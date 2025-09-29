use core::cell::RefCell;

use embassy_nrf::radio::{Instance, InterruptHandler};
use embassy_nrf::{interrupt, Peri};
use embassy_sync::blocking_mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::raw;

/// Radio error
// TODO: Extend the error codes with additional information
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    ScheduleTransmit,
    EnterReceive,
    Transmit,
    Receive,
}

// TODO expose the other variants in `pac::CCAMODE_A`
/// Clear Channel Assessment method
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Cca {
    /// Carrier sense
    CarrierSense,
    /// Energy Detection / Energy Above Threshold
    EnergyDetection {
        /// Energy measurements above this value mean that the channel is assumed to be busy.
        /// Note the measurement range is 0..0xFF - where 0 means that the received power was
        /// less than 10 dB above the selected receiver sensitivity. This value is not given in dBm,
        /// but can be converted. See the nrf52840 Product Specification Section 6.20.12.4
        /// for details.
        ed_threshold: u8,
    },
}

/// An IEEE 802.15.4 packet
///
/// This `Packet` is a PHY layer packet. It's made up of the physical header (PHR) and the PSDU
/// (PHY service data unit). The PSDU of this `Packet` will always include the MAC level CRC, AKA
/// the FCS (Frame Control Sequence) -- the CRC is fully computed in hardware and automatically
/// appended on transmission and verified on reception.
///
/// The API lets users modify the usable part (not the CRC) of the PSDU via the `deref` and
/// `copy_from_slice` methods. These methods will automatically update the PHR.
///
/// See figure 119 in the Product Specification of the nRF52840 for more details
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Packet {
    buffer: [u8; Self::SIZE],
}

// See figure 124 in nRF52840-PS
impl Packet {
    // For indexing purposes
    const PHY_HDR: usize = 0;
    const DATA: core::ops::RangeFrom<usize> = 1..;

    /// Maximum amount of usable payload (CRC excluded) a single packet can contain, in bytes
    pub const CAPACITY: u8 = 125;

    const CRC: u8 = 2; // Size of the CRC, which is *never* copied to / from RAM
    const MAX_PSDU_LEN: u8 = Self::CAPACITY + Self::CRC;
    const SIZE: usize = 1 /* PHR */ + Self::MAX_PSDU_LEN as usize;

    /// Create an empty packet (length = 0)
    pub const fn new() -> Self {
        let mut packet = Self {
            buffer: [0; Self::SIZE],
        };
        packet.buffer[Self::PHY_HDR] = Self::CRC;
        packet
    }

    /// Fill the packet payload with given `src` data
    ///
    /// # Panics
    ///
    /// This function panics if `src` is larger than `Self::CAPACITY`
    pub fn copy_from_slice(&mut self, src: &[u8]) {
        assert!(src.len() <= Self::CAPACITY as usize);
        let len = src.len() as u8;
        self.buffer[Self::DATA][..len as usize].copy_from_slice(&src[..len.into()]);
        self.set_len(len);
    }

    /// Return the size of this packet's payload
    pub const fn len(&self) -> u8 {
        self.buffer[Self::PHY_HDR] - Self::CRC
    }

    /// Return `true` if this packet's payload is empty
    pub const fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Change the size of the packet's payload
    ///
    /// # Panics
    ///
    /// This function panics if `len` is larger than `Self::CAPACITY`
    pub fn set_len(&mut self, len: u8) {
        assert!(len <= Self::CAPACITY);
        self.buffer[Self::PHY_HDR] = len + Self::CRC;
    }

    /// Return the LQI (Link Quality Indicator) of the received packet
    ///
    /// Note that the LQI is stored in the `Packet`'s internal buffer by the hardware so the value
    /// returned by this method is only valid after a `Radio.recv` operation. Operations that
    /// modify the `Packet`, like `copy_from_slice` or `set_len`+`deref_mut`, will overwrite the
    /// stored LQI value.
    ///
    /// Also note that the hardware will *not* compute a LQI for packets smaller than 3 bytes so
    /// this method will return an invalid value for those packets.
    pub const fn lqi(&self) -> u8 {
        self.buffer[1 /* PHY_HDR */ + self.len() as usize /* data */]
    }
}

impl Default for Packet {
    fn default() -> Self {
        Self::new()
    }
}

impl core::ops::Deref for Packet {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        &self.buffer[Self::DATA][..self.len() as usize]
    }
}

impl core::ops::DerefMut for Packet {
    fn deref_mut(&mut self) -> &mut [u8] {
        let len = self.len();
        &mut self.buffer[Self::DATA][..len as usize]
    }
}

/// Details of the received frame
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxFrameDetails {
    /// Received signal power in dBm
    pub power: i8,
    /// Timestamp taken when the last symbol of the frame was received
    pub time: Option<u64>,
}

/// Details of the received transmit ACK frame
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TxAckFrameDetails {
    /// Received signal power in dBm
    pub power: i8,
    /// Link Quality Indicator of the received ACK frame
    pub lqi: u8,
    /// Timestamp taken when the last symbol of the frame was received
    pub time: u64,
}

/// IEEE 802.15.4 radio driver.
pub struct Radio<'d, T: Instance> {
    _p: Peri<'d, T>,
}

impl<'d, T: Instance> Radio<'d, T> {
    /// Create a new IEEE 802.15.4 radio driver.
    pub fn new(
        radio: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        unsafe {
            raw::nrf_802154_init();
        }

        // // Enable NVIC interrupt
        // T::Interrupt::unpend();
        // unsafe { T::Interrupt::enable() };

        unsafe {
            raw::nrf_802154_channel_set(11);
            raw::nrf_802154_tx_power_set(0);
            raw::nrf_802154_cca_cfg_set(&raw::nrf_802154_cca_cfg_t {
                mode: raw::NRF_RADIO_CCA_MODE_CARRIER,
                ed_threshold: 0,
                corr_threshold: 0,
                corr_limit: 0,
            });
        }

        Self { _p: radio }
    }

    /// Change the radio channel
    pub fn set_channel(&mut self, channel: u8) {
        if !(11..=26).contains(&channel) {
            panic!("Bad 802.15.4 channel");
        }
        unsafe {
            raw::nrf_802154_channel_set(channel);
        }
    }

    /// Change the Clear Channel Assessment method
    pub fn set_cca(&mut self, _cca: Cca) {
        // TODO
    }

    /// Change the radio transmission power
    pub fn set_transmission_power(&mut self, power: i8) {
        unsafe {
            raw::nrf_802154_tx_power_set(power);
        }
    }

    /// Move the radio from any state to the DISABLED state
    fn disable(&mut self) {
        // TODO: Is this even supported in the C driver?
    }

    /// Receive one radio packet
    ///
    /// # Arguments
    /// - `packet`: A buffer where the received packet will be copied to. The buffer must be at
    ///   least `Packet::CAPACITY` bytes long.
    ///
    /// # Returns
    /// - `Ok(RxFrameDetails)` if a packet was successfully received
    /// - `Err(Error::EnterReceive)` if the radio could not enter receive mode
    /// - `Err(Error::Receive)` if the reception failed (CRC error, aborted, etc)
    pub async fn receive(&mut self, packet: &mut Packet) -> Result<RxFrameDetails, Error> {
        let receive_entered = unsafe { raw::nrf_802154_receive() };

        if !receive_entered {
            return Err(Error::EnterReceive);
        }

        let status = RadioState::wait(|state| {
            if matches!(state.status, RadioStatus::ReceiveDone { .. }) {
                packet.copy_from_slice(&state.rx);
            }

            matches!(
                state.status,
                RadioStatus::ReceiveFailed(_) | RadioStatus::ReceiveDone { .. }
            )
            .then_some(state.status)
        })
        .await;

        if let RadioStatus::ReceiveDone(details) = status {
            Ok(details)
        } else {
            Err(Error::Receive)
        }
    }

    /// Transmit one radio packet
    ///
    /// # Arguments
    /// - `packet`: The packet to transmit.
    /// - `cca`: If `true`, perform Clear Channel Assessment (CCA) before transmission.
    /// - `ack_packet`: If the radio is configured to wait for ACK frame in response to its transmission,
    ///   this buffer will be filled with the received ACK frame.
    ///   In either case, `None` can be passed if the user is not interested in the ACK frame.
    ///
    /// # Returns
    /// - `Ok(Some(TxAckFrameDetails))` if the packet was successfully transmitted and the radio is configured
    ///   to wait for an ACK frame, which was received.
    /// - `Ok(None)` if the packet was successfully transmitted and the radio is not configured
    ///   to wait for an ACK frame.
    /// - `Err(Error::ScheduleTransmit)` if the transmission could not be scheduled (radio busy, etc)
    /// - `Err(Error::Transmit)` if the transmission failed (no ACK received, etc)
    pub async fn transmit(
        &mut self,
        packet: &Packet,
        cca: bool,
        mut ack_packet: Option<&mut Packet>,
    ) -> Result<Option<TxAckFrameDetails>, Error> {
        // TODO: Potential race condition if the radio is scheduled to transmit but not transmitting yet
        let packet_data = RadioState::wait(|state| {
            if !matches!(state.status, RadioStatus::Transmitting) {
                state.tx.copy_from_slice(packet);

                let packet_data: &mut [u8] = &mut state.tx;

                Some(packet_data.as_mut_ptr())
            } else {
                None
            }
        })
        .await;

        let scheduled = unsafe {
            raw::nrf_802154_transmit_raw(
                packet_data,
                &raw::nrf_802154_transmit_metadata_t {
                    frame_props: raw::nrf_802154_transmitted_frame_props_t {
                        is_secured: false,
                        dynamic_data_is_set: false,
                    },
                    cca,
                    tx_power: raw::nrf_802154_tx_power_metadata_t {
                        use_metadata_value: false,
                        power: 0,
                    },
                    tx_channel: raw::nrf_802154_tx_channel_metadata_t {
                        use_metadata_value: false,
                        channel: 0,
                    },
                },
            )
        };

        if !scheduled {
            return Err(Error::ScheduleTransmit);
        }

        let status = RadioState::wait(|state| {
            if matches!(state.status, RadioStatus::TransmitDone(_)) {
                if let Some(ack_packet) = ack_packet.as_mut() {
                    ack_packet.copy_from_slice(&state.tx_ack);
                }
            }

            matches!(
                state.status,
                RadioStatus::TransmitFailed(_) | RadioStatus::TransmitDone(_)
            )
            .then_some(state.status)
        })
        .await;

        if let RadioStatus::TransmitDone(rx_pdu_details) = status {
            Ok(rx_pdu_details)
        } else {
            Err(Error::Transmit)
        }
    }
}

impl<T> Drop for Radio<'_, T>
where
    T: Instance,
{
    fn drop(&mut self) {
        self.disable();

        unsafe {
            raw::nrf_802154_deinit();
        }

        STATE.lock(|state| {
            let mut state = state.borrow_mut();
            state.status = RadioStatus::Idle;
        });
    }
}

// TODO: Think if we need `nrf_802154_state_t`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum RadioStatus {
    Idle,
    ReceiveFailed(raw::nrf_802154_tx_error_t),
    ReceiveDone(RxFrameDetails),
    CcaFailed(raw::nrf_802154_cca_error_t),
    CcaDone(bool),
    EnergyDetectionDetected(i8),
    EnergyDetectionFailed(raw::nrf_802154_ed_error_t),
    Transmitting,
    TxAckStarted,
    TransmitFailed(raw::nrf_802154_tx_error_t),
    TransmitDone(Option<TxAckFrameDetails>),
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct RadioState {
    status: RadioStatus,
    tx: Packet,
    tx_ack: Packet,
    rx: Packet,
}

impl RadioState {
    const fn new() -> Self {
        Self {
            status: RadioStatus::Idle,
            tx: Packet::new(),
            tx_ack: Packet::new(),
            rx: Packet::new(),
        }
    }

    async fn wait<F, R>(mut f: F) -> R
    where
        F: FnMut(&mut RadioState) -> Option<R>,
    {
        loop {
            if let Some(result) = STATE.lock(|state| f(&mut state.borrow_mut())) {
                break result;
            }

            STATE_SIGNAL.wait().await;
        }
    }

    fn update<F, R>(f: F)
    where
        F: FnOnce(&mut RadioState) -> R,
    {
        STATE.lock(|state| {
            let mut state = state.borrow_mut();
            f(&mut state);

            STATE_SIGNAL.signal(());
        });
    }
}

static STATE: blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<RadioState>> =
    blocking_mutex::Mutex::new(RefCell::new(RadioState::new()));

static STATE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[no_mangle]
unsafe extern "C" fn nrf_802154_cca_done(channel_free: bool) {
    RadioState::update(|state| state.status = RadioStatus::CcaDone(channel_free));
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_cca_failed(error: raw::nrf_802154_cca_error_t) {
    RadioState::update(|state| state.status = RadioStatus::CcaFailed(error));
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_energy_detected(p_result: *const raw::nrf_802154_energy_detected_t) {
    RadioState::update(|state| {
        state.status = RadioStatus::EnergyDetectionDetected(unsafe { p_result.as_ref().unwrap().ed_dbm })
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_energy_detection_failed(error: raw::nrf_802154_ed_error_t) {
    RadioState::update(|state| state.status = RadioStatus::EnergyDetectionFailed(error));
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_tx_ack_started() {
    RadioState::update(|state| state.status = RadioStatus::TxAckStarted);
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_received_raw(p_data: *mut u8, power: i8, len: u8) {
    RadioState::update(|state| {
        state
            .rx
            .copy_from_slice(core::slice::from_raw_parts(p_data, len as usize));

        state.status = RadioStatus::ReceiveDone(RxFrameDetails { power, time: None });

        unsafe {
            raw::nrf_802154_buffer_free_raw(p_data);
        }
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_received_timestamp_raw(p_data: *mut u8, power: i8, len: u8, time: u64) {
    RadioState::update(|state| {
        state
            .rx
            .copy_from_slice(core::slice::from_raw_parts(p_data, len as usize));

        state.status = RadioStatus::ReceiveDone(RxFrameDetails {
            power,
            time: Some(time),
        });

        unsafe {
            raw::nrf_802154_buffer_free_raw(p_data);
        }
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_receive_failed(error: raw::nrf_802154_rx_error_t, _id: u32) {
    RadioState::update(|state| state.status = RadioStatus::ReceiveFailed(error));
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_transmitted_raw(
    _p_frame: *mut u8,
    p_metadata: *const raw::nrf_802154_transmit_done_metadata_t,
) {
    RadioState::update(|state| {
        let p_metadata = unsafe { p_metadata.as_ref().unwrap() };
        if !p_metadata.data.transmitted.p_ack.is_null() {
            let len = p_metadata.data.transmitted.length;

            state.tx_ack.copy_from_slice(unsafe {
                core::slice::from_raw_parts(p_metadata.data.transmitted.p_ack, len as usize)
            });

            state.status = RadioStatus::TransmitDone(Some(TxAckFrameDetails {
                power: p_metadata.data.transmitted.power,
                lqi: p_metadata.data.transmitted.lqi,
                time: p_metadata.data.transmitted.time,
            }));

            unsafe {
                raw::nrf_802154_buffer_free_raw(p_metadata.data.transmitted.p_ack);
            }
        } else {
            state.tx_ack.set_len(0);
        }
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_transmit_failed(
    _p_frame: *mut u8,
    error: raw::nrf_802154_tx_error_t,
    _p_metadata: *const raw::nrf_802154_transmit_done_metadata_t,
) {
    RadioState::update(|state| state.status = RadioStatus::TransmitFailed(error));
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_tx_started(_p_frame: *const u8) {
    RadioState::update(|state| state.status = RadioStatus::Transmitting);
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_custom_part_of_radio_init() {}
