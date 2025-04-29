use core::cell::RefCell;
use core::ffi::CStr;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};
use core::task::{Poll, Waker};

use bt_hci::cmd::le::LeSetPeriodicAdvResponseData;
use bt_hci::cmd::Cmd;
use bt_hci::controller::blocking::TryError;
use bt_hci::controller::{blocking, Controller};
use bt_hci::event::EventParams;
use bt_hci::{AsHciBytes, FixedSizeValue, FromHciBytes, WriteHci};
use embassy_nrf::{peripherals, Peri};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::waitqueue::AtomicWaker;
use embedded_io::ErrorType;
use nrf_mpsl::MultiprotocolServiceLayer;
use rand_core::CryptoRngCore;
use raw::{
    sdc_cfg_adv_buffer_cfg_t, sdc_cfg_buffer_cfg_t, sdc_cfg_buffer_count_t, sdc_cfg_role_count_t, sdc_cfg_t,
    SDC_CFG_TYPE_NONE, SDC_DEFAULT_RESOURCE_CFG_TAG,
};

use crate::{raw, Error, RetVal};

static WAKER: AtomicWaker = AtomicWaker::new();
static SDC_RNG: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

/// Struct containing all peripherals required for the SDC to operate.
///
/// This is used to enforce at compile-time that your code doesn't use
/// these peripherals.
///
/// However, there's extra restrictions that are not enforced at compile-time
/// that you must ensure to fulfill manually:
///
/// - Do not use the `ECB`, `CCM` or `AAR` peripherals directly.
pub struct Peripherals<'d> {
    pub ppi_ch17: Peri<'d, peripherals::PPI_CH17>,
    pub ppi_ch18: Peri<'d, peripherals::PPI_CH18>,
    pub ppi_ch20: Peri<'d, peripherals::PPI_CH20>,
    pub ppi_ch21: Peri<'d, peripherals::PPI_CH21>,
    pub ppi_ch22: Peri<'d, peripherals::PPI_CH22>,
    pub ppi_ch23: Peri<'d, peripherals::PPI_CH23>,
    pub ppi_ch24: Peri<'d, peripherals::PPI_CH24>,
    pub ppi_ch25: Peri<'d, peripherals::PPI_CH25>,
    pub ppi_ch26: Peri<'d, peripherals::PPI_CH26>,
    pub ppi_ch27: Peri<'d, peripherals::PPI_CH27>,
    pub ppi_ch28: Peri<'d, peripherals::PPI_CH28>,
    pub ppi_ch29: Peri<'d, peripherals::PPI_CH29>,
}

impl<'d> Peripherals<'d> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        ppi_ch17: Peri<'d, peripherals::PPI_CH17>,
        ppi_ch18: Peri<'d, peripherals::PPI_CH18>,
        ppi_ch20: Peri<'d, peripherals::PPI_CH20>,
        ppi_ch21: Peri<'d, peripherals::PPI_CH21>,
        ppi_ch22: Peri<'d, peripherals::PPI_CH22>,
        ppi_ch23: Peri<'d, peripherals::PPI_CH23>,
        ppi_ch24: Peri<'d, peripherals::PPI_CH24>,
        ppi_ch25: Peri<'d, peripherals::PPI_CH25>,
        ppi_ch26: Peri<'d, peripherals::PPI_CH26>,
        ppi_ch27: Peri<'d, peripherals::PPI_CH27>,
        ppi_ch28: Peri<'d, peripherals::PPI_CH28>,
        ppi_ch29: Peri<'d, peripherals::PPI_CH29>,
    ) -> Self {
        Peripherals {
            ppi_ch17,
            ppi_ch18,
            ppi_ch20,
            ppi_ch21,
            ppi_ch22,
            ppi_ch23,
            ppi_ch24,
            ppi_ch25,
            ppi_ch26,
            ppi_ch27,
            ppi_ch28,
            ppi_ch29,
        }
    }
}

unsafe extern "C" fn fault_handler(file: *const core::ffi::c_char, line: u32) {
    panic!(
        "SoftdeviceController: {}:{}",
        // SAFETY: the SDC should always give us valid utf8 strings.
        unsafe { core::str::from_utf8_unchecked(CStr::from_ptr(file).to_bytes()) },
        line
    )
}

extern "C" fn sdc_callback() {
    WAKER.wake()
}

/// SAFETY: This function must only be called while a `SoftdeviceController` instance
/// is alive, with the same type parameter as was passed to `Builder::build`, and must
/// be synchronized with other MPSL accesses.
///
/// The softdevice controller calls this function exclusively from `mpsl_low_priority_process`,
/// which is appropriately synchronized.
unsafe extern "C" fn rand_blocking<R: CryptoRngCore + Send>(p_buff: *mut u8, length: u8) {
    let rng = SDC_RNG.load(Ordering::Acquire) as *mut R;
    if rng.is_null() {
        panic!("rand_blocking called from Softdevice Controller when no rng is set");
    }
    let buf = core::slice::from_raw_parts_mut(p_buff, usize::from(length));
    (*rng).fill_bytes(buf);
}

#[repr(align(8))]
pub struct Mem<const N: usize>(MaybeUninit<[u8; N]>);

impl<const N: usize> Mem<N> {
    pub fn new() -> Self {
        Self(MaybeUninit::uninit())
    }
}

impl<const N: usize> Default for Mem<N> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct Builder {
    // Prevent Send, Sync
    _private: PhantomData<*mut ()>,
}

impl Builder {
    pub fn new() -> Result<Self, Error> {
        let ret = unsafe { raw::sdc_init(Some(fault_handler)) };
        RetVal::from(ret).to_result().and(Ok(Builder { _private: PhantomData }))
    }

    pub fn central_count(self, count: u8) -> Result<Self, Error> {
        self.cfg_set(
            raw::SDC_CFG_TYPE_CENTRAL_COUNT,
            sdc_cfg_t {
                central_count: sdc_cfg_role_count_t { count },
            },
        )
    }

    pub fn peripheral_count(self, count: u8) -> Result<Self, Error> {
        self.cfg_set(
            raw::SDC_CFG_TYPE_PERIPHERAL_COUNT,
            sdc_cfg_t {
                peripheral_count: sdc_cfg_role_count_t { count },
            },
        )
    }

    pub fn buffer_cfg(
        self,
        tx_packet_size: u16,
        rx_packet_size: u16,
        tx_packet_count: u8,
        rx_packet_count: u8,
    ) -> Result<Self, Error> {
        self.cfg_set(
            raw::SDC_CFG_TYPE_BUFFER_CFG,
            sdc_cfg_t {
                buffer_cfg: sdc_cfg_buffer_cfg_t {
                    tx_packet_size,
                    rx_packet_size,
                    tx_packet_count,
                    rx_packet_count,
                },
            },
        )
    }

    pub fn adv_count(self, count: u8) -> Result<Self, Error> {
        self.cfg_set(
            raw::SDC_CFG_TYPE_ADV_COUNT,
            sdc_cfg_t {
                adv_count: sdc_cfg_role_count_t { count },
            },
        )
    }

    pub fn scan_buffer_cfg(self, count: u8) -> Result<Self, Error> {
        self.cfg_set(
            raw::SDC_CFG_TYPE_SCAN_BUFFER_CFG,
            sdc_cfg_t {
                scan_buffer_cfg: sdc_cfg_buffer_count_t { count },
            },
        )
    }

    pub fn adv_buffer_cfg(self, max_adv_data: u16) -> Result<Self, Error> {
        self.cfg_set(
            raw::SDC_CFG_TYPE_ADV_BUFFER_CFG,
            sdc_cfg_t {
                adv_buffer_cfg: sdc_cfg_adv_buffer_cfg_t { max_adv_data },
            },
        )
    }

    pub fn periodic_adv_count(self, count: u8) -> Result<Self, Error> {
        self.cfg_set(
            raw::SDC_CFG_TYPE_PERIODIC_ADV_COUNT,
            sdc_cfg_t {
                periodic_adv_count: sdc_cfg_role_count_t { count },
            },
        )
    }

    pub fn periodic_sync_count(self, count: u8) -> Result<Self, Error> {
        self.cfg_set(
            raw::SDC_CFG_TYPE_PERIODIC_SYNC_COUNT,
            sdc_cfg_t {
                periodic_sync_count: sdc_cfg_role_count_t { count },
            },
        )
    }

    pub fn periodic_sync_buffer_cfg(self, count: u8) -> Result<Self, Error> {
        self.cfg_set(
            raw::SDC_CFG_TYPE_PERIODIC_SYNC_BUFFER_CFG,
            sdc_cfg_t {
                periodic_sync_buffer_cfg: sdc_cfg_buffer_count_t { count },
            },
        )
    }

    pub fn periodic_adv_list_len(self, len: u8) -> Result<Self, Error> {
        self.cfg_set(
            raw::SDC_CFG_TYPE_PERIODIC_ADV_LIST_SIZE,
            sdc_cfg_t {
                periodic_adv_list_size: len,
            },
        )
    }

    pub fn support_adv(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_adv)
    }

    pub fn support_ext_adv(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_ext_adv)
    }

    pub fn support_peripheral(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_peripheral)
    }

    pub fn support_scan(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_scan)
    }

    pub fn support_ext_scan(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_ext_scan)
    }

    pub fn support_central(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_central)
    }

    pub fn support_ext_central(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_ext_central)
    }

    pub fn support_dle_central(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_dle_central)
    }

    pub fn support_dle_peripheral(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_dle_peripheral)
    }

    pub fn support_le_2m_phy(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_le_2m_phy)
    }

    pub fn support_le_coded_phy(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_le_coded_phy)
    }

    pub fn support_phy_update_central(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_phy_update_central)
    }

    pub fn support_phy_update_peripheral(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_phy_update_peripheral)
    }

    pub fn support_le_periodic_adv(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_le_periodic_adv)
    }

    pub fn support_le_periodic_sync(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_le_periodic_sync)
    }

    pub fn support_le_power_control_central(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_le_power_control_central)
    }

    pub fn support_le_power_control_peripheral(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_le_power_control_peripheral)
    }

    pub fn support_sca_central(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_sca_central)
    }

    pub fn support_sca_peripheral(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_sca_peripheral)
    }

    pub fn support_le_conn_cte_rsp_central(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_le_conn_cte_rsp_central)
    }

    pub fn support_le_conn_cte_rsp_peripheral(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_le_conn_cte_rsp_peripheral)
    }

    pub fn support_periodic_adv_sync_transfer_sender_central(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_periodic_adv_sync_transfer_sender_central)
    }

    pub fn support_periodic_adv_sync_transfer_sender_peripheral(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_periodic_adv_sync_transfer_sender_peripheral)
    }

    pub fn support_periodic_adv_sync_transfer_receiver_central(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_periodic_adv_sync_transfer_receiver_central)
    }

    pub fn support_periodic_adv_sync_transfer_receiver_peripheral(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_periodic_adv_sync_transfer_receiver_peripheral)
    }

    pub fn support_qos_channel_survey(self) -> Result<Self, Error> {
        self.support(raw::sdc_support_qos_channel_survey)
    }

    pub fn default_tx_power(self, dbm: i8) -> Result<Self, Error> {
        RetVal::from(unsafe { raw::sdc_default_tx_power_set(dbm) })
            .to_result()
            .and(Ok(self))
    }

    pub fn required_memory(&self) -> Result<usize, Error> {
        let ret = unsafe {
            raw::sdc_cfg_set(
                SDC_DEFAULT_RESOURCE_CFG_TAG as u8,
                SDC_CFG_TYPE_NONE as u8,
                core::ptr::null(),
            )
        };
        RetVal::from(ret).to_result().map(|x| x as usize)
    }

    /// SAFETY: `SoftdeviceController` must not have its lifetime end without running its destructor, e.g. using `mem::forget`.
    pub fn build<'d, R: CryptoRngCore + Send, const N: usize>(
        self,
        p: Peripherals<'d>,
        rng: &'d mut R,
        mpsl: &'d MultiprotocolServiceLayer,
        mem: &'d mut Mem<N>,
    ) -> Result<SoftdeviceController<'d>, Error> {
        // Peripherals are used by the Softdevice Controller library, so we merely take ownership and ignore them
        let _ = (p, mpsl);

        let required = self.required_memory()?;
        match N.cmp(&required) {
            core::cmp::Ordering::Less => {
                error!("Memory buffer too small. {} bytes needed.", required);
                return Err(Error::EINVAL);
            }
            core::cmp::Ordering::Equal => (),
            core::cmp::Ordering::Greater => {
                warn!("Memory buffer too big. {} bytes needed", required);
            }
        }

        unwrap!(
            SDC_RNG.compare_exchange(
                core::ptr::null_mut(),
                rng as *mut _ as _,
                Ordering::Release,
                Ordering::Relaxed
            ),
            "SoftdeviceController already initialized!"
        );
        let rand_source = raw::sdc_rand_source_t {
            rand_poll: Some(rand_blocking::<R>),
        };
        let ret = unsafe { raw::sdc_rand_source_register(&rand_source) };
        RetVal::from(ret).to_result()?;

        let ret = unsafe { raw::sdc_enable(Some(sdc_callback), mem.0.as_mut_ptr() as *mut _) };
        RetVal::from(ret).to_result().and(Ok(SoftdeviceController {
            using_ext_adv_cmds: Default::default(),
            periodic_adv_response_data_in_progress: AtomicBool::new(false),
            periodic_adv_response_data_complete: Signal::new(),
            _private: PhantomData,
        }))
    }

    #[inline]
    fn support(self, f: unsafe extern "C" fn() -> i32) -> Result<Self, Error> {
        RetVal::from(unsafe { f() }).to_result().and(Ok(self))
    }

    fn cfg_set(self, config_type: u32, value: sdc_cfg_t) -> Result<Self, Error> {
        let ret = unsafe { raw::sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG as u8, config_type as u8, &value) };
        RetVal::from(ret).to_result().and(Ok(self))
    }
}

pub struct SoftdeviceController<'d> {
    using_ext_adv_cmds: RefCell<Option<bool>>,
    periodic_adv_response_data_in_progress: AtomicBool,
    periodic_adv_response_data_complete: Signal<NoopRawMutex, (bt_hci::param::Status, bt_hci::param::SyncHandle)>,
    // Prevent Send, Sync
    _private: PhantomData<&'d *mut ()>,
}

impl Drop for SoftdeviceController<'_> {
    fn drop(&mut self) {
        unsafe { raw::sdc_disable() };
        SDC_RNG.store(core::ptr::null_mut(), Ordering::Release);
    }
}

#[inline(always)]
unsafe fn bytes_of<T: Copy>(t: &T) -> &[u8] {
    let len = core::mem::size_of::<T>();
    core::slice::from_raw_parts(t as *const _ as *const u8, len)
}

impl<'d> SoftdeviceController<'d> {
    pub fn build_revision() -> Result<[u8; raw::SDC_BUILD_REVISION_SIZE as usize], Error> {
        let mut rev = [0; raw::SDC_BUILD_REVISION_SIZE as usize];
        let ret = unsafe { raw::sdc_build_revision_get(rev.as_mut_ptr()) };
        RetVal::from(ret).to_result().and(Ok(rev))
    }

    pub fn hci_data_put(&self, buf: &[u8]) -> Result<(), Error> {
        assert!(buf.len() >= 4 && buf.len() >= 4 + usize::from(u16::from_le_bytes([buf[2], buf[3]])));
        RetVal::from(unsafe { raw::sdc_hci_data_put(buf.as_ptr()) })
            .to_result()
            .and(Ok(()))
    }

    pub fn hci_iso_data_put(&self, buf: &[u8]) -> Result<(), Error> {
        assert!(buf.len() >= 4 && buf.len() >= 4 + usize::from(u16::from_le_bytes([buf[2], buf[3]])));
        RetVal::from(unsafe { raw::sdc_hci_iso_data_put(buf.as_ptr()) })
            .to_result()
            .and(Ok(()))
    }

    pub fn try_hci_get(&self, buf: &mut [u8]) -> Result<bt_hci::PacketKind, Error> {
        assert!(buf.len() >= raw::HCI_MSG_BUFFER_MAX_SIZE as usize);
        let mut msg_type: raw::sdc_hci_msg_type_t = 0;

        let ret = unsafe { raw::sdc_hci_get(buf.as_mut_ptr(), (&mut msg_type) as *mut _ as *mut u8) };
        RetVal::from(ret).to_result()?;
        let kind = match msg_type {
            raw::SDC_HCI_MSG_TYPE_DATA => bt_hci::PacketKind::AclData,
            raw::SDC_HCI_MSG_TYPE_EVT => bt_hci::PacketKind::Event,
            _ => unreachable!(),
        };

        // Check for a CommandComplete packet for an LeSetPeriodicAdvResponseData command
        if let bt_hci::PacketKind::Event = kind {
            if let Ok((header, data)) = bt_hci::event::EventPacketHeader::from_hci_bytes(buf) {
                if header.code == bt_hci::event::CommandComplete::EVENT_CODE {
                    if let Ok(event) =
                        bt_hci::event::CommandComplete::from_hci_bytes_complete(&data[..usize::from(header.params_len)])
                    {
                        if event.cmd_opcode == LeSetPeriodicAdvResponseData::OPCODE {
                            if let Ok(return_params) = event.return_params::<LeSetPeriodicAdvResponseData>() {
                                self.periodic_adv_response_data_complete
                                    .signal((event.status, return_params));
                                return Err(Error::EAGAIN);
                            }
                        }
                    }
                }
            }
        }

        Ok(kind)
    }

    pub async fn hci_get(&self, buf: &mut [u8]) -> Result<bt_hci::PacketKind, Error> {
        poll_fn(|ctx| match self.try_hci_get(buf) {
            Err(Error::EAGAIN) => {
                WAKER.register(ctx.waker());
                Poll::Pending
            }
            res => Poll::Ready(res),
        })
        .await
    }

    pub fn register_waker(&self, waker: &Waker) {
        WAKER.register(waker);
    }

    #[inline(always)]
    unsafe fn raw_cmd(&self, f: unsafe extern "C" fn() -> u8) -> Result<(), bt_hci::param::Error> {
        bt_hci::param::Status::from(f()).to_result()
    }

    #[inline(always)]
    unsafe fn raw_cmd_params<P1: FixedSizeValue, P2: Copy>(
        &self,
        f: unsafe extern "C" fn(*const P2) -> u8,
        params: &P1,
    ) -> Result<(), bt_hci::param::Error> {
        debug_assert_eq!(core::mem::size_of::<P1>(), core::mem::size_of::<P2>());
        bt_hci::param::Status::from(f(params.as_hci_bytes().as_ptr() as *const _)).to_result()
    }

    #[inline(always)]
    unsafe fn raw_cmd_return<R1: FixedSizeValue, R2: Copy>(
        &self,
        f: unsafe extern "C" fn(*mut R2) -> u8,
    ) -> Result<R1, bt_hci::param::Error> {
        debug_assert_eq!(core::mem::size_of::<R1>(), core::mem::size_of::<R2>());
        let mut out = core::mem::zeroed();
        bt_hci::param::Status::from(f(&mut out)).to_result()?;
        Ok(unwrap!(R1::from_hci_bytes_complete(bytes_of(&out))))
    }

    #[inline(always)]
    unsafe fn raw_cmd_params_return<P1: FixedSizeValue, R1: FixedSizeValue, P2: Copy, R2: Copy>(
        &self,
        f: unsafe extern "C" fn(*const P2, *mut R2) -> u8,
        params: &P1,
    ) -> Result<R1, bt_hci::param::Error> {
        debug_assert_eq!(core::mem::size_of::<P1>(), core::mem::size_of::<P2>());
        debug_assert_eq!(core::mem::size_of::<R1>(), core::mem::size_of::<R2>());

        let mut out = core::mem::zeroed();
        bt_hci::param::Status::from(f(params.as_hci_bytes().as_ptr() as *const _, &mut out)).to_result()?;
        Ok(unwrap!(R1::from_hci_bytes_complete(bytes_of(&out))))
    }

    fn check_adv_cmd(&self, ext_adv: bool) -> Result<(), bt_hci::param::Error> {
        let mut using_ext_adv = self.using_ext_adv_cmds.borrow_mut();
        match *using_ext_adv {
            None => {
                *using_ext_adv = Some(ext_adv);
                Ok(())
            }
            Some(x) if x == ext_adv => Ok(()),
            _ => Err(bt_hci::param::Error::CMD_DISALLOWED),
        }
    }
}

impl ErrorType for SoftdeviceController<'_> {
    type Error = Error;
}

impl<'a> Controller for SoftdeviceController<'a> {
    async fn write_acl_data(&self, packet: &bt_hci::data::AclPacket<'_>) -> Result<(), Self::Error> {
        let mut buf = [0u8; raw::HCI_DATA_PACKET_MAX_SIZE as usize];
        packet.write_hci(buf.as_mut_slice()).map_err(|err| match err {
            embedded_io::SliceWriteError::Full => Error::ENOMEM,
            _ => unreachable!(),
        })?;
        self.hci_data_put(buf.as_slice())
    }

    async fn write_sync_data(&self, _packet: &bt_hci::data::SyncPacket<'_>) -> Result<(), Self::Error> {
        unimplemented!()
    }

    async fn write_iso_data(&self, packet: &bt_hci::data::IsoPacket<'_>) -> Result<(), Self::Error> {
        let mut buf = [0u8; raw::HCI_DATA_PACKET_MAX_SIZE as usize];
        packet.write_hci(buf.as_mut_slice()).map_err(|err| match err {
            embedded_io::SliceWriteError::Full => Error::ENOMEM,
            _ => unreachable!(),
        })?;
        self.hci_iso_data_put(buf.as_slice())
    }

    async fn read<'b>(&self, buf: &'b mut [u8]) -> Result<bt_hci::ControllerToHostPacket<'b>, Self::Error> {
        let kind = self.hci_get(buf).await?;
        bt_hci::ControllerToHostPacket::from_hci_bytes_with_kind(kind, buf)
            .map(|(x, _)| x)
            .map_err(|err| match err {
                bt_hci::FromHciBytesError::InvalidSize => Error::ENOMEM,
                bt_hci::FromHciBytesError::InvalidValue => Error::EINVAL,
            })
    }
}

impl blocking::Controller for SoftdeviceController<'_> {
    fn try_write_acl_data(&self, packet: &bt_hci::data::AclPacket<'_>) -> Result<(), blocking::TryError<Self::Error>> {
        let mut buf = [0u8; raw::HCI_DATA_PACKET_MAX_SIZE as usize];
        packet
            .write_hci(buf.as_mut_slice())
            .map_err(|err| match err {
                embedded_io::SliceWriteError::Full => Error::ENOMEM,
                _ => unreachable!(),
            })
            .map_err(into_try_err)?;
        self.hci_data_put(buf.as_slice()).map_err(into_try_err)
    }

    fn try_write_sync_data(
        &self,
        _packet: &bt_hci::data::SyncPacket<'_>,
    ) -> Result<(), blocking::TryError<Self::Error>> {
        unimplemented!()
    }

    fn try_write_iso_data(&self, packet: &bt_hci::data::IsoPacket<'_>) -> Result<(), blocking::TryError<Self::Error>> {
        let mut buf = [0u8; raw::HCI_DATA_PACKET_MAX_SIZE as usize];
        packet
            .write_hci(buf.as_mut_slice())
            .map_err(|err| match err {
                embedded_io::SliceWriteError::Full => Error::ENOMEM,
                _ => unreachable!(),
            })
            .map_err(into_try_err)?;
        self.hci_iso_data_put(buf.as_slice()).map_err(into_try_err)
    }

    fn try_read<'b>(
        &self,
        buf: &'b mut [u8],
    ) -> Result<bt_hci::ControllerToHostPacket<'b>, blocking::TryError<Self::Error>> {
        let kind = self.try_hci_get(buf).map_err(into_try_err)?;
        bt_hci::ControllerToHostPacket::from_hci_bytes_with_kind(kind, buf)
            .map(|(x, _)| x)
            .map_err(|err| match err {
                bt_hci::FromHciBytesError::InvalidSize => Error::ENOMEM,
                bt_hci::FromHciBytesError::InvalidValue => Error::EINVAL,
            })
            .map_err(into_try_err)
    }

    fn write_acl_data(&self, packet: &bt_hci::data::AclPacket) -> Result<(), Self::Error> {
        loop {
            match self.try_write_acl_data(packet) {
                Ok(v) => return Ok(v),
                Err(TryError::Error(e)) => return Err(e),
                Err(TryError::Busy) => {}
            }
        }
    }

    fn write_sync_data(&self, packet: &bt_hci::data::SyncPacket) -> Result<(), Self::Error> {
        loop {
            match self.try_write_sync_data(packet) {
                Ok(v) => return Ok(v),
                Err(TryError::Error(e)) => return Err(e),
                Err(TryError::Busy) => {}
            }
        }
    }

    fn write_iso_data(&self, packet: &bt_hci::data::IsoPacket) -> Result<(), Self::Error> {
        loop {
            match self.try_write_iso_data(packet) {
                Ok(v) => return Ok(v),
                Err(TryError::Error(e)) => return Err(e),
                Err(TryError::Busy) => {}
            }
        }
    }

    fn read<'b>(&self, buf: &'b mut [u8]) -> Result<bt_hci::ControllerToHostPacket<'b>, Self::Error> {
        loop {
            // Safety: the buffer can be reused after try_read has returned.
            let buf = unsafe { core::slice::from_raw_parts_mut(buf.as_mut_ptr(), buf.len()) };
            match self.try_read(buf) {
                Ok(v) => return Ok(v),
                Err(TryError::Error(e)) => return Err(e),
                Err(TryError::Busy) => {}
            }
        }
    }
}

fn into_try_err(e: Error) -> blocking::TryError<Error> {
    match e {
        Error::EAGAIN => blocking::TryError::Busy,
        other => blocking::TryError::Error(other),
    }
}

macro_rules! sdc_cmd {
    (async $cmd:ty => $raw:ident()) => {
        sdc_cmd!(async $cmd => raw_cmd($raw));
    };

    (async $cmd:ty => $raw:ident(x)) => {
        sdc_cmd!(async $cmd => raw_cmd_params($raw, $cmd));
    };

    (async $cmd:ty => $method:ident($raw:ident$(, $params:ty)?) $($ext_adv:literal)?) => {
        #[automatically_derived]
        #[allow(unused_variables)]
        impl<'d> bt_hci::controller::ControllerCmdAsync<$cmd> for $crate::sdc::SoftdeviceController<'d> {
            async fn exec(&self, cmd: &$cmd) -> Result<(), bt_hci::cmd::Error<Self::Error>> {
                $(self.check_adv_cmd($ext_adv)?;)?
                unsafe { self.$method(raw::$raw$(, <$params as bt_hci::cmd::Cmd>::params(cmd))?) }.map_err(bt_hci::cmd::Error::Hci)
            }
        }
    };

    ($cmd:ty => $raw:ident()) => {
        sdc_cmd!($cmd => raw_cmd($raw));
    };

    ($cmd:ty => $raw:ident(x)) => {
        sdc_cmd!($cmd => raw_cmd_params($raw, $cmd));
    };

    ($cmd:ty => $raw:ident() -> y) => {
        sdc_cmd!($cmd => raw_cmd_return($raw));
    };

    ($cmd:ty => $raw:ident(x) -> y) => {
        sdc_cmd!($cmd => raw_cmd_params_return($raw, $cmd));
    };

    ($cmd:ty => $method:ident($raw:ident$(, $params:ty)*) $($ext_adv:literal)?) => {
        #[automatically_derived]
        #[allow(unused_variables)]
        impl<'d> bt_hci::controller::ControllerCmdSync<$cmd> for $crate::sdc::SoftdeviceController<'d> {
            async fn exec(&self, cmd: &$cmd) -> Result<<$cmd as bt_hci::cmd::SyncCmd>::Return, bt_hci::cmd::Error<Self::Error>> {
                $(self.check_adv_cmd($ext_adv)?;)?
                let ret = unsafe { self.$method(raw::$raw$(, <$params as bt_hci::cmd::Cmd>::params(cmd))?) }?;
                Ok(ret)
            }
        }
    };
}

/// Bluetooth HCI Link Control commands (§7.1)
mod link_control {
    use bt_hci::cmd::link_control::*;

    use crate::raw;

    sdc_cmd!(Disconnect => sdc_hci_cmd_lc_disconnect(x));
    sdc_cmd!(async ReadRemoteVersionInformation => sdc_hci_cmd_lc_read_remote_version_information(x));
}

/// Bluetooth HCI Controller & Baseband commands (§7.3)
mod controller_baseband {
    use bt_hci::cmd::controller_baseband::*;
    use bt_hci::cmd::{Cmd, Error as CmdError};
    use bt_hci::controller::ControllerCmdSync;
    use bt_hci::param::ConnHandleCompletedPackets;
    use bt_hci::WriteHci;

    use crate::raw;

    impl<'d> ControllerCmdSync<Reset> for super::SoftdeviceController<'d> {
        async fn exec(&self, _cmd: &Reset) -> Result<<Reset as bt_hci::cmd::SyncCmd>::Return, CmdError<Self::Error>> {
            *self.using_ext_adv_cmds.borrow_mut() = None;
            unsafe { self.raw_cmd(raw::sdc_hci_cmd_cb_reset) }.map_err(bt_hci::cmd::Error::Hci)
        }
    }

    sdc_cmd!(SetEventMask => sdc_hci_cmd_cb_set_event_mask(x));
    sdc_cmd!(ReadTransmitPowerLevel => sdc_hci_cmd_cb_read_transmit_power_level(x) -> y);
    sdc_cmd!(SetControllerToHostFlowControl => sdc_hci_cmd_cb_set_controller_to_host_flow_control(x));
    sdc_cmd!(HostBufferSize => sdc_hci_cmd_cb_host_buffer_size(x));
    sdc_cmd!(SetEventMaskPage2 => sdc_hci_cmd_cb_set_event_mask_page_2(x));
    sdc_cmd!(ReadAuthenticatedPayloadTimeout => sdc_hci_cmd_cb_read_authenticated_payload_timeout(x) -> y);
    sdc_cmd!(WriteAuthenticatedPayloadTimeout => sdc_hci_cmd_cb_write_authenticated_payload_timeout(x) -> y);

    impl<'a, 'd> ControllerCmdSync<HostNumberOfCompletedPackets<'a>> for super::SoftdeviceController<'d> {
        async fn exec(
            &self,
            cmd: &HostNumberOfCompletedPackets<'a>,
        ) -> Result<<HostNumberOfCompletedPackets as bt_hci::cmd::SyncCmd>::Return, CmdError<Self::Error>> {
            const MAX_CONN_HANDLES: usize = 63;
            const N: usize = 1 + MAX_CONN_HANDLES + core::mem::size_of::<ConnHandleCompletedPackets>();
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_cb_host_number_of_completed_packets(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result().map_err(CmdError::Hci)
        }
    }
}

/// Bluetooth HCI Informational parameters (§7.4)
mod info {
    use bt_hci::cmd::info::*;

    use crate::raw;

    sdc_cmd!(ReadLocalVersionInformation => sdc_hci_cmd_ip_read_local_version_information() -> y);
    sdc_cmd!(ReadLocalSupportedCmds => sdc_hci_cmd_ip_read_local_supported_commands() -> y);
    sdc_cmd!(ReadLocalSupportedFeatures => sdc_hci_cmd_ip_read_local_supported_features() -> y);
    sdc_cmd!(ReadBdAddr => sdc_hci_cmd_ip_read_bd_addr() -> y);
}

/// Bluetooth HCI Status parameters (§7.5)
mod status {
    use bt_hci::cmd::status::*;

    use crate::raw;

    sdc_cmd!(ReadRssi => sdc_hci_cmd_sp_read_rssi(x) -> y);
}

/// Bluetooth HCI LE Controller commands (§7.8)
mod le {
    use core::sync::atomic::Ordering;

    use bt_hci::cmd::le::*;
    use bt_hci::cmd::{Cmd, Error as CmdError};
    use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
    use bt_hci::param::{AdvHandle, AdvSet, ConnHandle, InitiatingPhy, ScanningPhy, SyncHandle};
    use bt_hci::{FromHciBytes, WriteHci};

    use crate::raw;

    const MAX_PHY_COUNT: usize = 3;
    const MAX_ADV_SET: usize = 63;
    const MAX_ANTENNA_IDS: usize = 75;
    const MAX_SUBEVENTS: usize = 128;

    // Legacy advertising commands
    sdc_cmd!(LeSetAdvParams => raw_cmd_params(sdc_hci_cmd_le_set_adv_params, LeSetAdvParams) false);
    sdc_cmd!(LeReadAdvPhysicalChannelTxPower => raw_cmd_return(sdc_hci_cmd_le_read_adv_physical_channel_tx_power) false);
    sdc_cmd!(LeSetAdvData => raw_cmd_params(sdc_hci_cmd_le_set_adv_data, LeSetAdvData) false);
    sdc_cmd!(LeSetScanResponseData => raw_cmd_params(sdc_hci_cmd_le_set_scan_response_data, LeSetScanResponseData) false);
    sdc_cmd!(LeSetAdvEnable => raw_cmd_params(sdc_hci_cmd_le_set_adv_enable, LeSetAdvEnable) false);
    sdc_cmd!(LeSetScanParams => raw_cmd_params(sdc_hci_cmd_le_set_scan_params, LeSetScanParams) false);
    sdc_cmd!(LeSetScanEnable => raw_cmd_params(sdc_hci_cmd_le_set_scan_enable, LeSetScanEnable) false);
    sdc_cmd!(async LeCreateConn => raw_cmd_params(sdc_hci_cmd_le_create_conn, LeCreateConn) false);

    // Extended advertising commands
    sdc_cmd!(LeSetExtAdvParams => raw_cmd_params_return(sdc_hci_cmd_le_set_ext_adv_params, LeSetExtAdvParams) true);
    sdc_cmd!(LeSetExtAdvParamsV2 => raw_cmd_params_return(sdc_hci_cmd_le_set_ext_adv_params_v2, LeSetExtAdvParamsV2) true);
    sdc_cmd!(LeReadMaxAdvDataLength => raw_cmd_return(sdc_hci_cmd_le_read_max_adv_data_length) true);
    sdc_cmd!(LeReadNumberOfSupportedAdvSets => raw_cmd_return(sdc_hci_cmd_le_read_number_of_supported_adv_sets) true);
    sdc_cmd!(LeRemoveAdvSet => raw_cmd_params(sdc_hci_cmd_le_remove_adv_set, LeRemoveAdvSet) true);
    sdc_cmd!(LeClearAdvSets => raw_cmd(sdc_hci_cmd_le_clear_adv_sets) true);
    sdc_cmd!(LeSetPeriodicAdvParams => raw_cmd_params(sdc_hci_cmd_le_set_periodic_adv_params, LeSetPeriodicAdvParams) true);
    sdc_cmd!(LeSetPeriodicAdvParamsV2 => raw_cmd_params_return(sdc_hci_cmd_le_set_periodic_adv_params_v2, LeSetPeriodicAdvParamsV2) true);
    sdc_cmd!(LeSetPeriodicAdvEnable => raw_cmd_params(sdc_hci_cmd_le_set_periodic_adv_enable, LeSetPeriodicAdvEnable) true);
    sdc_cmd!(LeSetExtScanEnable => raw_cmd_params(sdc_hci_cmd_le_set_ext_scan_enable, LeSetExtScanEnable) true);
    sdc_cmd!(async LePeriodicAdvCreateSync => raw_cmd_params(sdc_hci_cmd_le_periodic_adv_create_sync, LePeriodicAdvCreateSync) true);
    sdc_cmd!(LePeriodicAdvCreateSyncCancel => raw_cmd(sdc_hci_cmd_le_periodic_adv_create_sync_cancel) true);
    sdc_cmd!(LePeriodicAdvTerminateSync => raw_cmd_params(sdc_hci_cmd_le_periodic_adv_terminate_sync, LePeriodicAdvTerminateSync) true);
    sdc_cmd!(LeAddDeviceToPeriodicAdvList => raw_cmd_params(sdc_hci_cmd_le_add_device_to_periodic_adv_list, LeAddDeviceToPeriodicAdvList) true);
    sdc_cmd!(LeRemoveDeviceFromPeriodicAdvList => raw_cmd_params(sdc_hci_cmd_le_remove_device_from_periodic_adv_list, LeRemoveDeviceFromPeriodicAdvList) true);
    sdc_cmd!(LeClearPeriodicAdvList => raw_cmd(sdc_hci_cmd_le_clear_periodic_adv_list) true);
    sdc_cmd!(LeReadPeriodicAdvListSize => raw_cmd_return(sdc_hci_cmd_le_read_periodic_adv_list_size) true);
    sdc_cmd!(LeSetPeriodicAdvSyncTransferParams => raw_cmd_params_return(sdc_hci_cmd_le_set_periodic_adv_sync_transfer_params, LeSetPeriodicAdvSyncTransferParams) true);
    sdc_cmd!(LeSetDefaultPeriodicAdvSyncTransferParams => raw_cmd_params(sdc_hci_cmd_le_set_default_periodic_adv_sync_transfer_params, LeSetDefaultPeriodicAdvSyncTransferParams) true);

    sdc_cmd!(LeSetEventMask => sdc_hci_cmd_le_set_event_mask(x));
    sdc_cmd!(LeReadBufferSize => sdc_hci_cmd_le_read_buffer_size() -> y);
    sdc_cmd!(LeReadLocalSupportedFeatures => sdc_hci_cmd_le_read_local_supported_features() -> y);
    sdc_cmd!(LeSetRandomAddr => sdc_hci_cmd_le_set_random_address(x));
    sdc_cmd!(LeCreateConnCancel => sdc_hci_cmd_le_create_conn_cancel());
    sdc_cmd!(LeReadFilterAcceptListSize => sdc_hci_cmd_le_read_filter_accept_list_size() -> y);
    sdc_cmd!(LeClearFilterAcceptList => sdc_hci_cmd_le_clear_filter_accept_list());
    sdc_cmd!(LeAddDeviceToFilterAcceptList => sdc_hci_cmd_le_add_device_to_filter_accept_list(x));
    sdc_cmd!(LeRemoveDeviceFromFilterAcceptList => sdc_hci_cmd_le_remove_device_from_filter_accept_list(x));
    sdc_cmd!(async LeConnUpdate => sdc_hci_cmd_le_conn_update(x));
    sdc_cmd!(LeSetHostChannelClassification => sdc_hci_cmd_le_set_host_channel_classification(x));
    sdc_cmd!(LeReadChannelMap => sdc_hci_cmd_le_read_channel_map(x) -> y);
    sdc_cmd!(async LeReadRemoteFeatures => sdc_hci_cmd_le_read_remote_features(x));
    sdc_cmd!(LeEncrypt => sdc_hci_cmd_le_encrypt(x) -> y);
    sdc_cmd!(LeRand => sdc_hci_cmd_le_rand() -> y);
    sdc_cmd!(async LeEnableEncryption => sdc_hci_cmd_le_enable_encryption(x));
    sdc_cmd!(LeLongTermKeyRequestReply => sdc_hci_cmd_le_long_term_key_request_reply(x) -> y);
    sdc_cmd!(LeLongTermKeyRequestNegativeReply => sdc_hci_cmd_le_long_term_key_request_negative_reply(x) -> y);
    sdc_cmd!(LeReadSupportedStates => sdc_hci_cmd_le_read_supported_states() -> y);
    sdc_cmd!(LeTestEnd => sdc_hci_cmd_le_test_end() -> y);
    sdc_cmd!(LeSetDataLength => sdc_hci_cmd_le_set_data_length(x) -> y);
    sdc_cmd!(LeReadSuggestedDefaultDataLength => sdc_hci_cmd_le_read_suggested_default_data_length() -> y);
    sdc_cmd!(LeWriteSuggestedDefaultDataLength => sdc_hci_cmd_le_write_suggested_default_data_length(x));
    sdc_cmd!(LeAddDeviceToResolvingList => sdc_hci_cmd_le_add_device_to_resolving_list(x));
    sdc_cmd!(LeRemoveDeviceFromResolvingList => sdc_hci_cmd_le_remove_device_from_resolving_list(x));
    sdc_cmd!(LeClearResolvingList => sdc_hci_cmd_le_clear_resolving_list());
    sdc_cmd!(LeReadResolvingListSize => sdc_hci_cmd_le_read_resolving_list_size() -> y);
    sdc_cmd!(LeSetAddrResolutionEnable => sdc_hci_cmd_le_set_address_resolution_enable(x));
    sdc_cmd!(LeSetResolvablePrivateAddrTimeout => sdc_hci_cmd_le_set_resolvable_private_address_timeout(x));
    sdc_cmd!(LeReadMaxDataLength => sdc_hci_cmd_le_read_max_data_length() -> y);
    sdc_cmd!(LeReadPhy => sdc_hci_cmd_le_read_phy(x) -> y);
    sdc_cmd!(LeSetDefaultPhy => sdc_hci_cmd_le_set_default_phy(x));
    sdc_cmd!(async LeSetPhy => sdc_hci_cmd_le_set_phy(x));
    sdc_cmd!(LeSetAdvSetRandomAddr => sdc_hci_cmd_le_set_adv_set_random_address(x));
    sdc_cmd!(LeReadTransmitPower => sdc_hci_cmd_le_read_transmit_power() -> y);
    sdc_cmd!(LeReadRfPathCompensation => sdc_hci_cmd_le_read_rf_path_compensation() -> y);
    sdc_cmd!(LeWriteRfPathCompensation => sdc_hci_cmd_le_write_rf_path_compensation(x));
    sdc_cmd!(LeSetPrivacyMode => sdc_hci_cmd_le_set_privacy_mode(x));
    sdc_cmd!(LeSetConnectionlessCteTransmitEnable => sdc_hci_cmd_le_set_connless_cte_transmit_enable(x));
    sdc_cmd!(LeConnCteResponseEnable => sdc_hci_cmd_le_conn_cte_response_enable(x) -> y);
    sdc_cmd!(LeReadAntennaInformation => sdc_hci_cmd_le_read_antenna_information() -> y);
    sdc_cmd!(LeSetPeriodicAdvReceiveEnable => sdc_hci_cmd_le_set_periodic_adv_receive_enable(x));
    sdc_cmd!(LePeriodicAdvSyncTransfer => sdc_hci_cmd_le_periodic_adv_sync_transfer(x) -> y);
    sdc_cmd!(LePeriodicAdvSetInfoTransfer => sdc_hci_cmd_le_periodic_adv_set_info_transfer(x) -> y);
    sdc_cmd!(async LeRequestPeerSca => sdc_hci_cmd_le_request_peer_sca(x));
    sdc_cmd!(LeEnhancedReadTransmitPowerLevel => sdc_hci_cmd_le_enhanced_read_transmit_power_level(x) -> y);
    sdc_cmd!(async LeReadRemoteTransmitPowerLevel => sdc_hci_cmd_le_read_remote_transmit_power_level(x));
    sdc_cmd!(LeSetPathLossReportingParams => sdc_hci_cmd_le_set_path_loss_reporting_params(x) -> y);
    sdc_cmd!(LeSetPathLossReportingEnable => sdc_hci_cmd_le_set_path_loss_reporting_enable(x) -> y);
    sdc_cmd!(LeSetTransmitPowerReportingEnable => sdc_hci_cmd_le_set_transmit_power_reporting_enable(x) -> y);
    sdc_cmd!(LeSetDataRelatedAddrChanges => sdc_hci_cmd_le_set_data_related_address_changes(x));
    sdc_cmd!(LeSetHostFeature => sdc_hci_cmd_le_set_host_feature(x));
    sdc_cmd!(LeSetHostFeatureV2 => sdc_hci_cmd_le_set_host_feature_v2(x));

    impl<'a, 'd> ControllerCmdSync<LeSetExtAdvData<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetExtAdvData<'a>) -> Result<(), CmdError<nrf_mpsl::Error>> {
            self.check_adv_cmd(true)?;
            const N: usize = 4 + 251;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_adv_data(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result().map_err(CmdError::Hci)
        }
    }

    impl<'a, 'd> ControllerCmdSync<LeSetExtScanResponseData<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetExtScanResponseData<'a>) -> Result<(), CmdError<nrf_mpsl::Error>> {
            self.check_adv_cmd(true)?;
            const N: usize = 4 + 251;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_scan_response_data(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result().map_err(CmdError::Hci)
        }
    }

    impl<'a, 'd> ControllerCmdSync<LeSetExtAdvEnable<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetExtAdvEnable<'a>) -> Result<(), CmdError<nrf_mpsl::Error>> {
            self.check_adv_cmd(true)?;
            const N: usize = 2 + MAX_ADV_SET * core::mem::size_of::<AdvSet>();
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_adv_enable(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result().map_err(CmdError::Hci)
        }
    }

    impl<'a, 'd> ControllerCmdSync<LeSetPeriodicAdvData<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetPeriodicAdvData<'a>) -> Result<(), CmdError<nrf_mpsl::Error>> {
            self.check_adv_cmd(true)?;
            const N: usize = 3 + 252;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_periodic_adv_data(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result().map_err(CmdError::Hci)
        }
    }

    impl<'d> ControllerCmdSync<LeSetExtScanParams> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetExtScanParams) -> Result<(), CmdError<nrf_mpsl::Error>> {
            self.check_adv_cmd(true)?;
            const N: usize = 3 + MAX_PHY_COUNT * core::mem::size_of::<ScanningPhy>();
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_scan_params(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result().map_err(CmdError::Hci)
        }
    }

    impl<'d> ControllerCmdAsync<LeExtCreateConn> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeExtCreateConn) -> Result<(), CmdError<nrf_mpsl::Error>> {
            self.check_adv_cmd(true)?;
            const N: usize = 10 + MAX_PHY_COUNT * core::mem::size_of::<InitiatingPhy>();
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_ext_create_conn(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result().map_err(CmdError::Hci)
        }
    }

    impl<'a, 'd> ControllerCmdSync<LeSetConnectionlessCteTransmitParams<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetConnectionlessCteTransmitParams<'a>) -> Result<(), CmdError<nrf_mpsl::Error>> {
            const N: usize = 5 + MAX_ANTENNA_IDS;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_connless_cte_transmit_params(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result().map_err(CmdError::Hci)
        }
    }

    impl<'a, 'd> ControllerCmdSync<LeSetConnCteTransmitParams<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetConnCteTransmitParams<'a>) -> Result<ConnHandle, CmdError<nrf_mpsl::Error>> {
            const N: usize = 5 + MAX_ANTENNA_IDS;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));

            let mut out = unsafe { core::mem::zeroed() };
            let ret = unsafe { raw::sdc_hci_cmd_le_set_conn_cte_transmit_params(buf.as_ptr() as *const _, &mut out) };

            bt_hci::param::Status::from(ret).to_result()?;
            Ok(unwrap!(ConnHandle::from_hci_bytes_complete(unsafe {
                super::bytes_of(&out)
            })))
        }
    }

    impl<'d> ControllerCmdAsync<LeExtCreateConnV2> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeExtCreateConnV2) -> Result<(), CmdError<nrf_mpsl::Error>> {
            self.check_adv_cmd(true)?;
            const N: usize = 12 + MAX_PHY_COUNT * 16;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));

            let ret = unsafe { raw::sdc_hci_cmd_le_ext_create_conn_v2(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result().map_err(CmdError::Hci)
        }
    }

    impl<'a, 'd> ControllerCmdSync<LeSetPeriodicAdvSubeventData<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetPeriodicAdvSubeventData<'a>) -> Result<AdvHandle, CmdError<nrf_mpsl::Error>> {
            self.check_adv_cmd(true)?;
            const N: usize = raw::HCI_CMD_MAX_SIZE as usize;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));

            let mut out = unsafe { core::mem::zeroed() };
            let ret = unsafe { raw::sdc_hci_cmd_le_set_periodic_adv_subevent_data(buf.as_ptr() as *const _, &mut out) };

            bt_hci::param::Status::from(ret).to_result()?;
            Ok(unwrap!(AdvHandle::from_hci_bytes_complete(unsafe {
                super::bytes_of(&out)
            })))
        }
    }

    impl<'a, 'd> ControllerCmdSync<LeSetPeriodicAdvResponseData<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetPeriodicAdvResponseData<'a>) -> Result<SyncHandle, CmdError<nrf_mpsl::Error>> {
            if self
                .periodic_adv_response_data_in_progress
                .swap(true, Ordering::Relaxed)
            {
                return Err(CmdError::Hci(bt_hci::param::Error::CONTROLLER_BUSY));
            }

            self.check_adv_cmd(true)?;
            const N: usize = raw::HCI_CMD_MAX_SIZE as usize;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));

            let mut out = unsafe { core::mem::zeroed() };
            let ret = unsafe { raw::sdc_hci_cmd_le_set_periodic_adv_response_data(buf.as_ptr() as *const _, &mut out) };
            bt_hci::param::Status::from(ret).to_result()?;

            // sdc_hci_cmd_le_set_periodic_adv_response_data generates an actual CommandComplete event, so wait for that.
            let (status, handle) = self.periodic_adv_response_data_complete.wait().await;
            self.periodic_adv_response_data_in_progress
                .store(false, Ordering::Relaxed);

            status.to_result().map(|_| handle).map_err(CmdError::Hci)
        }
    }

    impl<'a, 'd> ControllerCmdSync<LeSetPeriodicSyncSubevent<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetPeriodicSyncSubevent<'a>) -> Result<SyncHandle, CmdError<nrf_mpsl::Error>> {
            self.check_adv_cmd(true)?;
            const N: usize = 5 + MAX_SUBEVENTS;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));

            let mut out = unsafe { core::mem::zeroed() };
            let ret = unsafe { raw::sdc_hci_cmd_le_set_periodic_sync_subevent(buf.as_ptr() as *const _, &mut out) };

            bt_hci::param::Status::from(ret).to_result()?;
            Ok(unwrap!(SyncHandle::from_hci_bytes_complete(unsafe {
                super::bytes_of(&out)
            })))
        }
    }
}

/// Bluetooth HCI vendor specific commands
pub mod vendor {
    use bt_hci::cmd::Error as CmdError;
    use bt_hci::controller::ControllerCmdSync;
    use bt_hci::param::{BdAddr, ChannelMap, ConnHandle, Duration};
    use bt_hci::{cmd, param, FromHciBytes};

    use crate::raw;

    param!(
        struct ZephyrStaticAddr {
            addr: BdAddr,
            identity_root: [u8; 16],
        }
    );

    param!(
        struct ZephyrTxPower {
            handle_type: u8,
            handle: u16,
            selected_tx_power: i8,
        }
    );

    cmd! {
        /// https://docs.zephyrproject.org/apidoc/latest/hci__vs_8h.html
        ZephyrReadVersionInfo(VENDOR_SPECIFIC, 0x0001) {
            Params = ();
            ZephyrReadVersionInfoReturn {
                hw_platform: u16,
                hw_variant: u16,
                fw_variant: u8,
                fw_version: u8,
                fw_revision: u16,
                fw_build: u32,
            }
        }
    }

    cmd! {
        /// https://docs.zephyrproject.org/apidoc/latest/hci__vs_8h.html
        ZephyrReadSupportedCommands(VENDOR_SPECIFIC, 0x0002) {
            Params = ();
            Return = [u8; 64];
        }
    }

    cmd! {
        /// https://docs.zephyrproject.org/apidoc/latest/hci__vs_8h.html
        ZephyrWriteBdAddr(VENDOR_SPECIFIC, 0x0006) {
            Params = BdAddr;
            Return = ();
        }
    }

    cmd! {
        /// https://docs.zephyrproject.org/apidoc/latest/hci__vs_8h.html
        ZephyrReadStaticAddrs(VENDOR_SPECIFIC, 0x0009) {
            Params = ();
            ZephyrReadStaticAddrsReturn {
                num_addresses: u8,
                addr: ZephyrStaticAddr, // The softdevice controller always returns exactly 1 static address
            }
        }
    }

    cmd! {
        /// https://docs.zephyrproject.org/apidoc/latest/hci__vs_8h.html
        ZephyrReadKeyHierarchyRoots(VENDOR_SPECIFIC, 0x000a) {
            Params = ();
            ZephyrReadKeyHierarchyRootsReturn {
                ir: [u8; 16usize],
                er: [u8; 16usize],
            }
        }
    }

    cmd! {
        /// https://docs.zephyrproject.org/apidoc/latest/hci__vs_8h.html
        ZephyrReadChipTemp(VENDOR_SPECIFIC, 0x000b) {
            Params = ();
            Return = i8;
        }
    }

    cmd! {
        /// https://docs.zephyrproject.org/apidoc/latest/hci__vs_8h.html
        ZephyrWriteTxPower(VENDOR_SPECIFIC, 0x000e) {
            Params = ZephyrWriteTxPowerParams;
            Return = ZephyrTxPower;
        }
    }

    param! {
        struct ZephyrWriteTxPowerParams {
            handle_type: u8,
            handle: u16,
            tx_power_level: i8,
        }
    }

    cmd! {
        /// https://docs.zephyrproject.org/apidoc/latest/hci__vs_8h.html
        ZephyrReadTxPower(VENDOR_SPECIFIC, 0x000f) {
            Params = ZephyrReadTxPowerParams;
            Return = ZephyrTxPower;
        }
    }

    param! {
        struct ZephyrReadTxPowerParams {
            handle_type: u8,
            handle: u16,
        }
    }

    cmd! {
        NordicLlpmModeSet(VENDOR_SPECIFIC, 0x0101) {
            Params = bool;
            Return = ();
        }
    }

    cmd! {
        NordicConnUpdate(VENDOR_SPECIFIC, 0x0102) {
            Params = NordicConnUpdateParams;
            Return = ();
        }
    }

    param! {
        struct NordicConnUpdateParams {
            handle: ConnHandle,
            interval_us: u32,
            latency: u16,
            supervision_timeout: Duration<10_000>,
        }
    }

    cmd! {
        NordicConnEventExtend(VENDOR_SPECIFIC, 0x0103) {
            Params = bool;
            Return = ();
        }
    }

    cmd! {
        NordicQosConnEventReportEnable(VENDOR_SPECIFIC, 0x0104) {
            Params = bool;
            Return = ();
        }
    }

    cmd! {
        NordicEventLengthSet(VENDOR_SPECIFIC, 0x0105) {
            Params = u32;
            Return = ();
        }
    }

    cmd! {
        NordicPeriodicAdvEventLengthSet(VENDOR_SPECIFIC, 0x0106) {
            Params = u32;
            Return = ();
        }
    }

    cmd! {
        NordicPeripheralLatencyModeSet(VENDOR_SPECIFIC, 0x0109) {
            Params = NordicPeripheralLatencyModeSetParams;
            Return = ();
        }
    }

    param! {
        struct NordicPeripheralLatencyModeSetParams{
            handle: ConnHandle,
            mode: u8,
         }
    }

    cmd! {
        NordicWriteRemoteTxPower(VENDOR_SPECIFIC, 0x010a) {
            Params = NordicWriteRemoteTxPowerParams;
            Return = ();
        }
    }

    param! {
        struct NordicWriteRemoteTxPowerParams {
            handle: ConnHandle,
            phy: u8,
            delta: i8,
        }
    }

    cmd! {
        NordicSetAdvRandomness(VENDOR_SPECIFIC, 0x010c) {
            Params = NordicSetAdvRandomnessParams;
            Return = ();
        }
    }

    param! {
        struct NordicSetAdvRandomnessParams {
            adv_handle: u8,
            rand_us: u16,
        }
    }

    cmd! {
        NordicCompatModeWindowOffsetSet(VENDOR_SPECIFIC, 0x010d) {
            Params = bool;
            Return = ();
        }
    }

    cmd! {
        NordicQosChannelSurveyEnable(VENDOR_SPECIFIC, 0x010e) {
            NordicQosChannelSurveyEnableParams {
                enable: bool,
                interval_us: u32,
            }
            Return = ();
        }
    }

    cmd! {
        NordicSetPowerControlRequestParams(VENDOR_SPECIFIC, 0x0110) {
            NordicSetPowerControlRequestParamsParams {
                auto_enable: bool,
                apr_enable: bool,
                beta: u16,
                lower_limit: i8,
                upper_limit: i8,
                lower_target_rssi: i8,
                upper_target_rssi: i8,
                wait_period_ms: Duration<1_000>,
                apr_margin: u8,
            }
            Return = ();
        }
    }

    cmd! {
        NordicReadAverageRssi(VENDOR_SPECIFIC, 0x111) {
            Params = ConnHandle;
            NordicReadAverageRssiReturn {
                avg_rssi: i8,
            }
            Handle = handle: ConnHandle;
        }
    }

    cmd! {
        NordicCentralAclEventSpacingSet(VENDOR_SPECIFIC, 0x112) {
            Params = u32;
            Return = ();
        }
    }

    cmd! {
        NordicGetNextConnEventCounter(VENDOR_SPECIFIC, 0x114) {
            Params = ConnHandle;
            NordicGetNextConnEventCounterReturn {
                next_conn_event_counter: u16,
            }
            Handle = handle: ConnHandle;
        }
    }

    cmd! {
        NordicAllowParallelConnectionEstablishments(VENDOR_SPECIFIC, 0x115) {
            Params = bool;
            Return = ();
        }
    }

    cmd! {
        NordicMinValOfMaxAclTxPayloadSet(VENDOR_SPECIFIC, 0x116) {
            Params = u8;
            Return = ();
        }
    }

    cmd! {
        NordicIsoReadTxTimestamp(VENDOR_SPECIFIC, 0x117) {
            Params = ConnHandle;
            NordicIsoReadTxTimestampReturn {
                packet_sequence_number: u16,
                tx_time_stamp: u32,
            }
            Handle = handle: ConnHandle;
        }
    }

    cmd! {
        NordicBigReservedTimeSet(VENDOR_SPECIFIC, 0x118) {
            Params = u32;
            Return = ();
        }
    }

    cmd! {
        NordicCigReservedTimeSet(VENDOR_SPECIFIC, 0x119) {
            Params = u32;
            Return = ();
        }
    }

    cmd! {
        NordicCisSubeventLengthSet(VENDOR_SPECIFIC, 0x11a) {
            Params = u32;
            Return = ();
        }
    }

    cmd! {
        NordicScanChannelMapSet(VENDOR_SPECIFIC, 0x11b) {
            Params = ChannelMap;
            Return = ();
        }
    }

    cmd! {
        NordicScanAcceptExtAdvPacketsSet(VENDOR_SPECIFIC, 0x11c) {
            Params = bool;
            Return = ();
        }
    }

    cmd! {
        NordicSetRolePriority(VENDOR_SPECIFIC, 0x11d) {
            NordicSetRolePriorityParams {
                handle_type: u8,
                handle: u16,
                priority: u8,
            }
            Return = ();
        }
    }

    cmd! {
        NordicSetEventStartTask(VENDOR_SPECIFIC, 0x11e) {
            NordicSetEventStartTaskParams {
                handle_type: u8,
                handle: u16,
                task_address: u32,
            }
            Return = ();
        }
    }

    cmd! {
        NordicConnAnchorPointUpdateEventReportEnable(VENDOR_SPECIFIC, 0x11f) {
            Params = bool;
            Return = ();
        }
    }

    sdc_cmd!(ZephyrReadVersionInfo => sdc_hci_cmd_vs_zephyr_read_version_info() -> y);
    sdc_cmd!(ZephyrReadSupportedCommands => sdc_hci_cmd_vs_zephyr_read_supported_commands() -> y);
    sdc_cmd!(ZephyrWriteBdAddr => sdc_hci_cmd_vs_zephyr_write_bd_addr(x));
    sdc_cmd!(ZephyrReadKeyHierarchyRoots => sdc_hci_cmd_vs_zephyr_read_key_hierarchy_roots() -> y);
    sdc_cmd!(ZephyrReadChipTemp => sdc_hci_cmd_vs_zephyr_read_chip_temp() -> y);
    sdc_cmd!(ZephyrWriteTxPower => sdc_hci_cmd_vs_zephyr_write_tx_power(x) -> y);
    sdc_cmd!(ZephyrReadTxPower => sdc_hci_cmd_vs_zephyr_read_tx_power(x) -> y);
    sdc_cmd!(NordicLlpmModeSet => sdc_hci_cmd_vs_llpm_mode_set(x));
    sdc_cmd!(NordicConnUpdate => sdc_hci_cmd_vs_conn_update(x));
    sdc_cmd!(NordicConnEventExtend => sdc_hci_cmd_vs_conn_event_extend(x));
    sdc_cmd!(NordicQosConnEventReportEnable => sdc_hci_cmd_vs_qos_conn_event_report_enable(x));
    sdc_cmd!(NordicEventLengthSet => sdc_hci_cmd_vs_event_length_set(x));
    sdc_cmd!(NordicPeriodicAdvEventLengthSet => sdc_hci_cmd_vs_periodic_adv_event_length_set(x));
    sdc_cmd!(NordicPeripheralLatencyModeSet => sdc_hci_cmd_vs_peripheral_latency_mode_set(x));
    sdc_cmd!(NordicWriteRemoteTxPower => sdc_hci_cmd_vs_write_remote_tx_power(x));
    sdc_cmd!(NordicSetAdvRandomness => sdc_hci_cmd_vs_set_adv_randomness(x));
    sdc_cmd!(NordicCompatModeWindowOffsetSet => sdc_hci_cmd_vs_compat_mode_window_offset_set(x));
    sdc_cmd!(NordicQosChannelSurveyEnable => sdc_hci_cmd_vs_qos_channel_survey_enable(x));
    sdc_cmd!(NordicSetPowerControlRequestParams => sdc_hci_cmd_vs_set_power_control_request_params(x));
    sdc_cmd!(NordicReadAverageRssi => sdc_hci_cmd_vs_read_average_rssi(x) -> y);
    sdc_cmd!(NordicCentralAclEventSpacingSet => sdc_hci_cmd_vs_central_acl_event_spacing_set(x));
    sdc_cmd!(NordicGetNextConnEventCounter => sdc_hci_cmd_vs_get_next_conn_event_counter(x) -> y);
    sdc_cmd!(NordicAllowParallelConnectionEstablishments => sdc_hci_cmd_vs_allow_parallel_connection_establishments(x));
    sdc_cmd!(NordicMinValOfMaxAclTxPayloadSet => sdc_hci_cmd_vs_min_val_of_max_acl_tx_payload_set(x));
    sdc_cmd!(NordicIsoReadTxTimestamp => sdc_hci_cmd_vs_iso_read_tx_timestamp(x) -> y);
    sdc_cmd!(NordicBigReservedTimeSet => sdc_hci_cmd_vs_big_reserved_time_set(x));
    sdc_cmd!(NordicCigReservedTimeSet => sdc_hci_cmd_vs_cig_reserved_time_set(x));
    sdc_cmd!(NordicCisSubeventLengthSet => sdc_hci_cmd_vs_cis_subevent_length_set(x));
    sdc_cmd!(NordicScanChannelMapSet => sdc_hci_cmd_vs_scan_channel_map_set(x));
    sdc_cmd!(NordicScanAcceptExtAdvPacketsSet => sdc_hci_cmd_vs_scan_accept_ext_adv_packets_set(x));
    sdc_cmd!(NordicSetRolePriority => sdc_hci_cmd_vs_set_role_priority(x));
    sdc_cmd!(NordicSetEventStartTask => sdc_hci_cmd_vs_set_event_start_task(x));
    sdc_cmd!(NordicConnAnchorPointUpdateEventReportEnable => sdc_hci_cmd_vs_conn_anchor_point_update_event_report_enable(x));

    impl<'d> ControllerCmdSync<ZephyrReadStaticAddrs> for super::SoftdeviceController<'d> {
        async fn exec(
            &self,
            _cmd: &ZephyrReadStaticAddrs,
        ) -> Result<<ZephyrReadStaticAddrs as bt_hci::cmd::SyncCmd>::Return, CmdError<Self::Error>> {
            const N: usize = core::mem::size_of::<ZephyrReadStaticAddrsReturn>();
            let mut out = [0; N];
            let ret = unsafe { raw::sdc_hci_cmd_vs_zephyr_read_static_addresses(out.as_mut_ptr() as *mut _) };
            bt_hci::param::Status::from(ret).to_result()?;
            Ok(unwrap!(ZephyrReadStaticAddrsReturn::from_hci_bytes_complete(&out)))
        }
    }
}
