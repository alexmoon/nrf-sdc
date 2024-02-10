use core::ffi::CStr;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicPtr, Ordering};
use core::task::Poll;

use embassy_nrf::{peripherals, Peripheral, PeripheralRef};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::waitqueue::AtomicWaker;
use nrf_mpsl::MultiprotocolServiceLayer;
use raw::{
    sdc_cfg_adv_buffer_cfg_t, sdc_cfg_buffer_cfg_t, sdc_cfg_buffer_count_t, sdc_cfg_event_length_t,
    sdc_cfg_role_count_t, sdc_cfg_t, SDC_CFG_TYPE_NONE, SDC_DEFAULT_RESOURCE_CFG_TAG,
};

use crate::rng_pool::RngPool;
use crate::{hci, pac, raw, Error, RetVal};

static WAKER: AtomicWaker = AtomicWaker::new();
static FLASH_STATUS: Signal<ThreadModeRawMutex, Result<(), Error>> = Signal::new();
static RNG_POOL: AtomicPtr<RngPool> = AtomicPtr::new(core::ptr::null_mut());

pub struct Peripherals<'d> {
    pub ecb: pac::ECB,
    pub aar: pac::AAR,
    pub nvmc: PeripheralRef<'d, peripherals::NVMC>,

    pub ppi_ch17: PeripheralRef<'d, peripherals::PPI_CH17>,
    pub ppi_ch18: PeripheralRef<'d, peripherals::PPI_CH18>,
    pub ppi_ch20: PeripheralRef<'d, peripherals::PPI_CH20>,
    pub ppi_ch21: PeripheralRef<'d, peripherals::PPI_CH21>,
    pub ppi_ch22: PeripheralRef<'d, peripherals::PPI_CH22>,
    pub ppi_ch23: PeripheralRef<'d, peripherals::PPI_CH23>,
    pub ppi_ch24: PeripheralRef<'d, peripherals::PPI_CH24>,
    pub ppi_ch25: PeripheralRef<'d, peripherals::PPI_CH25>,
    pub ppi_ch26: PeripheralRef<'d, peripherals::PPI_CH26>,
    pub ppi_ch27: PeripheralRef<'d, peripherals::PPI_CH27>,
    pub ppi_ch28: PeripheralRef<'d, peripherals::PPI_CH28>,
    pub ppi_ch29: PeripheralRef<'d, peripherals::PPI_CH29>,
}

impl<'d> Peripherals<'d> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        ecb: pac::ECB,
        aar: pac::AAR,
        nvmc: impl Peripheral<P = peripherals::NVMC> + 'd,
        ppi_ch17: impl Peripheral<P = peripherals::PPI_CH17> + 'd,
        ppi_ch18: impl Peripheral<P = peripherals::PPI_CH18> + 'd,
        ppi_ch20: impl Peripheral<P = peripherals::PPI_CH20> + 'd,
        ppi_ch21: impl Peripheral<P = peripherals::PPI_CH21> + 'd,
        ppi_ch22: impl Peripheral<P = peripherals::PPI_CH22> + 'd,
        ppi_ch23: impl Peripheral<P = peripherals::PPI_CH23> + 'd,
        ppi_ch24: impl Peripheral<P = peripherals::PPI_CH24> + 'd,
        ppi_ch25: impl Peripheral<P = peripherals::PPI_CH25> + 'd,
        ppi_ch26: impl Peripheral<P = peripherals::PPI_CH26> + 'd,
        ppi_ch27: impl Peripheral<P = peripherals::PPI_CH27> + 'd,
        ppi_ch28: impl Peripheral<P = peripherals::PPI_CH28> + 'd,
        ppi_ch29: impl Peripheral<P = peripherals::PPI_CH29> + 'd,
    ) -> Self {
        Peripherals {
            ecb,
            aar,
            nvmc: nvmc.into_ref(),
            ppi_ch17: ppi_ch17.into_ref(),
            ppi_ch18: ppi_ch18.into_ref(),
            ppi_ch20: ppi_ch20.into_ref(),
            ppi_ch21: ppi_ch21.into_ref(),
            ppi_ch22: ppi_ch22.into_ref(),
            ppi_ch23: ppi_ch23.into_ref(),
            ppi_ch24: ppi_ch24.into_ref(),
            ppi_ch25: ppi_ch25.into_ref(),
            ppi_ch26: ppi_ch26.into_ref(),
            ppi_ch27: ppi_ch27.into_ref(),
            ppi_ch28: ppi_ch28.into_ref(),
            ppi_ch29: ppi_ch29.into_ref(),
        }
    }
}

unsafe extern "C" fn fault_handler(file: *const core::ffi::c_char, line: u32) {
    panic!(
        "SoftdeviceController: {}:{}",
        CStr::from_ptr(file).to_str().unwrap_or("bad filename"),
        line
    )
}

extern "C" fn sdc_callback() {
    WAKER.wake()
}

unsafe extern "C" fn rand_prio_low_get(p_buff: *mut u8, length: u8) -> u8 {
    let rng = RNG_POOL.load(Ordering::Acquire);
    if !rng.is_null() {
        let buf = core::slice::from_raw_parts_mut(p_buff, usize::from(length));
        (*rng).try_fill_bytes(buf) as u8
    } else {
        0
    }
}

unsafe extern "C" fn rand_prio_high_get(p_buff: *mut u8, length: u8) -> u8 {
    let rng = RNG_POOL.load(Ordering::Acquire);
    if !rng.is_null() {
        let buf = core::slice::from_raw_parts_mut(p_buff, usize::from(length));
        (*rng).try_fill_bytes(buf) as u8
    } else {
        0
    }
}

unsafe extern "C" fn rand_blocking(p_buff: *mut u8, length: u8) {
    let rng = RNG_POOL.load(Ordering::Acquire);
    if !rng.is_null() {
        let buf = core::slice::from_raw_parts_mut(p_buff, usize::from(length));
        (*rng).blocking_fill_bytes(buf);
    } else {
        panic!("rand_blocking called from Softdevice Controller when no RngPool is set");
    }
}

extern "C" fn flash_callback(status: u32) {
    let res = match status {
        raw::SDC_SOC_FLASH_CMD_STATUS_SUCCESS => Ok(()),
        raw::SDC_SOC_FLASH_CMD_STATUS_TIMEOUT => Err(Error::ETIMEDOUT),
        _ => Err(Error::EINVAL),
    };
    FLASH_STATUS.signal(res);
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
        tx_packet_size: u8,
        rx_packet_size: u8,
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

    pub fn event_duration(self, microseconds: u32) -> Result<Self, Error> {
        self.cfg_set(
            raw::SDC_CFG_TYPE_EVENT_LENGTH,
            sdc_cfg_t {
                event_length: sdc_cfg_event_length_t {
                    event_length_us: microseconds,
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

    pub fn coex_adv_mode_configure(self, mode: core::ops::ControlFlow<(), ()>) -> Result<Self, Error> {
        let val = match mode {
            core::ops::ControlFlow::Break(()) => false,
            core::ops::ControlFlow::Continue(()) => true,
        };

        RetVal::from(unsafe { raw::sdc_coex_adv_mode_configure(val) })
            .to_result()
            .and(Ok(self))
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
    pub fn build<'d>(
        self,
        p: Peripherals<'d>,
        rng: &'d RngPool,
        mpsl: &'d MultiprotocolServiceLayer,
        mem: &'d mut [u8],
    ) -> Result<SoftdeviceController<'d>, Error> {
        // Peripherals are used by the Softdevice Controller library, so we merely take ownership and ignore them
        let _ = (p, mpsl);

        let required = self.required_memory()?;
        match mem.len().cmp(&required) {
            core::cmp::Ordering::Less => {
                error!("Memory buffer too small. {} bytes needed.", required);
                return Err(Error::EINVAL);
            }
            core::cmp::Ordering::Equal => (),
            core::cmp::Ordering::Greater => {
                warn!("Memory buffer too big. {} bytes needed", required);
            }
        }

        RNG_POOL.store(rng as *const _ as *mut _, Ordering::Release);
        let rand_source = raw::sdc_rand_source_t {
            rand_prio_low_get: Some(rand_prio_low_get),
            rand_prio_high_get: Some(rand_prio_high_get),
            rand_poll: Some(rand_blocking),
        };
        let ret = unsafe { raw::sdc_rand_source_register(&rand_source) };
        RetVal::from(ret).to_result()?;

        let ret = unsafe { raw::sdc_enable(Some(sdc_callback), mem.as_mut_ptr()) };
        RetVal::from(ret)
            .to_result()
            .and(Ok(SoftdeviceController { _private: PhantomData }))
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
    // Prevent Send, Sync
    _private: PhantomData<&'d *mut ()>,
}

impl<'d> Drop for SoftdeviceController<'d> {
    fn drop(&mut self) {
        unsafe { raw::sdc_disable() };
        RNG_POOL.store(core::ptr::null_mut(), Ordering::Release);
    }
}

impl<'d> SoftdeviceController<'d> {
    pub fn build_revision() -> Result<[u8; raw::SDC_BUILD_REVISION_SIZE as usize], Error> {
        let mut rev = [0; raw::SDC_BUILD_REVISION_SIZE as usize];
        let ret = unsafe { raw::sdc_build_revision_get(rev.as_mut_ptr()) };
        RetVal::from(ret).to_result().and(Ok(rev))
    }

    pub fn hci_cmd_put(&self, buf: &[u8]) -> Result<(), Error> {
        assert!(buf.len() >= 3 && buf.len() >= 3 + usize::from(buf[2]));
        RetVal::from(unsafe { raw::sdc_hci_cmd_put(buf.as_ptr()) })
            .to_result()
            .and(Ok(()))
    }

    pub fn hci_data_put(&self, buf: &[u8]) -> Result<(), Error> {
        assert!(buf.len() >= 4 && buf.len() >= 4 + usize::from(u16::from_le_bytes([buf[2], buf[3]])));
        RetVal::from(unsafe { raw::sdc_hci_data_put(buf.as_ptr()) })
            .to_result()
            .and(Ok(()))
    }

    pub fn try_hci_get(&self, buf: &mut [u8]) -> Result<hci::MsgKind, Error> {
        assert!(buf.len() >= raw::HCI_MSG_BUFFER_MAX_SIZE as usize);
        let mut msg_type: raw::sdc_hci_msg_type_t = 0;
        let ret = unsafe { raw::sdc_hci_get(buf.as_mut_ptr(), &mut msg_type) };
        RetVal::from(ret).to_result().map(|_| match msg_type {
            raw::SDC_HCI_MSG_TYPE_DATA => hci::MsgKind::Data,
            raw::SDC_HCI_MSG_TYPE_EVT => hci::MsgKind::Event,
            _ => unreachable!(),
        })
    }

    pub async fn hci_get(&self, buf: &mut [u8]) -> Result<hci::MsgKind, Error> {
        poll_fn(|ctx| match self.try_hci_get(buf) {
            Err(Error::EAGAIN) => {
                WAKER.register(ctx.waker());
                Poll::Pending
            }
            res => Poll::Ready(res),
        })
        .await
    }

    pub async fn flash_write(&self, addr: u32, buf: &[u8]) -> Result<(), Error> {
        let ret = unsafe {
            raw::sdc_soc_flash_write_async(addr, buf.as_ptr() as *const _, buf.len() as u32, Some(flash_callback))
        };
        RetVal::from(ret).to_result()?;
        FLASH_STATUS.reset();
        FLASH_STATUS.wait().await
    }

    pub async fn flash_erase(&self, addr: u32) -> Result<(), Error> {
        let ret = unsafe { raw::sdc_soc_flash_page_erase_async(addr, Some(flash_callback)) };
        RetVal::from(ret).to_result()?;
        FLASH_STATUS.reset();
        FLASH_STATUS.wait().await
    }

    pub fn ecb_block_encrypt(&self, key: &[u8; 16], cleartext: &[u8], ciphertext: &mut [u8]) -> Result<(), Error> {
        assert_eq!(cleartext.len(), 16);
        assert_eq!(ciphertext.len(), 16);
        let ret = unsafe { raw::sdc_soc_ecb_block_encrypt(key.as_ptr(), cleartext.as_ptr(), ciphertext.as_mut_ptr()) };
        RetVal::from(ret).to_result().and(Ok(()))
    }
}

/// Bluetooth HCI Link Control commands (§7.1)
impl<'d> SoftdeviceController<'d> {
    pub fn disconnect(&self, conn_handle: hci::ConnHandle, reason: hci::Error) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_lc_disconnect_t {
            conn_handle: conn_handle.to_raw(),
            reason: reason.into(),
        };
        let ret = unsafe { raw::sdc_hci_cmd_lc_disconnect(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn read_remote_version_information(&self, conn_handle: hci::ConnHandle) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_lc_read_remote_version_information_t {
            conn_handle: conn_handle.to_raw(),
        };
        let ret = unsafe { raw::sdc_hci_cmd_lc_read_remote_version_information(&params) };
        hci::Status::from(ret).to_result()
    }
}

/// Bluetooth HCI Controller & Baseband commands (§7.3)
impl<'d> SoftdeviceController<'d> {
    pub fn set_event_mask(&self, mask: hci::EventMask) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_cb_set_event_mask_t { raw: mask.to_raw() };
        let ret = unsafe { raw::sdc_hci_cmd_cb_set_event_mask(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn reset(&self) -> Result<(), hci::Error> {
        let ret = unsafe { raw::sdc_hci_cmd_cb_reset() };
        hci::Status::from(ret).to_result()
    }

    pub fn read_transmit_power_level(&self, conn_handle: hci::ConnHandle, maximum: bool) -> Result<i8, hci::Error> {
        let params = raw::sdc_hci_cmd_cb_read_transmit_power_level_t {
            conn_handle: conn_handle.to_raw(),
            type_: u8::from(maximum),
        };
        let mut out = raw::sdc_hci_cmd_cb_read_transmit_power_level_return_t {
            conn_handle: 0,
            tx_power_level: 0,
        };
        let ret = unsafe { raw::sdc_hci_cmd_cb_read_transmit_power_level(&params, &mut out) };
        hci::Status::from(ret).to_result().map(|_| out.tx_power_level)
    }

    pub fn set_controller_to_host_flow_control(&self, enable: bool) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_cb_set_controller_to_host_flow_control_t {
            flow_control_enable: u8::from(enable),
        };
        let ret = unsafe { raw::sdc_hci_cmd_cb_set_controller_to_host_flow_control(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn host_buffer_size(
        &self,
        host_acl_data_packet_length: u16,
        host_sync_data_packet_length: u8,
        host_total_num_acl_data_packets: u16,
        host_total_num_sync_data_packets: u16,
    ) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_cb_host_buffer_size_t {
            host_acl_data_packet_length,
            host_sync_data_packet_length,
            host_total_num_acl_data_packets,
            host_total_num_sync_data_packets,
        };
        let ret = unsafe { raw::sdc_hci_cmd_cb_host_buffer_size(&params) };
        hci::Status::from(ret).to_result()
    }

    // NOTE: sdc_hci_cmd_cb_host_number_of_completed_packets not supported due to variable array parameter

    pub fn set_event_mask_page_2(&self, mask: hci::EventMaskPage2) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_cb_set_event_mask_page_2_t { raw: mask.to_raw() };
        let ret = unsafe { raw::sdc_hci_cmd_cb_set_event_mask_page_2(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn read_authenticated_payload_timeout(&self, conn_handle: hci::ConnHandle) -> Result<u16, hci::Error> {
        let params = raw::sdc_hci_cmd_cb_read_authenticated_payload_timeout_t {
            conn_handle: conn_handle.to_raw(),
        };
        let mut out = raw::sdc_hci_cmd_cb_read_authenticated_payload_timeout_return_t {
            conn_handle: 0,
            authenticated_payload_timeout: 0,
        };
        let ret = unsafe { raw::sdc_hci_cmd_cb_read_authenticated_payload_timeout(&params, &mut out) };
        hci::Status::from(ret)
            .to_result()
            .map(|_| out.authenticated_payload_timeout)
    }

    pub fn write_authenticated_payload_timeout(
        &self,
        conn_handle: hci::ConnHandle,
        authenticated_payload_timeout: u16,
    ) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_cb_write_authenticated_payload_timeout_t {
            conn_handle: conn_handle.to_raw(),
            authenticated_payload_timeout,
        };
        let mut out = raw::sdc_hci_cmd_cb_write_authenticated_payload_timeout_return_t { conn_handle: 0 };
        let ret = unsafe { raw::sdc_hci_cmd_cb_write_authenticated_payload_timeout(&params, &mut out) };
        hci::Status::from(ret).to_result()
    }
}

/// Bluetooth HCI Informational parameters (§7.4)
impl<'d> SoftdeviceController<'d> {
    pub fn read_local_version_information(&self) -> Result<hci::LocalVersionInformation, hci::Error> {
        let mut out = raw::sdc_hci_cmd_ip_read_local_version_information_return_t {
            hci_version: 0,
            hci_subversion: 0,
            lmp_version: 0,
            company_identifier: 0,
            lmp_subversion: 0,
        };
        let ret = unsafe { raw::sdc_hci_cmd_ip_read_local_version_information(&mut out) };
        hci::Status::from(ret).to_result().map(|_| {
            hci::LocalVersionInformation::new(
                out.hci_version,
                out.hci_subversion,
                out.lmp_version,
                out.company_identifier,
                out.lmp_subversion,
            )
        })
    }

    pub fn read_local_supported_commands(&self) -> Result<hci::CmdMask, hci::Error> {
        let mut out = raw::sdc_hci_cmd_ip_read_local_supported_commands_return_t { raw: [0; 64] };
        let ret = unsafe { raw::sdc_hci_cmd_ip_read_local_supported_commands(&mut out) };
        hci::Status::from(ret)
            .to_result()
            .map(|_| hci::CmdMask::new(unsafe { out.raw }))
    }

    pub fn read_local_supported_features(&self) -> Result<hci::LmpFeatureMask, hci::Error> {
        let mut out = raw::sdc_hci_cmd_ip_read_local_supported_features_return_t { raw: [0; 8] };
        let ret = unsafe { raw::sdc_hci_cmd_ip_read_local_supported_features(&mut out) };
        hci::Status::from(ret)
            .to_result()
            .map(|_| hci::LmpFeatureMask::new(unsafe { out.raw }))
    }

    pub fn read_bd_addr(&self) -> Result<hci::BdAddr, hci::Error> {
        let mut out = raw::sdc_hci_cmd_ip_read_bd_addr_return_t { bd_addr: [0; 6] };
        let ret = unsafe { raw::sdc_hci_cmd_ip_read_bd_addr(&mut out) };
        hci::Status::from(ret)
            .to_result()
            .map(|_| hci::BdAddr::new(out.bd_addr))
    }
}

/// Bluetooth HCI Status parameters (§7.5)
impl<'d> SoftdeviceController<'d> {
    pub fn read_rssi(&self, conn_handle: hci::ConnHandle) -> Result<i8, hci::Error> {
        let params = raw::sdc_hci_cmd_sp_read_rssi_t {
            handle: conn_handle.to_raw(),
        };
        let mut out = raw::sdc_hci_cmd_sp_read_rssi_return_t { handle: 0, rssi: 0 };
        let ret = unsafe { raw::sdc_hci_cmd_sp_read_rssi(&params, &mut out) };
        hci::Status::from(ret).to_result().map(|_| out.rssi)
    }
}

/// Bluetooth HCI LE Controller commands (§7.8)
#[allow(clippy::too_many_arguments)]
impl<'d> SoftdeviceController<'d> {
    pub fn le_set_event_mask(&self, mask: hci::LeEventMask) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_le_set_event_mask_t { raw: mask.to_raw() };
        let ret = unsafe { raw::sdc_hci_cmd_le_set_event_mask(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn le_read_buffer_size(&self) -> Result<hci::LeReadBufferSize, hci::Error> {
        let mut out = raw::sdc_hci_cmd_le_read_buffer_size_return_t {
            le_acl_data_packet_length: 0,
            total_num_le_acl_data_packets: 0,
        };
        let ret = unsafe { raw::sdc_hci_cmd_le_read_buffer_size(&mut out) };
        hci::Status::from(ret)
            .to_result()
            .map(|_| hci::LeReadBufferSize::new(out.le_acl_data_packet_length, out.total_num_le_acl_data_packets, 0, 0))
    }

    pub fn le_read_local_supported_features(&self) -> Result<hci::LeFeatureMask, hci::Error> {
        let mut out = raw::sdc_hci_cmd_le_read_local_supported_features_return_t { raw: [0; 8] };
        let ret = unsafe { raw::sdc_hci_cmd_le_read_local_supported_features(&mut out) };
        hci::Status::from(ret)
            .to_result()
            .map(|_| hci::LeFeatureMask::new(unsafe { out.raw }))
    }

    pub fn le_set_random_address(&self, random_address: hci::BdAddr) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_le_set_random_address_t {
            random_address: random_address.to_raw(),
        };
        let ret = unsafe { raw::sdc_hci_cmd_le_set_random_address(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn le_set_adv_params(
        &self,
        adv_interval_min: hci::Duration,
        adv_interval_max: hci::Duration,
        adv_type: hci::AdvertisingType,
        own_address_type: hci::AddressType,
        peer_address_type: hci::AddressType,
        peer_address: hci::BdAddr,
        adv_channel_map: hci::AdvertisingChannelMap,
        adv_filter_policy: hci::AdvertisingFilterPolicy,
    ) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_le_set_adv_params_t {
            adv_interval_min: adv_interval_min.as_u16(),
            adv_interval_max: adv_interval_max.as_u16(),
            adv_type: adv_type.to_raw(),
            own_address_type: own_address_type.to_raw(),
            peer_address_type: peer_address_type.to_raw(),
            peer_address: peer_address.to_raw(),
            adv_channel_map: adv_channel_map.to_raw(),
            adv_filter_policy: adv_filter_policy.to_raw(),
        };
        let ret = unsafe { raw::sdc_hci_cmd_le_set_adv_params(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn le_read_adv_physical_channel_tx_power(&self) -> Result<i8, hci::Error> {
        let mut out = raw::sdc_hci_cmd_le_read_adv_physical_channel_tx_power_return_t { tx_power_level: 0 };
        let ret = unsafe { raw::sdc_hci_cmd_le_read_adv_physical_channel_tx_power(&mut out) };
        hci::Status::from(ret).to_result().map(|_| out.tx_power_level)
    }

    pub fn le_set_adv_data(&self, adv_data: &[u8]) -> Result<(), hci::Error> {
        assert!(adv_data.len() <= 31);
        let mut params = raw::sdc_hci_cmd_le_set_adv_data_t {
            adv_data_length: adv_data.len() as u8,
            adv_data: [0; 31],
        };
        params.adv_data[..adv_data.len()].copy_from_slice(adv_data);
        let ret = unsafe { raw::sdc_hci_cmd_le_set_adv_data(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn le_set_scan_response_data(&self, scan_response_data: &[u8]) -> Result<(), hci::Error> {
        assert!(scan_response_data.len() <= 31);
        let mut params = raw::sdc_hci_cmd_le_set_scan_response_data_t {
            scan_response_data_length: scan_response_data.len() as u8,
            scan_response_data: [0; 31],
        };
        params.scan_response_data[..scan_response_data.len()].copy_from_slice(scan_response_data);
        let ret = unsafe { raw::sdc_hci_cmd_le_set_scan_response_data(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn le_set_adv_enable(&self, adv_enable: bool) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_le_set_adv_enable_t {
            adv_enable: u8::from(adv_enable),
        };
        let ret = unsafe { raw::sdc_hci_cmd_le_set_adv_enable(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn le_set_scan_params(
        &self,
        le_scan_type: hci::LeScanType,
        le_scan_interval: hci::Duration,
        le_scan_window: hci::Duration,
        own_address_type: hci::AddressType,
        scanning_filter_policy: hci::ScanningFilterPolicy,
    ) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_le_set_scan_params_t {
            le_scan_type: le_scan_type.to_raw(),
            le_scan_interval: le_scan_interval.as_u16(),
            le_scan_window: le_scan_window.as_u16(),
            own_address_type: own_address_type.to_raw(),
            scanning_filter_policy: scanning_filter_policy.to_raw(),
        };
        let ret = unsafe { raw::sdc_hci_cmd_le_set_scan_params(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn le_set_scan_enable(&self, le_scan_enable: bool, filter_duplicates: bool) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_le_set_scan_enable_t {
            le_scan_enable: u8::from(le_scan_enable),
            filter_duplicates: u8::from(filter_duplicates),
        };
        let ret = unsafe { raw::sdc_hci_cmd_le_set_scan_enable(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn le_create_conn(
        &self,
        le_scan_interval: hci::Duration,
        le_scan_window: hci::Duration,
        initiator_filter_policy: bool,
        peer_address_type: hci::AddressType,
        peer_address: hci::BdAddr,
        own_address_type: hci::AddressType,
        conn_interval_min: hci::Duration<2048>,
        conn_interval_max: hci::Duration<2048>,
        max_latency: u16,
        supervision_timeout: hci::Duration<16>,
        min_ce_len: hci::Duration,
        max_ce_len: hci::Duration,
    ) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_le_create_conn_t {
            le_scan_interval: le_scan_interval.as_u16(),
            le_scan_window: le_scan_window.as_u16(),
            initiator_filter_policy: u8::from(initiator_filter_policy),
            peer_address_type: peer_address_type.to_raw(),
            peer_address: peer_address.to_raw(),
            own_address_type: own_address_type.to_raw(),
            conn_interval_min: conn_interval_min.as_u16(),
            conn_interval_max: conn_interval_max.as_u16(),
            max_latency,
            supervision_timeout: supervision_timeout.as_u16(),
            min_ce_length: min_ce_len.as_u16(),
            max_ce_length: max_ce_len.as_u16(),
        };
        let ret = unsafe { raw::sdc_hci_cmd_le_create_conn(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn le_create_conn_cancel(&self) -> Result<(), hci::Error> {
        let ret = unsafe { raw::sdc_hci_cmd_le_create_conn_cancel() };
        hci::Status::from(ret).to_result()
    }

    pub fn le_read_filter_accept_list_size(&self) -> Result<u8, hci::Error> {
        let mut out = raw::sdc_hci_cmd_le_read_filter_accept_list_size_return_t {
            filter_accept_list_size: 0,
        };
        let ret = unsafe { raw::sdc_hci_cmd_le_read_filter_accept_list_size(&mut out) };
        hci::Status::from(ret).to_result().map(|_| out.filter_accept_list_size)
    }

    pub fn le_clear_filter_accept_list(&self) -> Result<(), hci::Error> {
        let ret = unsafe { raw::sdc_hci_cmd_le_clear_filter_accept_list() };
        hci::Status::from(ret).to_result()
    }

    pub fn le_add_device_to_filter_accept_list(
        &self,
        address_type: hci::AddressType,
        address: hci::BdAddr,
    ) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_le_add_device_to_filter_accept_list_t {
            address_type: address_type.to_raw(),
            address: address.to_raw(),
        };
        let ret = unsafe { raw::sdc_hci_cmd_le_add_device_to_filter_accept_list(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn le_remove_device_from_filter_accept_list(
        &self,
        address_type: hci::AddressType,
        address: hci::BdAddr,
    ) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_le_remove_device_from_filter_accept_list_t {
            address_type: address_type.to_raw(),
            address: address.to_raw(),
        };
        let ret = unsafe { raw::sdc_hci_cmd_le_remove_device_from_filter_accept_list(&params) };
        hci::Status::from(ret).to_result()
    }

    pub fn le_conn_update(
        &self,
        conn_handle: hci::ConnHandle,
        conn_interval_min: hci::Duration,
        conn_interval_max: hci::Duration,
        max_latency: u16,
        supervision_timeout: hci::Duration<16>,
        min_ce_len: hci::Duration,
        max_ce_len: hci::Duration,
    ) -> Result<(), hci::Error> {
        let params = raw::sdc_hci_cmd_le_conn_update_t {
            conn_handle: conn_handle.to_raw(),
            conn_interval_min: conn_interval_min.as_u16(),
            conn_interval_max: conn_interval_max.as_u16(),
            max_latency,
            supervision_timeout: supervision_timeout.as_u16(),
            min_ce_length: min_ce_len.as_u16(),
            max_ce_length: max_ce_len.as_u16(),
        };
        let ret = unsafe { raw::sdc_hci_cmd_le_conn_update(&params) };
        hci::Status::from(ret).to_result()
    }
    //     pub fn le_set_host_channel_classification(&self,
    //         p_params: *const sdc_hci_cmd_le_set_host_channel_classification_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_host_channel_classification(
    //             p_params: *const sdc_hci_cmd_le_set_host_channel_classification_t,
    //         )
    //     }
    //     pub fn le_read_channel_map(&self,
    //         p_params: *const sdc_hci_cmd_le_read_channel_map_t,
    //         p_return: *mut sdc_hci_cmd_le_read_channel_map_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_channel_map(
    //             p_params: *const sdc_hci_cmd_le_read_channel_map_t,
    //             p_return: *mut sdc_hci_cmd_le_read_channel_map_return_t,
    //         )
    //     }
    //     pub fn le_read_remote_features(&self, p_params: *const sdc_hci_cmd_le_read_remote_features_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_remote_features(p_params: *const sdc_hci_cmd_le_read_remote_features_t)
    //     }
    //     pub fn le_encrypt(&self,
    //         p_params: *const sdc_hci_cmd_le_encrypt_t,
    //         p_return: *mut sdc_hci_cmd_le_encrypt_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_encrypt(
    //             p_params: *const sdc_hci_cmd_le_encrypt_t,
    //             p_return: *mut sdc_hci_cmd_le_encrypt_return_t,
    //         )
    //     }
    //     pub fn le_rand(&self, p_return: *mut sdc_hci_cmd_le_rand_return_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_rand(p_return: *mut sdc_hci_cmd_le_rand_return_t)
    //     }
    //     pub fn le_enable_encryption(&self, p_params: *const sdc_hci_cmd_le_enable_encryption_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_enable_encryption(p_params: *const sdc_hci_cmd_le_enable_encryption_t)
    //     }
    //     pub fn le_long_term_key_request_reply(&self,
    //         p_params: *const sdc_hci_cmd_le_long_term_key_request_reply_t,
    //         p_return: *mut sdc_hci_cmd_le_long_term_key_request_reply_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_long_term_key_request_reply(
    //             p_params: *const sdc_hci_cmd_le_long_term_key_request_reply_t,
    //             p_return: *mut sdc_hci_cmd_le_long_term_key_request_reply_return_t,
    //         )
    //     }
    //     pub fn le_long_term_key_request_negative_reply(&self,
    //         p_params: *const sdc_hci_cmd_le_long_term_key_request_negative_reply_t,
    //         p_return: *mut sdc_hci_cmd_le_long_term_key_request_negative_reply_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_long_term_key_request_negative_reply(
    //             p_params: *const sdc_hci_cmd_le_long_term_key_request_negative_reply_t,
    //             p_return: *mut sdc_hci_cmd_le_long_term_key_request_negative_reply_return_t,
    //         )
    //     }
    //     pub fn le_read_supported_states(&self,
    //         p_return: *mut sdc_hci_cmd_le_read_supported_states_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_supported_states(p_return: *mut sdc_hci_cmd_le_read_supported_states_return_t)
    //     }
    //     pub fn le_test_end(&self, p_return: *mut sdc_hci_cmd_le_test_end_return_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_test_end(p_return: *mut sdc_hci_cmd_le_test_end_return_t)
    //     }
    //     pub fn le_set_data_length(&self,
    //         p_params: *const sdc_hci_cmd_le_set_data_length_t,
    //         p_return: *mut sdc_hci_cmd_le_set_data_length_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_data_length(
    //             p_params: *const sdc_hci_cmd_le_set_data_length_t,
    //             p_return: *mut sdc_hci_cmd_le_set_data_length_return_t,
    //         )
    //     }
    //     pub fn le_read_suggested_default_data_length(&self,
    //         p_return: *mut sdc_hci_cmd_le_read_suggested_default_data_length_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_suggested_default_data_length(
    //             p_return: *mut sdc_hci_cmd_le_read_suggested_default_data_length_return_t,
    //         )
    //     }
    //     pub fn le_write_suggested_default_data_length(&self,
    //         p_params: *const sdc_hci_cmd_le_write_suggested_default_data_length_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_write_suggested_default_data_length(
    //             p_params: *const sdc_hci_cmd_le_write_suggested_default_data_length_t,
    //         )
    //     }
    //     pub fn le_add_device_to_resolving_list(&self,
    //         p_params: *const sdc_hci_cmd_le_add_device_to_resolving_list_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_add_device_to_resolving_list(p_params: *const sdc_hci_cmd_le_add_device_to_resolving_list_t)
    //     }
    //     pub fn le_remove_device_from_resolving_list(&self,
    //         p_params: *const sdc_hci_cmd_le_remove_device_from_resolving_list_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_remove_device_from_resolving_list(
    //             p_params: *const sdc_hci_cmd_le_remove_device_from_resolving_list_t,
    //         )
    //     }
    //     pub fn le_clear_resolving_list(&self, ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_clear_resolving_list()
    //     }
    //     pub fn le_read_resolving_list_size(&self,
    //         p_return: *mut sdc_hci_cmd_le_read_resolving_list_size_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_resolving_list_size(p_return: *mut sdc_hci_cmd_le_read_resolving_list_size_return_t)
    //     }
    //     pub fn le_set_address_resolution_enable(&self,
    //         p_params: *const sdc_hci_cmd_le_set_address_resolution_enable_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_address_resolution_enable(
    //             p_params: *const sdc_hci_cmd_le_set_address_resolution_enable_t,
    //         )
    //     }
    //     pub fn le_set_resolvable_private_address_timeout(&self,
    //         p_params: *const sdc_hci_cmd_le_set_resolvable_private_address_timeout_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_resolvable_private_address_timeout(
    //             p_params: *const sdc_hci_cmd_le_set_resolvable_private_address_timeout_t,
    //         )
    //     }
    //     pub fn le_read_max_data_length(&self,
    //         p_return: *mut sdc_hci_cmd_le_read_max_data_length_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_max_data_length(p_return: *mut sdc_hci_cmd_le_read_max_data_length_return_t)
    //     }
    //     pub fn le_read_phy(&self,
    //         p_params: *const sdc_hci_cmd_le_read_phy_t,
    //         p_return: *mut sdc_hci_cmd_le_read_phy_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_phy(
    //             p_params: *const sdc_hci_cmd_le_read_phy_t,
    //             p_return: *mut sdc_hci_cmd_le_read_phy_return_t,
    //         )
    //     }
    //     pub fn le_set_default_phy(&self, p_params: *const sdc_hci_cmd_le_set_default_phy_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_default_phy(p_params: *const sdc_hci_cmd_le_set_default_phy_t)
    //     }
    //     pub fn le_set_phy(&self, p_params: *const sdc_hci_cmd_le_set_phy_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_phy(p_params: *const sdc_hci_cmd_le_set_phy_t)
    //     }
    //     pub fn le_set_adv_set_random_address(&self,
    //         p_params: *const sdc_hci_cmd_le_set_adv_set_random_address_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_adv_set_random_address(p_params: *const sdc_hci_cmd_le_set_adv_set_random_address_t)
    //     }
    //     pub fn le_set_ext_adv_params(&self,
    //         p_params: *const sdc_hci_cmd_le_set_ext_adv_params_t,
    //         p_return: *mut sdc_hci_cmd_le_set_ext_adv_params_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_ext_adv_params(
    //             p_params: *const sdc_hci_cmd_le_set_ext_adv_params_t,
    //             p_return: *mut sdc_hci_cmd_le_set_ext_adv_params_return_t,
    //         )
    //     }
    //     pub fn le_set_ext_adv_data(&self, p_params: *const sdc_hci_cmd_le_set_ext_adv_data_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_ext_adv_data(p_params: *const sdc_hci_cmd_le_set_ext_adv_data_t)
    //     }
    //     pub fn le_set_ext_scan_response_data(&self,
    //         p_params: *const sdc_hci_cmd_le_set_ext_scan_response_data_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_ext_scan_response_data(p_params: *const sdc_hci_cmd_le_set_ext_scan_response_data_t)
    //     }
    //     pub fn le_set_ext_adv_enable(&self, p_params: *const sdc_hci_cmd_le_set_ext_adv_enable_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_ext_adv_enable(p_params: *const sdc_hci_cmd_le_set_ext_adv_enable_t)
    //     }
    //     pub fn le_read_max_adv_data_length(&self,
    //         p_return: *mut sdc_hci_cmd_le_read_max_adv_data_length_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_max_adv_data_length(p_return: *mut sdc_hci_cmd_le_read_max_adv_data_length_return_t)
    //     }
    //     pub fn le_read_number_of_supported_adv_sets(&self,
    //         p_return: *mut sdc_hci_cmd_le_read_number_of_supported_adv_sets_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_number_of_supported_adv_sets(
    //             p_return: *mut sdc_hci_cmd_le_read_number_of_supported_adv_sets_return_t,
    //         )
    //     }
    //     pub fn le_remove_adv_set(&self, p_params: *const sdc_hci_cmd_le_remove_adv_set_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_remove_adv_set(p_params: *const sdc_hci_cmd_le_remove_adv_set_t)
    //     }
    //     pub fn le_clear_adv_sets(&self, ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_clear_adv_sets()
    //     }
    //     pub fn le_set_periodic_adv_params(&self,
    //         p_params: *const sdc_hci_cmd_le_set_periodic_adv_params_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_periodic_adv_params(p_params: *const sdc_hci_cmd_le_set_periodic_adv_params_t)
    //     }
    //     pub fn le_set_periodic_adv_data(&self, p_params: *const sdc_hci_cmd_le_set_periodic_adv_data_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_periodic_adv_data(p_params: *const sdc_hci_cmd_le_set_periodic_adv_data_t)
    //     }
    //     pub fn le_set_periodic_adv_enable(&self,
    //         p_params: *const sdc_hci_cmd_le_set_periodic_adv_enable_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_periodic_adv_enable(p_params: *const sdc_hci_cmd_le_set_periodic_adv_enable_t)
    //     }
    //     pub fn le_set_ext_scan_params(&self, p_params: *const sdc_hci_cmd_le_set_ext_scan_params_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_ext_scan_params(p_params: *const sdc_hci_cmd_le_set_ext_scan_params_t)
    //     }
    //     pub fn le_set_ext_scan_enable(&self, p_params: *const sdc_hci_cmd_le_set_ext_scan_enable_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_ext_scan_enable(p_params: *const sdc_hci_cmd_le_set_ext_scan_enable_t)
    //     }
    //     pub fn le_ext_create_conn(&self, p_params: *const sdc_hci_cmd_le_ext_create_conn_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_ext_create_conn(p_params: *const sdc_hci_cmd_le_ext_create_conn_t)
    //     }
    //     pub fn le_periodic_adv_create_sync(&self,
    //         p_params: *const sdc_hci_cmd_le_periodic_adv_create_sync_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_periodic_adv_create_sync(p_params: *const sdc_hci_cmd_le_periodic_adv_create_sync_t)
    //     }
    //     pub fn le_periodic_adv_create_sync_cancel(&self, ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_periodic_adv_create_sync_cancel()
    //     }
    //     pub fn le_periodic_adv_terminate_sync(&self,
    //         p_params: *const sdc_hci_cmd_le_periodic_adv_terminate_sync_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_periodic_adv_terminate_sync(p_params: *const sdc_hci_cmd_le_periodic_adv_terminate_sync_t)
    //     }
    //     pub fn le_add_device_to_periodic_adv_list(&self,
    //         p_params: *const sdc_hci_cmd_le_add_device_to_periodic_adv_list_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_add_device_to_periodic_adv_list(
    //             p_params: *const sdc_hci_cmd_le_add_device_to_periodic_adv_list_t,
    //         )
    //     }
    //     pub fn le_remove_device_from_periodic_adv_list(&self,
    //         p_params: *const sdc_hci_cmd_le_remove_device_from_periodic_adv_list_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_remove_device_from_periodic_adv_list(
    //             p_params: *const sdc_hci_cmd_le_remove_device_from_periodic_adv_list_t,
    //         )
    //     }
    //     pub fn le_clear_periodic_adv_list(&self, ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_clear_periodic_adv_list()
    //     }
    //     pub fn le_read_periodic_adv_list_size(&self,
    //         p_return: *mut sdc_hci_cmd_le_read_periodic_adv_list_size_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_periodic_adv_list_size(
    //             p_return: *mut sdc_hci_cmd_le_read_periodic_adv_list_size_return_t,
    //         )
    //     }
    //     pub fn le_read_transmit_power(&self,
    //         p_return: *mut sdc_hci_cmd_le_read_transmit_power_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_transmit_power(p_return: *mut sdc_hci_cmd_le_read_transmit_power_return_t)
    //     }
    //     pub fn le_read_rf_path_compensation(&self,
    //         p_return: *mut sdc_hci_cmd_le_read_rf_path_compensation_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_rf_path_compensation(p_return: *mut sdc_hci_cmd_le_read_rf_path_compensation_return_t)
    //     }
    //     pub fn le_write_rf_path_compensation(&self,
    //         p_params: *const sdc_hci_cmd_le_write_rf_path_compensation_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_write_rf_path_compensation(p_params: *const sdc_hci_cmd_le_write_rf_path_compensation_t)
    //     }
    //     pub fn le_set_privacy_mode(&self, p_params: *const sdc_hci_cmd_le_set_privacy_mode_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_privacy_mode(p_params: *const sdc_hci_cmd_le_set_privacy_mode_t)
    //     }
    //     pub fn le_set_connless_cte_transmit_params(&self,
    //         p_params: *const sdc_hci_cmd_le_set_connless_cte_transmit_params_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_connless_cte_transmit_params(
    //             p_params: *const sdc_hci_cmd_le_set_connless_cte_transmit_params_t,
    //         )
    //     }
    //     pub fn le_set_connless_cte_transmit_enable(&self,
    //         p_params: *const sdc_hci_cmd_le_set_connless_cte_transmit_enable_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_connless_cte_transmit_enable(
    //             p_params: *const sdc_hci_cmd_le_set_connless_cte_transmit_enable_t,
    //         )
    //     }
    //     pub fn le_set_conn_cte_transmit_params(&self,
    //         p_params: *const sdc_hci_cmd_le_set_conn_cte_transmit_params_t,
    //         p_return: *mut sdc_hci_cmd_le_set_conn_cte_transmit_params_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_conn_cte_transmit_params(
    //             p_params: *const sdc_hci_cmd_le_set_conn_cte_transmit_params_t,
    //             p_return: *mut sdc_hci_cmd_le_set_conn_cte_transmit_params_return_t,
    //         )
    //     }
    //     pub fn le_conn_cte_response_enable(&self,
    //         p_params: *const sdc_hci_cmd_le_conn_cte_response_enable_t,
    //         p_return: *mut sdc_hci_cmd_le_conn_cte_response_enable_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_conn_cte_response_enable(
    //             p_params: *const sdc_hci_cmd_le_conn_cte_response_enable_t,
    //             p_return: *mut sdc_hci_cmd_le_conn_cte_response_enable_return_t,
    //         )
    //     }
    //     pub fn le_read_antenna_information(&self,
    //         p_return: *mut sdc_hci_cmd_le_read_antenna_information_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_antenna_information(p_return: *mut sdc_hci_cmd_le_read_antenna_information_return_t)
    //     }
    //     pub fn le_set_periodic_adv_receive_enable(&self,
    //         p_params: *const sdc_hci_cmd_le_set_periodic_adv_receive_enable_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_periodic_adv_receive_enable(
    //             p_params: *const sdc_hci_cmd_le_set_periodic_adv_receive_enable_t,
    //         )
    //     }
    //     pub fn le_periodic_adv_sync_transfer(&self,
    //         p_params: *const sdc_hci_cmd_le_periodic_adv_sync_transfer_t,
    //         p_return: *mut sdc_hci_cmd_le_periodic_adv_sync_transfer_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_periodic_adv_sync_transfer(
    //             p_params: *const sdc_hci_cmd_le_periodic_adv_sync_transfer_t,
    //             p_return: *mut sdc_hci_cmd_le_periodic_adv_sync_transfer_return_t,
    //         )
    //     }
    //     pub fn le_periodic_adv_set_info_transfer(&self,
    //         p_params: *const sdc_hci_cmd_le_periodic_adv_set_info_transfer_t,
    //         p_return: *mut sdc_hci_cmd_le_periodic_adv_set_info_transfer_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_periodic_adv_set_info_transfer(
    //             p_params: *const sdc_hci_cmd_le_periodic_adv_set_info_transfer_t,
    //             p_return: *mut sdc_hci_cmd_le_periodic_adv_set_info_transfer_return_t,
    //         )
    //     }
    //     pub fn le_set_periodic_adv_sync_transfer_params(&self,
    //         p_params: *const sdc_hci_cmd_le_set_periodic_adv_sync_transfer_params_t,
    //         p_return: *mut sdc_hci_cmd_le_set_periodic_adv_sync_transfer_params_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_periodic_adv_sync_transfer_params(
    //             p_params: *const sdc_hci_cmd_le_set_periodic_adv_sync_transfer_params_t,
    //             p_return: *mut sdc_hci_cmd_le_set_periodic_adv_sync_transfer_params_return_t,
    //         )
    //     }
    //     pub fn le_set_default_periodic_adv_sync_transfer_params(&self,
    //         p_params: *const sdc_hci_cmd_le_set_default_periodic_adv_sync_transfer_params_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_default_periodic_adv_sync_transfer_params(
    //             p_params: *const sdc_hci_cmd_le_set_default_periodic_adv_sync_transfer_params_t,
    //         )
    //     }
    //     pub fn le_request_peer_sca(&self, p_params: *const sdc_hci_cmd_le_request_peer_sca_t) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_request_peer_sca(p_params: *const sdc_hci_cmd_le_request_peer_sca_t)
    //     }
    //     pub fn le_enhanced_read_transmit_power_level(&self,
    //         p_params: *const sdc_hci_cmd_le_enhanced_read_transmit_power_level_t,
    //         p_return: *mut sdc_hci_cmd_le_enhanced_read_transmit_power_level_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_enhanced_read_transmit_power_level(
    //             p_params: *const sdc_hci_cmd_le_enhanced_read_transmit_power_level_t,
    //             p_return: *mut sdc_hci_cmd_le_enhanced_read_transmit_power_level_return_t,
    //         )
    //     }
    //     pub fn le_read_remote_transmit_power_level(&self,
    //         p_params: *const sdc_hci_cmd_le_read_remote_transmit_power_level_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_read_remote_transmit_power_level(
    //             p_params: *const sdc_hci_cmd_le_read_remote_transmit_power_level_t,
    //         )
    //     }
    //     pub fn le_set_path_loss_reporting_params(&self,
    //         p_params: *const sdc_hci_cmd_le_set_path_loss_reporting_params_t,
    //         p_return: *mut sdc_hci_cmd_le_set_path_loss_reporting_params_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_path_loss_reporting_params(
    //             p_params: *const sdc_hci_cmd_le_set_path_loss_reporting_params_t,
    //             p_return: *mut sdc_hci_cmd_le_set_path_loss_reporting_params_return_t,
    //         )
    //     }
    //     pub fn le_set_path_loss_reporting_enable(&self,
    //         p_params: *const sdc_hci_cmd_le_set_path_loss_reporting_enable_t,
    //         p_return: *mut sdc_hci_cmd_le_set_path_loss_reporting_enable_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_path_loss_reporting_enable(
    //             p_params: *const sdc_hci_cmd_le_set_path_loss_reporting_enable_t,
    //             p_return: *mut sdc_hci_cmd_le_set_path_loss_reporting_enable_return_t,
    //         )
    //     }
    //     pub fn le_set_transmit_power_reporting_enable(&self,
    //         p_params: *const sdc_hci_cmd_le_set_transmit_power_reporting_enable_t,
    //         p_return: *mut sdc_hci_cmd_le_set_transmit_power_reporting_enable_return_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_transmit_power_reporting_enable(
    //             p_params: *const sdc_hci_cmd_le_set_transmit_power_reporting_enable_t,
    //             p_return: *mut sdc_hci_cmd_le_set_transmit_power_reporting_enable_return_t,
    //         )
    //     }
    //     pub fn le_set_data_related_address_changes(&self,
    //         p_params: *const sdc_hci_cmd_le_set_data_related_address_changes_t,
    //     ) -> Result<(), hci::Error> {
    //         raw::sdc_hci_cmd_le_set_data_related_address_changes(
    //             p_params: *const sdc_hci_cmd_le_set_data_related_address_changes_t,
    //         )
    //     }
}

/// Bluetooth HCI vendor specific commands
impl<'d> SoftdeviceController<'d> {
    // TODO
    // fn zephyr_read_version_info(p_return: *mut sdc_hci_cmd_vs_zephyr_read_version_info_return_t) -> u8;
    // fn zephyr_read_supported_commands( p_return: *mut sdc_hci_cmd_vs_zephyr_read_supported_commands_return_t, ) -> u8;
    pub fn zephyr_write_bd_addr(&self, bd_addr: hci::BdAddr) -> Result<(), hci::Error> {
        let ret = unsafe {
            raw::sdc_hci_cmd_vs_zephyr_write_bd_addr(&raw::sdc_hci_cmd_vs_zephyr_write_bd_addr_t {
                bd_addr: bd_addr.to_raw(),
            })
        };
        hci::Status::from(ret).to_result()
    }
    // fn zephyr_read_static_addresses( p_return: *mut sdc_hci_cmd_vs_zephyr_read_static_addresses_return_t, ) -> u8;
    // fn zephyr_read_key_hierarchy_roots( p_return: *mut sdc_hci_cmd_vs_zephyr_read_key_hierarchy_roots_return_t, ) -> u8;
    // fn zephyr_read_chip_temp(p_return: *mut sdc_hci_cmd_vs_zephyr_read_chip_temp_return_t) -> u8;
    // fn zephyr_write_tx_power( p_params: *const sdc_hci_cmd_vs_zephyr_write_tx_power_t, p_return: *mut sdc_hci_cmd_vs_zephyr_write_tx_power_return_t, ) -> u8;
    // fn zephyr_read_tx_power( p_params: *const sdc_hci_cmd_vs_zephyr_read_tx_power_t, p_return: *mut sdc_hci_cmd_vs_zephyr_read_tx_power_return_t, ) -> u8;
    // fn read_supported_vs_commands( p_return: *mut sdc_hci_cmd_vs_read_supported_vs_commands_return_t, ) -> u8;
    // fn llpm_mode_set(p_params: *const sdc_hci_cmd_vs_llpm_mode_set_t) -> u8;
    // fn conn_update(p_params: *const sdc_hci_cmd_vs_conn_update_t) -> u8;
    // fn conn_event_extend(p_params: *const sdc_hci_cmd_vs_conn_event_extend_t) -> u8;
    // fn qos_conn_event_report_enable( p_params: *const sdc_hci_cmd_vs_qos_conn_event_report_enable_t, ) -> u8;
    // fn event_length_set(p_params: *const sdc_hci_cmd_vs_event_length_set_t) -> u8;
    // fn periodic_adv_event_length_set( p_params: *const sdc_hci_cmd_vs_periodic_adv_event_length_set_t, ) -> u8;
    // fn coex_scan_mode_config(p_params: *const sdc_hci_cmd_vs_coex_scan_mode_config_t) -> u8;
    // fn coex_priority_config(p_params: *const sdc_hci_cmd_vs_coex_priority_config_t) -> u8;
    // fn peripheral_latency_mode_set(p_params: *const sdc_hci_cmd_vs_peripheral_latency_mode_set_t) -> u8;
    // fn write_remote_tx_power(p_params: *const sdc_hci_cmd_vs_write_remote_tx_power_t) -> u8;
    // fn set_auto_power_control_request_param( p_params: *const sdc_hci_cmd_vs_set_auto_power_control_request_param_t, ) -> u8;
    // fn set_adv_randomness(p_params: *const sdc_hci_cmd_vs_set_adv_randomness_t) -> u8;
}
