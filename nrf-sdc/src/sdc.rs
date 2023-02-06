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

use self::sealed::Sealed;
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
        unwrap!(CStr::from_ptr(file).to_str()),
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
        assert!(buf.len() >= 4 && buf.len() >= 4 + usize::from(buf[2]) + (usize::from(buf[3]) << 8));
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

pub enum FlashError {}
mod sealed {
    pub trait Sealed {}
}

pub trait VendorExt: Sealed {
    // TODO
    // fn zephyr_read_version_info(p_return: *mut sdc_hci_cmd_vs_zephyr_read_version_info_return_t) -> u8;
    // fn zephyr_read_supported_commands( p_return: *mut sdc_hci_cmd_vs_zephyr_read_supported_commands_return_t, ) -> u8;
    // fn zephyr_write_bd_addr(p_params: *const sdc_hci_cmd_vs_zephyr_write_bd_addr_t) -> u8;
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

impl<'d> Sealed for SoftdeviceController<'d> {}
impl<'d> VendorExt for SoftdeviceController<'d> {}
