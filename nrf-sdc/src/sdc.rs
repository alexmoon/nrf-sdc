use core::ffi::CStr;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicPtr, Ordering};
use core::task::{Poll, Waker};

use bt_hci::{AsHciBytes, FixedSizeValue, FromHciBytes};
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
use crate::{pac, raw, Error, RetVal};

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

    pub fn try_hci_get(&self, buf: &mut [u8]) -> Result<bt_hci::PacketKind, Error> {
        assert!(buf.len() >= raw::HCI_MSG_BUFFER_MAX_SIZE as usize);
        let mut msg_type: raw::sdc_hci_msg_type_t = 0;
        let ret = unsafe { raw::sdc_hci_get(buf.as_mut_ptr(), &mut msg_type) };
        RetVal::from(ret).to_result().map(|_| match msg_type {
            raw::SDC_HCI_MSG_TYPE_DATA => bt_hci::PacketKind::AclData,
            raw::SDC_HCI_MSG_TYPE_EVT => bt_hci::PacketKind::Event,
            _ => unreachable!(),
        })
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

    #[inline(always)]
    unsafe fn raw_cmd(&self, f: unsafe extern "C" fn() -> u8) -> Result<(), bt_hci::param::Error> {
        bt_hci::param::Status::from(f()).to_result()
    }

    #[inline(always)]
    unsafe fn raw_cmd_params<P1: FixedSizeValue, P2: Copy>(
        &self,
        f: unsafe extern "C" fn(*const P2) -> u8,
        params: P1,
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
        params: P1,
    ) -> Result<R1, bt_hci::param::Error> {
        debug_assert_eq!(core::mem::size_of::<P1>(), core::mem::size_of::<P2>());
        debug_assert_eq!(core::mem::size_of::<R1>(), core::mem::size_of::<R2>());

        let mut out = core::mem::zeroed();
        bt_hci::param::Status::from(f(params.as_hci_bytes().as_ptr() as *const _, &mut out)).to_result()?;
        Ok(unwrap!(R1::from_hci_bytes_complete(bytes_of(&out))))
    }
}

macro_rules! sdc_cmd {
    ($name:ident => $raw:ident::<$cmd:ty>()) => {
        pub fn $name(&self) -> Result<(), Error> {
            unsafe { self.raw_cmd(raw::$raw) }
        }
    };

    ($name:ident => $raw:ident::<$cmd:ty>($param:ident)) => {
        pub fn $name(&self, params: <$cmd as bt_hci::cmd::Cmd>::Params) -> Result<(), Error> {
            unsafe { self.raw_cmd_params(raw::$raw, params) }
        }
    };

    ($name:ident => $raw:ident::<$cmd:ty>() -> $ret:ident) => {
        pub fn $name(&self) -> Result<<$cmd as bt_hci::cmd::SyncCmd>::Return<'static>, Error> {
            unsafe { self.raw_cmd_return(raw::$raw) }
        }
    };

    ($name:ident => $raw:ident::<$cmd:ty>($param:ident) -> $ret:ident) => {
        pub fn $name(&self, params: <$cmd as bt_hci::cmd::Cmd>::Params) -> Result<<$cmd as bt_hci::cmd::SyncCmd>::Return<'static>, Error> {
            unsafe { self.raw_cmd_params_return(raw::$raw, params) }
        }
    };

    ($($name:ident => $raw:ident::<$cmd:ty>($($param:ident)?) $(-> $ret:ident)?),+ $(,)?) => {
        $(sdc_cmd! { $name => $raw::<$cmd>($($param)?) $(-> $ret)? })+
    };
}

mod link_control {
    use crate::raw;
    use bt_hci::cmd::link_control::*;
    use bt_hci::param::Error;

    /// Bluetooth HCI Link Control commands (§7.1)
    impl<'d> super::SoftdeviceController<'d> {
        sdc_cmd! {
            disconnect => sdc_hci_cmd_lc_disconnect::<Disconnect>(x),
            read_remote_version_information => sdc_hci_cmd_lc_read_remote_version_information::<ReadRemoteVersionInformation>(x),
        }
    }
}

mod controller_baseband {
    use crate::raw;
    use bt_hci::cmd::controller_baseband::*;
    use bt_hci::param::{ConnHandleCompletedPackets, Error};
    use bt_hci::WriteHci;

    /// Bluetooth HCI Controller & Baseband commands (§7.3)
    impl<'d> super::SoftdeviceController<'d> {
        sdc_cmd! {
            set_event_mask => sdc_hci_cmd_cb_set_event_mask::<SetEventMask>(x),
            reset => sdc_hci_cmd_cb_reset::<Reset>(),
            read_transmit_power_level => sdc_hci_cmd_cb_read_transmit_power_level::<ReadTransmitPowerLevel>(x) -> y,
            set_controller_to_host_flow_control => sdc_hci_cmd_cb_set_controller_to_host_flow_control::<SetControllerToHostFlowControl>(x),
            host_buffer_size => sdc_hci_cmd_cb_host_buffer_size::<HostBufferSize>(x),
            set_event_mask_page_2 => sdc_hci_cmd_cb_set_event_mask_page_2::<SetEventMaskPage2>(x),
            read_authenticated_payload_timeout => sdc_hci_cmd_cb_read_authenticated_payload_timeout::<ReadAuthenticatedPayloadTimeout>(x) -> y,
            write_authenticated_payload_timeout => sdc_hci_cmd_cb_write_authenticated_payload_timeout::<WriteAuthenticatedPayloadTimeout>(x) -> y,
        }

        pub fn host_number_of_completed_packets(&self, params: &[ConnHandleCompletedPackets]) -> Result<(), Error> {
            const MAX_CONN_HANDLES: usize = 63;
            const N: usize = 1 + MAX_CONN_HANDLES + core::mem::size_of::<ConnHandleCompletedPackets>();
            let mut buf = [0; N];
            unwrap!(params.write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_cb_host_number_of_completed_packets(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }
    }
}

mod info {
    use crate::raw;
    use bt_hci::cmd::info::*;
    use bt_hci::param::Error;

    /// Bluetooth HCI Informational parameters (§7.4)
    impl<'d> super::SoftdeviceController<'d> {
        sdc_cmd! {
            read_local_version_information => sdc_hci_cmd_ip_read_local_version_information::<ReadLocalVersionInformation>() -> y,
            read_local_supported_commands => sdc_hci_cmd_ip_read_local_supported_commands::<ReadLocalSupportedCmds>() -> y,
            read_local_supported_features => sdc_hci_cmd_ip_read_local_supported_features::<ReadLocalSupportedFeatures>() -> y,
            read_bd_addr => sdc_hci_cmd_ip_read_bd_addr::<ReadBdAddr>() -> y,
        }
    }
}

mod status {
    use crate::raw;
    use bt_hci::cmd::status::*;
    use bt_hci::param::Error;

    /// Bluetooth HCI Status parameters (§7.5)
    impl<'d> super::SoftdeviceController<'d> {
        sdc_cmd!(read_rssi => sdc_hci_cmd_sp_read_rssi::<ReadRssi>(x) -> y);
    }
}

mod le {
    use crate::raw;
    use bt_hci::cmd::le::*;
    use bt_hci::param::{AdvSet, ConnHandle, Error, InitiatingPhy, ScanningPhy};
    use bt_hci::{FromHciBytes, WriteHci};

    const MAX_PHY_COUNT: usize = 3;
    const MAX_ADV_SET: usize = 63;
    const MAX_ANTENNA_IDS: usize = 75;

    /// Bluetooth HCI LE Controller commands (§7.8)
    impl<'d> super::SoftdeviceController<'d> {
        sdc_cmd! {
            le_set_event_mask => sdc_hci_cmd_le_set_event_mask::<LeSetEventMask>(x),
            le_read_buffer_size => sdc_hci_cmd_le_read_buffer_size::<LeReadBufferSize>() -> y,
            le_read_local_supported_features => sdc_hci_cmd_le_read_local_supported_features::<LeReadLocalSupportedFeatures>() -> y,
            le_set_random_address => sdc_hci_cmd_le_set_random_address::<LeSetRandomAddr>(x),
            le_set_adv_params => sdc_hci_cmd_le_set_adv_params::<LeSetAdvParams>(x),
            le_read_adv_physical_channel_tx_power => sdc_hci_cmd_le_read_adv_physical_channel_tx_power::<LeReadAdvPhysicalChannelTxPower>() -> y,
            le_set_adv_data => sdc_hci_cmd_le_set_adv_data::<LeSetAdvData>(x),
            le_set_scan_response_data => sdc_hci_cmd_le_set_scan_response_data::<LeSetScanResponseData>(x),
            le_set_adv_enable => sdc_hci_cmd_le_set_adv_enable::<LeSetAdvEnable>(x),
            le_set_scan_params => sdc_hci_cmd_le_set_scan_params::<LeSetScanParams>(x),
            le_set_scan_enable => sdc_hci_cmd_le_set_scan_enable::<LeSetScanEnable>(x),
            le_create_conn => sdc_hci_cmd_le_create_conn::<LeCreateConn>(x),
            le_create_conn_cancel => sdc_hci_cmd_le_create_conn_cancel::<LeCreateConnCancel>(),
            le_read_filter_accept_list_size => sdc_hci_cmd_le_read_filter_accept_list_size::<LeReadFilterAcceptListSize>() -> y,
            le_clear_filter_accept_list => sdc_hci_cmd_le_clear_filter_accept_list::<LeClearFilterAcceptList>(),
            le_add_device_to_filter_accept_list => sdc_hci_cmd_le_add_device_to_filter_accept_list::<LeAddDeviceToFilterAcceptList>(x),
            le_remove_device_from_filter_accept_list => sdc_hci_cmd_le_remove_device_from_filter_accept_list::<LeRemoveDeviceFromFilterAcceptList>(x),
            le_conn_update => sdc_hci_cmd_le_conn_update::<LeConnUpdate>(x),
            le_set_host_channel_classification => sdc_hci_cmd_le_set_host_channel_classification::<LeSetHostChannelClassification>(x),
            le_read_channel_map => sdc_hci_cmd_le_read_channel_map::<LeReadChannelMap>(x) -> y,
            le_read_remote_features => sdc_hci_cmd_le_read_remote_features::<LeReadRemoteFeatures>(x),
            le_encrypt => sdc_hci_cmd_le_encrypt::<LeEncrypt>(x) -> y,
            le_rand => sdc_hci_cmd_le_rand::<LeRand>() -> y,
            le_enable_encryption => sdc_hci_cmd_le_enable_encryption::<LeEnableEncryption>(x),
            le_long_term_key_request_reply => sdc_hci_cmd_le_long_term_key_request_reply::<LeLongTermKeyRequestReply>(x) -> y,
            le_long_term_key_request_negative_reply => sdc_hci_cmd_le_long_term_key_request_negative_reply::<LeLongTermKeyRequestNegativeReply>(x) -> y,
            le_read_supported_states => sdc_hci_cmd_le_read_supported_states::<LeReadSupportedStates>() -> y,
            le_test_end => sdc_hci_cmd_le_test_end::<LeTestEnd>() -> y,
            le_set_data_length => sdc_hci_cmd_le_set_data_length::<LeSetDataLength>(x) -> y,
            le_read_suggested_default_data_length => sdc_hci_cmd_le_read_suggested_default_data_length::<LeReadSuggestedDefaultDataLength>() -> y,
            le_write_suggested_default_data_length => sdc_hci_cmd_le_write_suggested_default_data_length::<LeWriteSuggestedDefaultDataLength>(x),
            le_add_device_to_resolving_list => sdc_hci_cmd_le_add_device_to_resolving_list::<LeAddDeviceToResolvingList>(x),
            le_remove_device_from_resolving_list => sdc_hci_cmd_le_remove_device_from_resolving_list::<LeRemoveDeviceFromResolvingList>(x),
            le_clear_resolving_list => sdc_hci_cmd_le_clear_resolving_list::<LeClearResolvingList>(),
            le_read_resolving_list_size => sdc_hci_cmd_le_read_resolving_list_size::<LeReadResolvingListSize>() -> y,
            le_set_address_resolution_enable => sdc_hci_cmd_le_set_address_resolution_enable::<LeSetAddrResolutionEnable>(x),
            le_set_resolvable_private_address_timeout => sdc_hci_cmd_le_set_resolvable_private_address_timeout::<LeSetResolvablePrivateAddrTimeout>(x),
            le_read_max_data_length => sdc_hci_cmd_le_read_max_data_length::<LeReadMaxDataLength>() -> y,
            le_read_phy => sdc_hci_cmd_le_read_phy::<LeReadPhy>(x) -> y,
            le_set_default_phy => sdc_hci_cmd_le_set_default_phy::<LeSetDefaultPhy>(x),
            le_set_phy => sdc_hci_cmd_le_set_phy::<LeSetPhy>(x),
            le_set_adv_set_random_address => sdc_hci_cmd_le_set_adv_set_random_address::<LeSetAdvSetRandomAddr>(x),
            le_set_ext_adv_params => sdc_hci_cmd_le_set_ext_adv_params::<LeSetExtAdvParams>(x) -> y,
            le_read_max_adv_data_length => sdc_hci_cmd_le_read_max_adv_data_length::<LeReadMaxAdvDataLength>() -> y,
            le_read_number_of_supported_adv_sets => sdc_hci_cmd_le_read_number_of_supported_adv_sets::<LeReadNumberOfSupportedAdvSets>() -> y,
            le_remove_adv_set => sdc_hci_cmd_le_remove_adv_set::<LeRemoveAdvSet>(x),
            le_clear_adv_sets => sdc_hci_cmd_le_clear_adv_sets::<LeClearAdvSets>(),
            le_set_periodic_adv_params => sdc_hci_cmd_le_set_periodic_adv_params::<LeSetPeriodicAdvParams>(x),
            le_set_periodic_adv_enable => sdc_hci_cmd_le_set_periodic_adv_enable::<LeSetPeriodicAdvEnable>(x),
            le_set_ext_scan_enable => sdc_hci_cmd_le_set_ext_scan_enable::<LeSetExtScanEnable>(x),
            le_periodic_adv_create_sync => sdc_hci_cmd_le_periodic_adv_create_sync::<LePeriodicAdvCreateSync>(x),
            le_periodic_adv_create_sync_cancel => sdc_hci_cmd_le_periodic_adv_create_sync_cancel::<LePeriodicAdvCreateSyncCancel>(),
            le_periodic_adv_terminate_sync => sdc_hci_cmd_le_periodic_adv_terminate_sync::<LePeriodicAdvTerminateSync>(x),
            le_add_device_to_periodic_adv_list => sdc_hci_cmd_le_add_device_to_periodic_adv_list::<LeAddDeviceToPeriodicAdvList>(x),
            le_remove_device_from_periodic_adv_list => sdc_hci_cmd_le_remove_device_from_periodic_adv_list::<LeRemoveDeviceFromPeriodicAdvList>(x),
            le_clear_periodic_adv_list => sdc_hci_cmd_le_clear_periodic_adv_list::<LeClearPeriodicAdvList>(),
            le_read_periodic_adv_list_size => sdc_hci_cmd_le_read_periodic_adv_list_size::<LeReadPeriodicAdvListSize>() -> y,
            le_read_transmit_power => sdc_hci_cmd_le_read_transmit_power::<LeReadTransmitPower>() -> y,
            le_read_rf_path_compensation => sdc_hci_cmd_le_read_rf_path_compensation::<LeReadRfPathCompensation>() -> y,
            le_write_rf_path_compensation => sdc_hci_cmd_le_write_rf_path_compensation::<LeWriteRfPathCompensation>(x),
            le_set_privacy_mode => sdc_hci_cmd_le_set_privacy_mode::<LeSetPrivacyMode>(x),
            le_set_connless_cte_transmit_enable => sdc_hci_cmd_le_set_connless_cte_transmit_enable::<LeSetConnectionlessCteTransmitEnable>(x),
            le_conn_cte_response_enable => sdc_hci_cmd_le_conn_cte_response_enable::<LeConnCteResponseEnable>(x) -> y,
            le_read_antenna_information => sdc_hci_cmd_le_read_antenna_information::<LeReadAntennaInformation>() -> y,
            le_set_periodic_adv_receive_enable => sdc_hci_cmd_le_set_periodic_adv_receive_enable::<LeSetPeriodicAdvReceiveEnable>(x),
            le_periodic_adv_sync_transfer => sdc_hci_cmd_le_periodic_adv_sync_transfer::<LePeriodicAdvSyncTransfer>(x) -> y,
            le_periodic_adv_set_info_transfer => sdc_hci_cmd_le_periodic_adv_set_info_transfer::<LePeriodicAdvSetInfoTransfer>(x) -> y,
            le_set_periodic_adv_sync_transfer_params => sdc_hci_cmd_le_set_periodic_adv_sync_transfer_params::<LeSetPeriodicAdvSyncTransferParams>(x) -> y,
            le_set_default_periodic_adv_sync_transfer_params => sdc_hci_cmd_le_set_default_periodic_adv_sync_transfer_params::<LeSetDefaultPeriodicAdvSyncTransferParams>(x),
            le_request_peer_sca => sdc_hci_cmd_le_request_peer_sca::<LeRequestPeerSca>(x),
            le_enhanced_read_transmit_power_level => sdc_hci_cmd_le_enhanced_read_transmit_power_level::<LeEnhancedReadTransmitPowerLevel>(x) -> y,
            le_read_remote_transmit_power_level => sdc_hci_cmd_le_read_remote_transmit_power_level::<LeReadRemoteTransmitPowerLevel>(x),
            le_set_path_loss_reporting_params => sdc_hci_cmd_le_set_path_loss_reporting_params::<LeSetPathLossReportingParams>(x) -> y,
            le_set_path_loss_reporting_enable => sdc_hci_cmd_le_set_path_loss_reporting_enable::<LeSetPathLossReportingEnable>(x) -> y,
            le_set_transmit_power_reporting_enable => sdc_hci_cmd_le_set_transmit_power_reporting_enable::<LeSetTransmitPowerReportingEnable>(x) -> y,
            le_set_data_related_address_changes => sdc_hci_cmd_le_set_data_related_address_changes::<LeSetDataRelatedAddrChanges>(x),
        }

        pub fn le_set_ext_adv_data(&self, params: LeSetExtAdvDataParams) -> Result<(), Error> {
            const N: usize = 4 + 251;
            let mut buf = [0; N];
            unwrap!(params.write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_adv_data(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }

        pub fn le_set_ext_scan_response_data(&self, params: LeSetExtScanResponseDataParams) -> Result<(), Error> {
            const N: usize = 4 + 251;
            let mut buf = [0; N];
            unwrap!(params.write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_scan_response_data(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }

        pub fn le_set_ext_adv_enable(&self, params: LeSetExtAdvEnableParams) -> Result<(), Error> {
            const N: usize = 2 + MAX_ADV_SET * core::mem::size_of::<AdvSet>();
            let mut buf = [0; N];
            unwrap!(params.write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_adv_enable(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }

        pub fn le_set_periodic_adv_data(&self, params: LeSetPeriodicAdvDataParams) -> Result<(), Error> {
            const N: usize = 3 + 252;
            let mut buf = [0; N];
            unwrap!(params.write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_periodic_adv_data(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }

        pub fn le_set_ext_scan_params(&self, params: LeSetExtScanParamsParams) -> Result<(), Error> {
            const N: usize = 3 + MAX_PHY_COUNT * core::mem::size_of::<ScanningPhy>();
            let mut buf = [0; N];
            unwrap!(params.write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_scan_params(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }

        pub fn le_ext_create_conn(&self, params: LeExtCreateConnParams) -> Result<(), Error> {
            const N: usize = 10 + MAX_PHY_COUNT * core::mem::size_of::<InitiatingPhy>();
            let mut buf = [0; N];
            unwrap!(params.write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_ext_create_conn(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }

        pub fn le_set_connless_cte_transmit_params(
            &self,
            params: LeSetConnectionlessCteTransmitParamsParams,
        ) -> Result<(), Error> {
            const N: usize = 5 + MAX_ANTENNA_IDS;
            let mut buf = [0; N];
            unwrap!(params.write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_connless_cte_transmit_params(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }

        pub fn le_set_conn_cte_transmit_params(
            &self,
            params: LeSetConnCteTransmitParamsParams,
        ) -> Result<ConnHandle, Error> {
            const N: usize = 5 + MAX_ANTENNA_IDS;
            let mut buf = [0; N];
            unwrap!(params.write_hci(buf.as_mut_slice()));

            let mut out = unsafe { core::mem::zeroed() };
            let ret = unsafe { raw::sdc_hci_cmd_le_set_conn_cte_transmit_params(buf.as_ptr() as *const _, &mut out) };

            bt_hci::param::Status::from(ret).to_result()?;
            Ok(unwrap!(ConnHandle::from_hci_bytes_complete(unsafe {
                super::bytes_of(&out)
            })))
        }
    }
}

pub mod vendor {
    use crate::raw;
    use bt_hci::param::{BdAddr, ConnHandle, Duration, Error};
    use bt_hci::{cmd, param, FromHciBytes};

    param!(
        #[repr(C, packed)]
        struct ZephyrStaticAddr {
            addr: BdAddr,
            identity_root: [u8; 16],
        }
    );

    #[repr(transparent)]
    #[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct ZephyrStaticAddrs<'a>(&'a [ZephyrStaticAddr]);

    impl<'a> core::ops::Deref for ZephyrStaticAddrs<'a> {
        type Target = [ZephyrStaticAddr];

        fn deref(&self) -> &Self::Target {
            self.0
        }
    }

    impl<'de> bt_hci::FromHciBytes<'de> for ZephyrStaticAddrs<'de> {
        fn from_hci_bytes(data: &'de [u8]) -> Result<(Self, &'de [u8]), bt_hci::FromHciBytesError> {
            let (len, rest) = u8::from_hci_bytes(data)?;
            let len = usize::from(len);
            let bytes = len * core::mem::size_of::<ZephyrStaticAddr>();
            if rest.len() < bytes {
                return Err(bt_hci::FromHciBytesError::InvalidSize);
            }

            let (data, rest) = rest.split_at(bytes);
            let addrs = unsafe { core::slice::from_raw_parts(data.as_ptr() as *const ZephyrStaticAddr, len) };
            Ok((ZephyrStaticAddrs(addrs), rest))
        }
    }

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
        }
    }

    impl bt_hci::cmd::SyncCmd for ZephyrReadStaticAddrs {
        type Return<'ret> = ZephyrStaticAddrs<'ret>;
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
        NordicReadSupportedCommands(VENDOR_SPECIFIC, 0x0100) {
            Params = ();
            Return = [u8; 64];
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
            supervision_timeout: Duration<10_1000>,
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
            Params = bool;
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
        NordicCoexScanModeConfig(VENDOR_SPECIFIC, 0x0107) {
            Params = u8;
            Return = ();
        }
    }

    cmd! {
        NordicCoexPriorityConfig(VENDOR_SPECIFIC, 0x0108) {
            Params = NordicCoexPriorityConfigParams;
            Return = ();
        }
    }

    param! {
        struct NordicCoexPriorityConfigParams {
            role: u8,
            priority: u8,
            escalation_threshold: u8,
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
        NordicSetAutoPowerControlRequestParam(VENDOR_SPECIFIC, 0x010b) {
            Params = NordicSetAutoPowerControlRequestParamParams;
            Return = ();
        }
    }

    param! {
        struct NordicSetAutoPowerControlRequestParamParams {
            enable: bool,
            beta: u16,
            lower_limit: i8,
            upper_limit: i8,
            lower_target_rssi: i8,
            upper_target_rssi: i8,
            wait_period: u8,
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

    /// Bluetooth HCI vendor specific commands
    impl<'d> super::SoftdeviceController<'d> {
        sdc_cmd! {
            zephyr_read_version_info => sdc_hci_cmd_vs_zephyr_read_version_info::<ZephyrReadVersionInfo>() -> y,
            zephyr_read_supported_commands => sdc_hci_cmd_vs_zephyr_read_supported_commands::<ZephyrReadSupportedCommands>() -> y,
            zephyr_write_bd_addr => sdc_hci_cmd_vs_zephyr_write_bd_addr::<ZephyrWriteBdAddr>(x),
            zephyr_read_key_hierarchy_roots => sdc_hci_cmd_vs_zephyr_read_key_hierarchy_roots::<ZephyrReadKeyHierarchyRoots>() -> y,
            zephyr_read_chip_temp => sdc_hci_cmd_vs_zephyr_read_chip_temp::<ZephyrReadChipTemp>() -> y,
            zephyr_write_tx_power => sdc_hci_cmd_vs_zephyr_write_tx_power::<ZephyrWriteTxPower>(x) -> y,
            zephyr_read_tx_power => sdc_hci_cmd_vs_zephyr_read_tx_power::<ZephyrReadTxPower>(x) -> y,
            read_supported_vs_commands => sdc_hci_cmd_vs_read_supported_vs_commands::<NordicReadSupportedCommands>() -> y,
            llpm_mode_set => sdc_hci_cmd_vs_llpm_mode_set::<NordicLlpmModeSet>(x),
            conn_update => sdc_hci_cmd_vs_conn_update::<NordicConnUpdate>(x),
            conn_event_extend => sdc_hci_cmd_vs_conn_event_extend::<NordicConnEventExtend>(x),
            qos_conn_event_report_enable => sdc_hci_cmd_vs_qos_conn_event_report_enable::<NordicQosConnEventReportEnable>(x),
            event_length_set => sdc_hci_cmd_vs_event_length_set::<NordicEventLengthSet>(x),
            periodic_adv_event_length_set => sdc_hci_cmd_vs_periodic_adv_event_length_set::<NordicPeriodicAdvEventLengthSet>(x),
            coex_scan_mode_config => sdc_hci_cmd_vs_coex_scan_mode_config::<NordicCoexScanModeConfig>(x),
            coex_priority_config => sdc_hci_cmd_vs_coex_priority_config::<NordicCoexPriorityConfig>(x),
            peripheral_latency_mode_set => sdc_hci_cmd_vs_peripheral_latency_mode_set::<NordicPeripheralLatencyModeSet>(x),
            write_remote_tx_power => sdc_hci_cmd_vs_write_remote_tx_power::<NordicWriteRemoteTxPower>(x),
            set_auto_power_control_request_param => sdc_hci_cmd_vs_set_auto_power_control_request_param::<NordicSetAutoPowerControlRequestParam>(x),
            set_adv_randomness => sdc_hci_cmd_vs_set_adv_randomness::<NordicSetAdvRandomness>(x),
        }

        /// # Safety
        ///
        /// `buf` must be large enough for the list of static addresses returned by the controller.
        pub unsafe fn zephyr_read_static_addresses<'a>(
            &self,
            buf: &'a mut [u8],
        ) -> Result<<ZephyrReadStaticAddrs as bt_hci::cmd::SyncCmd>::Return<'a>, Error> {
            bt_hci::param::Status::from(raw::sdc_hci_cmd_vs_zephyr_read_static_addresses(buf.as_ptr() as *mut _))
                .to_result()?;
            Ok(unwrap!(ZephyrStaticAddrs::from_hci_bytes(buf)).0)
        }
    }
}
