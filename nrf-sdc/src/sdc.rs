use core::ffi::CStr;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicPtr, Ordering};
use core::task::{Poll, Waker};

use bt_hci::{AsHciBytes, Controller, FixedSizeValue, FromHciBytes, WriteHci};
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
    pub fn build<'d, const N: usize>(
        self,
        p: Peripherals<'d>,
        rng: &'d RngPool,
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

        RNG_POOL.store(rng as *const _ as *mut _, Ordering::Release);
        let rand_source = raw::sdc_rand_source_t {
            rand_prio_low_get: Some(rand_prio_low_get),
            rand_prio_high_get: Some(rand_prio_high_get),
            rand_poll: Some(rand_blocking),
        };
        let ret = unsafe { raw::sdc_rand_source_register(&rand_source) };
        RetVal::from(ret).to_result()?;

        let ret = unsafe { raw::sdc_enable(Some(sdc_callback), mem.0.as_mut_ptr() as *mut _) };
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
}

impl<'a> Controller for SoftdeviceController<'a> {
    type Error = Error;

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

    async fn write_iso_data(&self, _packet: &bt_hci::data::IsoPacket<'_>) -> Result<(), Self::Error> {
        unimplemented!()
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

macro_rules! sdc_cmd {
    (async $cmd:ty => $raw:ident()) => {
        impl<'d> bt_hci::ControllerCmdAsync<$cmd> for $crate::sdc::SoftdeviceController<'d> {
            async fn exec(&self, _cmd: &$cmd) -> Result<(), bt_hci::param::Error> {
                unsafe { self.raw_cmd(raw::$raw) }
            }
        }
    };

    (async $cmd:ty => $raw:ident(x)) => {
        impl<'d> bt_hci::ControllerCmdAsync<$cmd> for $crate::sdc::SoftdeviceController<'d> {
            async fn exec(&self, cmd: &$cmd) -> Result<(), bt_hci::param::Error> {
                unsafe { self.raw_cmd_params(raw::$raw, <$cmd as bt_hci::cmd::Cmd>::params(cmd)) }
            }
        }
    };

    ($cmd:ty => $raw:ident()) => {
        impl<'d> bt_hci::ControllerCmdSync<$cmd> for $crate::sdc::SoftdeviceController<'d> {
            async fn exec(&self, _cmd: &$cmd) -> Result<(), bt_hci::param::Error> {
                unsafe { self.raw_cmd(raw::$raw) }
            }
        }
    };

    ($cmd:ty => $raw:ident(x)) => {
        impl<'d> bt_hci::ControllerCmdSync<$cmd> for $crate::sdc::SoftdeviceController<'d> {
            async fn exec(&self, cmd: &$cmd) -> Result<(), bt_hci::param::Error> {
                unsafe { self.raw_cmd_params(raw::$raw, <$cmd as bt_hci::cmd::Cmd>::params(cmd)) }
            }
        }
    };

    ($cmd:ty => $raw:ident() -> y) => {
        impl<'d> bt_hci::ControllerCmdSync<$cmd> for $crate::sdc::SoftdeviceController<'d> {
            async fn exec(&self, _cmd: &$cmd) -> Result<<$cmd as bt_hci::cmd::SyncCmd>::Return, bt_hci::param::Error> {
                unsafe { self.raw_cmd_return(raw::$raw) }
            }
        }
    };

    ($cmd:ty => $raw:ident(x) -> y) => {
        impl<'d> bt_hci::ControllerCmdSync<$cmd> for $crate::sdc::SoftdeviceController<'d> {
            async fn exec(&self, cmd: &$cmd) -> Result<<$cmd as bt_hci::cmd::SyncCmd>::Return, bt_hci::param::Error> {
                unsafe { self.raw_cmd_params_return(raw::$raw, <$cmd as bt_hci::cmd::Cmd>::params(cmd)) }
            }
        }
    };
}

/// Bluetooth HCI Link Control commands (§7.1)
mod link_control {
    use crate::raw;
    use bt_hci::cmd::link_control::*;

    sdc_cmd!(Disconnect => sdc_hci_cmd_lc_disconnect(x));
    sdc_cmd!(async ReadRemoteVersionInformation => sdc_hci_cmd_lc_read_remote_version_information(x));
}

/// Bluetooth HCI Controller & Baseband commands (§7.3)
mod controller_baseband {
    use crate::raw;
    use bt_hci::cmd::{controller_baseband::*, Cmd};
    use bt_hci::param::ConnHandleCompletedPackets;
    use bt_hci::{ControllerCmdSync, WriteHci};

    sdc_cmd!(SetEventMask => sdc_hci_cmd_cb_set_event_mask(x));
    sdc_cmd!(Reset => sdc_hci_cmd_cb_reset());
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
        ) -> Result<<HostNumberOfCompletedPackets as bt_hci::cmd::SyncCmd>::Return, bt_hci::param::Error> {
            const MAX_CONN_HANDLES: usize = 63;
            const N: usize = 1 + MAX_CONN_HANDLES + core::mem::size_of::<ConnHandleCompletedPackets>();
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_cb_host_number_of_completed_packets(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }
    }
}

/// Bluetooth HCI Informational parameters (§7.4)
mod info {
    use crate::raw;
    use bt_hci::cmd::info::*;

    sdc_cmd!(ReadLocalVersionInformation => sdc_hci_cmd_ip_read_local_version_information() -> y);
    sdc_cmd!(ReadLocalSupportedCmds => sdc_hci_cmd_ip_read_local_supported_commands() -> y);
    sdc_cmd!(ReadLocalSupportedFeatures => sdc_hci_cmd_ip_read_local_supported_features() -> y);
    sdc_cmd!(ReadBdAddr => sdc_hci_cmd_ip_read_bd_addr() -> y);
}

/// Bluetooth HCI Status parameters (§7.5)
mod status {
    use crate::raw;
    use bt_hci::cmd::status::*;

    sdc_cmd!(ReadRssi => sdc_hci_cmd_sp_read_rssi(x) -> y);
}

/// Bluetooth HCI LE Controller commands (§7.8)
mod le {
    use crate::raw;
    use bt_hci::cmd::{le::*, Cmd};
    use bt_hci::param::{AdvHandle, AdvSet, ConnHandle, Error, InitiatingPhy, ScanningPhy, SyncHandle};
    use bt_hci::{FromHciBytes, WriteHci};

    const MAX_PHY_COUNT: usize = 3;
    const MAX_ADV_SET: usize = 63;
    const MAX_ANTENNA_IDS: usize = 75;
    const MAX_SUBEVENTS: usize = 128;

    sdc_cmd!(LeSetEventMask => sdc_hci_cmd_le_set_event_mask(x));
    sdc_cmd!(LeReadBufferSize => sdc_hci_cmd_le_read_buffer_size() -> y);
    sdc_cmd!(LeReadLocalSupportedFeatures => sdc_hci_cmd_le_read_local_supported_features() -> y);
    sdc_cmd!(LeSetRandomAddr => sdc_hci_cmd_le_set_random_address(x));
    sdc_cmd!(LeSetAdvParams => sdc_hci_cmd_le_set_adv_params(x));
    sdc_cmd!(LeReadAdvPhysicalChannelTxPower => sdc_hci_cmd_le_read_adv_physical_channel_tx_power() -> y);
    sdc_cmd!(LeSetAdvData => sdc_hci_cmd_le_set_adv_data(x));
    sdc_cmd!(LeSetScanResponseData => sdc_hci_cmd_le_set_scan_response_data(x));
    sdc_cmd!(LeSetAdvEnable => sdc_hci_cmd_le_set_adv_enable(x));
    sdc_cmd!(LeSetScanParams => sdc_hci_cmd_le_set_scan_params(x));
    sdc_cmd!(LeSetScanEnable => sdc_hci_cmd_le_set_scan_enable(x));
    sdc_cmd!(async LeCreateConn => sdc_hci_cmd_le_create_conn(x));
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
    sdc_cmd!(LeSetExtAdvParams => sdc_hci_cmd_le_set_ext_adv_params(x) -> y);
    sdc_cmd!(LeReadMaxAdvDataLength => sdc_hci_cmd_le_read_max_adv_data_length() -> y);
    sdc_cmd!(LeReadNumberOfSupportedAdvSets => sdc_hci_cmd_le_read_number_of_supported_adv_sets() -> y);
    sdc_cmd!(LeRemoveAdvSet => sdc_hci_cmd_le_remove_adv_set(x));
    sdc_cmd!(LeClearAdvSets => sdc_hci_cmd_le_clear_adv_sets());
    sdc_cmd!(LeSetPeriodicAdvParams => sdc_hci_cmd_le_set_periodic_adv_params(x));
    sdc_cmd!(LeSetPeriodicAdvEnable => sdc_hci_cmd_le_set_periodic_adv_enable(x));
    sdc_cmd!(LeSetExtScanEnable => sdc_hci_cmd_le_set_ext_scan_enable(x));
    sdc_cmd!(async LePeriodicAdvCreateSync => sdc_hci_cmd_le_periodic_adv_create_sync(x));
    sdc_cmd!(LePeriodicAdvCreateSyncCancel => sdc_hci_cmd_le_periodic_adv_create_sync_cancel());
    sdc_cmd!(LePeriodicAdvTerminateSync => sdc_hci_cmd_le_periodic_adv_terminate_sync(x));
    sdc_cmd!(LeAddDeviceToPeriodicAdvList => sdc_hci_cmd_le_add_device_to_periodic_adv_list(x));
    sdc_cmd!(LeRemoveDeviceFromPeriodicAdvList => sdc_hci_cmd_le_remove_device_from_periodic_adv_list(x));
    sdc_cmd!(LeClearPeriodicAdvList => sdc_hci_cmd_le_clear_periodic_adv_list());
    sdc_cmd!(LeReadPeriodicAdvListSize => sdc_hci_cmd_le_read_periodic_adv_list_size() -> y);
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
    sdc_cmd!(LeSetPeriodicAdvSyncTransferParams => sdc_hci_cmd_le_set_periodic_adv_sync_transfer_params(x) -> y);
    sdc_cmd!(LeSetDefaultPeriodicAdvSyncTransferParams => sdc_hci_cmd_le_set_default_periodic_adv_sync_transfer_params(x));
    sdc_cmd!(async LeRequestPeerSca => sdc_hci_cmd_le_request_peer_sca(x));
    sdc_cmd!(LeEnhancedReadTransmitPowerLevel => sdc_hci_cmd_le_enhanced_read_transmit_power_level(x) -> y);
    sdc_cmd!(async LeReadRemoteTransmitPowerLevel => sdc_hci_cmd_le_read_remote_transmit_power_level(x));
    sdc_cmd!(LeSetPathLossReportingParams => sdc_hci_cmd_le_set_path_loss_reporting_params(x) -> y);
    sdc_cmd!(LeSetPathLossReportingEnable => sdc_hci_cmd_le_set_path_loss_reporting_enable(x) -> y);
    sdc_cmd!(LeSetTransmitPowerReportingEnable => sdc_hci_cmd_le_set_transmit_power_reporting_enable(x) -> y);
    sdc_cmd!(LeSetDataRelatedAddrChanges => sdc_hci_cmd_le_set_data_related_address_changes(x));
    sdc_cmd!(LeSetPeriodicAdvParamsV2 => sdc_hci_cmd_le_set_periodic_adv_params_v2(x) -> y);

    impl<'a, 'd> bt_hci::ControllerCmdSync<LeSetExtAdvData<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetExtAdvData<'a>) -> Result<(), Error> {
            const N: usize = 4 + 251;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_adv_data(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }
    }

    impl<'a, 'd> bt_hci::ControllerCmdSync<LeSetExtScanResponseData<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetExtScanResponseData<'a>) -> Result<(), Error> {
            const N: usize = 4 + 251;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_scan_response_data(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }
    }

    impl<'a, 'd> bt_hci::ControllerCmdSync<LeSetExtAdvEnable<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetExtAdvEnable<'a>) -> Result<(), Error> {
            const N: usize = 2 + MAX_ADV_SET * core::mem::size_of::<AdvSet>();
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_adv_enable(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }
    }

    impl<'a, 'd> bt_hci::ControllerCmdSync<LeSetPeriodicAdvData<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetPeriodicAdvData<'a>) -> Result<(), Error> {
            const N: usize = 3 + 252;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_periodic_adv_data(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }
    }

    impl<'d> bt_hci::ControllerCmdSync<LeSetExtScanParams> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetExtScanParams) -> Result<(), Error> {
            const N: usize = 3 + MAX_PHY_COUNT * core::mem::size_of::<ScanningPhy>();
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_ext_scan_params(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }
    }

    impl<'d> bt_hci::ControllerCmdAsync<LeExtCreateConn> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeExtCreateConn) -> Result<(), Error> {
            const N: usize = 10 + MAX_PHY_COUNT * core::mem::size_of::<InitiatingPhy>();
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_ext_create_conn(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }
    }

    impl<'a, 'd> bt_hci::ControllerCmdSync<LeSetConnectionlessCteTransmitParams<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetConnectionlessCteTransmitParams<'a>) -> Result<(), Error> {
            const N: usize = 5 + MAX_ANTENNA_IDS;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));
            let ret = unsafe { raw::sdc_hci_cmd_le_set_connless_cte_transmit_params(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }
    }

    impl<'a, 'd> bt_hci::ControllerCmdSync<LeSetConnCteTransmitParams<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetConnCteTransmitParams<'a>) -> Result<ConnHandle, Error> {
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

    impl<'d> bt_hci::ControllerCmdAsync<LeExtCreateConnV2> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeExtCreateConnV2) -> Result<(), Error> {
            const N: usize = 12 + MAX_PHY_COUNT * 16;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));

            let ret = unsafe { raw::sdc_hci_cmd_le_ext_create_conn_v2(buf.as_ptr() as *const _) };
            bt_hci::param::Status::from(ret).to_result()
        }
    }

    impl<'a, 'd> bt_hci::ControllerCmdSync<LeSetPeriodicAdvSubeventData<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetPeriodicAdvSubeventData<'a>) -> Result<AdvHandle, Error> {
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

    impl<'a, 'd> bt_hci::ControllerCmdSync<LeSetPeriodicAdvResponseData<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetPeriodicAdvResponseData<'a>) -> Result<SyncHandle, Error> {
            const N: usize = raw::HCI_CMD_MAX_SIZE as usize;
            let mut buf = [0; N];
            unwrap!(cmd.params().write_hci(buf.as_mut_slice()));

            let mut out = unsafe { core::mem::zeroed() };
            let ret = unsafe { raw::sdc_hci_cmd_le_set_periodic_adv_response_data(buf.as_ptr() as *const _, &mut out) };

            bt_hci::param::Status::from(ret).to_result()?;
            Ok(unwrap!(SyncHandle::from_hci_bytes_complete(unsafe {
                super::bytes_of(&out)
            })))
        }
    }

    impl<'a, 'd> bt_hci::ControllerCmdSync<LeSetPeriodicSyncSubevent<'a>> for super::SoftdeviceController<'d> {
        async fn exec(&self, cmd: &LeSetPeriodicSyncSubevent<'a>) -> Result<SyncHandle, Error> {
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

    cmd! {
        NordicCompatModeWindowOffsetSet(VENDOR_SPECIFIC, 0x010d) {
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
    sdc_cmd!(NordicReadSupportedCommands => sdc_hci_cmd_vs_read_supported_vs_commands() -> y);
    sdc_cmd!(NordicLlpmModeSet => sdc_hci_cmd_vs_llpm_mode_set(x));
    sdc_cmd!(NordicConnUpdate => sdc_hci_cmd_vs_conn_update(x));
    sdc_cmd!(NordicConnEventExtend => sdc_hci_cmd_vs_conn_event_extend(x));
    sdc_cmd!(NordicQosConnEventReportEnable => sdc_hci_cmd_vs_qos_conn_event_report_enable(x));
    sdc_cmd!(NordicEventLengthSet => sdc_hci_cmd_vs_event_length_set(x));
    sdc_cmd!(NordicPeriodicAdvEventLengthSet => sdc_hci_cmd_vs_periodic_adv_event_length_set(x));
    sdc_cmd!(NordicCoexScanModeConfig => sdc_hci_cmd_vs_coex_scan_mode_config(x));
    sdc_cmd!(NordicCoexPriorityConfig => sdc_hci_cmd_vs_coex_priority_config(x));
    sdc_cmd!(NordicPeripheralLatencyModeSet => sdc_hci_cmd_vs_peripheral_latency_mode_set(x));
    sdc_cmd!(NordicWriteRemoteTxPower => sdc_hci_cmd_vs_write_remote_tx_power(x));
    sdc_cmd!(NordicSetAutoPowerControlRequestParam => sdc_hci_cmd_vs_set_auto_power_control_request_param(x));
    sdc_cmd!(NordicSetAdvRandomness => sdc_hci_cmd_vs_set_adv_randomness(x));
    sdc_cmd!(NordicCompatModeWindowOffsetSet => sdc_hci_cmd_vs_compat_mode_window_offset_set(x));

    impl<'d> super::SoftdeviceController<'d> {
        pub fn zephyr_read_static_addresses<'a>(&self, buf: &'a mut [u8]) -> Result<ZephyrStaticAddrs<'a>, Error> {
            assert!(buf.len() >= raw::HCI_EVENT_MAX_SIZE as usize);

            let ret = unsafe { raw::sdc_hci_cmd_vs_zephyr_read_static_addresses(buf.as_ptr() as *mut _) };
            bt_hci::param::Status::from(ret).to_result()?;
            Ok(unwrap!(ZephyrStaticAddrs::from_hci_bytes(buf)).0)
        }
    }
}
