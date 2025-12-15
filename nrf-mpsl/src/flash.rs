//! Flash operations using the timeslot API.
use core::cell::RefCell;
use core::future::poll_fn;
use core::mem::MaybeUninit;
use core::ops::ControlFlow;
use core::slice;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use cortex_m::peripheral::NVIC;
use embassy_nrf::interrupt::Interrupt;
#[cfg(feature = "nrf52")]
use embassy_nrf::nvmc::{FLASH_SIZE, PAGE_SIZE};
#[cfg(feature = "nrf54l-s")]
use embassy_nrf::nvmc::{FLASH_SIZE, PAGE_SIZE};
#[cfg(feature = "nrf52")]
use embassy_nrf::pac::nvmc::vals::Wen;
#[cfg(feature = "nrf52")]
use embassy_nrf::peripherals::NVMC;
#[cfg(feature = "nrf54l-s")]
use embassy_nrf::peripherals::RRAMC as NVMC;
use embassy_nrf::{pac, Peri};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::waitqueue::WakerRegistration;
use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind};

use crate::{raw, MultiprotocolServiceLayer, RetVal};

// A custom RawMutex implementation that also masks the radio timer interrupt which invokes the
// timeslot callback.
struct RadioTimerRawMutex;
unsafe impl RawMutex for RadioTimerRawMutex {
    const INIT: Self = RadioTimerRawMutex;
    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        #[cfg(not(feature = "nrf54l-s"))]
        const TIMER: u32 = Interrupt::TIMER0 as u32;
        #[cfg(feature = "nrf54l-s")]
        const TIMER: u32 = Interrupt::TIMER10 as u32;
        unsafe {
            const INDEX: usize = TIMER as usize / 32;
            const MASK: u32 = 1 << (TIMER % 32);

            let nvic = &*NVIC::PTR;
            nvic.icer[INDEX].write(MASK);
            compiler_fence(Ordering::SeqCst);
            let r = f();
            compiler_fence(Ordering::SeqCst);
            nvic.iser[INDEX].write(MASK);
            r
        }
    }
}

/// Error type for flash operations.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashError {
    /// An error from the MPSL API.
    Mpsl(crate::Error),
    /// The operation tried to access an address outside of the flash memory.
    OutOfBounds,
    /// The address or buffer is not aligned to a word boundary.
    Unaligned,
}

/// Represents a flash that can be used with the MPSL timeslot API to
/// ensure it does not affect radio transmissions.
///
/// # Requirements
///
/// The MPSL must be initialized with timeslots support before creating a `Flash` instance.
/// Use `MultiprotocolServiceLayer::with_timeslots()` to create an MPSL instance with timeslot
/// support.
///
/// # Example
///
/// ```rust,no_run
/// // Initialize MPSL with timeslots
/// let mpsl = MultiprotocolServiceLayer::with_timeslots(
///     mpsl_p,
///     Irqs,
///     lfclk_cfg,
///     session_mem
/// ).unwrap();
///
/// // Create a Flash instance
/// let flash = Flash::take(&mpsl, p.NVMC);
/// ```
pub struct Flash<'d> {
    _mpsl: &'d MultiprotocolServiceLayer<'d>,
    _p: Peri<'d, NVMC>,
}

/// Global state of the timeslot flash operation
struct State {
    inner: Mutex<RadioTimerRawMutex, RefCell<InnerState>>,
}

/// Inner state.
struct InnerState {
    taken: bool,
    operation: FlashOp,
    slot_duration_us: u32,
    waker: WakerRegistration,
    result: Option<Result<(), FlashError>>,
    timeslot_request: raw::mpsl_timeslot_request_t,
    return_param: raw::mpsl_timeslot_signal_return_param_t,
}

/// A flash operation being performed.
enum FlashOp {
    None,
    Erase {
        /// Next address to erase.
        address: u32,
        /// How many partial erase cycles performed on the current address.
        elapsed: u32,
        /// Destination address, exclusive
        to: u32,
    },
    Write {
        /// Destination address
        dest: *mut u32,
        /// Source address
        src: *const u32,
        /// The number of words to write
        words: u32,
    },
}

// Safety:
// Timeslot request and return parameters are only modified before and after timeslot starts,
// or within the callback.
unsafe impl Send for InnerState {}
unsafe impl Sync for InnerState {}

const TIMESLOT_FLASH_HFCLK_CFG: u8 = raw::MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE as u8;
const TIMESLOT_FLASH_PRIORITY: u8 = raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8;

// Values derived from nRF SDK
const TIMESLOT_TIMEOUT_PRIORITY_NORMAL_US: u32 = 30000;
// Extra slack for non-flash activity
const TIMESLOT_SLACK_US: u32 = 1000;

// Values according to nRF52 product specification
#[cfg(feature = "nrf52")]
const ERASE_PAGE_DURATION_US: u32 = 85_000;
#[cfg(feature = "nrf52")]
const WRITE_WORD_DURATION_US: u32 = 41;

// See RRAM vs RAM section at https://argenox.com/blog/nordic-announces-nrf54l
// 65 us for single (unbuffered) word writes and 22 us for sequential address ordered (buffered)
// word writes. We do the former.
#[cfg(feature = "nrf54l-s")]
const WRITE_WORD_DURATION_US: u32 = 65;

#[cfg(not(feature = "nrf52832"))]
const ERASE_PARTIAL_PAGE_DURATION_MS: u32 = 10;
#[cfg(not(feature = "nrf52832"))]
const ERASE_PARTIAL_PAGE_DURATION_US: u32 = ERASE_PARTIAL_PAGE_DURATION_MS * 1000;

const WORD_SIZE: usize = 4;

#[cfg(feature = "nrf54l-s")]
const WRITE_LINE_SIZE: usize = 16;

// Values derived from Zephyr
#[cfg(not(feature = "nrf52832"))]
const TIMESLOT_LENGTH_ERASE_US: u32 = ERASE_PARTIAL_PAGE_DURATION_US;

#[cfg(feature = "nrf52832")]
const TIMESLOT_LENGTH_ERASE_US: u32 = ERASE_PAGE_DURATION_US;

const TIMESLOT_LENGTH_WRITE_US: u32 = 7500;

static STATE: State = State::new();

impl<'d> Flash<'d> {
    /// Creates a new `Flash` instance, taking ownership of the NVMC peripheral.
    ///
    /// This method should only be called once.
    ///
    /// # Panics
    ///
    /// This method will panic if it is called more than once.
    pub fn take(_mpsl: &'d MultiprotocolServiceLayer<'d>, _p: Peri<'d, NVMC>) -> Flash<'d> {
        STATE.with_inner(|state| {
            if state.taken {
                panic!("nrf_mpsl::Flash::take() called multiple times.")
            }
            state.taken = true;
        });

        Self { _mpsl, _p }
    }

    /// Reads data from flash.
    ///
    /// This operation does not require a timeslot.
    pub fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FlashError> {
        if offset as usize >= FLASH_SIZE || offset as usize + bytes.len() > FLASH_SIZE {
            return Err(FlashError::OutOfBounds);
        }

        let flash_data = unsafe { slice::from_raw_parts(offset as *const u8, bytes.len()) };
        bytes.copy_from_slice(flash_data);
        Ok(())
    }

    // Perform a flash operation using the timeslot API
    //
    // This method creates a timeslot session, reserves a timeslot to run the flash operation,
    // and waits until the operation is done. Any result associated with the operation is propagated
    // to the caller.
    //
    // If the future is cancelled, or the operation completes, the session is closed.
    async fn do_op(&mut self, slot_duration_us: u32, op: FlashOp) -> Result<(), FlashError> {
        // Wait until no flash operation is running
        poll_fn(|cx| {
            STATE.with_inner(|state| {
                state.waker.register(cx.waker());
                if let FlashOp::None = state.operation {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
        })
        .await;

        let mut session_id: u8 = 0;
        let ret =
            unsafe { raw::mpsl_timeslot_session_open(Some(timeslot_session_callback), (&mut session_id) as *mut _) };
        RetVal::from(ret).to_result()?;

        // Make sure that session is closed if the future is dropped from here on.
        let _drop = OnDrop::new(|| {
            let _ = unsafe { raw::mpsl_timeslot_session_close(session_id) };
        });

        // Prepare the operation and start the timeslot
        let request = STATE.with_inner(|state| {
            state.result = None;
            state.operation = op;
            state.slot_duration_us = slot_duration_us;
            state.timeslot_request.request_type = raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8;
            state.timeslot_request.params.earliest = raw::mpsl_timeslot_request_earliest_t {
                hfclk: TIMESLOT_FLASH_HFCLK_CFG,
                priority: TIMESLOT_FLASH_PRIORITY,
                length_us: slot_duration_us + TIMESLOT_SLACK_US,
                timeout_us: TIMESLOT_TIMEOUT_PRIORITY_NORMAL_US,
            };
            core::ptr::from_ref(&state.timeslot_request)
        });

        let ret = unsafe { raw::mpsl_timeslot_request(session_id, request) };
        RetVal::from(ret).to_result()?;

        // Wait until the operation has produced a result.
        poll_fn(|cx| {
            STATE.with_inner(|state| {
                state.waker.register(cx.waker());
                match state.result.take() {
                    Some(result) => Poll::Ready(result),
                    None => Poll::Pending,
                }
            })
        })
        .await?;

        // No need to run the drop handler now
        _drop.defuse();

        unsafe {
            let ret = raw::mpsl_timeslot_session_close(session_id);
            RetVal::from(ret).to_result()?;
        }

        Ok(())
    }

    /// Erases a region of flash.
    ///
    /// This operation is aligned to page boundaries. The `from` and `to` addresses must
    /// be page-aligned.
    pub async fn erase(&mut self, from: u32, to: u32) -> Result<(), FlashError> {
        if to < from || to as usize > FLASH_SIZE {
            return Err(FlashError::OutOfBounds);
        }
        #[cfg(not(feature = "nrf54l-s"))]
        if from as usize % PAGE_SIZE != 0 || to as usize % PAGE_SIZE != 0 {
            return Err(FlashError::Unaligned);
        }
        #[cfg(feature = "nrf54l-s")]
        if from as usize % WRITE_LINE_SIZE != 0 || to as usize % WRITE_LINE_SIZE != 0 {
            return Err(FlashError::Unaligned);
        }

        self.do_op(
            TIMESLOT_LENGTH_ERASE_US,
            FlashOp::Erase {
                elapsed: 0,
                address: from,
                to,
            },
        )
        .await?;
        Ok(())
    }

    /// Writes data to flash.
    ///
    /// This operation is aligned to word boundaries. The `offset` and `data.len()` must
    /// be word-aligned.
    pub async fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), FlashError> {
        if offset as usize + data.len() > FLASH_SIZE {
            return Err(FlashError::OutOfBounds);
        }
        #[cfg(not(feature = "nrf54l-s"))]
        if offset as usize % WORD_SIZE != 0 || data.len() % WORD_SIZE != 0 {
            return Err(FlashError::Unaligned);
        }
        #[cfg(feature = "nrf54l-s")]
        if offset as usize % WRITE_LINE_SIZE != 0 || data.len() % WRITE_LINE_SIZE != 0 {
            return Err(FlashError::Unaligned);
        }

        let src = data.as_ptr() as *const u32;
        let dest = offset as *mut u32;
        let words = data.len() as u32 / WORD_SIZE as u32;
        self.do_op(TIMESLOT_LENGTH_WRITE_US, FlashOp::Write { dest, words, src })
            .await?;
        Ok(())
    }
}

#[cfg(not(feature = "nrf54l-s"))]
type Nvmc = pac::nvmc::Nvmc;
#[cfg(feature = "nrf54l-s")]
type Nvmc = pac::rramc::Rramc;

#[inline(always)]
fn wait_ready(p: Nvmc) {
    while !p.ready().read().ready() {}
}

#[cfg(feature = "nrf54l-s")]
#[inline(always)]
fn wait_ready_write(p: Nvmc) {
    while !p.readynext().read().readynext() {}
    while !p.bufstatus().writebufempty().read().empty() {}
}

#[cfg(not(feature = "nrf54l-s"))]
#[inline(always)]
fn enable_erase(p: Nvmc) {
    p.config().write(|w| w.set_wen(Wen::EEN));
}

#[inline(always)]
fn enable_read(p: Nvmc) {
    #[cfg(not(feature = "nrf54l-s"))]
    p.config().write(|w| w.set_wen(Wen::REN));
    #[cfg(feature = "nrf54l-s")]
    p.config().write(|w| w.set_wen(false));
}

#[inline(always)]
fn enable_write(p: Nvmc) {
    #[cfg(not(feature = "nrf54l-s"))]
    p.config().write(|w| w.set_wen(Wen::WEN));
    #[cfg(feature = "nrf54l-s")]
    p.config().write(|w| w.set_wen(true));
}

unsafe extern "C" fn timeslot_session_callback(
    session_id: u8,
    signal: u32,
) -> *mut raw::mpsl_timeslot_signal_return_param_t {
    // Read current time spent inside a slot
    //
    // Safety: guaranteed by MPSL to provide values when called inside slot callback.
    unsafe fn get_timeslot_time_us() -> u32 {
        #[cfg(not(feature = "nrf54l-s"))]
        let p = pac::TIMER0;
        #[cfg(feature = "nrf54l-s")]
        let p = pac::TIMER10;
        p.tasks_capture(0).write_value(1);
        p.cc(0).read()
    }

    match signal {
        raw::MPSL_TIMESLOT_SIGNAL_START => STATE.with_inner(|state| {
            match state
                .operation
                .perform(|| get_timeslot_time_us(), state.slot_duration_us)
            {
                ControlFlow::Continue(_) => {
                    state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
                    state.timeslot_request.params.earliest.priority = raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8;
                    state.timeslot_request.params.earliest.timeout_us = TIMESLOT_TIMEOUT_PRIORITY_NORMAL_US;
                    state.return_param.params.request.p_next = &mut state.timeslot_request;
                }
                ControlFlow::Break(_) => {
                    #[cfg(not(feature = "nrf54l-s"))]
                    let p = pac::NVMC;
                    #[cfg(feature = "nrf54l-s")]
                    let p = pac::RRAMC;
                    enable_read(p);
                    wait_ready(p);
                    state.result.replace(Ok(()));
                    state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
                    state.waker.wake();
                }
            }
            &mut state.return_param as *mut _
        }),
        raw::MPSL_TIMESLOT_SIGNAL_SESSION_IDLE => core::ptr::null_mut(),

        raw::MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED => {
            STATE.with_inner(|state| {
                state.operation = FlashOp::None;
                state.waker.wake();
            });
            core::ptr::null_mut()
        }
        raw::MPSL_TIMESLOT_SIGNAL_CANCELLED | raw::MPSL_TIMESLOT_SIGNAL_BLOCKED => {
            STATE.with_inner(|state| {
                state.timeslot_request.params.earliest.priority = raw::MPSL_TIMESLOT_PRIORITY_HIGH as u8;
                state.timeslot_request.params.earliest.timeout_us = raw::MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US;
                let ret = unsafe { raw::mpsl_timeslot_request(session_id, &state.timeslot_request) };
                assert!(ret == 0);
            });
            core::ptr::null_mut()
        }
        raw::MPSL_TIMESLOT_SIGNAL_OVERSTAYED => {
            panic!("Used too much of our timeslot");
        }
        _ => core::ptr::null_mut(),
    }
}

impl State {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(InnerState {
                taken: false,
                operation: FlashOp::None,
                result: None,
                slot_duration_us: 0,
                waker: WakerRegistration::new(),
                timeslot_request: raw::mpsl_timeslot_request_t {
                    request_type: raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8,
                    params: raw::mpsl_timeslot_request_t__bindgen_ty_1 {
                        earliest: raw::mpsl_timeslot_request_earliest_t {
                            hfclk: TIMESLOT_FLASH_HFCLK_CFG,
                            priority: TIMESLOT_FLASH_PRIORITY,
                            length_us: 0,
                            timeout_us: TIMESLOT_TIMEOUT_PRIORITY_NORMAL_US,
                        },
                    },
                },
                return_param: raw::mpsl_timeslot_signal_return_param_t {
                    callback_action: 0,
                    params: raw::mpsl_timeslot_signal_return_param_t__bindgen_ty_1 {
                        request: raw::mpsl_timeslot_signal_return_param_t__bindgen_ty_1__bindgen_ty_1 {
                            p_next: core::ptr::null_mut(),
                        },
                    },
                },
            })),
        }
    }

    fn with_inner<F: FnOnce(&mut InnerState) -> R, R>(&self, f: F) -> R {
        self.inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            f(&mut inner)
        })
    }
}

impl FlashOp {
    //
    // nrf52 erase and write
    //
    #[cfg(not(any(feature = "nrf52832", feature = "nrf54l-s")))]
    fn erase<F: Fn() -> u32>(
        get_time: F,
        slot_duration_us: u32,
        elapsed: &mut u32,
        address: &mut u32,
        to: u32,
    ) -> core::ops::ControlFlow<()> {
        let p = pac::NVMC;
        loop {
            // Enable erase and erase next page
            enable_erase(p);
            p.erasepagepartialcfg().write(|w| w.0 = ERASE_PARTIAL_PAGE_DURATION_MS);
            wait_ready(p);

            p.erasepagepartial().write_value(*address);
            wait_ready(p);
            enable_read(p);

            *elapsed += ERASE_PARTIAL_PAGE_DURATION_US;
            if *elapsed > ERASE_PAGE_DURATION_US {
                *address += PAGE_SIZE as u32;
                if *address >= to {
                    return ControlFlow::Break(());
                }
            }
            if get_time() + ERASE_PARTIAL_PAGE_DURATION_US >= slot_duration_us {
                return ControlFlow::Continue(());
            }
        }
    }

    // No partial erase for this chip, just do one page at a time
    #[cfg(feature = "nrf52832")]
    fn erase<F: Fn() -> u32>(
        _get_time: F,
        _slot_duration_us: u32,
        _elapsed: &mut u32,
        address: &mut u32,
        to: u32,
    ) -> core::ops::ControlFlow<()> {
        let p = pac::NVMC;
        enable_erase(p);
        wait_ready(p);
        p.erasepage().write_value(*address);
        wait_ready(p);
        enable_read(p);
        *address += PAGE_SIZE as u32;
        if *address >= to {
            ControlFlow::Break(())
        } else {
            ControlFlow::Continue(())
        }
    }

    #[cfg(not(feature = "nrf54l-s"))]
    fn write<F: Fn() -> u32>(
        get_time: F,
        slot_duration_us: u32,
        dest: &mut *mut u32,
        src: &mut *const u32,
        words: &mut u32,
    ) -> core::ops::ControlFlow<()> {
        let p = pac::NVMC;
        let mut i = 0;
        // Do at least one write to avoid getting stuck. The timeslot parameters guarantees we
        // should be able to do at least one operation.
        if *words > 0 {
            loop {
                enable_write(p);
                wait_ready(p);
                unsafe {
                    let w = core::ptr::read_unaligned(src.add(i));
                    core::ptr::write_volatile(dest.add(i), w);
                }
                wait_ready(p);
                i += 1;
                if get_time() + WRITE_WORD_DURATION_US >= slot_duration_us || ((i as u32) >= *words) {
                    break;
                }
            }
        }

        unsafe {
            *src = src.add(i);
            *dest = dest.add(i);
            *words -= i as u32;
        }

        if *words == 0 {
            ControlFlow::Break(())
        } else {
            ControlFlow::Continue(())
        }
    }

    //
    // nrf54l erase and write
    //
    // RRAM can overwrite in place, so we don't actually need to erase before writing like
    // NorFlash. Here, we emulate erasing by overwriting with 0xFFFFFFFF.
    #[cfg(feature = "nrf54l-s")]
    fn erase<F: Fn() -> u32>(
        get_time: F,
        slot_duration_us: u32,
        _elapsed: &mut u32,
        from: &mut u32,
        to: u32,
    ) -> core::ops::ControlFlow<()> {
        let p = pac::RRAMC;

        enable_write(p);
        wait_ready(p);

        const ERASE_WORD: u32 = 0xFFFFFFFF;

        while *from < to {
            unsafe {
                let dest = *from as *mut u32;
                for i in 0..(WRITE_LINE_SIZE / WORD_SIZE) {
                    core::ptr::write_volatile(dest.add(i), ERASE_WORD);
                }
                *from += WRITE_LINE_SIZE as u32;
            }
            wait_ready_write(p);
            if get_time() + (4 * WRITE_WORD_DURATION_US) >= slot_duration_us {
                break;
            }
        }

        enable_read(p);
        wait_ready(p);

        if *from >= to {
            ControlFlow::Break(())
        } else {
            ControlFlow::Continue(())
        }
    }

    #[cfg(feature = "nrf54l-s")]
    fn write<F: Fn() -> u32>(
        get_time: F,
        slot_duration_us: u32,
        dest: &mut *mut u32,
        src: &mut *const u32,
        words: &mut u32,
    ) -> core::ops::ControlFlow<()> {
        let p = pac::RRAMC;

        if *words > 0 {
            let mut i = 0;

            enable_write(p);
            wait_ready(p);

            loop {
                unsafe {
                    let w = core::ptr::read_unaligned(src.add(i));
                    core::ptr::write_volatile(dest.add(i), w);
                }
                i += 1;
                if i % 4 == 0 {
                    wait_ready_write(p);

                    if get_time() + (4 * WRITE_WORD_DURATION_US) >= slot_duration_us || ((i as u32) >= *words) {
                        break;
                    }
                }
            }

            enable_read(p);
            wait_ready(p);

            unsafe {
                *src = src.add(i);
                *dest = dest.add(i);
                *words -= i as u32;
            }
        }

        if *words == 0 {
            ControlFlow::Break(())
        } else {
            ControlFlow::Continue(())
        }
    }

    fn perform<F: Fn() -> u32>(&mut self, get_time: F, slot_duration_us: u32) -> core::ops::ControlFlow<()> {
        match self {
            Self::Erase { elapsed, address, to } => {
                // Do at least one erase to avoid getting stuck. The timeslot parameters guarantees we should be able to at least one operation.
                if *address >= *to {
                    return ControlFlow::Break(());
                }
                Self::erase(get_time, slot_duration_us, elapsed, address, *to)
            }
            Self::Write { dest, src, words } => Self::write(get_time, slot_duration_us, dest, src, words),
            FlashOp::None => ControlFlow::Break(()),
        }
    }
}

impl ErrorType for Flash<'_> {
    type Error = FlashError;
}

impl embedded_storage_async::nor_flash::MultiwriteNorFlash for Flash<'_> {}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::Mpsl(_) => NorFlashErrorKind::Other,
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::Unaligned => NorFlashErrorKind::NotAligned,
        }
    }
}

impl embedded_storage::nor_flash::ReadNorFlash for Flash<'_> {
    const READ_SIZE: usize = 1;
    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        Self::read(self, offset, bytes)
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE
    }
}

impl embedded_storage_async::nor_flash::ReadNorFlash for Flash<'_> {
    const READ_SIZE: usize = 1;
    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        Self::read(self, offset, bytes)
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE
    }
}

impl embedded_storage_async::nor_flash::NorFlash for Flash<'_> {
    #[cfg(not(feature = "nrf54l-s"))]
    const WRITE_SIZE: usize = WORD_SIZE;
    #[cfg(feature = "nrf54l-s")]
    const WRITE_SIZE: usize = WRITE_LINE_SIZE;
    const ERASE_SIZE: usize = PAGE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        Self::erase(self, from, to).await
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        Self::write(self, offset, bytes).await
    }
}

impl From<crate::Error> for FlashError {
    fn from(e: crate::Error) -> Self {
        Self::Mpsl(e)
    }
}

/// A guard that runs a closure when it is dropped.
#[must_use = "to delay the drop handler invocation to the end of the scope"]
pub(crate) struct OnDrop<F: FnOnce()> {
    f: MaybeUninit<F>,
}

impl<F: FnOnce()> OnDrop<F> {
    /// Creates a new `OnDrop` guard.
    ///
    /// The closure `f` will be called when the guard is dropped.
    pub fn new(f: F) -> Self {
        Self { f: MaybeUninit::new(f) }
    }

    /// Prevents the closure from being called when the guard is dropped.
    pub fn defuse(self) {
        core::mem::forget(self)
    }
}

impl<F: FnOnce()> Drop for OnDrop<F> {
    fn drop(&mut self) {
        unsafe { self.f.as_ptr().read()() }
    }
}
