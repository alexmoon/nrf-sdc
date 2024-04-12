use crate::pac;
use crate::raw;
use crate::Error;
use crate::MultiprotocolServiceLayer;
use crate::RetVal;
use core::mem::MaybeUninit;
use core::cell::RefCell;
use core::future::poll_fn;
use core::ops::ControlFlow;
use core::slice;
use core::task::Poll;
use embassy_nrf::{
    into_ref,
    nvmc::{FLASH_SIZE, PAGE_SIZE},
    peripherals::NVMC,
    Peripheral, PeripheralRef,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::waitqueue::WakerRegistration;
use embedded_storage::nor_flash::{NorFlashError, NorFlashErrorKind, ErrorType};

/// Error type for Flash operations.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashError {
    /// Error from the MPSL api
    Mpsl(crate::Error),
    /// Operation using a location not in flash.
    OutOfBounds,
    /// Unaligned operation or using unaligned buffers.
    Unaligned,
}


/// Represents a flash that can be used with the MPSL timeslot API to
/// ensure it does not affect radio transmissions.
pub struct Flash<'d> {
    _mpsl: &'d MultiprotocolServiceLayer<'d>,
    _p: PeripheralRef<'d, NVMC>,
}

/// Global state of the timeslot flash operation
struct State {
    inner: Mutex<CriticalSectionRawMutex, RefCell<InnerState>>,
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
        words: u32,
    },
}

// Safety:
// Timeslot request and return parameters are only modified before and after timeslot starts,
// or within the callback.
unsafe impl Send for State {}
unsafe impl Sync for State {}

// Values derived from nRF SDK
const TIMESLOT_TIMEOUT_PRIORITY_NORMAL_US: u32 = 30000;
// Extra slack for non-flash activity
const TIMESLOT_SLACK_US: u32 = 1000;

// Values according to nRF52 product specification
const ERASE_PAGE_DURATION_US: u32 = 85_000;
const WRITE_WORD_DURATION_US: u32 = 41;
const ERASE_PARTIAL_PAGE_DURATION_MS: u32 = 10;
const ERASE_PARTIAL_PAGE_DURATION_US: u32 = ERASE_PARTIAL_PAGE_DURATION_MS * 1000;
const WORD_SIZE: u32 = 4;

// Values derived from Zephyr
const TIMESLOT_LENGTH_ERASE_US: u32 = ERASE_PARTIAL_PAGE_DURATION_US;
const TIMESLOT_LENGTH_WRITE_US: u32 = 7500;

static STATE: State = State::new();

impl<'d> Flash<'d> {
    /// Constructs a flash instance.
    ///
    /// # Panics
    ///
    /// Panics if called more than once.
    pub fn take(_mpsl: &'d MultiprotocolServiceLayer<'d>, _p: impl Peripheral<P = NVMC> + 'd) -> Flash<'d> {
        STATE.with_inner(|state| {
            if state.taken {
                panic!("nrf_mpsl::Flash::take() called multiple times.")
            }
            state.taken = true;
        });

        into_ref!(_p);

        Self { _mpsl, _p }
    }

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
        let session_id: u8 = poll_fn(|cx| {
            STATE.with_inner(|state| {
                if let FlashOp::None = state.operation {
                    let mut session_id: u8 = 0;
                    let ret = unsafe { raw::mpsl_timeslot_session_open(Some(timeslot_session_callback), (&mut session_id) as *mut _) };
                    match RetVal::from(ret).to_result() {
                        Ok(_) => {
                            Poll::Ready(Ok(session_id))
                        }
                        Err(Error::ENOMEM) => {
                            state.waker.register(cx.waker());
                            Poll::Pending
                        }
                        Err(e) => {
                            Poll::Ready(Err(e))
                        }
                    }
                } else {
                    Poll::Pending
                }
            })
        }).await?;

        let _drop = OnDrop::new(|| {
            STATE.with_inner(|state| {
                state.operation = FlashOp::None;
            });
            let _ = unsafe { raw::mpsl_timeslot_session_close(session_id) };
        });

        // info!("Requesting timeslot");
        STATE.with_inner(|state| {
            state.result = None;
            state.operation = op;
            state.slot_duration_us = slot_duration_us;
            state.timeslot_request.request_type = raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8;
            state.timeslot_request.params.earliest = raw::mpsl_timeslot_request_earliest_t {
                hfclk: raw::MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE as u8,
                priority: raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8,
                length_us: slot_duration_us + TIMESLOT_SLACK_US,
                timeout_us: TIMESLOT_TIMEOUT_PRIORITY_NORMAL_US,
            };
            let ret = unsafe { raw::mpsl_timeslot_request(session_id, &state.timeslot_request) };
            RetVal::from(ret).to_result()
        })?;
        // info!("Timeslot requested");

        poll_fn(|cx| {
            STATE.with_inner(|state| {
                match state.result.take() {
                    Some(result) => {
                        state.operation = FlashOp::None;
                        Poll::Ready(result)
                    }
                    None => {
                        state.waker.register(cx.waker());
                        Poll::Pending
                    }
                }
            })
        })
        .await?;

        // No need to run the drop handler now
        _drop.defuse();

        unsafe {
            // info!("MPSL closing session {}", session_id);
            let ret = raw::mpsl_timeslot_session_close(session_id);
            RetVal::from(ret).to_result()?;
            // info!("MPSL closed session {}", session_id);
        }

        Ok(())
    }

    pub async fn erase(&mut self, from: u32, to: u32) -> Result<(), FlashError> {
        if to < from || to as usize > FLASH_SIZE {
            return Err(FlashError::OutOfBounds);
        }
        if from as usize % PAGE_SIZE != 0 || to as usize % PAGE_SIZE != 0 {
            return Err(FlashError::Unaligned);
        }

        self.do_op(TIMESLOT_LENGTH_ERASE_US, FlashOp::Erase { elapsed: 0, address: from, to }).await?;
        Ok(())
    }



    pub async fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), FlashError> {
        if offset as usize + data.len() > FLASH_SIZE {
            return Err(FlashError::OutOfBounds);
        }
        if offset as usize % 4 != 0 || data.len() as usize % 4 != 0 {
            return Err(FlashError::Unaligned);
        }

        let src = data.as_ptr() as *const u32;
        let dest = offset as *mut u32;
        let words = data.len() as u32 / WORD_SIZE;
        self.do_op(TIMESLOT_LENGTH_WRITE_US, FlashOp::Write { dest, words, src }).await?;
        Ok(())

    }
}

unsafe extern "C" fn timeslot_session_callback(
    session_id: u8,
    signal: u32,
) -> *mut raw::mpsl_timeslot_signal_return_param_t {

    // Read current time spent inside a slot
    //
    // Safety: guaranteed by MPSL to provide values when called inside slot callback.
    unsafe fn get_timeslot_time_us() -> u32 {
        let p = unsafe { &*pac::TIMER0::ptr() };
        p.tasks_capture[0].write(|w| w.tasks_capture().set_bit());
        p.cc[0].read().cc().bits()
    }

    match signal {
        raw::MPSL_TIMESLOT_SIGNAL_START => {
            STATE.with_inner(|state| {
                match state.operation.perform(|| get_timeslot_time_us(), state.slot_duration_us) {
                    ControlFlow::Continue(_) => {
                        state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
                        state.timeslot_request.params.earliest.priority = raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8;
                        state.timeslot_request.params.earliest.timeout_us = TIMESLOT_TIMEOUT_PRIORITY_NORMAL_US;
                        state.return_param.params.request.p_next = &mut state.timeslot_request;
                    }
                    ControlFlow::Break(_) => {
                        state.result.replace(Ok(()));
                        state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
                        state.waker.wake();
                    }
                }
                return &mut state.return_param as *mut _;
            })
        }
        raw::MPSL_TIMESLOT_SIGNAL_SESSION_IDLE => {
            core::ptr::null_mut()
        }

        raw::MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED => {
            STATE.with_inner(|state| {
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
        _ => {
            core::ptr::null_mut()
        }
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
                            hfclk: raw::MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE as u8,
                            priority: raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8,
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

    fn regs() -> &'static pac::nvmc::RegisterBlock {
        unsafe { &*pac::NVMC::ptr() }
    }
}

impl FlashOp {
    fn perform<F: Fn() -> u32>(&mut self, get_time: F, slot_duration_us: u32) -> core::ops::ControlFlow<()> {
        match self {
            Self::Erase { elapsed, address, to} => {
                // Enable erase and erase next page
                let p = State::regs();
                // Do at least one erase to avoid getting stuck. The timeslot parameters guarantees we should be able to at least one operation.
                loop {
                    p.config.write(|w| w.wen().een());
                    p.erasepagepartialcfg.write(|w| unsafe { w.bits(ERASE_PARTIAL_PAGE_DURATION_MS) });
                    while p.ready.read().ready().is_busy() {}

                    p.erasepagepartial.write(|w| unsafe { w.bits(*address) });
                    while p.ready.read().ready().is_busy() {}
                    p.config.write(|w| w.wen().ren());

                    *elapsed += ERASE_PARTIAL_PAGE_DURATION_US;
                    if *elapsed > ERASE_PAGE_DURATION_US {
                        *address = *address + PAGE_SIZE as u32;
                        if *address >= *to {
                            return ControlFlow::Break(())
                        }
                    }
                    if get_time() + ERASE_PARTIAL_PAGE_DURATION_US >= slot_duration_us {
                        break;
                    }
                }
                ControlFlow::Continue(())

            }
            Self::Write { dest, src, words } => {
                let p = State::regs();
                let mut i = 0;
                // Do at least one erase to avoid getting stuck. The timeslot parameters guarantees we should be able to at least one operation.
                loop {
                    p.config.write(|w| w.wen().wen());
                    while p.ready.read().ready().is_busy() {}
                    unsafe {
                        let w = core::ptr::read_unaligned(src.add(i));
                        core::ptr::write_volatile(dest.add(i), w);
                    }
                    while p.ready.read().ready().is_busy() {}
                    i += 1;
                    if get_time() + WRITE_WORD_DURATION_US >= slot_duration_us || ((i as u32) >= *words) {
                        break;
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
            FlashOp::None => ControlFlow::Break(()),
        }
    }
}

impl<'d> ErrorType for Flash<'d> {
    type Error = FlashError;
}

impl<'d> embedded_storage_async::nor_flash::MultiwriteNorFlash for Flash<'d> {}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::Mpsl(_) => NorFlashErrorKind::Other,
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::Unaligned => NorFlashErrorKind::NotAligned,
        }
    }
}

impl<'d> embedded_storage::nor_flash::ReadNorFlash for Flash<'d> {
    const READ_SIZE: usize = 1;
    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        Self::read(self, offset, bytes)
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE
    }
}

impl<'d> embedded_storage_async::nor_flash::ReadNorFlash for Flash<'d> {
    const READ_SIZE: usize = 1;
    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        Self::read(self, offset, bytes)
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE
    }
}

impl<'d> embedded_storage_async::nor_flash::NorFlash for Flash<'d> {
    const WRITE_SIZE: usize = 4;
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

/// A type to delay the drop handler invocation.
#[must_use = "to delay the drop handler invocation to the end of the scope"]
pub struct OnDrop<F: FnOnce()> {
    f: MaybeUninit<F>,
}

impl<F: FnOnce()> OnDrop<F> {
    /// Create a new instance.
    pub fn new(f: F) -> Self {
        Self { f: MaybeUninit::new(f) }
    }

    /// Prevent drop handler from running.
    pub fn defuse(self) {
        core::mem::forget(self)
    }
}

impl<F: FnOnce()> Drop for OnDrop<F> {
    fn drop(&mut self) {
        unsafe { self.f.as_ptr().read()() }
    }
}
