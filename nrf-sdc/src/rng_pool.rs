use core::future::poll_fn;
use core::ptr;
use core::sync::atomic::{AtomicPtr, AtomicU32, AtomicUsize, Ordering};
use core::task::Poll;

use embassy_hal_common::{Peripheral, PeripheralRef};
use embassy_nrf::interrupt::{self, InterruptExt};
use embassy_nrf::peripherals::RNG;
use embassy_sync::waitqueue::AtomicWaker;

use crate::pac;

fn regs() -> &'static pac::rng::RegisterBlock {
    unsafe { &*pac::RNG::PTR }
}

static STATE: State = State {
    base: AtomicPtr::new(ptr::null_mut()),
    len: AtomicUsize::new(0),
    head: AtomicUsize::new(0),
    tail: AtomicUsize::new(0),
    trigger: AtomicPtr::new(ptr::null_mut()),
    waker: AtomicWaker::new(),
};

/// # Safety
///
/// - When `base` is non-null, it must point to a byte slice of at least `len` bytes
/// - `head` and `tail` must be in the range `0..len`
/// - References to `base[tail]` may exist only in `RngPool::on_interrupt`
struct State {
    base: AtomicPtr<u8>,
    len: AtomicUsize,
    head: AtomicUsize,
    tail: AtomicUsize,
    trigger: AtomicPtr<u8>,
    waker: AtomicWaker,
}

/// A wrapper around an nRF RNG peripheral.
///
/// It has a non-blocking API, and a blocking api through `rand`.
pub struct RngPool<'d> {
    irq: PeripheralRef<'d, interrupt::RNG>,
    threshold: usize,
    read: AtomicUsize,
    blocking: AtomicU32,
}

impl<'d> Drop for RngPool<'d> {
    fn drop(&mut self) {
        // Reset RNG peripheral state
        self.stop();
        self.disable_irq();
        self.bias_correction(false);

        // Disable and remove the irq handler
        self.irq.disable();
        self.irq.remove_handler();

        // The interrupt is now disabled and can't preempt us anymore, so the order doesn't matter here.
        STATE.base.store(ptr::null_mut(), Ordering::Release);
        STATE.head.store(0, Ordering::Release);
        STATE.tail.store(0, Ordering::Release);
        STATE.trigger.store(ptr::null_mut(), Ordering::Release);
    }
}

impl<'d> RngPool<'d> {
    /// Creates a new RngPool driver from the `RNG` peripheral and interrupt.
    ///
    /// The length of `pool` must be a power of two.
    ///
    /// SAFETY: `RngPool` must not have its lifetime end without running its destructor, e.g. using `mem::forget`.
    pub fn new(
        _rng: impl Peripheral<P = RNG> + 'd,
        irq: impl Peripheral<P = interrupt::RNG> + 'd,
        pool: &'d mut [u8],
        threshold: usize,
    ) -> Self {
        assert!(pool.len().is_power_of_two());

        let irq = irq.into_ref();
        irq.disable();

        let base = pool.as_mut_ptr();

        let this = Self {
            irq,
            threshold,
            read: AtomicUsize::new(0),
            blocking: AtomicU32::new(0),
        };

        this.stop();
        this.disable_irq();

        // The interrupt is not yet enabled, so the order doesn't matter here.
        STATE.base.store(base, Ordering::Release);
        STATE.len.store(pool.len(), Ordering::Release);
        STATE.head.store(0, Ordering::Release);
        STATE.tail.store(0, Ordering::Release);
        STATE.trigger.store(core::ptr::null_mut(), Ordering::Release);

        this.irq.set_handler(Self::on_interrupt);
        this.irq.unpend();
        this.irq.enable();

        this
    }

    fn on_interrupt(_: *mut ()) {
        // Clear the event.
        regs().events_valrdy.reset();

        // Mutate the slice within a critical section, so that the RngPool isn't dropped in between us loading the
        // pointer and actually dereferencing it.
        let (ptr, len, head, tail) = critical_section::with(|_| {
            let base = STATE.base.load(Ordering::Acquire);
            let len = STATE.len.load(Ordering::Acquire);
            let head = STATE.head.load(Ordering::Acquire);
            let tail = STATE.tail.load(Ordering::Acquire);

            if base.is_null() {
                (core::ptr::null_mut(), 0, 0, 0)
            } else {
                // # Safety
                //
                // - `!base.is_null()` means the `RngPool` owns the buffer
                // - `tail` < N
                // - `on_interrupt` has exclusive access to the value pointed to by `tail`
                unsafe {
                    let ptr = base.add(tail);
                    *ptr = regs().value.read().value().bits();
                    (ptr, len, head, tail)
                }
            }
        });

        if ptr.is_null() {
            regs().tasks_stop.write(|w| unsafe { w.bits(1) });
            return;
        }

        let new_tail = ring_add(tail, 1, len);
        let is_full = new_tail == head || ring_add(new_tail, 1, len) == head;

        // Only update `tail` if it is distict from `head`
        // (this keeps the value pointed to by `tail` exclusive to `on_interrupt`)
        if new_tail != head {
            STATE.tail.store(new_tail, Ordering::Release);
        }

        if is_full {
            STATE.waker.wake();
            regs().tasks_stop.write(|w| unsafe { w.bits(1) });
        } else if ptr == STATE.trigger.load(Ordering::Acquire) {
            STATE.waker.wake();
        }
    }

    fn stop(&self) {
        regs().tasks_stop.write(|w| unsafe { w.bits(1) })
    }

    fn start(&self) {
        regs().tasks_start.write(|w| unsafe { w.bits(1) })
    }

    fn enable_irq(&self) {
        regs().intenset.write(|w| w.valrdy().set());
    }

    fn disable_irq(&self) {
        regs().intenclr.write(|w| w.valrdy().clear());
    }

    /// Enable or disable the RNG's bias correction.
    ///
    /// Bias correction removes any bias towards a '1' or a '0' in the bits generated.
    /// However, this makes the generation of numbers slower.
    ///
    /// Defaults to disabled.
    pub fn bias_correction(&self, enable: bool) {
        regs().config.write(|w| w.dercen().bit(enable))
    }

    pub fn capacity(&self) -> usize {
        STATE.len.load(Ordering::Acquire)
    }

    /// Fills `dest` with random bytes from the pool, returning the number of bytes written.
    pub fn try_fill_bytes(&self, dest: &mut [u8]) -> usize {
        if dest.is_empty() {
            return 0; // Nothing to fill
        }

        let capacity = self.capacity();
        let (start, end, len, reentrant) = critical_section::with(|_| {
            let head = STATE.head.load(Ordering::Acquire);
            let tail = STATE.tail.load(Ordering::Acquire);
            let read = self.read.load(Ordering::Acquire);

            let reentrant = head != read; // if read is ahead of head then we've pre-empted a lower priority call
            let available = ring_len(read, tail, capacity);
            let len = dest.len().min(available);

            let end = ring_add(read, len, capacity);
            self.read.store(end, Ordering::Release);

            (read, end, len, reentrant)
        });

        // Safety: This gets a slice or slices to the portion of the ring buffer between `start` and `end`. This is
        // guaranteed to exclude `tail`, because `end` <= `tail` from the perspective of the ring buffer.
        unsafe {
            if start <= end {
                let ptr = STATE.base.load(Ordering::Acquire);
                let slice = core::slice::from_raw_parts(ptr.add(start), end - start);
                dest[..len].copy_from_slice(slice);
            } else {
                let ptr = STATE.base.load(Ordering::Acquire);
                let slice = core::slice::from_raw_parts(ptr.add(start), capacity - start);
                dest[..(capacity - start)].copy_from_slice(slice);
                let slice = core::slice::from_raw_parts(ptr, end);
                dest[(capacity - start)..len].copy_from_slice(slice);
            }
        }

        if !reentrant {
            // We were the only (or lowest-priority) call to `try_fill_bytes`, so its up to us to update `head`
            critical_section::with(|_| {
                // The new value of `head` is the value of `read`, which may have been pushed further by
                // higher-priority reentrant calls to `try_fill_bytes`.
                let head = self.read.load(Ordering::Acquire);
                STATE.head.store(head, Ordering::Release);

                // Check the available length of the pool. If it's less than or equal to `threshold`, start the RNG to
                // refill the pool.
                let tail = STATE.tail.load(Ordering::Acquire);
                let available = ring_len(head, tail, capacity);
                if available <= self.threshold && self.blocking.load(Ordering::Acquire) == 0 {
                    self.enable_irq();
                    self.start();
                }
            });
        }

        len
    }

    /// Asynchronously fills `dest` with random bytes.
    pub async fn fill_bytes(&self, dest: &mut [u8]) {
        let mut len = 0;
        let capacity = self.capacity();

        poll_fn(|cx| {
            STATE.waker.register(cx.waker());

            len += self.try_fill_bytes(&mut dest[len..]);
            if len == dest.len() {
                // We're done.
                STATE.trigger.store(core::ptr::null_mut(), Ordering::Release);
                Poll::Ready(())
            } else {
                let remaining = dest.len() - len;
                if remaining < capacity {
                    let trigger = ring_add(STATE.head.load(Ordering::Acquire), remaining, capacity);
                    // Safety: `trigger < N`
                    let ptr = unsafe { STATE.base.load(Ordering::Acquire).add(trigger) };
                    STATE.trigger.store(ptr, Ordering::Release);
                }

                Poll::Pending
            }
        })
        .await;
    }

    /// Fills `dest` with random bytes, blocking until `dest` is full.
    pub fn blocking_fill_bytes(&mut self, dest: &mut [u8]) {
        let n = self.try_fill_bytes(dest);

        if n < dest.len() {
            self.blocking.fetch_add(1, Ordering::AcqRel);
            self.disable_irq();
            self.start();

            for byte in dest[n..].iter_mut() {
                let regs = regs();
                while regs.events_valrdy.read().bits() == 0 {}
                regs.events_valrdy.reset();
                *byte = regs.value.read().value().bits();
            }

            if self.blocking.fetch_sub(1, Ordering::AcqRel) == 1 {
                self.enable_irq(); // Refill the pool
            }
        }
    }
}

impl<'d> rand_core::RngCore for RngPool<'d> {
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.blocking_fill_bytes(dest);
    }

    fn next_u32(&mut self) -> u32 {
        let mut bytes = [0; 4];
        self.blocking_fill_bytes(&mut bytes);
        // We don't care about the endianness, so just use the native one.
        u32::from_ne_bytes(bytes)
    }

    fn next_u64(&mut self) -> u64 {
        let mut bytes = [0; 8];
        self.blocking_fill_bytes(&mut bytes);
        u64::from_ne_bytes(bytes)
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.blocking_fill_bytes(dest);
        Ok(())
    }
}

impl<'d> rand_core::CryptoRng for RngPool<'d> {}

/// Offsets `base` by `count`, wrapping at `capacity`
///
/// Note: Only valid for capacities that are powers of two.
fn ring_add(base: usize, count: usize, capacity: usize) -> usize {
    (base + count) & (capacity - 1)
}

/// Finds the difference between `end` and `start`, wrapping around `capacity`
///
/// Note: Only valid for capacities that are powers of two.
fn ring_len(start: usize, end: usize, capacity: usize) -> usize {
    (end + capacity - start) & (capacity - 1)
}
