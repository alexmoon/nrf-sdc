use core::arch::asm;
use core::sync::atomic::{compiler_fence, AtomicBool, Ordering};

use cortex_m::peripheral::NVIC;
use embassy_nrf::interrupt::Interrupt;

cfg_if::cfg_if! {
    if #[cfg(feature = "nrf54l")] {
        const CS_LEN: usize = 8;
        const RESERVED_IRQS: [u32; CS_LEN] = {
            let mut irqs = [0; CS_LEN];
            irqs[Interrupt::RADIO_0 as usize / 32] = 1  << (Interrupt::RADIO_0 as usize % 32);
            irqs[Interrupt::GRTC_3 as usize / 32] = 1  << (Interrupt::GRTC_3 as usize % 32);
            irqs[Interrupt::TIMER10 as usize / 32] = 1  << (Interrupt::TIMER10 as usize % 32);
            irqs
        };
    } else {
        const CS_LEN: usize = 2;
        const RESERVED_IRQS: [u32; 2] =
            [
                (1 << (Interrupt::RADIO as u8)) | (1 << (Interrupt::RTC0 as u8)) | (1 << (Interrupt::TIMER0 as u8))
                0,
            ];
    }
}

static mut CS_MASK: [u32; CS_LEN] = [0; CS_LEN];
static CS_FLAG: AtomicBool = AtomicBool::new(false);

#[inline]
unsafe fn raw_critical_section<R>(f: impl FnOnce() -> R) -> R {
    // TODO: assert that we're in privileged level
    // Needed because disabling irqs in non-privileged level is a noop, which would break safety.

    let primask: u32;
    asm!("mrs {}, PRIMASK", out(reg) primask);

    asm!("cpsid i");

    // Prevent compiler from reordering operations inside/outside the critical section.
    compiler_fence(Ordering::SeqCst);

    let r = f();

    compiler_fence(Ordering::SeqCst);

    if primask & 1 == 0 {
        asm!("cpsie i");
    }

    r
}

struct CriticalSection;
critical_section::set_impl!(CriticalSection);

unsafe impl critical_section::Impl for CriticalSection {
    unsafe fn acquire() -> bool {
        let nvic = &*NVIC::PTR;
        let nested_cs = CS_FLAG.load(Ordering::SeqCst);

        if !nested_cs {
            raw_critical_section(|| {
                CS_FLAG.store(true, Ordering::Relaxed);

                // Store the state of irqs.
                for i in 0..CS_LEN {
                    CS_MASK[i] = nvic.icer[i].read();
                }

                // Disable only not-reserved irqs.
                for i in 0..RESERVED_IRQS.len() {
                    nvic.icer[i].write(!RESERVED_IRQS[i]);
                }
            });
        }

        compiler_fence(Ordering::SeqCst);

        nested_cs
    }

    unsafe fn release(nested_cs: bool) {
        compiler_fence(Ordering::SeqCst);

        let nvic = &*NVIC::PTR;
        if !nested_cs {
            raw_critical_section(|| {
                CS_FLAG.store(false, Ordering::Relaxed);
                // restore only non-reserved irqs.
                for i in 0..CS_LEN {
                    nvic.iser[i].write(CS_MASK[i] & !RESERVED_IRQS[i]);
                }
            });
        }
    }
}
