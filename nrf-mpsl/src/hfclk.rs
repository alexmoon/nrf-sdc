use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;

use crate::error::{Error, RetVal};
use crate::raw;

static SIGNAL: Signal<ThreadModeRawMutex, ()> = Signal::new();
static ENABLED: AtomicBool = AtomicBool::new(false);

pub struct Hfclk {
    // Prevent Send, Sync
    _private: PhantomData<*mut ()>,
}

impl Drop for Hfclk {
    fn drop(&mut self) {
        let ret = unsafe { raw::mpsl_clock_hfclk_release() };
        if let Err(err) = RetVal::from(ret).to_result() {
            warn!("Error releasing Hfclk: {:?}", err);
        }
        ENABLED.store(false, Ordering::Release);
    }
}

impl Hfclk {
    pub(crate) fn new() -> Result<Self, Error> {
        if ENABLED.swap(true, Ordering::Acquire) {
            // Only one Hfclk request is allowed at a time
            return Err(Error::EINVAL);
        }

        extern "C" fn on_hfclk_started() {
            SIGNAL.signal(());
        }

        let ret = unsafe { raw::mpsl_clock_hfclk_request(Some(on_hfclk_started)) };
        RetVal::from(ret).to_result().and(Ok(Hfclk { _private: PhantomData }))
    }

    pub async fn wait() {
        let mut is_running = 0;
        let ret = unsafe { raw::mpsl_clock_hfclk_is_running(&mut is_running) };
        if let Err(err) = RetVal::from(ret).to_result() {
            warn!("Error checking hfclk status: {:?}", err)
        } else if is_running == 0 {
            SIGNAL.wait().await
        }
    }
}
