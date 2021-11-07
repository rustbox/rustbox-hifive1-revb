
use lock_api::{GuardSend, RawMutex};
use core::cell::UnsafeCell;
use riscv;

/// False means the lock is unlocked and is available for getting
/// When acquired, the lock is locked.
/// 
pub struct RawSpinLock {
    locked: UnsafeCell<bool>
}

unsafe impl Sync for RawSpinLock {}

unsafe impl RawMutex for RawSpinLock {
    const INIT: RawSpinLock = RawSpinLock { locked: UnsafeCell::new(false) };

    type GuardMarker = GuardSend;

    fn lock(&self) {
        while !self.try_lock() {
            unsafe {
                riscv::asm::wfi();
            }
        }
    }

    fn try_lock(&self) -> bool {
        riscv::interrupt::free(|_| {
            let locked = self.locked.get();
            unsafe {
                if *locked == false {
                    *locked = true;
                    true
                } else {
                    false
                }
            }
        })
    }

    unsafe fn unlock(&self) {
        self.locked.get().write(false);
    }
}



pub type SpinLock<T> = lock_api::Mutex<RawSpinLock, T>;
pub type SpinlockGuard<'a, T> = lock_api::MutexGuard<'a, RawSpinLock, T>;
