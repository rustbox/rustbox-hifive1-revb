#![no_std]
#![no_main]

extern crate panic_halt;

use hifive1::hal::device::DevicePeripherals;
use riscv_rt::entry;
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::hal::core::CorePeripherals;
use hifive1::hal::core::plic::Priority;
use hifive1::hal::e310x::Interrupt;
use hifive1::{sprint, sprintln, pin};
use core::sync::atomic::{AtomicUsize, Ordering};
use riscv::register::{mstatus, mie};

static COUNTER: AtomicUsize = AtomicUsize::new(0);
static COUNTER2: AtomicUsize = AtomicUsize::new(0);

#[no_mangle]
pub unsafe extern "C" fn MachineTimer() {
    COUNTER.fetch_add(1, Ordering::SeqCst);

    let mut clint = CorePeripherals::steal().clint;
    clint.mtimecmp.set_mtimecmp(clint.mtime.mtime() + 65536 / 2);
}

#[no_mangle]
pub unsafe extern "C" fn MachineExternal() {
    let mut plic = CorePeripherals::steal().plic;
    let intr = plic.claim.claim().unwrap();
    // sprintln!("Ext");
    match intr {
        Interrupt::RTC => {
            COUNTER2.fetch_add(1, Ordering::SeqCst);

            let rtc = &*hifive1::hal::e310x::RTC::ptr();
            rtc.rtccmp.modify(|r, w| w.bits(r.bits() + 65536));
            core::sync::atomic::compiler_fence(Ordering::SeqCst);
        }
        Interrupt::UART0 => {
            let w = unsafe {
                DeviceResources::steal().peripherals.UART0.rxdata.read().data().bits()
            };
            match w {
                8 | 127 => {
                    sprint!("{}{}{}", 8 as char, ' ', 8 as char);
                },
                10 | 13 => {
                    sprintln!();
                },
                _ => {
                    sprint!("{}", w as char);
                }
            }
        }
        _ => {
            sprintln!("Unknown interrupt #{:?}!", intr);
            panic!("Unknown interrupt");
        }
    }
    plic.claim.complete(intr);
}

#[entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let pins = dr.pins;

    // Configure clocks
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 64.mhz().into());

    let serial_t = pin!(pins, uart0_tx).into_iof0();
    let serial_r = pin!(pins, uart0_rx).into_iof0();
    let serial = hifive1::hal::serial::Serial::new(
        p.UART0, 
        (serial_t, serial_r),
        115_200.bps(),
        clocks
    );
    let serial = serial.listen();
    let (uart, (tx, rx)) = serial.free();

    // Configure UART for stdout
    hifive1::stdout::configure(uart, tx, rx, 115_200.bps(), clocks);

    unsafe {
        let uart = DeviceResources::steal().peripherals.UART0;
        // Turn on interrupts for reading from uart
        uart.ie.write(|w| w.txwm().bit(false).rxwm().bit(true));
        // Enable receive, and set interrupt watermark to 1, so we should interrupt on any enqued data
        uart.rxctrl.write(|w| w.enable().bit(true).counter().bits(0));
    }


    sprintln!("\nhello world!");

    // Disable watchdog
    let wdg = p.WDOG;
    wdg.wdogcfg.modify(|_, w| w.enalways().clear_bit());

    let mut rtc = p.RTC.constrain();
    rtc.enable();
    rtc.set_scale(0);
    rtc.set_rtc(0);
    rtc.set_rtccmp(10000);
    rtc.enable();

    let mut clint = dr.core_peripherals.clint;
    clint.mtimecmp.set_mtimecmp(clint.mtime.mtime() + 10000);

    unsafe {
        mie::set_mtimer();
        mstatus::set_mie();

        let rplic = &*hifive1::hal::e310x::PLIC::ptr();
        for p in rplic.priority.iter() {
            p.write(|w| w.bits(0));
        }

        let mut plic = CorePeripherals::steal().plic;
        plic.rtc.set_priority(Priority::P7);
        plic.rtc.enable();
        plic.uart0.enable();
        plic.uart0.set_priority(Priority::P7);
        plic.threshold.set(Priority::P0);
        plic.mext.enable();
    }

    loop {
        unsafe {
            riscv::asm::wfi();
        }
        let cnt = COUNTER.load(Ordering::SeqCst);
        let cnt2 = COUNTER2.load(Ordering::SeqCst);
        // sprint!("\rCounter: {}, {}           ", cnt, cnt2);
    }
}