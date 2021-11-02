#![no_std]
#![no_main]
#![feature(asm)]

extern crate panic_halt;

use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::{pin, sprintln};
use riscv_rt::entry;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;



#[entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let pins = dr.pins;

    // Configure clocks
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    // Configure UART for stdout
    hifive1::stdout::configure(
        p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );

    sprintln!("hello world!, riscv!");

    let afloat: f32 = 32.12345;

    sprintln!("This is a float {}", afloat);

    let r = foo_test();
    sprintln!("found {:?}", r);


    loop {
        unsafe {
            riscv::asm::wfi();
        }
    }
}

#[no_mangle]
fn foo_test() -> Result<bool, bool> {
    let mut foo = AtomicBool::new(true);
    foo.compare_exchange(true, false, Ordering::Acquire, Ordering::Acquire)
}
