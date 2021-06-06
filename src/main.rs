#![no_std]
#![no_main]
#![feature(panic_info_message,asm)]

extern crate panic_halt;

// use hifive1::hal::e310x::interrupt;
use hifive1::sprint;
use riscv_rt::entry;
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::{pin, sprintln};

use riscv::interrupt;



#[no_mangle]
extern "C" fn eh_personality() {}


// #[panic_handler]
// fn panic(info: &core::panic::PanicInfo) -> ! {
//     sprint!("Aborting: ");
//     if let Some(p) = info.location() {
//         sprintln!(
//                  "line {}, file {}: {}",
//                  p.line(),
//                  p.file(),
//                  info.message().unwrap()
//         );
//     }
//     else {
//         sprintln!("no information available.");
//     }
//     abort();
// }


// #[no_mangle]
// extern "C"
// fn abort() -> ! {
//     loop {
//         unsafe {
//             asm!("wfi");
//         }
//     }
// }

// ///////////////////////////////////
// / CONSTANTS
// ///////////////////////////////////

// ///////////////////////////////////
// / ENTRY POINT
// ///////////////////////////////////
// extern "C"
#[no_mangle]
#[entry]
fn kmain() -> ! {
    // Main should initialize all sub-systems and get
    // ready to start scheduling. The last thing this
    // should do is start the timer.

    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let pins = dr.pins;

    // Configure clocks
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    // Configure UART for stdout
    let mut _rx = hifive1::stdout::configure(
        p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );

    sprintln!("Hello World, This is Rust Box");


    loop {
        unsafe {
            riscv::asm::wfi();
        }


        // interrupt::free(|_| {
        //     if let Ok(w) = rx.read() {
        //         match w {
        //             8 => {
        //                 sprint!("{}{}{}", 8 as char, ' ', 8 as char);
        //             },
        //             10 | 13 => {
        //                 sprintln!();
        //             },
        //             _ => {
        //                 sprint!("{}", w as char);
        //             }
        //         }
        //     }
        // });
    }
}



// ///////////////////////////////////
// / RUST MODULES
// ///////////////////////////////////


