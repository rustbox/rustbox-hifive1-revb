#![no_std]
#![no_main]
#![feature(panic_info_message,asm)]

use hifive1::hal::core::clint;
use hifive1::hal::e310x::CLINT;
use hifive1::sprint;
use riscv::register::mstatus;
use riscv_rt::entry;
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::{pin, pins, sprintln};
use hifive1::Led;
use spin::Mutex;
use lazy_static::lazy_static;



#[no_mangle]
extern "C" fn eh_personality() {}


#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    sprint!("Aborting: ");
    if let Some(p) = info.location() {
        sprintln!(
                 "line {}, file {}: {}",
                 p.line(),
                 p.file(),
                 info.message().unwrap()
        );
    }
    else {
        sprintln!("no information available.");
    }
    stop();
}


#[no_mangle]
extern "C"
fn stop() -> ! {
    loop {
        unsafe {
            asm!("wfi");
        }
    }
}

#[no_mangle]
fn MachineTimer() {
    // sprint!(".");

    riscv::interrupt::free(|_| {
        update_time_compare(CLOCK_SPEED / 1000) 
        // 1000 Timer interupts per second
        // 
    })
}

#[no_mangle]
fn UserTimer() {
    sprint!("Ut")
}

#[no_mangle]
fn SupervisorTimer() {
    sprint!("St");
}

#[no_mangle]
fn MachineExternal() {
    sprint!(".");
}

#[no_mangle]
fn DefaultHandler() {
    sprint!("Default!")
}

// interrupt!(WATCHDOG, periodic);

fn periodic() {
    sprint!(".");
}

// interrupt!(UART0, keyboard);

fn keyboard() {
    sprint!(".");
}

fn set_priv_to_machine() {

    sprintln!("Machine Priv Mode: {:?}", mstatus::read().mpp());
    sprintln!("Supervisor Priv Mode: {:?}", mstatus::read().spp());
    unsafe {
        sprintln!("Setting to Machine/Supervisor");
        mstatus::set_mpp(mstatus::MPP::Machine);
        mstatus::set_spp(mstatus::SPP::Supervisor);
    }
    sprintln!("Machine Priv Mode: {:?}", mstatus::read().mpp());
    sprintln!("Supervisor Priv Mode: {:?}", mstatus::read().spp());
}

fn enable_risc_interrupts() {
    unsafe {
        riscv::register::mie::set_mtimer();
        riscv::register::mstatus::set_mie();
        sprintln!("Timer Interrupt Enabled");
    }
}

fn update_time_compare(delta: u64) {
    let current = current_mtime();
    // sprintln!("Current Time: {}", current);
    // sprintln!("Current Compare: {}", current_time_cmp());

    let next = current + delta;
    // sprintln!("Next Compare: {}", next);
    
    set_time_cmp(next);

}

fn current_mtime() -> u64 {
    clint::MTIME.mtime()
}

fn current_time_cmp() -> u64 {
    unsafe {
        let lo = (*CLINT::ptr()).mtimecmp.read().bits() as u64;
        let hi = (*CLINT::ptr()).mtimecmph.read().bits() as u64;
        // move `hi` to the upper bits, and then bitwise or with lo
        hi << 32 | lo
    }
}

fn set_time_cmp(value: u64) {
    unsafe {
        // Volume II: RISC-V Privileged Architectures V1.10 p.31, figure 3.15
        (*CLINT::ptr()).mtimecmp.write(|w| w.bits(0xffff_ffff));
        // Hi bits
        (*CLINT::ptr()).mtimecmph.write(|w| w.bits((value >> 32) as u32));
        // Lo bits
        (*CLINT::ptr()).mtimecmp.write(|w| w.bits(value as u32));
    }
}

// ///////////////////////////////////
// / CONSTANTS
// ///////////////////////////////////


static CLOCK_SPEED: u64 = 32_768;

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
    let mut rx = hifive1::stdout::configure(
        p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );

    let rgb_pins = pins!(pins, (led_red, led_green, led_blue));
    let mut tleds = hifive1::rgb(rgb_pins.0, rgb_pins.1, rgb_pins.2);

    tleds.2.on();

    set_priv_to_machine();
    update_time_compare((clocks.lfclk().0 / 1000) as u64);
    enable_risc_interrupts();

    sprintln!("Hello World, This is Rust Box");

    loop {
        riscv::interrupt::free(|_| {
            if let Ok(w) = rx.read() {
                match w {
                    8 | 127 => {
                        sprint!("{}{}{}", 8 as char, ' ', 8 as char);
                    },
                    10 | 13 => {
                        sprintln!();
                        tleds.0.off();
                        tleds.1.off();
                        tleds.2.on();
                    },
                    _ => {
                        sprint!("{}", w as char);
                        let c = (w % 7) + 1; // 1 to 8
                        if c as u8 & 1 == 1 {
                            tleds.2.on();
                        } else {
                            tleds.2.off();
                        }
                        if c as u8 & 2 == 2 {
                            tleds.1.on();
                        } else {
                            tleds.1.off();
                        }
                        if c as u8 & 4 == 4 {
                            tleds.0.on();
                        } else {
                            tleds.0.off();
                        }
                    }
                }
            }
        });

        unsafe {
            riscv::asm::wfi();
        }
    }
}



// ///////////////////////////////////
// / RUST MODULES
// ///////////////////////////////////


