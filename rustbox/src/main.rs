#![no_std]
#![no_main]
#![feature(panic_info_message,asm,alloc_error_handler,alloc_prelude)]

use core::sync::atomic::AtomicBool;

use alloc::string::String;
use hifive1::hal::core::clint;
use hifive1::hal::e310x::CLINT;
use hifive1::hal::e310x::PLIC;
use hifive1::hal::e310x::Interrupt;
use hifive1::hal::core::plic::Priority;
use hifive1::sprint;
use riscv::register::mcause;
use riscv::register::mcause::Exception;
use riscv::register::mstatus;
use riscv::register::mtval;
use riscv::register::pmpcfg0;
use riscv::register::satp;
use riscv_rt::entry;
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::{pin, pins, sprintln};
use hifive1::Led;
use hifive1::hal::e310x::interrupt;
use spin::Mutex;
use lazy_static::lazy_static;
use linked_list_allocator::LockedHeap;
use alloc::alloc::{GlobalAlloc, Layout};
use alloc::prelude::v1::*;

extern crate alloc;


#[alloc_error_handler]
fn alloc_error(layout: Layout) -> ! {
    panic!("Bad Allocation! layout: {:?}", layout);
}

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

extern "C" {
    static _heap_size: usize;
    static _sheap: usize;
}

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
    sprint!(".");
    riscv::interrupt::free(|_| {
        update_time_compare(CLOCK_SPEED / 1000);
        // 1000 Timer interupts per second
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
    riscv::interrupt::free(|_| {
        let mut dr = unsafe {
            DeviceResources::steal()
        };
    
        if let Some(interrupt) = dr.core_peripherals.plic.claim.claim() {
            match interrupt {
                Interrupt::UART0 => {
                    let pins = dr.pins;
                    let rgb_pins = pins!(pins, (led_red, led_green, led_blue));
                    let mut tleds = hifive1::rgb(rgb_pins.0, rgb_pins.1, rgb_pins.2);
    
                    let w = dr.peripherals.UART0.rxdata.read().data().bits();
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
                _ => {
                    sprintln!("Unhandled Interrupt Handler for {:?}", interrupt);
                    panic!("Unknown Interrupt Handler");
                }
            }
            dr.core_peripherals.plic.claim.complete(interrupt);
        } else {
            panic!("Why is there no interrupt cause??");
        }
    })
}

#[no_mangle]
fn DefaultHandler() {
    sprint!("Default!")
}

#[no_mangle]
fn ExceptionHandler(trap_frame: &riscv_rt::TrapFrame) -> ! {
    let cause = Exception::from(mcause::read().code());
    let mv = mtval::read();
    panic!("Exception Found! Cause: {:?}, Value: {:#08X}", cause, mv);
}

fn set_priv_to_machine() {

    sprintln!("Machine Priv Mode: {:?}", mstatus::read().mpp());
    unsafe {
        sprintln!("Setting to Machine/Supervisor");
        mstatus::set_mpp(mstatus::MPP::Machine);
    }
    sprintln!("Machine Priv Mode: {:?}", mstatus::read().mpp());
}

fn enable_risc_interrupts() {
    unsafe {
        riscv::register::mie::set_mtimer();
        riscv::register::mie::set_mext();
        riscv::register::mie::set_uext();
        riscv::register::mstatus::set_mie();
        riscv::register::mstatus::set_uie();
        sprintln!("Interrupts Enabled");
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

fn enabled_plic_interrupts() -> [(Interrupt, bool); 52] {
    let (enable1, enable2) = unsafe {
        let en = &(*PLIC::ptr()).enable;
        (en[0].read().bits(), en[1].read().bits())
    };
    let mut interrupts: [(Interrupt, bool); 52] = [(Interrupt::GPIO0, false); 52];
    for ext in 0..32 {
        let i: usize = 1 << ext;
        let is_enabled = i & (enable1 as usize) == i;
        match Interrupt::try_from(1 + ext as u8) {
            Ok(t) => {interrupts[ext] = (t, is_enabled)},
            Err(_) => {sprintln!("error: {}", 1 + ext)}
        }
    }
    for ext in 0..21 {
        let i: usize = 1 << ext;
        let is_enabled = i & (enable2 as usize) == i;
        match Interrupt::try_from(1 + 32 + ext as u8) {
            Ok(t) => {interrupts[ext + 32] = (t, is_enabled)},
            Err(_) => {sprintln!("error: {}", 1 + ext)}
        }
    }
    interrupts
}

fn set_time_cmp(value: u64) {
    unsafe {
        // Volume II: RISC-V Privileged Architectures V1.10 p.31, figure 3.15
        (*CLINT::ptr()).mtimecmp.write(|w| w.bits(0xffff_ffff));
        // Hi bits
        (*CLINT::ptr()).mtimecmph.write(|w| w.bits((value >> 32) as u32));
        // Loupdate_time_compare bits
        (*CLINT::ptr()).mtimecmp.write(|w| w.bits(value as u32));
    }
}

fn initialize_plic_enable_and_threshold() {
    
    unsafe {
        let dr = DeviceResources::steal();
        let mut plic = dr.core_peripherals.plic;
        plic.threshold.set(Priority::P0);
    }
    assert_eq!(0, unsafe {(*PLIC::ptr()).threshold.read().bits()});
    unsafe {
        (*PLIC::ptr()).enable[0].write(|w| w.bits(0));
        (*PLIC::ptr()).enable[1].write(|w| w.bits(0));
    }
    assert_eq!(0, unsafe {(*PLIC::ptr()).enable[0].read().bits()});
    assert_eq!(0, unsafe {(*PLIC::ptr()).enable[1].read().bits()});
}

fn configure_uart_receiver_interrupt_enable() {
    unsafe {
        let dr = DeviceResources::steal();
        let uart = dr.peripherals.UART0;
        let mut plic = dr.core_peripherals.plic;

        plic.uart0.enable();
        plic.uart0.set_priority(Priority::P4);
        uart.ie.write(|w| w.txwm().bit(false).rxwm().bit(true));
        uart.rxctrl.write(|w| w.enable().bit(true).counter().bits(0));
    }
}

/// Align (set to a multiple of some power of two)
/// This takes an order which is the exponent to 2^order
/// Therefore, all alignments must be made as a power of two.
/// This function always rounds up.
pub const fn align_val(val: usize, order: usize) -> usize {
	let o = (1usize << order) - 1;
	(val + o) & !o
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

    // Configure UART0 for stdout
    let _ = hifive1::stdout::configure(p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );

    let rgb_pins = pins!(pins, (led_red, led_green, led_blue));
    let mut tleds = hifive1::rgb(rgb_pins.0, rgb_pins.1, rgb_pins.2);

    sprintln!("We changed something!");

    unsafe {
        // pmpcfg0::read().
        // let m = satp::read().bits();

        // sprintln!("Mode is {:?}", m);
        // let start = &_sheap as *const usize;
        // let size = &_heap_size as *const usize;
        // let aligned_start = align_val(start as usize, 5);
        // let aligned_size = (size as usize + start as usize).saturating_sub(aligned_start);
        // sprintln!("Heap Start {:#08x}", start as usize);
        // sprintln!("Heap Size {:#08x}", size as usize);
        // ALLOCATOR.lock().init(start as usize, size as usize);
    }
    
    set_priv_to_machine();
    initialize_plic_enable_and_threshold();
    configure_uart_receiver_interrupt_enable();
    let enableds = enabled_plic_interrupts();
    sprintln!("{:?}", enableds);
    update_time_compare(CLOCK_SPEED / 1000);
    enable_risc_interrupts();

    tleds.2.on();

    sprintln!("Welcome to Rust Box");
    sprintln!("Hello world");
    sprintln!("Clock speed measured: {}", clocks.measure_coreclk().0);

    // let greeting = String::from("Hello World");
    
    // let greet_loc = &greeting as *const String;
    // sprintln!("greet location: {:?}", greet_loc);
    

    // sprintln!("{}", greeting);

    loop {
        unsafe {
            riscv::asm::wfi();
        }
    }
}



// ///////////////////////////////////
// / RUST MODULES
// ///////////////////////////////////


