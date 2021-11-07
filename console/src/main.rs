#![no_std]
#![no_main]
#![feature(panic_info_message,asm,alloc_error_handler,core_intrinsics,allocator_api,ptr_metadata)]

use core::alloc::GlobalAlloc;

use alloc::string::String;
use hifive1::Led;
use hifive1::hal::e310x::CLINT;
use hifive1::hal::e310x::PLIC;
use hifive1::hal::e310x::Interrupt;
use hifive1::hal::DeviceResources;
use hifive1::hal::prelude::*;
use hifive1::hal::core::clint;
use hifive1::hal::core::plic::Priority;
use hifive1::{sprint, pin, pins, sprintln};
use riscv::register::mie;
use riscv::register::mip;
use riscv_rt::entry;
use buddyalloc::Heap;

extern crate alloc;

use alloc::alloc::Layout;
use terminal::Terminal;

mod repl;
mod input;
mod terminal;
mod fsm;
mod lock;
mod tree;
mod hash;
mod game;
mod shell;


static mut TERMINAL: Option<Terminal> = None;

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

extern "C" {
    static _heap_size: usize;
    static _sheap: usize;
}

#[derive(Debug)]
struct LockedHeap<const N: usize>(lock::SpinLock<Heap<N>>);

unsafe impl<const N: usize> GlobalAlloc for LockedHeap<N> {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let mut heap = self.0.lock();
        let ptr = heap.allocate(layout)
            .ok()
            .map_or(0 as *mut u8, |a| a);
        ptr
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        let mut heap = self.0.lock();
        heap.deallocate(ptr, layout)
    }
}

#[global_allocator]
static mut ALLOCATOR: LockedHeap<8> = unsafe {
    // LockedHeap::new_unchecked(_sheap as *mut u8, _heap_size as *mut u8)
    LockedHeap(lock::SpinLock::new(Heap::new_unchecked((0x80000900 as usize) as *mut u8, 12_288)))
};


#[alloc_error_handler]
fn alloc_error(layout: Layout) -> ! {
    panic!("Bad Allocation! layout: {:?}", layout);
}


#[no_mangle]
fn MachineTimer() {

    riscv::interrupt::free(|_| {

        update_time_compare(CLOCK_SPEED / 100);
    });
}

fn read_line() -> String {
    let mut s = String::new();
    unsafe {
        if let Some(ref mut term) = TERMINAL {
            term.read_line(&mut s); 
        }
    }
    s
}

fn get_char() -> char {
    let mut c = 0 as char;
    unsafe {
        if let Some(ref mut term) = TERMINAL {
            c = term.get_char();
        }
    }
    c
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

                    let w = dr.peripherals.UART0.rxdata.read().data().bits();
                    unsafe {
                        if let Some(ref mut term) = TERMINAL {
                            term.input(w);
                        }
                    }
                },
                Interrupt::PWM1CMP1 => {
                    let pwms1 = dr.peripherals.PWM1.pwms.read().bits();
                    let pwm1_count = dr.peripherals.PWM1.count.read().bits();
                    sprintln!("PM1CMP1 interrupted! Count is {}, Scaled is {}", pwm1_count, pwms1);
                    stop();
                },
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

fn update_time_compare(delta: u64) {
    let current = current_mtime();
    let next = current + delta;

    set_time_cmp(next);
}

pub fn current_mtime() -> u64 {
    clint::MTIME.mtime()
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

fn enable_pmw_interrupt(pwm: u32, cmp: u32) {
    let pmw_interrupt_enable_offset = 40;
    let pwm = pwm.clamp(0, 3);
    let cmp = cmp.clamp(0, 3);
    unsafe {
        // let dr = DeviceResources::steal();
        // let mut plic = dr.core_peripherals.plic;
        
        // dr.peripherals.PWM0.cfg.read().zerocmp();
        sprintln!("hello");
        let interrupt_bit = pmw_interrupt_enable_offset + 4 * pwm + cmp - 32;
        (*PLIC::ptr()).enable[1].modify(|r, w| {
            w.bits(r.bits() | (1 << interrupt_bit))
        });
        (*PLIC::ptr()).priority[(interrupt_bit + 32) as usize].write(|w| w.priority().p4());
        sprintln!("enabled");
    }
}


fn enable_risc_interrupts() {
    unsafe {
        riscv::register::mie::set_mtimer();
        riscv::register::mie::set_mext();
        // riscv::register::mie::set_uext();
        riscv::register::mstatus::set_mie();
        // riscv::register::mstatus::set_uie();
        sprintln!("Interrupts Enabled");
    }
}

pub fn block_until<F>(f: F) 
    where 
        F: Fn() -> bool {
    
    let mut condition = false;
    unsafe {
        loop {
            riscv::asm::wfi();

            riscv::interrupt::free(|_| {
                condition = f()
            });

            if condition {
                break;
            }
        }
    }
}

fn delay(ms: u32) {
    let ticks = ((ms as u64) * CLOCK_SPEED) / 1000;
    update_time_compare(ticks);
    unsafe {
        mie::set_mtimer();
    }

    loop {
        unsafe {
            riscv::asm::wfi();
        }

        if mip::read().mtimer() {
            break;
        }
    }

    unsafe {
        mie::clear_mtimer();
    }
}

static CLOCK_SPEED: u64 = 32_768;


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
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320_000.khz().into());

    // Configure UART0 for stdout
    let _ = hifive1::stdout::configure(p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );

    // Init Terminal
    sprintln!("Init Terminal");
    unsafe {
        TERMINAL.insert(Terminal::new());
    }

    let rgb_pins = pins!(pins, (led_red, led_green, led_blue));
    let mut tleds = hifive1::rgb(rgb_pins.0, rgb_pins.1, rgb_pins.2);
    tleds.1.on();

    initialize_plic_enable_and_threshold();
    configure_uart_receiver_interrupt_enable();
    let enableds = enabled_plic_interrupts();
    sprintln!("{:?}", enableds);

    sprintln!("Hello, world!");
    sprintln!("Welcome to Rustbox Console");

    update_time_compare(CLOCK_SPEED / 1000);
    enable_risc_interrupts();



    loop {
        game::game();

        unsafe {
            riscv::asm::wfi();
        }

        sprint!("Enter an array: ");
        let arr_in = read_line();
        let arr = shell::parse_array(&arr_in.trim());
        sprintln!("Array: {:?}", arr);
    }
}
