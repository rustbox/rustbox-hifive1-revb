#![no_std]
#![no_main]
#![feature(panic_info_message,alloc_error_handler,core_intrinsics)]

use core::arch::asm;

use embedded_hal::blocking::delay;
use hifive1::Led;
use hifive1::hal::core::clint::MTIME;
use hifive1::hal::core::counters::MCYCLE;
use hifive1::hal::delay::Sleep;
use hifive1::hal::e310x::CLINT;
use hifive1::hal::e310x::GPIO0;
use hifive1::hal::e310x::PLIC;
use hifive1::hal::e310x::Interrupt;
use hifive1::hal::DeviceResources;
use hifive1::hal::e310x::PWM0;
use hifive1::hal::e310x::PWM1;
use hifive1::hal::gpio;
use hifive1::hal::prelude::*;
use hifive1::hal::core::clint;
use hifive1::hal::core::plic::Priority;
use hifive1::{sprint, pin, pins, sprintln};
use riscv::asm::wfi;
use riscv::register::mcycle;
use riscv::register::mie;
use riscv::register::mip;
use riscv_rt::entry;
use spin::{Mutex};

use crate::vga::{Brightness, Red2, Green2, Blue2, red_pins2, green_pins2, blue_pins2};

#[macro_use]
extern crate lazy_static;

static mut NEXT_PIXEL_RED: u8 = 0;
static mut NEXT_PIXEL_GREEN: u8 = 0;
static mut NEXT_PIXEL_BLUE: u8 = 0;
static mut HSYNC_STATE: bool = false;
static mut VSYNC_STATE: bool = false;

static mut CURRENT_COLUMN: u16 = 0;
#[no_mangle]
static mut CURRENT_LINE: u16 = 0;
static mut FRAME: u16 = 0;

// static mut HSYNC_LINE: vga::HSync = vga::HSync { pin: vga::hsync_pins(DeviceResources::steal().pins.pin12) };

mod vga;
mod color;


// const VGA_CONFIGURATION: vga::VgaConfiguration = vga::VgaConfiguration {
//     horizontal: vga::DirectionConfiguration {
//         visible: 640,
//         front_porch: 16,
//         sync: 96,
//         back_porch: 48,
//         hardware_scale: 4,
//         software_scale: 1
//     },
//     vertical: vga::DirectionConfiguration {
//         visible: 480,
//         front_porch: 10,
//         sync: 2,
//         back_porch: 33,
//         hardware_scale: 1,
//         software_scale: 5
//     }
// };


// pub const LOGICAL_WIDTH: u8 = (VGA_CONFIGURATION.horizontal.visible / (VGA_CONFIGURATION.horizontal.hardware_scale * VGA_CONFIGURATION.horizontal.software_scale)) as u8;
// pub const LOGICAL_LINES: u16 = VGA_CONFIGURATION.vertical.visible / (VGA_CONFIGURATION.vertical.hardware_scale * VGA_CONFIGURATION.vertical.software_scale);



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
            wfi();
        }
    }
}

static mut TIME_COUNT: u64 = 0;
static mut TIME_INTERRUPT_COUNT: u64 = 0;
static TICKS: u64 = CLOCK_SPEED / 10;

#[no_mangle]
fn MachineTimer() {
    // sprint!(".");
    // let mtime = MTIME;
    // RTC = 32,768 Hz
    // riscv::interrupt::free(|_| {
    //     unsafe {
    //         TIME_INTERRUPT_COUNT += 1;
    //         TIME_COUNT = current_mtime();
    //     }
    //     // update_time_compare(CLOCK_SPEED / 1000);
    //     // let start_cycles = mcycle::read64();
    //     // let start_time = current_mtime();
        
    //     // while start_time + ticks > current_mtime() {}
        
    //     // let stop_cycles = mcycle::read64();
    //     // let stop_time = current_mtime();
        
    //     // let delta_cycles = stop_cycles.wrapping_sub(start_cycles);
    //     // let delta_time = stop_time.wrapping_sub(start_time);
        
    //     // let cycles_per_tick = delta_cycles as f32 / delta_time as f32;
    //     // sprintln!("{} cycles / {} ticks = {}", delta_cycles, delta_time, cycles_per_tick);
    //     // sprintln!("cycles per clock: {}", cycles_per_tick);
    //     // 1000 Timer interupts per second
    //     update_time_compare(TICKS);
    //     // let previous = unsafe {
    //     //     let temp = TIME_COUNT;
    //     //     TIME_COUNT = current;
    //     //     temp
    //     // };

    //     // let end_time = current_mtime();
    //     // let delta_time = current.wrapping_sub(previous);
    //     // sprintln!("Delta Ticks: {}", delta_time);
    //     // *previous = current_mtime();
    // })
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
                },
                Interrupt::PWM1CMP1 => {
                    let pwms1 = dr.peripherals.PWM1.pwms.read().bits();
                    let pwm1_count = dr.peripherals.PWM1.count.read().bits();
                    sprintln!("PM1CMP1 interrupted! Count is {}, Scaled is {}", pwm1_count, pwms1);
                    stop();
                },
                Interrupt::GPIO18 => {
                    // This starts a new line
                    unsafe {
                        (*GPIO0::ptr()).rise_ip.write(|w| w.pin18().set_bit());

                        inc_line_count();
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
#[inline]
unsafe fn inc_line_count() {
    if CURRENT_LINE == 480 - 1 {
        CURRENT_LINE = 0;
    } else {
        CURRENT_LINE += 1;
    }
}


fn enabled_plic_interrupts() -> [(Interrupt, bool); 52] {
    sprintln!("enter enableds");
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

#[no_mangle]
fn current_mtime() -> u64 {
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

// fn enable_gpio_interrupt()

fn enable_risc_interrupts() {
    unsafe {
        // riscv::register::mie::set_mtimer();
        riscv::register::mie::set_mext();
        // riscv::register::mie::set_uext();
        riscv::register::mstatus::set_mie();
        // riscv::register::mstatus::set_uie();
        sprintln!("Interrupts Enabled");
    }
}

fn setup_gpio18_interrupt() {
    // GPIO_0 starts at interrupt 8, so gpio18 is interrupt 26
    unsafe {
        (*GPIO0::ptr()).rise_ie.write(|w| w.pin18().set_bit());
        (*PLIC::ptr()).enable[0].modify(|r, w| {
            w.bits(r.bits() | (1 << 26))
        });
        (*PLIC::ptr()).priority[26].write(|w| w.priority().p6());
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
            wfi();
        }

        if mip::read().mtimer() {
            break;
        }
    }

    unsafe {
        mie::clear_mtimer();
    }
}

fn delay_ticks(ticks: u32) {
    let ticks = ticks as u64;
    update_time_compare(ticks);
    unsafe {
        mie::set_mtimer();
    }

    loop {
        unsafe {
            wfi();
        }

        if mip::read().mtimer() {
            break;
        }
    }

    sprint!(".");

    unsafe {
        mie::clear_mtimer();
    }
}

static CLOCK_SPEED: u64 = 32_768;

fn setup_pmw(pmw: &PWM1) {
    sprintln!("Setup pmw");
    pmw.cfg.write(|w| unsafe { w
        .enalways().set_bit()
        .scale().bits(5)
        .cmp0gang().set_bit()
        .cmp1gang().set_bit()
    });
    pmw.cmp1.write(|w| unsafe { 
        w.value().bits(32768)
    });
}

extern "C" {
    static _heap_size: usize;
    static _sheap: usize;
}

const buffer_size: usize = 16372;

#[repr(transparent)]
struct LineBuffer {
    pixels: [u8; buffer_size]
}

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
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 302_100.khz().into());

    // Configure UART0 for stdout
    let _ = hifive1::stdout::configure(p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );

    // unsafe {
    //     HSYNC_LINE = Some(vga::HSync { pin: vga::hsync_pins(pins.pin10) });
    //     VSYNC_LINE = Some(vga::VSync { pin: vga::vsync_pins(pins.pin13) });
    // };

    // sprintln!("config = {:?}", &VGA_CONFIGURATION);
    // let (_beam_state, line_state) = vga::beam_state(0, 491, &VGA_CONFIGURATION);
    // sprintln!("Test Line State: {:?}", line_state);


    initialize_plic_enable_and_threshold();
    // configure_uart_receiver_interrupt_enable();
    let enableds = enabled_plic_interrupts();
    // setup_pmw(&p.PWM1);
    sprintln!("{:?}", enableds);
    
    // let rgb_pins = pins!(pins, (led_red, led_green, led_blue));
    // let mut tleds = hifive1::rgb(rgb_pins.0, rgb_pins.1, rgb_pins.2);
    sprintln!("Hello, world!");
    sprintln!("Welcome to Rustbox Terminal");
    let core_speed = clocks.measure_coreclk();
    sprintln!("Core clock speed is {}", core_speed.0);
    
    // tleds.2.on();
    update_time_compare(CLOCK_SPEED / 1000);
    enable_risc_interrupts();
    // setup_gpio18_interrupt();
    let _ = current_mtime();

    // let config = vga::VgaConfiguration {
    //     horizontal: vga::DirectionConfiguration::new(
    //         25.422045680238, 
    //         0.63555114200596, 
    //         3.8133068520357, 
    //         1.9066534260179, 
    //         25.175,
    //         1.0
    //     ),
    //     vertical: vga::DirectionConfiguration::new(
    //         15.253227408143,
    //         0.31777557100298,
    //         0.063555114200596,
    //         1.0486593843098,
    //         31.46875,
    //         1.0
    //     )
    // };

    // let mut gpio_0 = pins!(pins, (dig8)).into_output();

    // let mut red = vga::Red {
    //     pins: vga::red_pins(pins!(pins, (dig8, dig9, dig10)))
    // };

    // let mut hsync = vga::HSync {
    //     pin: vga::hsync_pins(pins!(pins, (dig18)))
    // };

    // let mut vsync = vga::VSync {
    //     pin: vga::vsync_pins(pins!(pins, (dig19)))
    // };

    
    sprintln!("set pin 3");
    pins!(pins, (dig3)).into_iof1();
    sprintln!("after set pin");

    // let x = pins!(pins, (dig4)).into_output();

    // sprintln!("setup pin 18");
    // let pixel = pins!(pins, (dig2)).into_floating_input();

    
    
    sprintln!("after setup");

    // sprintln!("set pin 9");
    // pins!(pins, (dig9)).into_iof1();
    // sprintln!("after set pin");

    // let mut i: u32 = 0;
    
    // let buffer = unsafe { &mut *(0x8000000c as *mut LineBuffer) };

    // let frame_writer = vga::LineBufferWriter {
    //     address_lines: vga::ColorAddress {
    //         pins: vga::address_pins(pins!(pins, (dig0, dig1, dig2, dig3, dig4, dig5, dig6, dig7)))
    //     },
    //     rgb: (
    //         vga::Red {pins: vga::red_pins(pins!(pins, (dig8, dig9, dig10)))},
    //         vga::Green {pins: vga::green_pins(pins!(pins, (dig15, dig16, dig17)))},
    //         vga::Blue {pins: vga::blue_pins(pins!(pins, (dig11, dig12, dig13))) }
    //     ),
    //     logical_line: 0,
    //     hardware_line: 0,
    //     frame: unsafe { &mut *(0x8000000c as *mut vga::Frame) }
    // };
    
    

    // sprintln!("line = {:?}", buffer.pixels[0]);
    // sprintln!("size is {}", buffer.pixels.len());

    // buffer.pixels[4321] = 42;

    // let end = &buffer.pixels[buffer_size - 128 .. buffer_size];
    // sprintln!("end of buffer: {:?}", end);

    // sprintln!("Whole frame: {:?}", buffer.pixels);



    let mut red = Red2 {
        pins: red_pins2(pins!(pins, (dig8, dig9)))
    };

    let mut green = Green2 {
        pins: green_pins2(pins!(pins, (dig10, dig11)))
    };

    let mut blue = Blue2 {
        pins: blue_pins2(pins!(pins, (dig12, dig13)))
    };

    red.set_level(0);
    green.set_level(0);
    blue.set_level(0);

    let mut line_buffer: [u32; 160] = [0; 160];
    line_buffer[64] = 12;

    let mut reset = pin!(pins, dig2).into_output();
    let _ = reset.set_high();
    for i in 0..32 {
        set_color2(i);
        set_color2_and_wait(i);
    }
    set_color2(0);
    let _ = reset.set_low();

    // Should draw a green bar down the middle?
    loop {
        // The idea here is that a single line should be 9600 cycles (12 cycles per pixel, 
        // we're running at 12x vga speed)
        // But software pixels are 5 hw pixels, or 60 cycles
        // So, init i (1 cycle) + 160 * (3 (lw, compute index) + 1 (set_color) + 1 (inc i) + 1 (bne 160) + N (nop) )
        // And we can add a constant set of NOP at the end of a line to try and equal 9600 cycles
        // Currently N (nops inside loop) = 53, and overhead nops = 160, 59*160 + 160 = 9600 theoretically
        let mut i = 0; // 1
        while i < 160 {
            let c = line_buffer[i]; // 3 (since lw on e310 is 2 cycles)
            set_color2(c); // 1
            unsafe {
                asm!(   // 53
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                )
            }
            i += 1; // 1; bne i, 160: 1
            unsafe {
                // 160
                asm!(
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                );
            }
        }
    }
}

#[no_mangle]
fn set_color(rgb: (&mut Red2, &mut Green2, &mut Blue2), color: u8) {
    // let r = (color & rshift) >> 4;
    // let g = (color & gshift) >> 2;
    // let b = color & bshift;
    // rgb.0.set_level(r);
    // rgb.1.set_level(g);
    // rgb.2.set_level(b);

}

#[no_mangle]
#[inline]
fn set_color2(color: u32) {
    // let gpio_val = 
    //     (color as u32 & red1 as u32) >> 5 |
    //     (color as u32 & red0 as u32) >> 3 |
    //     (color as u32 & green1 as u32) << 6 |
    //     (color as u32 & green0 as u32) << 8 |
    //     (color as u32 & blue1 as u32) << 19 |
    //     (color as u32 & blue0 as u32) << 21;
    // sprintln!("gpio_val = {:#032b}", gpio_val);
    let gpio_val = color;

    unsafe {
        // 0x1001_2000 + 0x0c = 
        let gpio_out = 0x1001_200C as *mut u32;
        *gpio_out = gpio_val;
    }
}

#[no_mangle]
#[inline]
fn set_color2_and_wait(color: u32) {
    unsafe {
        // GPIO is 0x1001_2000 and offset 0x0c is the output register:
        let gpio_out = 0x1001_200C as *mut u32;
        *gpio_out = color;
        asm!(
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            // "nop",
        )
    }
}

#[no_mangle]
#[inline]
fn pixel_wait() {
    unsafe {
        asm!(
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
        )
    }
}

#[no_mangle]
#[inline]
fn set_wide_pixel(color: u32) {
    set_color2_and_wait(color);
    set_color2_and_wait(color);
    set_color2_and_wait(color);
    set_color2_and_wait(color);
    set_color2_and_wait(color);
}

// sprintln!("Average line time for last frame {}", line_time as f32 / 320.0);
// sprintln!("Average ms per frame {}", (time_stop - time_start) as f32 * 0.030517578 / frames as f32);

fn timed_section(start: u64, target: u64) {
    while mcycle::read64() - start < target {}
}