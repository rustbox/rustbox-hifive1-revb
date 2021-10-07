#![no_std]
#![no_main]
#![feature(panic_info_message,asm,alloc_error_handler,core_intrinsics)]

use core::intrinsics::volatile_store;

use embedded_hal::blocking::delay;
use hifive1::Led;
use hifive1::hal::core::clint::MTIME;
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


use crate::vga::Brightness;
use crate::vga::address_pins;

#[macro_use]
extern crate lazy_static;

static mut NEXT_PIXEL_RED: u8 = 0;
static mut NEXT_PIXEL_GREEN: u8 = 0;
static mut NEXT_PIXEL_BLUE: u8 = 0;
static mut HSYNC_STATE: bool = false;
static mut VSYNC_STATE: bool = false;

static mut CURRENT_COLUMN: u16 = 0;
static mut CURRENT_LINE: u16 = 0;
static mut FRAME: u16 = 0;

// static mut HSYNC_LINE: vga::HSync = vga::HSync { pin: vga::hsync_pins(DeviceResources::steal().pins.pin12) };

mod vga;
mod color;

static mut HSYNC_LINE: Option<vga::HSync> = None;
static mut VSYNC_LINE: Option<vga::VSync> = None;

const VGA_CONFIGURATION: vga::VgaConfiguration = vga::VgaConfiguration {
    horizontal: vga::DirectionConfiguration {
        visible: 640,
        front_porch: 16,
        sync: 96,
        back_porch: 48,
        hardware_scale: 4,
        software_scale: 1
    },
    vertical: vga::DirectionConfiguration {
        visible: 480,
        front_porch: 10,
        sync: 2,
        back_porch: 33,
        hardware_scale: 1,
        software_scale: 5
    }
};


pub const LOGICAL_WIDTH: u8 = (VGA_CONFIGURATION.horizontal.visible / (VGA_CONFIGURATION.horizontal.hardware_scale * VGA_CONFIGURATION.horizontal.software_scale)) as u8;
pub const LOGICAL_LINES: u16 = VGA_CONFIGURATION.vertical.visible / (VGA_CONFIGURATION.vertical.hardware_scale * VGA_CONFIGURATION.vertical.software_scale);



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
                    unsafe {
                        (*GPIO0::ptr()).rise_ip.write(|w| w.pin18().set_bit());
                        // sprintln!("Handling pixel interrupt");

                        if let Some(hsync_line) = HSYNC_LINE.as_mut() {
                            if HSYNC_STATE {
                                hsync_line.on();
                                // sprintln!("Horizontal Sync on, pixel {}", CURRENT_COLUMN)
                            } else {
                                hsync_line.off();
                                // sprintln!("Horizontal Sync off, pixel {}", CURRENT_COLUMN)
                            }
                        }

                        if CURRENT_COLUMN == 0 {
                            // Only update the during the first column
                            if let Some(vsync_line) = VSYNC_LINE.as_mut() {
                                // sprintln!("Line {}, Vsync: {}", CURRENT_LINE, VSYNC_STATE);
                                if VSYNC_STATE {
                                    vsync_line.on();
                                } else {
                                    vsync_line.off();
                                }
                            }
                        }

                        CURRENT_COLUMN = (CURRENT_COLUMN + 1) % (VGA_CONFIGURATION.horizontal.size() / VGA_CONFIGURATION.horizontal.hardware_scale);
                        if CURRENT_COLUMN == 0 {
                            CURRENT_LINE = (CURRENT_LINE + 1) % (VGA_CONFIGURATION.vertical.size() / VGA_CONFIGURATION.vertical.hardware_scale);
                            if CURRENT_LINE == 0 {
                                FRAME += 1;
                                sprintln!("Frame {}", FRAME);
                            }
                        }

                        let (beam_state, line_state) = vga::beam_state(CURRENT_COLUMN, CURRENT_LINE, &VGA_CONFIGURATION);
                        // sprintln!("line_state: {:?}", line_state);
                        HSYNC_STATE = beam_state == vga::BeamState::Sync;
                        VSYNC_STATE = line_state.is_some() && (line_state.unwrap() == vga::LineState::Sync);
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
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320_000.khz().into());

    // Configure UART0 for stdout
    let _ = hifive1::stdout::configure(p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );

    unsafe {
        HSYNC_LINE = Some(vga::HSync { pin: vga::hsync_pins(pins.pin10) });
        VSYNC_LINE = Some(vga::VSync { pin: vga::vsync_pins(pins.pin13) });
    };

    sprintln!("config = {:?}", &VGA_CONFIGURATION);
    let (_beam_state, line_state) = vga::beam_state(0, 491, &VGA_CONFIGURATION);
    sprintln!("Test Line State: {:?}", line_state);


    initialize_plic_enable_and_threshold();
    // configure_uart_receiver_interrupt_enable();
    let enableds = enabled_plic_interrupts();
    setup_pmw(&p.PWM1);
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
    setup_gpio18_interrupt();
    // let timer_start = current_mtime();

    let config = vga::VgaConfiguration {
        horizontal: vga::DirectionConfiguration::new(
            25.422045680238, 
            0.63555114200596, 
            3.8133068520357, 
            1.9066534260179, 
            25.175,
            1.0
        ),
        vertical: vga::DirectionConfiguration::new(
            15.253227408143,
            0.31777557100298,
            0.063555114200596,
            1.0486593843098,
            31.46875,
            1.0
        )
    };

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
    const horizontal: usize = 160;
    const vertical: usize = 96;
    
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
    
    unsafe {
        let start = &_sheap as *const usize;
        sprintln!("\"heap\" start: {:?}", start);
    }

    let mut duration: u64 = 0;
    let mut frames = 0;
    sprintln!("Initialize frame buffer in memory");

    // sprintln!("line = {:?}", buffer.pixels[0]);
    // sprintln!("size is {}", buffer.pixels.len());

    loop {
        let start = mcycle::read64();
        for line in 0..buffer_size {
            // buffer.pixels[line] = 0;
            // for pixel in 0..160 {
            //     // sprintln!("pixel is {}", pixel);
            //     // buffer.pixels[line][pixel] = 0;
            // }
        }
        let end = mcycle::read64();
        frames += 1;
        if frames > 0 {
            duration += (end - start);
        }
        if frames >= 20 {
            break;
        }
    }

    // buffer.pixels[4321] = 42;
    let magic = unsafe { & *(0x800010ed as *const u8) };
    sprintln!("Magic value at 0x800010ed: {}", magic);

    sprintln!("frames {}", frames);
    sprintln!("duration: {}", duration);
    let cycles_per_frame = duration as f32 / (frames) as f32;
    sprintln!("Average cycles per frame: {}", cycles_per_frame);

    // let end = &buffer.pixels[buffer_size - 128 .. buffer_size];
    // sprintln!("end of buffer: {:?}", end);

    // sprintln!("Whole frame: {:?}", buffer.pixels);

    loop {

        // for line in buffer {
        //     for pixel in line {
        //         let c = &color::COLOR_MAP[pixel as usize];
        //         red.set_level(c.0);
        //     }
        // }
        // if i == 200 {
        //     p.PWM1.cfg.write(|w|  w.cmp1ip().clear_bit() );
        //     sprintln!("Enabling pmw1 cmp1 interrupt");
        //     enable_pmw_interrupt(1, 1);
        // }
        // pwm.
        // let mut frames = 0;
        // let time_start = MTIME.mtime();
        // let mut line_time = 0;
        // loop {
    
        //     line_time = vga::frame(&config);
        //     frames += 1;
    
        //     if frames > 1000 {
        //         break;
        //     }
        // }
        // let time_stop = MTIME.mtime();
        // cycles / frame * (1 sec / 32,768 cycles) * (1000 ms/sec)

        // vga::frame(&config, &mut hsync, &mut vsync);

        // Wait a bit before repeating
        // while MTIME.mtime() - time_stop < 16_000 {}

        // Try to do something before it triggers the interrupt
        // Wait until we get at least 500 scaled pwm1 ticks
        // while p.PWM1.pwms.read().bits() < 500 {}
        // // Then set the count back to 0
        // let t = p.PWM1.count.read().bits();
        // let ts = p.PWM1.pwms.read().bits();
        // i += 1;
        // sprintln!("{}) Scaled is {}, count is {}", i, ts, t);
        // sprintln!("Attempt to set to 0");
        // p.PWM1.count.write(|w| unsafe { w.bits(0) });
        unsafe {
            wfi()
        }
    }
}

// sprintln!("Average line time for last frame {}", line_time as f32 / 320.0);
// sprintln!("Average ms per frame {}", (time_stop - time_start) as f32 * 0.030517578 / frames as f32);

fn timed_section(start: u64, target: u64) {
    while mcycle::read64() - start < target {}
}