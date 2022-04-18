#![no_std]
#![no_main]
#![feature(panic_info_message,alloc_error_handler,core_intrinsics)]

use core::arch::asm;

use hifive1::hal::e310x::CLINT;
use hifive1::hal::e310x::GPIO0;
use hifive1::hal::e310x::PLIC;
use hifive1::hal::e310x::Interrupt;
use hifive1::hal::DeviceResources;
use hifive1::hal::e310x::PWM1;
use hifive1::hal::prelude::*;
use hifive1::hal::core::clint;
use hifive1::hal::core::plic::Priority;
use hifive1::{sprint, pin, pins, sprintln};
use riscv::asm::wfi;
use riscv::register::mcycle;
use riscv::register::mie;
use riscv::register::mip;
use riscv_rt::entry;
use heapless::Deque;

use text::CharVis;

use crate::vga::{Brightness, Red2, Green2, Blue2, red_pins2, green_pins2, blue_pins2};

#[macro_use]
extern crate lazy_static;


#[no_mangle]
static mut CURRENT_PIXEL: usize = 0;
#[no_mangle]
static mut CURRENT_LINE_CYCLE: usize = 0;
#[no_mangle]
static mut CURRENT_LINE_PIXEL: usize = 0;
#[no_mangle]
static mut CURRENT_LINE: usize = 0;
#[no_mangle]
static mut HW_LINE: usize = 0;
#[no_mangle]
static mut VIDEO_BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
#[no_mangle]
static mut LINE_NUMBER: usize = 0;

static mut LINE_TIME: u64 = 0;

static mut WRITING: bool = true;

static mut LAST_TIMER: u64 = 0;

#[no_mangle]
static mut TEXT_BUFFER: [[u16; text::COLUMNS]; text::ROWS] = [[0; text::COLUMNS]; text::ROWS];

#[no_mangle]
static mut CURSOR: (usize, usize) = (0, 0);

static mut CURSOR_TIMER: Timer = Timer::new(800, true);

#[no_mangle]
static mut PENDING_TEXT: Deque<(usize, usize), 64> = Deque::new();

static TEXT_COLOR_INDEX: u8 = 2; // Green

static TEXT_BACKGROUND_INDEX: u8 = 0; // Black


pub const WIDTH: usize = 128;
pub const HEIGHT: usize = 96;
const BUFFER_SIZE: usize = WIDTH * HEIGHT;



mod vga;
mod color;
mod text;

struct Timer {
    /// Set time
    set_time: u64,
    /// Time in ms
    time: u64,
    repeat: bool,
    alarm: bool,
}

impl Timer {
    const fn new(set: u64, repeat: bool) -> Timer {
        Timer {
            set_time: set * 33,
            time: set * 33,
            repeat,
            alarm: false
        }
    }

    fn tick(&mut self, delta: u64) -> bool {
        self.time = self.time.saturating_sub(delta);
        self.alarm = self.time == 0;
        self.alarm
    }

    fn clear_alarm(&mut self) {
        if self.alarm {
            self.alarm = false;
            if self.repeat {
                self.time = self.set_time;
            }
        }
    }
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
            wfi();
        }
    }
}


#[no_mangle]
fn MachineTimer() {
    riscv::interrupt::free(|_| {
        unsafe {
            let current = current_mtime();
            let delta = current - LAST_TIMER;
            LAST_TIMER = current;
            let alarm = CURSOR_TIMER.tick(delta);
            if alarm {
                CURSOR_TIMER.clear_alarm();
                TEXT_BUFFER[CURSOR.1][CURSOR.0] = text::toggle_reverse_char(TEXT_BUFFER[CURSOR.1][CURSOR.0]);
                let _ = PENDING_TEXT.push_front(CURSOR);
            }
            update_time_compare(3);
        }
        
    })
}


#[no_mangle]
fn MachineExternal() {
    riscv::interrupt::free(|_| {
        set_color2(0b00110000);

        let claim = 0x0C20_0004 as *mut u32;
        let inter = unsafe { *claim };
        match inter {
            3 => unsafe {
                // rxdata register offset = 0x04
                let uart_data_rx = 0x10013004 as *mut u8;
                let w = *uart_data_rx;
                match w {
                    127 => {
                        // Space, then back up one
                        sprint!("{}{}", ' ', 8 as char);
                        vprint_char(' ');
                        CURSOR.0 -= 1;
                    },
                    8 => {
                        sprint!("{}{}{}", 8 as char, ' ', 8 as char);
                        CURSOR.0 = (CURSOR.0 - 1).clamp(0, text::COLUMNS);
                        vprint_char(' ');
                        CURSOR.0 = (CURSOR.0 - 1).clamp(0, text::COLUMNS);
                    }
                    10 | 13 => {
                        sprintln!();
                        CURSOR.0 = 0;
                        CURSOR.1 = (CURSOR.1 + 1).clamp(0, text::ROWS);
                    },
                    _ => {
                        sprint!("{}", w as char);
                        vprint_char(w as char);
                    }
                }
            },
            26 => {
                unsafe {
                    let gpio_rise_ip = 0x1001_201C as *mut u32;
                    // sprintln!("pin 18: {:#032b}", *gpio_rise_ip);
                    *gpio_rise_ip = 1 << 18;
                    // sprintln!("_after: {:#032b}", *gpio_rise_ip);
                    // sprint!("*")
                    
                    // This is the time between line interrupts
                    let duration = current_mtime() - LINE_TIME;
                    if duration > 10 {
                        // Each line should take ~0.96 RTC cycles. So if we've gone from the last line
                        // Back to the first, that vertical blank area will take way longer than .96 cycles
                        // since the vertical blank area is 45 lines. So for good measure we just check
                        // that we have "many" RTC cycles since the last interrupt. That indicates we're
                        // at the top of the frame. So let's zero out both the current line and current pixel.
                        LINE_NUMBER = 0;
                        CURRENT_PIXEL = 0;
                        CURRENT_LINE_CYCLE = 0;
                        HW_LINE = 0;
                        WRITING = true;
                        // Turn off the timer interrupt when we start the frame
                        riscv::register::mie::clear_mtimer();
                        // sprint!("x");
                    }
                    
                    LINE_TIME = current_mtime();
                }
            },
            _ => {
                sprintln!("Unhandled Interrupt Handler for {:?}", inter);
                panic!("Unknown Interrupt Handler");
            }
        }
        unsafe {
            *claim = inter;
        }

        set_color2(0b00000000);
    })
}


fn enabled_plic_interrupts() -> [(Interrupt, bool); 52] {
    sprintln!("enter enableds");
    let (enable1, enable2) = unsafe {
        let en = &(*PLIC::ptr()).enable;
        (en[0].read().bits(), en[1].read().bits())
    };
    sprintln!("Enabled 0: {:#032b}", &enable1);
    let mut interrupts: [(Interrupt, bool); 52] = [(Interrupt::GPIO0, false); 52];
    for ext in 0..32 {
        let i: usize = 1 << ext;
        let is_enabled = i & (enable1 as usize) == i;
        match Interrupt::try_from(0 + ext as u8) {
            Ok(t) => {interrupts[ext] = (t, is_enabled)},
            Err(_) => {sprintln!("error: {}", ext)}
        }
    }
    for ext in 0..20 {
        let i: usize = 1 << ext;
        let is_enabled = i & (enable2 as usize) == i;
        match Interrupt::try_from(32 + ext as u8) {
            Ok(t) => {interrupts[ext + 32] = (t, is_enabled)},
            Err(_) => {sprintln!("error: {}", ext)}
        }
    }
    interrupts
}

fn plic_priorities() -> [(usize, u8); 52] {
    let mut pr = [(0, 0); 52];
    unsafe {
        for p in 0..52 {
            let v = (*PLIC::ptr()).priority[p].read().priority().bits();
            pr[p] = (p, v);
        }
    }
    pr
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
        // riscv::register::mie::set_msoft();
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

fn setup_gpio20_interrupt() {
    // GPIO_0 starts at interrupt 8, so gpio18 is interrupt 26
    unsafe {
        (*GPIO0::ptr()).rise_ie.write(|w| w.pin20().set_bit());
        (*PLIC::ptr()).enable[0].modify(|r, w| {
            w.bits(r.bits() | (1 << 28))
        });
        (*PLIC::ptr()).priority[28].write(|w| w.priority().p6());
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

fn vprint_char(c: char) {
    unsafe {
        riscv::interrupt::free(|_| {
            // Write the character to text memory
            text::write_char(c, 
                CURSOR.1, 
                CURSOR.0, 
                &mut TEXT_BUFFER, 
                TEXT_COLOR_INDEX, 
                TEXT_BACKGROUND_INDEX
            );
            // Update the cursor position. Cursor moves to the right, unless at the end of a row
            // in which case we go down a row and go back to the first column until the end
            let old_cursor = CURSOR;
            if CURSOR.0 == text::COLUMNS - 1 {
                CURSOR.0 = 0;
                CURSOR.1 = (CURSOR.1 + 1).clamp(0, text::ROWS - 1);
            } else {
                CURSOR.0 += 1;
            }
            // Add the changed cell to the pending changes queue
            let _ = PENDING_TEXT.push_front(old_cursor);
        })
    }
}

extern "C" {
    static _heap_size: usize;
    static _sheap: usize;
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



    initialize_plic_enable_and_threshold();
    configure_uart_receiver_interrupt_enable();
    setup_gpio18_interrupt();
    // setup_gpio20_interrupt();
    let enableds = enabled_plic_interrupts();
    sprintln!("{:?}", enableds);

    let priorities = plic_priorities();
    sprintln!("{:?}", priorities);
    
    sprintln!("Hello, world!");
    sprintln!("Welcome to Rustbox Terminal");
    let core_speed = clocks.measure_coreclk();
    sprintln!("Core clock speed is {}", core_speed.0);
    
    update_time_compare(CLOCK_SPEED / 1000);
    enable_risc_interrupts();
    let _ = current_mtime();

    sprintln!("set pin 3");
    pins!(pins, (dig3)).into_iof1();
    sprintln!("after set pin");

    sprintln!("Cursor = {:?}", unsafe {CURSOR});

    unsafe {
        for r in 0..text::ROWS {
            for c in 0..text::COLUMNS {
                text::write_char(' ', r, c, &mut TEXT_BUFFER, 2, 0);
            }
        }
        // text::write_char('A', 0, 0, &mut TEXT_BUFFER, 2, 0);
        // CURSOR = (0, 0);
        zero_buffer(&mut VIDEO_BUFFER);

    }
    sprintln!("after setup");


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

    let _line = pin!(pins, dig2).into_floating_input();
    let _frame = pin!(pins, dig4).into_floating_input();
    for i in 0..32 {
        set_color2(i);
    }
    set_color2(0);

    unsafe {
        LINE_NUMBER = 0;
    }

    loop {
        // sprint!("-");
        unsafe {
            if WRITING {
                riscv::interrupt::free(|_| {
                    write_line_and_update_state();
                });
            } else {
                while !PENDING_TEXT.is_empty() {
                    match PENDING_TEXT.pop_back() {
                        None => {
                            break;
                        },
                        Some((col, row)) => {
                            let cv = CharVis::from(TEXT_BUFFER[row][col]);
                            cv.draw(&mut VIDEO_BUFFER, col, row);
                        }
                    }
                }
            }
            wfi();
        }
    }
}

#[inline]
fn write_line_and_update_state() {
    write_line();
    set_color2(0);
    // sprint!("?");
    unsafe {
        CURRENT_LINE_CYCLE += 1;
        HW_LINE += 1;
        if HW_LINE == 480 {
            // When we reach the end of the frame, turn on timer interrupts to perform other activities
            // sprint!("0");
            riscv::register::mie::set_mtimer();
            WRITING = false;
        }
        if CURRENT_LINE_CYCLE == 5 {
            // When we roll over the cycle to 0, inc current_line
            CURRENT_LINE_CYCLE = 0;
            LINE_NUMBER += 1;
            if LINE_NUMBER == 96 {
                if HW_LINE != 480 {
                    panic!("hw line is {}, and is expected to be 480", HW_LINE);
                }
                LINE_NUMBER = 0;
            }
            CURRENT_PIXEL = LINE_NUMBER as usize * WIDTH;
        }
    }
}

fn zero_buffer(buf: &mut [u8]) {
    buf.fill(0);
}

fn load_image(buffer: &mut [u8]) {
    let bs = include_bytes!("../image.bmp");
    sprintln!("bs size: {}", bs.len());
    if bs.len() >= BUFFER_SIZE {
        buffer.copy_from_slice(&bs[0..BUFFER_SIZE]);
    } else {
        for i in 0..bs.len() {
            buffer[i] = bs[i];
        }
        for i in bs.len()..BUFFER_SIZE {
            buffer[i] = 0;
        }
    }
}

#[no_mangle]
#[inline]
fn write_line() {
    unsafe {
        let pixel_end = CURRENT_PIXEL + 128;
        // sprintln!("current pixel {}", CURRENT_PIXEL);

        for p in CURRENT_PIXEL..pixel_end {
            let color = VIDEO_BUFFER[p];
            set_color2(color as u32);
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
                // "nop",
            )
        }
    }
}

#[no_mangle]
#[inline]
fn set_color2(color: u32) {
    unsafe {
        // 0x1001_2000 + 0x0c = 0x1001_200C
        let gpio_out = 0x1001_200C as *mut u32;
        *gpio_out = color;
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

fn timed_section(start: u64, target: u64) {
    while mcycle::read64() - start < target {}
}