#![no_std]
#![no_main]

/*
*  TODO comment
*/

extern crate panic_halt;

use bare_metal::Nr;
use hifive1::hal::core::plic::Priority;
use hifive1::hal::core::CorePeripherals;
use hifive1::hal::e310x::Interrupt;
use hifive1::hal::e310x::GPIO0;
use hifive1::hal::gpio::{gpio0::Pin1, Input, PullUp};
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::Led;
use hifive1::{pin, pins};
use hifive1::{sprint, sprintln};
use riscv_rt::entry;

#[no_mangle]
pub unsafe extern "C" fn MachineExternal() {
    let mut plic = CorePeripherals::steal().plic;
    let intr = plic.claim.claim().unwrap();
    match intr {
        Interrupt::GPIO1 => {
            sprintln!("Hi!");
            let dr = DeviceResources::steal();
            let pins = dr.pins;

            let rgb_pins = pins!(pins, (led_red, led_green, led_blue));
            let mut tleds = hifive1::rgb(rgb_pins.0, rgb_pins.1, rgb_pins.2);

            if BUTTON
                .as_ref()
                .expect("interrupt enabled after replace")
                .is_low()
                .expect("infallable")
            {
                tleds.0.on();
                tleds.1.off();
                tleds.2.off();
            } else {
                tleds.0.off();
                tleds.1.off();
                tleds.2.on();
            }
        }
        _ => {
            sprintln!("Unknown interrupt #{}!", intr.nr());
            // panic!("Unknown interrupt");
        }
    }
    plic.claim.complete(intr);
}

static mut BUTTON: Option<Pin1<Input<PullUp>>> = None;

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

    // GPIO PIN1 -> DIG9 physical on board (both hifive1 and hifive1-revB)
    let button = pin!(pins, dig9).into_pull_up_input();

    sprint!("hello world {:?}\n", button.is_low());

    unsafe {
        let rgpio = &*hifive1::hal::e310x::GPIO0::ptr();
        // rgpio.rise_ie.write(|w| w.pin1().set_bit());
        // rgpio.fall_ie.write(|w| w.pin1().set_bit());
        rgpio.input_en.write(|w| w.pin1().set_bit());
        rgpio.pullup.write(|w| w.pin1().set_bit());
        rgpio.high_ie.write(|w| w.pin1().set_bit());
        rgpio.low_ie.write(|w| w.pin1().set_bit());

        // rgpio.low_ip.read()
    }

    unsafe {
        BUTTON.replace(button);
    }

    // get all 3 led pins in a tuple (each pin is it's own type here)
    let rgb_pins = pins!(pins, (led_red, led_green, led_blue));
    let mut tleds = hifive1::rgb(rgb_pins.0, rgb_pins.1, rgb_pins.2);
    tleds.0.off();
    tleds.1.on();
    tleds.2.off();

    unsafe {
        let rplic = &*hifive1::hal::e310x::PLIC::ptr();
        for p in rplic.priority.iter() {
            // p.write(|w| w.bits(0));
            p.write(|w| w.bits(Priority::P7.into()));
        }
        rplic.priority[1].write(|w| w.bits(Priority::P0.into()));
        rplic.priority[46].write(|w| w.bits(Priority::P0.into())); // TODO: vas ist das?

        // rplic.priority[9].write(|w| w.bits(Priority::P7.into()));
        rplic.enable[0].write(|w| w.bits(0b1 << 9));
        // rplic.enable[1].write(|w| w.bits(0b1 << 9));

        let mut plic = CorePeripherals::steal().plic;

        plic.threshold.set(Priority::P0);
        plic.mext.enable();

        riscv::register::mstatus::set_mie();
    }

    loop {
        unsafe {
            riscv::asm::wfi();
        }
    }
}
