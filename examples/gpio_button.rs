#![no_std]
#![no_main]

/*
*  TODO comment
*/

extern crate panic_halt;

use hifive1::hal::delay::Sleep;
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::sprint;
use hifive1::Led;
use hifive1::{pin, pins};
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let pins = dr.pins;

    // Configure clocks
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    // GPIO PIN1 -> DIG9 physical on board (both hifive1 and hifive1-revB)
    let button = pin!(pins, dig9).into_pull_up_input();

    // get all 3 led pins in a tuple (each pin is it's own type here)
    let rgb_pins = pins!(pins, (led_red, led_green, led_blue));
    let mut tleds = hifive1::rgb(rgb_pins.0, rgb_pins.1, rgb_pins.2);

    // Configure UART for stdout
    hifive1::stdout::configure(
        p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );

    sprint!("hello button {:?}\n", button.is_low());

    // // get the local interrupts struct
    let clint = dr.core_peripherals.clint;

    // // get the sleep struct
    let mut sleep = Sleep::new(clint.mtimecmp, clocks);

    const PERIOD: u32 = 1000; // 1s
    loop {
        if let Ok(b) = button.is_low() {
            if b {
                tleds.0.on();
                tleds.1.off();
                tleds.2.off();
            } else {
                tleds.0.off();
                tleds.1.off();
                tleds.2.on();
            }
        }

        // sleep for 1s
        sleep.delay_ms(PERIOD);
    }
}
