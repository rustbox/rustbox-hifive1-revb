use core::ops::DerefMut;
use core::ops::Deref;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::PinState;
use hifive1::hal::{core::clint::MTIME, e310x::clint::mtime};
use riscv::register::{mcycle, mip};
use hifive1::hal::DeviceResources;
use hifive1::{sprint, pin, pins, sprintln};
use hifive1::hal::gpio::gpio0::*;
use hifive1::hal::gpio::{NoInvert, Output, Regular, Unknown};

use crate::{set_time_cmp};
// use crate::{LOGICAL_WIDTH, LOGICAL_LINES};

pub fn red_pins3(pins: (Pin0<Unknown>, Pin1<Unknown>, Pin2<Unknown>)) -> RedPins3 {
    (pins.0.into_output(), pins.1.into_output(), pins.2.into_output())
}

pub fn green_pins3(pins: (Pin9<Unknown>, Pin10<Unknown>, Pin11<Unknown>)) -> GreenPins3 {
    (pins.0.into_output(), pins.1.into_output(), pins.2.into_output())
}

pub fn blue_pins3(pins: (Pin3<Unknown>, Pin4<Unknown>, Pin5<Unknown>)) -> BluePins3 {
    (pins.0.into_output(), pins.1.into_output(), pins.2.into_output())
}

pub fn red_pins2(pins: (Pin0<Unknown>, Pin1<Unknown>)) -> RedPins2 {
    (pins.0.into_output(), pins.1.into_output())
}

pub fn green_pins2(pins: (Pin2<Unknown>, Pin3<Unknown>)) -> GreenPins2 {
    (pins.0.into_output(), pins.1.into_output(), )
}

pub fn blue_pins2(pins: (Pin4<Unknown>, Pin5<Unknown>)) -> BluePins2 {
    (pins.0.into_output(), pins.1.into_output())
}

pub fn address_pins(pins: (Pin16<Unknown>, Pin17<Unknown>, Pin18<Unknown>,
    Pin19<Unknown>, Pin20<Unknown>, Pin21<Unknown>,
    Pin22<Unknown>, Pin23<Unknown>)) -> ColorAddressPins {

    (pins.0.into_output(), pins.1.into_output(), pins.2.into_output(), 
        pins.3.into_output(), pins.4.into_output(), pins.5.into_output(),
        pins.6.into_output(), pins.7.into_output())
}

pub fn hsync_pins(pins: Pin10<Unknown>) -> HSyncPin {
    pins.into_output()
}

pub fn vsync_pins(pins: Pin13<Unknown>) -> VSyncPin {
    pins.into_output()
}

/// Values should be in number of clock cycles
#[derive(Copy, Clone, Debug)]
pub struct DirectionConfiguration {
    pub visible: u16,
    pub front_porch: u16,
    pub sync: u16,
    pub back_porch: u16,
    pub hardware_scale: u16,
    pub software_scale: u16
}

impl DirectionConfiguration {
    /// Given in whatever units such that when multiplied together you get dimensionless
    /// Numbers of cycles for the given time frame
    pub fn new(visible: f32, front_porch: f32, sync: f32, back_porch: f32, clock_speed: f32, scale: f32) -> DirectionConfiguration {
        DirectionConfiguration {
            visible: (visible * clock_speed * scale) as u16,
            front_porch: (front_porch * clock_speed * scale) as u16,
            sync: (sync * clock_speed * scale) as u16,
            back_porch: (back_porch * clock_speed * scale) as u16,
            hardware_scale: 1,
            software_scale: 1
        }
    }

    /// Visible goes from 0..=(visible - 1), so Front Porch starts at `visible`
    pub fn start_front_porch(&self) -> u16 {
        self.visible
    }

    pub fn end_front_porch(&self) -> u16 {
        self.start_front_porch() + self.front_porch - 1
    }

    /// Before Sync goes from 0 to visible + front porch - 1
    pub fn start_sync(&self) -> u16 {
        self.visible + self.front_porch
    }

    pub fn end_sync(&self) -> u16 {
        self.start_sync() + self.sync -1
    }

    /// Before back porch goes from 0 to visible + front porch + sync
    pub fn start_back_porch(&self) -> u16 {
        self.visible + self.front_porch + self.sync
    }

    pub fn end_back_porch(&self) -> u16 {
        self.start_back_porch() + self.back_porch - 1
    }

    pub fn size(&self) -> u16 {
        self.visible + self.front_porch + self.sync + self.back_porch
    }
}

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

#[derive(Copy, Clone, Debug)]
pub struct VgaConfiguration {
    pub horizontal: DirectionConfiguration,
    pub vertical: DirectionConfiguration
}

// #[repr(transparent)]
// pub struct Frame {
//     pub buffer: [[u8; LOGICAL_WIDTH as usize]; LOGICAL_LINES as usize]
// }

// pub struct LineBufferWriter {
//     address_lines: ColorAddress,
//     rgb: (Red3, Green3, Blue3),
//     logical_line: u16,
//     hardware_line: u16,
//     frame: &'static Frame
// }

// impl LineBufferWriter {
//     fn write_line(&mut self) {
//         for pix in 0..LOGICAL_WIDTH {
//             // turn on address
//             self.address_lines.set_address(pix as u8);
//             // Translate color index into color
//             let color_index = self.frame.buffer[self.logical_line as usize][pix as usize];
//             let color = &crate::color::COLOR_MAP[color_index as usize];
//             // turn on color lines
//             self.rgb.0.set_level(color.0);
//             self.rgb.1.set_level(color.1);
//             self.rgb.2.set_level(color.2);
//         }
//     }
// }


/// The Horizontal state of the VGA signal
// #[derive(Copy, Clone, PartialEq, Debug)]
// pub enum BeamState {
//     Visible,
//     Blank,
//     Sync,
// }

/// The Vertical state of the VGA signal
// #[derive(Clone, Copy, PartialEq, Debug)]
// pub enum LineState {
//     Blank,
//     Sync
// }

/// Compute the state the Beam (vertical and horizontal) is in based on the current pixel, column and line.
/// The horizontal state can be `Visible`, `Blank`, or `Sync` and is determined by the VgaConfiguration
/// visible, front_porch, sync, and back_porch values. When a pixel is in the back or front porch the 
/// BeamState is Blank.
///
/// The vertical state is expressed as an Option of `Blank` or `Sync` when the line is in the front porch, 
/// back porch, or sync period, respectively. If a line is not in either period then nothing in particular
/// needs to happen and is the Option None.
// pub fn beam_state(column: u16, line: u16, configuration: &VgaConfiguration) -> (BeamState, Option<LineState>) {
//     // Both Vertical and Horizontal are "Blank" if we're in the Vertical blanking period
//     if line >= configuration.vertical.start_front_porch() / configuration.vertical.hardware_scale && line <= configuration.vertical.end_front_porch() / configuration.vertical.hardware_scale
//         || line >= configuration.vertical.start_back_porch() / configuration.vertical.hardware_scale && line <= configuration.vertical.end_back_porch() / configuration.vertical.hardware_scale {

//         (BeamState::Blank, Some(LineState::Blank))
//     } else {
//         let beam_state = if column <= configuration.horizontal.visible / configuration.horizontal.hardware_scale - 1 {
//             BeamState::Visible
//         } else if column >= configuration.horizontal.start_front_porch() / configuration.horizontal.hardware_scale && column <= configuration.horizontal.end_front_porch() / configuration.horizontal.hardware_scale {
//             BeamState::Blank
//         } else if column >= configuration.horizontal.start_sync() / configuration.horizontal.hardware_scale && column <= configuration.horizontal.end_sync() / configuration.horizontal.hardware_scale {
//             BeamState::Sync
//         } else {
//             BeamState::Blank
//         };

//         // sprintln!("{:?}", configuration.vertical);
//         // sprintln!("line: {}, scaled sync start {}, scaled sync end {}", line, configuration.vertical.start_sync() / configuration.vertical.hardware_scale, configuration.vertical.end_sync() / configuration.vertical.hardware_scale);

//         let line_state = if line >= configuration.vertical.start_sync() / configuration.vertical.hardware_scale && line <= configuration.vertical.end_sync() / configuration.vertical.hardware_scale {
            
//             Some(LineState::Sync)
//         } else {
//             None
//         };

//         (beam_state, line_state)
//     }
// }

/// Given the values of red, green, and blue, and the vertical and horizontal sync values, engage or
/// disengage the specific Output Pins
fn output_color3(red: &mut Red3, green: &mut Green3, blue: &mut Blue3, red_value: u8, green_value: u8, blue_value: u8) {
    red.set_level(red_value);
    green.set_level(green_value);
    blue.set_level(blue_value);
}



pub struct VSync {
    pub pin: VSyncPin
}

impl VSync {
    /// Sync on means the signal goes low
    pub fn on(&mut self) -> &mut VSync {
        let _ = self.pin.set_low();
        self
    }

    /// Sync off means the signal goes high
    pub fn off(&mut self) -> &mut VSync {
        let _ = self.pin.set_high();
        self
    }
}


pub struct HSync {
    pub pin: HSyncPin
}

impl HSync {
    /// Sync on means the signal goes low
    pub fn on(&mut self) -> &mut HSync {
        let _ = self.pin.set_low();
        self
    }

    /// Sync off means the signal goes high
    pub fn off(&mut self) -> &mut HSync {
        let _ = self.pin.set_high();
        self
    }
}


pub trait Brightness {
    fn set_level(&mut self, level: u8);
}

/// dig8, dig9, dig10
pub type RedPins3 = (Pin0<Output<Regular<NoInvert>>>, Pin1<Output<Regular<NoInvert>>>, Pin2<Output<Regular<NoInvert>>>);

/// dig11, dig12, dig13
pub type BluePins3 = (Pin3<Output<Regular<NoInvert>>>, Pin4<Output<Regular<NoInvert>>>, Pin5<Output<Regular<NoInvert>>>);

/// dig15, dig16, dig17
pub type GreenPins3 = (Pin9<Output<Regular<NoInvert>>>, Pin10<Output<Regular<NoInvert>>>, Pin11<Output<Regular<NoInvert>>>);

/// dig0, dig1
pub type RedPins2 = (Pin0<Output<Regular<NoInvert>>>, Pin1<Output<Regular<NoInvert>>>);

/// dig2, dig3
pub type GreenPins2 = (Pin2<Output<Regular<NoInvert>>>, Pin3<Output<Regular<NoInvert>>>);

/// dig4, dig5
pub type BluePins2 = (Pin4<Output<Regular<NoInvert>>>, Pin5<Output<Regular<NoInvert>>>);



/// digs: [0, 1, 2, 3, 4, 5, 6, 7]
pub type ColorAddressPins = (Pin16<Output<Regular<NoInvert>>>, Pin17<Output<Regular<NoInvert>>>, Pin18<Output<Regular<NoInvert>>>, 
    Pin19<Output<Regular<NoInvert>>>, Pin20<Output<Regular<NoInvert>>>, Pin21<Output<Regular<NoInvert>>>,
    Pin22<Output<Regular<NoInvert>>>, Pin23<Output<Regular<NoInvert>>>);

/// dig16
pub type HSyncPin = Pin10<Output<Regular<NoInvert>>>;

/// dig19
pub type VSyncPin = Pin13<Output<Regular<NoInvert>>>;

pub struct Red3 {
    pub pins: RedPins3
}

pub struct Green3 {
    pub pins: GreenPins3
}

pub struct Blue3 {
    pub pins: BluePins3
}

pub struct Red2 {
    pub pins: RedPins2
}

pub struct Green2 {
    pub pins: GreenPins2
}

pub struct Blue2 {
    pub pins: BluePins2
}

pub struct ColorAddress {
    /// index 0 is MSB
    pub pins: ColorAddressPins
}

impl ColorAddress {
    fn set_address(&mut self, address: u8) {
        let i7 = address & 0b10000000 != 0;
        let i6 = address & 0b01000000 != 0;
        let i5 = address & 0b00100000 != 0;
        let i4 = address & 0b00010000 != 0;
        let i3 = address & 0b00001000 != 0;
        let i2 = address & 0b00000100 != 0;
        let i1 = address & 0b00000010 != 0;
        let i0 = address & 0b00000001 != 0;

        if i7 {
            let _ = self.pins.1.set_high();
        } else {
            let _ = self.pins.1.set_low();
        }

        if i6 {
            let _ = self.pins.2.set_high();
        } else {
            let _ = self.pins.2.set_low();
        }

        if i5 {
            let _ = self.pins.3.set_high();
        } else {
            let _ = self.pins.3.set_low();
        }

        if i4 {
            let _ = self.pins.4.set_high();
        } else {
            let _ = self.pins.4.set_low();
        }

        if i3 {
            let _ = self.pins.5.set_high();
        } else {
            let _ = self.pins.5.set_low();
        }

        if i2 {
            let _ = self.pins.6.set_high();
        } else {
            let _ = self.pins.6.set_low();
        }

        if i1 {
            let _ = self.pins.7.set_high();
        } else {
            let _ = self.pins.7.set_low();
        }

        if i0 {
            let _ = self.pins.7.set_high();
        } else {
            let _ = self.pins.7.set_low();
        }
    }
}

impl Brightness for Red3 {
    fn set_level(&mut self, level: u8) {
        if level % 2 == 1 {
            let _ = self.pins.2.set_high();
        } else {
            let _ = self.pins.2.set_low();
        }

        if (level >> 1) % 2 == 1 {
            let _ = self.pins.1.set_high();
        } else {
            let _ = self.pins.1.set_low();
        }

        if (level >> 2) % 2 == 1 {
            let _ = self.pins.0.set_high();
        } else {
            let _ = self.pins.0.set_low();
        }
    }
}

impl Brightness for Green3 {
    fn set_level(&mut self, level: u8) {
        if level % 2 == 1 {
            let _ = self.pins.2.set_high();
        } else {
            let _ = self.pins.2.set_low();
        }

        if (level >> 1) % 2 == 1 {
            let _ = self.pins.1.set_high();
        } else {
            let _ = self.pins.1.set_low();
        }

        if (level >> 2) % 2 == 1 {
            let _ = self.pins.0.set_high();
        } else {
            let _ = self.pins.0.set_low();
        }
    }
}

impl Brightness for Blue3 {
    fn set_level(&mut self, level: u8) {
        if level % 2 == 1 {
            let _ = self.pins.2.set_high();
        } else {
            let _ = self.pins.2.set_low();
        }

        if (level >> 1) % 2 == 1 {
            let _ = self.pins.1.set_high();
        } else {
            let _ = self.pins.1.set_low();
        }

        if (level >> 2) % 2 == 1 {
            let _ = self.pins.0.set_high();
        } else {
            let _ = self.pins.0.set_low();
        }
    }
}

impl Brightness for Red2 {
    #[no_mangle]
    fn set_level(&mut self, level: u8) {
        // PinState;
        let pin0 = PinState::from((level & 0b0000001) == 1);
        let pin1 = PinState::from((level & 0b0000010) == 2);

        // sprintln!("pin0: {:?}, pin1: {:?}", pin0, pin1);

        let _ = self.pins.0.set_state(pin0);
        let _ = self.pins.1.set_state(pin1);
    }
}

impl Brightness for Green2 {
    fn set_level(&mut self, level: u8) {
        // PinState;
        let pin0 = PinState::from((level & 0b0000001) == 1);
        let pin1 = PinState::from((level & 0b0000010) == 2);

        let _ = self.pins.0.set_state(pin0);
        let _ = self.pins.1.set_state(pin1);
    }
}

impl Brightness for Blue2 {
    fn set_level(&mut self, level: u8) {
        // PinState;
        let pin0 = PinState::from((level & 0b0000001) == 1);
        let pin1 = PinState::from((level & 0b0000010) == 2);

        let _ = self.pins.0.set_state(pin0);
        let _ = self.pins.1.set_state(pin1);
    }
}



fn blanking_for_cycles(delta: u64) {
    let start = MTIME.mtime();
    set_time_cmp(start + delta);
    unsafe {
        // Turn on timer interrupts
        riscv::register::mie::set_mtimer();
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
        // Turn off timer interrupts
        riscv::register::mie::clear_mtimer();
    }
}

fn line(config: &VgaConfiguration, hsync: &mut HSync) {
    let frame_start = mcycle::read64();
    // Wait for visible duration
    while mcycle::read64() - frame_start < config.horizontal.visible as u64 {}
    
    let fp_start = mcycle::read64();
    while mcycle::read64() - fp_start < config.horizontal.front_porch as u64 {}

    let sync_start = mcycle::read64();
    // Turn on sync
    hsync.on();
    while mcycle::read64() - sync_start < config.horizontal.sync as u64 {}
    hsync.off();
    // Turn off sync

    let back_porch_start = mcycle::read64();
    while mcycle::read64() - back_porch_start < config.horizontal.back_porch as u64 {}
}

fn blanking_hsync(config: &VgaConfiguration, hsync: &mut HSync) {
    let line_start = mcycle::read64();
    let blank_time = config.horizontal.visible + config.horizontal.front_porch;
    while mcycle::read64() - line_start < blank_time as u64 {}

    let sync_start = mcycle::read64();
    //Turn on HSync
    hsync.on();
    while mcycle::read64() - sync_start < config.horizontal.sync as u64 {}
    hsync.off();
    // Turn off HSync

    let back_porch_start = mcycle::read64();
    while mcycle::read64() - back_porch_start < config.horizontal.back_porch as u64 {}
}

pub fn frame(config: &VgaConfiguration, hsync: &mut HSync, vsync: &mut VSync) -> u64 {
    
    // Visible
    // let mut n = 0;
    // let frame_start_time = MTIME.mtime();
    let frame_start = mcycle::read64();
    while mcycle::read64() - frame_start < config.vertical.visible as u64 {
        line(config, hsync);
    }
    let frame_end = mcycle::read64();

    // Blanking
    let front_porch_start = mcycle::read64();
    while mcycle::read64() - front_porch_start < config.vertical.front_porch as u64 {
        blanking_hsync(config, hsync);
    }

    // Turn on vertical sync
    let vsync_start = mcycle::read64();
    vsync.on();
    while mcycle::read64() - vsync_start < config.vertical.sync as u64 {
        blanking_hsync(config, hsync)
    }
    vsync.off();
    // Turn off Vertical sync

    // Blanking
    let back_porch_start = mcycle::read64();
    while mcycle::read64() - back_porch_start < config.vertical.back_porch as u64 {
        blanking_hsync(config, hsync)
    }

    return 0;
}

#[cfg(test)]
mod test_vga {
    use super::*;

    #[test]
    fn test_beam_state() {
        let line = 490;
        let config = vga::VgaConfiguration {
            horizontal: vga::DirectionConfiguration {
                visible: 640,
                front_porch: 16,
                sync: 96,
                back_porch: 48,
                hardware_scale: 2,
                software_scale: 2
            },
            vertical: vga::DirectionConfiguration {
                visible: 480,
                front_porch: 10,
                sync: 2,
                back_porch: 33,
                hardware_scale: 1,
                software_scale: 1
            }
        };

        let (beam_state, line_state) = beam_state(0, line, config);
        assert_eq!(line_state, Some(LineState::Sync))
    }
}
