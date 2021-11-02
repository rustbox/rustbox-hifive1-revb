use core::str;

use alloc::string::String;
use alloc::vec::Vec;
use hifive1::{sprint, sprintln};

use crate::input;
use crate::fsm::{Trie, Input, Output, MachineState};
use crate::block_until;

static CONTROL_SEQUENCES: &[(&[&str], Output)] = &[
    (&["ESC", "ESC"], Output::ESC),
    (&["ESC", "[", "3", "~"], Output::DEL),
    (&["ESC", "[", "D"], Output::CURSOR_LEFT(1)),
    (&["ESC", "[", "C"], Output::CURSOR_RIGHT(1)),
    (&["ESC", "[", "A"], Output::CURSOR_UP(1)),
    (&["ESC", "[", "B"], Output::CURSOR_DOWN(1)),
];


trait Effect<E> {
    fn effect(&self) -> E;
}


enum TerminalAction {
    /// Echo byte to terminal, and add byte to buffer
    Echo(u8),
    /// Do not echo byte, but add byte to buffer
    Silent(u8),
    /// Delete the character currently at the cursor
    Delete,
    /// Do not echo and do not add byte to buffer
    Wait
}

pub struct Terminal {
    parser: Trie<Input, Output>,
    buffer: input::LineBuffer,
    stdin: Vec<u8>
}

impl Terminal {
    pub fn new() -> Terminal {
        Terminal { 
            parser: Trie::from_strs(CONTROL_SEQUENCES), 
            buffer: input::LineBuffer::new(),
            stdin: Vec::new()
        }
    }

    pub fn input(&mut self, c: u8) {
        let input = Input::from(c);
        let (out_action, state) = self.parser.input_next(input);
        if let Some(o) = out_action {
            match o {
                &Output::CURSOR_LEFT(1) => {
                    self.buffer.move_cursor_left(1);
                    sprint!("{}", bytes_to_str(&[27, 91, 68]));
                }
                &Output::CURSOR_LEFT(_) => {},
                &Output::CURSOR_RIGHT(1) => {
                    self.buffer.move_cursor_right(1);
                    sprint!("{}", bytes_to_str(&[27, 91, 67]))
                }
                &Output::CURSOR_RIGHT(_) => {},
                &Output::CURSOR_UP(_) => {},
                &Output::CURSOR_DOWN(_) => {},
                &Output::DEL => {
                    self.buffer.delete();
                    sprint!("{}", bytes_to_str(&[27, 91, 51, 126]))
                }
                _ => {}
            }
        }
        // Only echo input to terminal if MachineState::Listening
        match state {
            MachineState::Listening(Input::NewLine) => {
                let ch: u8 = c.into();
                self.buffer.push(ch);

                riscv::interrupt::free(|_| {

                    while let Some(c) = self.buffer.read_char() {
                        self.stdin.push(c);
                    }
                });

                sprintln!();
            }
            MachineState::Listening(Input::Char(127)) => {
                // Backspace
                self.buffer.backspace();
                if self.buffer.cursor_position() > 0 {
                    sprint!("{}{}{}", 8 as char, 32 as char, 8 as char);
                }
            }
            MachineState::Listening(c) => {
                let ch: u8 = c.into();
                self.buffer.push(ch);
                sprint!("{}", char::from(c));
            },
            _ => {}
        }
    }

    pub fn read_line(&mut self, buffer: &mut String) -> u8 {
        block_until(|| self.stdin.len() > 0);

        let mut read = 0;
        let mut iter = self.stdin.iter();
        while let Some(c) = iter.next() {
            buffer.push(*c as char);
            read += 1;
            if *c as char == '\n' {
                break;
            }
        }

        self.stdin.clear();
        read
    }
}

fn bytes_to_str(bytes: &[u8]) -> &str {
    unsafe {
        str::from_utf8_unchecked(bytes)
    }
}








