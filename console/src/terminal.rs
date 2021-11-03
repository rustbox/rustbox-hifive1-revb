use core::str;

use alloc::string::{String, ToString};
use alloc::vec::Vec;
use hifive1::{sprint, sprintln};

use crate::input;
use crate::fsm::{Trie, Input, Output, MachineState};
use crate::block_until;

static CONTROL_SEQUENCES: &[(&[&str], Output)] = &[
    (&["ESC", "ESC"], Output::ESC),
    (&["ESC", "[", "3", "~"], Output::DEL),
    (&["ESC", "[", "D"], Output::CursorLeft(1)),
    (&["ESC", "[", "C"], Output::CursorRight(1)),
    (&["ESC", "[", "A"], Output::CursorUp(1)),
    (&["ESC", "[", "B"], Output::CursorDown(1)),
    (&["ESC", "[", "2", "~"], Output::INSERT),
    (&["ESC", "[", "0", "K"], Output::EraseFromCursor),
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
                &Output::CursorLeft(1) => {
                    let amount = self.buffer.move_cursor_left(1);
                    if amount > 0 {
                        sprint!("{}", bytes_to_str(&[27, 91, 68]));
                    }
                }
                &Output::CursorLeft(_) => {},
                &Output::CursorRight(1) => {
                    let amount = self.buffer.move_cursor_right(1);
                    if amount > 0 {
                        sprint!("{}", bytes_to_str(&[27, 91, 67]))
                    }
                }
                &Output::CursorRight(_) => {},
                &Output::CursorUp(_) => {},
                &Output::CursorDown(_) => {},
                &Output::DEL => {
                    let deleted = self.buffer.delete();
                    if deleted.is_some() {
                        //Delete from cursor to end of line
                        sprint!("{}", bytes_to_str(&[27, 91, 48, 'K' as u8]));
                        // print buffer from cursor to the end
                        let buf_to_end = self.buffer.slice_from_cursor();
                        sprint!("{}", bytes_to_str(buf_to_end));
                        // Return cursor to real cursor position
                        let amt = (self.buffer.len() - self.buffer.cursor_position()).to_string();
                        sprint!("{}", bytes_to_str(&[ &[27, 91], amt.as_bytes(), &[68] ].concat()));
                    }
                }
                &Output::INSERT => {
                    sprint!("{}", bytes_to_str(&[27, 91, 50, 126]));
                    sprintln!("Insert!");
                },
                &Output::EraseFromCursor => {
                    sprint!("{}", bytes_to_str(&[27, 91, 48, 75]));
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
                let removed = self.buffer.backspace();
                if removed.is_some() {
                    sprint!("{}{}{}", 8 as char, 32 as char, 8 as char);
                }
            }
            MachineState::Listening(c) => {
                let ch: u8 = c.into();
                if self.buffer.cursor_position() == self.buffer.len() {
                    self.buffer.push(ch);
                    sprint!("{}", ch as char);
                } else {
                    // Delete to end of the line
                    sprint!("{}", bytes_to_str(&[27, 91, 48, 'K' as u8]));
                    // push byte and print it
                    self.buffer.push(ch);
                    sprint!("{}", ch as char);
                    // print buffer slice from cursor position to the end
                    let buf_to_end = self.buffer.slice_from_cursor();
                    sprint!("{}", bytes_to_str(buf_to_end));
                    // Move cursor back to previous position
                    let amt = (self.buffer.len() - self.buffer.cursor_position()).to_string();
                    sprint!("{}", bytes_to_str(&[ &[27, 91], amt.as_bytes(), &[68] ].concat()));
                }
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

    pub fn get_char(&mut self) -> char {
        block_until(|| self.buffer.len() > 0);
        self.buffer.flush().0[0] as char
    }
}

fn bytes_to_str(bytes: &[u8]) -> &str {
    unsafe {
        str::from_utf8_unchecked(bytes)
    }
}








