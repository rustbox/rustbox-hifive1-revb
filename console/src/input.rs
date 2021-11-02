
pub const BUFFER_SIZE: usize = 256;

use heapless::{Deque, Vec};
use hifive1::sprintln;

pub struct LineBuffer {
    buffer: Vec<u8, BUFFER_SIZE>,
    /// Cursor is the position in the buffer where the next pushed character is about to go
    cursor: u8,
    out: Deque<u8, BUFFER_SIZE>,
    settings: Settings
}

struct Settings {
    buffer_input: bool
}

impl Default for Settings {
    fn default() -> Settings {
        Settings {
            buffer_input: true
        }
    }
}


impl LineBuffer {
    pub fn new() -> LineBuffer {
        LineBuffer {
            buffer: Vec::new(),
            cursor: 1,
            out: Deque::new(),
            settings: Settings::default()
        }
    }

    /// Push a byte onto the buffer at the cursor location, incrementing the cursor
    /// 
    /// If the cursor is in the middle of the buffer, then we by default insert the byte
    /// increasing the size of the buffer.
    /// 
    /// If the cursor is at the end of the buffer then we merely push the character onto the end of the buffer
    pub fn push(&mut self, c: u8) {

        if c == 10 || c == 13 {
            let (contents, length) = self.flush();
            for i  in 0..length as usize {
                let _ = self.out.push_front(contents[i]);
            }
            let _ = self.out.push_front('\n' as u8);
        } else {
            self.cursor = self.cursor.clamp(0, self.buffer.len() as u8);
            let _ = self.buffer.push(c);

            let mut i = self.buffer.len() - 1;
            while i as u8 > self.cursor {
                self.buffer.swap(i, i - 1);
                i -= 1;
            }

            self.cursor += 1;
        }

    }

    /// Remove and shift bytes in the buffer. Does not move cursor.
    pub fn remove(&mut self, index: usize) -> Option<u8> {
        if index >= self.buffer.len() {
            return None
        }
        // Move the index item to the end
        for i in index..self.buffer.len() - 1 {
            self.buffer.swap(i, i+1)
        }

        self.buffer.pop()
    }

    /// In place remove byte. Cursor stays at the same position.
    /// If cursor is at the end, delete does nothing.
    pub fn delete(&mut self) {
        self.remove(self.cursor as usize);
    }

    /// Moves cursor back one while removing byte that is behind cursor.
    /// If Cursor is at beginning of buffer, then backspace does nothing.
    pub fn backspace(&mut self) {
        if self.cursor > 0 {
            self.remove(self.cursor as usize - 1);
            self.cursor -= 1;
        }
    }

    pub fn remove_current(&mut self) -> Option<u8> {
        
        self.remove((self.cursor as usize - 1).clamp(0, self.buffer.len() -1))
    }

    pub fn move_cursor_left(&mut self, amount: u8) -> u8 {
        self.cursor = (self.cursor - amount).clamp(0, self.buffer.len() as u8-1);
        self.cursor
    }

    pub fn move_cursor_right(&mut self, amount: u8) -> u8 {
        self.cursor = (self.cursor + amount).clamp(0, self.buffer.len() as u8-1);
        self.cursor
    }

    pub fn cursor_position(&self) -> u8 {
        self.cursor
    }

    /// Flushes contents of buffer into an array. This will clear the buffer,
    /// and reset the cursor to 0.
    /// The returned array 
    pub fn flush(&mut self) -> ([u8; BUFFER_SIZE], u8) {
        let mut a = [0; BUFFER_SIZE];
        let length = self.buffer.len();
        // Copy contents over. Everything leftover will be 0 in the output
        for i in 0..length {
            a[i] = self.buffer[i];
        }
        self.buffer.clear();
        self.cursor = 0;

        (a, length as u8)
    }

    /// Read the next character in the buffer, starting at position 0, moving everything else down
    /// This removes that character from the buffer
    pub fn read_next(&mut self) -> Option<u8> {
        self.remove(0)
    }

    pub fn read_char(&mut self) -> Option<u8> {
        self.out.pop_back()
    }

    pub fn has_output(&self) -> bool {
        self.out.len() > 0
    }

    pub fn slice(&self) -> &[u8] {
        self.buffer.as_slice()
    }

    pub fn len(&self) -> u8 {
        self.buffer.len() as u8
    }

    // fn swap(&mut self, a: u8, b: u8) {
    //     let a = a.clamp(0, self.buffer.len() as u8) as usize;
    //     let b = b.clamp(0, self.buffer.len() as u8) as usize;

    //     let temp = self.buffer[a];
    //     self.buffer[a] = self.buffer[b];
    //     self.buffer[b] = temp;
    // }
}