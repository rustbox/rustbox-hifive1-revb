
pub const BUFFER_SIZE: usize = 256;

use heapless::{Deque, Vec};

/// LineBuffer represents the set of characters that are present in an active
/// line on a Terminal. Where a character will be placed next depnds on the
/// location of the `cursor` in the line.
/// 
/// This contains a buffer that is actively modified when adding characters.
/// When `\n` is pushed, by default the buffer is flushed and the contents are
/// placed in the `out` queue. This can then be dequeued from to retreive input.
pub struct LineBuffer {
    buffer: Vec<u8, BUFFER_SIZE>,
    /// Cursor is the position in the buffer where the next pushed character is about to go
    cursor: u8,
    out: Deque<u8, BUFFER_SIZE>,
}


impl LineBuffer {

    /// Create a new empty Buffer with the cursor set to the column 0
    pub fn new() -> LineBuffer {
        LineBuffer {
            buffer: Vec::new(),
            cursor: 0,
            out: Deque::new(),
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
            // Only flush if we have contents
            if self.len() > 0 {
                let (contents, length) = self.flush();
                for i  in 0..length as usize {
                    let _ = self.out.push_front(contents[i]);
                }
                let _ = self.out.push_front('\n' as u8);
            }
        } else {
            self.cursor = self.cursor.clamp(0, self.buffer.len() as u8);
            // add character to the end of the buffer
            let _ = self.buffer.push(c);
            debug_assert!(self.buffer.len() > 0);

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
    pub fn delete(&mut self) -> Option<u8> {
        self.remove(self.cursor as usize)
    }

    /// Moves cursor back one while removing byte that is behind cursor.
    /// If Cursor is at beginning of buffer, then backspace does nothing.
    pub fn backspace(&mut self) -> Option<u8> {
        if self.cursor > 0 {
            self.cursor -= 1;
            self.remove(self.cursor as usize)
        } else {
            None
        }
    }

    pub fn remove_current(&mut self) -> Option<u8> {
        
        self.remove((self.cursor as usize - 1).clamp(0, self.buffer.len() -1))
    }

    /// Moves the cursor to the left by `amount`, bounded by the beginning of the buffer.
    pub fn move_cursor_left(&mut self, amount: u8) -> u8 {
        let amt = amount.clamp(0, self.cursor);
        self.cursor -= amt;
        amt
    }

    /// Moves the cursor to the right by `amount`, bounded the length of the buffer.
    pub fn move_cursor_right(&mut self, amount: u8) -> u8 {
        let right_bound = if self.buffer.len() > 0 {
            self.buffer.len()
        } else {
            0
        };
        debug_assert!(right_bound as u8 >= self.cursor, "The `right_bound` {} < `cursor` {}; buffer len is {}", right_bound, self.cursor, self.buffer.len());
        let max = right_bound as u8 - self.cursor;
        let amt = amount.clamp(0, max);
        self.cursor += amt;
        amt
    }

    /// The current position of the cursor
    pub fn cursor_position(&self) -> u8 {
        self.cursor
    }

    /// Flushes contents of buffer into an array. This will clear the buffer,
    /// and reset the cursor to 0.
    /// This returns the the buffer as an array with `BUFFER_SIZE` as well as
    /// the number of bytes flushed.
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

    /// Read the next byte from the `out` queue
    pub fn read_char(&mut self) -> Option<u8> {
        self.out.pop_back()
    }

    /// True if the output queue has bytes to be read.
    pub fn has_output(&self) -> bool {
        self.out.len() > 0
    }

    /// Borrow the contents of the buffer as a slice of bytes
    pub fn slice(&self) -> &[u8] {
        self.buffer.as_slice()
    }

    /// Borrow the contents of the buffer starting at the position of the cursor
    pub fn slice_from_cursor(&self) -> &[u8] {
        &self.buffer[self.cursor as usize..self.buffer.len()]
    }

    /// Current length of of the buffer
    pub fn len(&self) -> u8 {
        self.buffer.len() as u8
    }
}