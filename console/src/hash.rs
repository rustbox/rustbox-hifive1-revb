
use core::hash::{BuildHasher, Hasher};
use core::num::Wrapping;

pub struct SDBMHasher(Wrapping<u32>);

impl Hasher for SDBMHasher {
    #[inline]
    fn finish(&self) -> u64 {
        (self.0).0 as u64
    }

    #[inline]
    fn write(&mut self, bytes: &[u8]) {
        for byte in bytes.iter() {
            self.0 = Wrapping(*byte as u32) + (self.0 << 6) + (self.0 << 16) - self.0;
        }
    }
}

impl Default for SDBMHasher {
    fn default() -> SDBMHasher {
        SDBMHasher(Wrapping(0))
    }
}

impl BuildHasher for SDBMHasher {
    type Hasher = SDBMHasher;

    fn build_hasher(&self) -> Self::Hasher {
        SDBMHasher::default()
    }
}

