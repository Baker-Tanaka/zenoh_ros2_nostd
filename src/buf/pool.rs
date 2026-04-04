//! Fixed-size buffer pool using `heapless` collections.
//!
//! Provides pre-allocated, reusable byte buffers to avoid heap allocation
//! in the hot path (publish/subscribe).

use heapless::Vec;

/// A fixed-capacity byte buffer.
///
/// `N` is the maximum payload size in bytes.
pub type Buffer<const N: usize> = Vec<u8, N>;

/// A pool of reusable byte buffers.
///
/// - `N`: capacity of each individual buffer (bytes).
/// - `COUNT`: number of buffers in the pool.
///
/// Buffers are checked out for use and returned when done.
pub struct BufferPool<const N: usize, const COUNT: usize> {
    buffers: [Option<Vec<u8, N>>; COUNT],
}

impl<const N: usize, const COUNT: usize> BufferPool<N, COUNT> {
    const NONE: Option<Vec<u8, N>> = None;

    /// Create a new buffer pool with all buffers available.
    pub fn new() -> Self {
        let mut buffers = [Self::NONE; COUNT];
        for slot in buffers.iter_mut() {
            *slot = Some(Vec::new());
        }
        Self { buffers }
    }

    /// Acquire a buffer from the pool.
    ///
    /// Returns `None` if all buffers are checked out.
    pub fn acquire(&mut self) -> Option<Vec<u8, N>> {
        for slot in self.buffers.iter_mut() {
            if slot.is_some() {
                return slot.take();
            }
        }
        None
    }

    /// Return a buffer to the pool.
    ///
    /// The buffer is cleared before being placed back.
    /// Returns `Err` if the pool is already full (logic error).
    pub fn release(&mut self, mut buf: Vec<u8, N>) -> Result<(), Vec<u8, N>> {
        buf.clear();
        for slot in self.buffers.iter_mut() {
            if slot.is_none() {
                *slot = Some(buf);
                return Ok(());
            }
        }
        Err(buf)
    }

    /// Number of available (not checked out) buffers.
    pub fn available(&self) -> usize {
        self.buffers.iter().filter(|s| s.is_some()).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool_acquire_release() {
        let mut pool: BufferPool<128, 4> = BufferPool::new();
        assert_eq!(pool.available(), 4);

        let buf1 = pool.acquire().unwrap();
        assert_eq!(pool.available(), 3);

        let buf2 = pool.acquire().unwrap();
        assert_eq!(pool.available(), 2);

        pool.release(buf1).unwrap();
        assert_eq!(pool.available(), 3);

        pool.release(buf2).unwrap();
        assert_eq!(pool.available(), 4);
    }

    #[test]
    fn test_pool_exhaustion() {
        let mut pool: BufferPool<64, 2> = BufferPool::new();

        let _b1 = pool.acquire().unwrap();
        let _b2 = pool.acquire().unwrap();
        assert!(pool.acquire().is_none());
    }

    #[test]
    fn test_release_clears_buffer() {
        let mut pool: BufferPool<64, 2> = BufferPool::new();

        let mut buf = pool.acquire().unwrap();
        buf.extend_from_slice(&[1, 2, 3]).unwrap();
        assert_eq!(buf.len(), 3);

        pool.release(buf).unwrap();

        // Re-acquired buffer should be empty
        let buf = pool.acquire().unwrap();
        assert_eq!(buf.len(), 0);
    }

    #[test]
    fn test_single_buffer_pool() {
        let mut pool: BufferPool<32, 1> = BufferPool::new();
        assert_eq!(pool.available(), 1);

        let buf = pool.acquire().unwrap();
        assert_eq!(pool.available(), 0);
        assert!(pool.acquire().is_none());

        pool.release(buf).unwrap();
        assert_eq!(pool.available(), 1);
    }

    #[test]
    fn test_buffer_capacity() {
        let mut pool: BufferPool<16, 1> = BufferPool::new();
        let mut buf = pool.acquire().unwrap();

        // Should be able to fill up to capacity
        let data = [0xAA; 16];
        buf.extend_from_slice(&data).unwrap();
        assert_eq!(buf.len(), 16);

        // Should fail to exceed capacity
        assert!(buf.push(0xFF).is_err());
    }
}
