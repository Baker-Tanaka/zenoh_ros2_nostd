//! WASI monotonic clock utilities.
//!
//! Provides time-related helpers using the WASI `clock_time_get` syscall
//! as the time source. This is used for keepalive timing and timer
//! callbacks when running on a WASI host.

/// WASI clock ID for the monotonic clock.
const CLOCKID_MONOTONIC: u32 = 1;

#[cfg(target_os = "wasi")]
#[link(wasm_import_module = "wasi_snapshot_preview1")]
extern "C" {
    fn clock_time_get(id: u32, precision: u64, time: *mut u64) -> u16;
}

/// Get the current monotonic time in nanoseconds.
///
/// Uses WASI `clock_time_get(CLOCKID_MONOTONIC, 0)`.
/// Returns 0 if the syscall fails.
pub fn monotonic_ns() -> u64 {
    #[cfg(target_os = "wasi")]
    {
        let mut time: u64 = 0;
        let rc = unsafe { clock_time_get(CLOCKID_MONOTONIC, 0, &mut time) };
        if rc == 0 {
            time
        } else {
            0
        }
    }
    #[cfg(not(target_os = "wasi"))]
    {
        0
    }
}
