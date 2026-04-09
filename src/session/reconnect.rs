//! Reconnection policy with exponential backoff.

use embassy_time::{Duration, Timer};

/// Reconnection policy with configurable exponential backoff.
#[derive(Clone, Copy)]
pub struct ReconnectPolicy {
    /// Initial delay before the first reconnection attempt.
    pub initial_delay_ms: u64,
    /// Maximum delay between reconnection attempts.
    pub max_delay_ms: u64,
    /// Multiplier applied to the delay after each failed attempt.
    pub backoff_factor: u32,
    /// Maximum number of reconnection attempts (0 = unlimited).
    pub max_attempts: u32,
    // Internal state
    current_delay_ms: u64,
    attempt_count: u32,
}

impl ReconnectPolicy {
    /// Create a new reconnect policy.
    pub fn new(initial_delay_ms: u64, max_delay_ms: u64) -> Self {
        Self {
            initial_delay_ms,
            max_delay_ms,
            backoff_factor: 2,
            max_attempts: 0,
            current_delay_ms: initial_delay_ms,
            attempt_count: 0,
        }
    }

    /// Default policy: 1s initial, 30s max, factor 2, unlimited attempts.
    pub fn default_policy() -> Self {
        Self::new(1_000, 30_000)
    }

    /// Reset the backoff state (call after a successful connection).
    pub fn reset(&mut self) {
        self.current_delay_ms = self.initial_delay_ms;
        self.attempt_count = 0;
    }

    /// Check if we should attempt another reconnection.
    pub fn should_retry(&self) -> bool {
        self.max_attempts == 0 || self.attempt_count < self.max_attempts
    }

    /// Wait for the current backoff delay, then advance the state.
    pub async fn wait_and_advance(&mut self) {
        Timer::after(Duration::from_millis(self.current_delay_ms)).await;
        self.attempt_count += 1;
        self.current_delay_ms =
            (self.current_delay_ms * self.backoff_factor as u64).min(self.max_delay_ms);
    }

    /// Current attempt number (0-based).
    pub fn attempt(&self) -> u32 {
        self.attempt_count
    }
}
