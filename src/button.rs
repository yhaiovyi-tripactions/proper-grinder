//! Generic Software Button Debounce Module
//! 
//! This module provides a platform-agnostic button debouncer that works with any GPIO pin
//! implementing `embedded-hal::digital::InputPin` and any RTIC monotonic timer.
//! 
//! # Features
//! - Software debouncing with configurable timeout
//! - Support for both active-HIGH and active-LOW buttons
//! - Generic over GPIO pin types and monotonic timers
//! - Async/await support with RTIC 2.x
//! - Event-driven API (Press/Release events)
//! 
//! # Example
//! ```rust,no_run
//! let button_pin = gpiob.pb2.into_pull_up_input();
//! let mut button = DebouncedButton::new_active_high(button_pin, 20);
//! 
//! // In an async task:
//! if let Some(event) = button.check_event::<Mono>().await {
//!     match event {
//!         ButtonEvent::Press => println!("Button pressed!"),
//!         ButtonEvent::Release => println!("Button released!"),
//!     }
//! }
//! ```

use core::{
    option::Option::{self, Some, None},
    result::Result::{Ok, Err},
    convert::From,
    cmp::{PartialEq, Eq},
    fmt::Debug,
    clone::Clone,
    marker::Copy,
};
use embedded_hal::digital::InputPin;
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit;

/// Button debounce states
pub enum ButtonState {
    /// Button is not pressed
    Released,
    /// Button is pressed  
    Pressed,
}

impl PartialEq for ButtonState {
    fn eq(&self, other: &Self) -> bool {
        core::mem::discriminant(self) == core::mem::discriminant(other)
    }
}

impl Eq for ButtonState {}

impl Clone for ButtonState {
    fn clone(&self) -> Self {
        *self
    }
}

impl Copy for ButtonState {}

impl Debug for ButtonState {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ButtonState::Released => f.write_str("Released"),
            ButtonState::Pressed => f.write_str("Pressed"),
        }
    }
}

/// Button events after debouncing
pub enum ButtonEvent {
    /// Button was just pressed (transition from Released to Pressed)
    Press,
    /// Button was just released (transition from Pressed to Released)
    Release,
}

impl PartialEq for ButtonEvent {
    fn eq(&self, other: &Self) -> bool {
        core::mem::discriminant(self) == core::mem::discriminant(other)
    }
}

impl Eq for ButtonEvent {}

impl Clone for ButtonEvent {
    fn clone(&self) -> Self {
        *self
    }
}

impl Copy for ButtonEvent {}

impl Debug for ButtonEvent {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ButtonEvent::Press => f.write_str("Press"),
            ButtonEvent::Release => f.write_str("Release"),
        }
    }
}

/// Software button debouncer with RTIC async support
/// Generic over both PIN type and MONOTONIC timer
pub struct DebouncedButton<PIN> {
    pin: PIN,
    last_state: ButtonState,
    stable_state: ButtonState,
    debounce_time_ms: u32,
    active_low: bool,
}

impl<PIN> DebouncedButton<PIN>
where
    PIN: InputPin,
{
    /// Create a new debounced button (active low - pullup resistor)
    pub fn new(pin: PIN, debounce_time_ms: u32) -> Self {
        Self {
            pin,
            last_state: ButtonState::Released,
            stable_state: ButtonState::Released,
            debounce_time_ms,
            active_low: true,
        }
    }

    /// Create a new debounced button (active high - pulldown resistor)
    pub fn new_active_high(pin: PIN, debounce_time_ms: u32) -> Self {
        Self {
            pin,
            last_state: ButtonState::Released,
            stable_state: ButtonState::Released,
            debounce_time_ms,
            active_low: false,
        }
    }

    /// Read the current raw button state
    fn read_raw_state(&mut self) -> ButtonState {
        match self.pin.is_high() {
            Ok(high) => {
                if self.active_low {
                    if high { ButtonState::Released } else { ButtonState::Pressed }
                } else if high { ButtonState::Pressed } else { ButtonState::Released }
            }
            Err(_) => self.last_state,
        }
    }

    /// Check for button events with debouncing
    /// Generic over any RTIC monotonic timer
    pub async fn check_event<MONO>(&mut self) -> Option<ButtonEvent>
    where
        MONO: Monotonic,
        MONO::Duration: From<fugit::MillisDurationU32>,
    {
        let current_raw = self.read_raw_state();
        
        if current_raw != self.last_state {
            self.last_state = current_raw;
            
            // Generic delay that works with any monotonic
            let delay = MONO::Duration::from(fugit::MillisDurationU32::millis(self.debounce_time_ms));
            MONO::delay(delay).await;
            
            let debounced_state = self.read_raw_state();
            
            if debounced_state == current_raw && debounced_state != self.stable_state {
                let event = match debounced_state {
                    ButtonState::Pressed => Some(ButtonEvent::Press),
                    ButtonState::Released => Some(ButtonEvent::Release),
                };
                
                self.stable_state = debounced_state;
                return event;
            }
        }
        
        self.last_state = current_raw;
        None
    }
} 