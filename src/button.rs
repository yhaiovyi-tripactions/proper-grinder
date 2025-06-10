//! Interrupt-Driven Button Handler with Debouncing
//!
//! This module provides a truly interrupt-driven button handler using EXTI interrupts
//! combined with timer-based debouncing. No polling loops in main tasks.
//!
//! # Features
//! - Hardware interrupt-driven (EXTI) for immediate response
//! - Timer-based debouncing to eliminate button bounce
//! - Async channel communication between ISR and tasks
//! - Support for both active-HIGH and active-LOW buttons
//! - Zero CPU overhead when button not pressed
//!
//! # Architecture
//! 1. GPIO pin configured for EXTI interrupt on both edges
//! 2. Interrupt handler starts debounce timer and records raw state
//! 3. Timer callback checks if state is stable and sends final event
//! 4. Async task receives only stable, debounced events

use core::{
    clone::Clone,
    cmp::{Eq, PartialEq},
    fmt::Debug,
    marker::Copy,
};

/// Button events after debouncing
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ButtonEvent {
    /// Button was pressed (stable transition to pressed state)
    Press,
    /// Button was released (stable transition to released state)
    Release,
}

/// Button states for debouncing
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ButtonState {
    /// Button is not pressed
    Released,
    /// Button is pressed
    Pressed,
}

/// Interrupt-driven button with timer-based debouncing
/// 
/// This handles the complete debouncing flow:
/// 1. EXTI interrupt detects any edge
/// 2. Debounce timer starts
/// 3. After debounce delay, final state is checked and event sent
pub struct InterruptButton {
    /// Current stable state of the button
    stable_state: ButtonState,
    /// State recorded during last interrupt
    pending_state: ButtonState,
    /// Whether button is active low (pullup) or active high (pulldown)
    active_low: bool,
    /// Debounce time in milliseconds
    debounce_time_ms: u32,
    /// Whether debounce timer is currently running
    debounce_pending: bool,
}

impl InterruptButton {
    /// Create new interrupt button for active-low (pullup resistor) configuration
    pub fn new_active_low(debounce_time_ms: u32) -> Self {
        Self {
            stable_state: ButtonState::Released,
            pending_state: ButtonState::Released,
            active_low: true,
            debounce_time_ms,
            debounce_pending: false,
        }
    }

    /// Create new interrupt button for active-high (pulldown resistor) configuration  
    pub fn new_active_high(debounce_time_ms: u32) -> Self {
        Self {
            stable_state: ButtonState::Released,
            pending_state: ButtonState::Released,
            active_low: false,
            debounce_time_ms,
            debounce_pending: false,
        }
    }

    /// Convert raw GPIO pin state to logical button state
    fn pin_to_button_state(&self, pin_high: bool) -> ButtonState {
        if self.active_low {
            if pin_high {
                ButtonState::Released
            } else {
                ButtonState::Pressed
            }
        } else if pin_high {
            ButtonState::Pressed
        } else {
            ButtonState::Released
        }
    }

    /// Handle GPIO interrupt - should be called from EXTI interrupt handler
    /// 
    /// Returns true if debounce timer should be started
    pub fn handle_interrupt(&mut self, pin_high: bool) -> bool {
        let new_state = self.pin_to_button_state(pin_high);
        
        // Only start debounce if state changed and we're not already debouncing
        if new_state != self.stable_state && !self.debounce_pending {
            self.pending_state = new_state;
            self.debounce_pending = true;
            true // Start debounce timer
        } else {
            false // Ignore spurious interrupts during debounce
        }
    }

    /// Handle debounce timer completion
    /// 
    /// Should be called when debounce timer expires.
    /// Returns Some(event) if button state change is confirmed.
    pub fn handle_debounce_complete(&mut self, current_pin_high: bool) -> Option<ButtonEvent> {
        self.debounce_pending = false;
        
        let current_state = self.pin_to_button_state(current_pin_high);
        
        // Check if state is stable (matches what we saw during interrupt)
        if current_state == self.pending_state && current_state != self.stable_state {
            let event = match current_state {
                ButtonState::Pressed => ButtonEvent::Press,
                ButtonState::Released => ButtonEvent::Release,
            };
            
            self.stable_state = current_state;
            Some(event)
        } else {
            // State changed during debounce - ignore this transition
            None
        }
    }

    /// Get debounce time in milliseconds
    pub fn debounce_time_ms(&self) -> u32 {
        self.debounce_time_ms
    }

    /// Check if debounce timer is currently running
    pub fn is_debounce_pending(&self) -> bool {
        self.debounce_pending
    }

    /// Get current stable button state
    pub fn stable_state(&self) -> ButtonState {
        self.stable_state
    }
}

/// Helper function to send button events from interrupt context
/// 
/// This should be called from the EXTI interrupt handler
pub fn send_button_event(event: ButtonEvent) -> Result<(), ButtonEvent> {
    // This will be implemented in main.rs using the specific channel
    // The interrupt handler will call this function
    Err(event) // Placeholder - actual implementation in main.rs
}

/// Helper trait for GPIO pins that can be used as interrupt sources
pub trait InterruptPin {
    /// Configure the pin as an interrupt source
    fn setup_interrupt(&mut self);
    
    /// Clear pending interrupt flag
    fn clear_interrupt(&mut self);
    
    /// Check if this pin caused the interrupt
    fn check_interrupt(&self) -> bool;
}

