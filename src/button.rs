//! Reusable Interrupt-Driven Button Handler for RTIC
//!
//! This module provides a production-ready, interrupt-driven button handler specifically
//! designed for RTIC (Real-Time Interrupt-driven Concurrency) applications. It eliminates
//! all polling loops and provides first-party RTIC resource sharing support.
//!
//! # Key Features
//! 
//! - **🚫 Zero Polling** - Pure interrupt-driven architecture with no loops
//! - **⚡ RTIC Native** - Designed for RTIC shared resources and task spawning patterns
//! - **🔄 Software Debouncing** - Eliminates mechanical switch bounce with configurable timing
//! - **🎯 Event-Driven** - Tasks spawn only when needed, then exit immediately
//! - **🧩 Hardware Agnostic** - Accepts pin states as parameters, no direct HAL dependencies
//! - **⚙️ Dual Configuration** - Supports both active-high and active-low button configurations
//!
//! # RTIC Integration Pattern
//!
//! This module follows the RTIC pattern of:
//! 1. **EXTI Interrupt Handler** → calls `handle_button_interrupt()` → spawns debounce task
//! 2. **Debounce Task** → waits, then calls `handle_button_debounce()` → spawns action task  
//! 3. **Action Task** → calls `handle_button_event()` → executes final action → exits
//!
//! # Usage Example
//!
//! ```rust,no_run
//! use button::{InterruptButton, handle_button_interrupt, handle_button_debounce, handle_button_event};
//!
//! // In RTIC shared resources
//! #[shared]
//! struct Shared {
//!     button_handler: InterruptButton,
//!     button_pin: SomeGpioPin,
//! }
//!
//! // EXTI interrupt handler
//! #[task(binds = EXTI2, shared = [button_handler, button_pin])]
//! fn button_interrupt(mut ctx: button_interrupt::Context) {
//!     let pin_state = ctx.shared.button_pin.lock(|pin| {
//!         pin.clear_interrupt_pending_bit();
//!         pin.is_high()
//!     });
//!     
//!     let should_debounce = ctx.shared.button_handler.lock(|handler| {
//!         handle_button_interrupt(handler, pin_state)
//!     });
//!     
//!     if let Some(pin_state) = should_debounce {
//!         debounce_task::spawn(pin_state).ok();
//!     }
//! }
//! ```

use core::{
    clone::Clone,
    cmp::{Eq, PartialEq},
    fmt::Debug,
    marker::Copy,
};

/// Button events after successful debouncing
/// 
/// These events represent stable, confirmed button state transitions
/// that have passed the debounce filter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ButtonEvent {
    /// Button was pressed (stable transition to pressed state)
    /// 
    /// This event is generated when the button state has been stable
    /// in the pressed position for the full debounce period.
    Press,
    
    /// Button was released (stable transition to released state)
    /// 
    /// This event is generated when the button state has been stable
    /// in the released position for the full debounce period.
    Release,
}

/// Internal button states for debouncing state machine
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ButtonState {
    /// Button is in released state
    Released,
    /// Button is in pressed state  
    Pressed,
}

/// Actions that can result from button events
/// 
/// This enum provides common action types that applications might want to take
/// in response to button events. For more complex behavior, implement a custom
/// ButtonEventHandler in your application.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ButtonAction {
    /// Toggle some application state
    /// 
    /// This generic action indicates the application should toggle something
    /// (LED, motor, state machine, etc.) in response to the button press.
    Toggle,
    
    /// No action required
    /// 
    /// This indicates the event was processed but requires no
    /// application-level response.
    None,
}

/// RTIC-native interrupt-driven button handler with software debouncing
/// 
/// This struct manages the complete button debouncing state machine and is designed
/// to be stored in RTIC shared resources. It provides all the state needed for
/// proper interrupt-driven button handling without any polling loops.
/// 
/// # Thread Safety
/// 
/// This struct is designed to be used with RTIC's resource sharing mechanisms.
/// All mutations should be done within RTIC resource locks to ensure thread safety
/// in the interrupt-driven environment.
/// 
/// # Real-Time Constraints
/// 
/// All methods on this struct are designed to execute quickly and deterministically,
/// making them suitable for use in interrupt context and real-time tasks.
pub struct InterruptButton {
    /// Current stable state of the button
    stable_state: ButtonState,
    /// State recorded during the most recent interrupt
    pending_state: ButtonState,
    /// Button configuration: true for active-low (pullup), false for active-high (pulldown)
    active_low: bool,
    /// Debounce time in milliseconds
    debounce_time_ms: u32,
    /// Whether a debounce operation is currently in progress
    debounce_pending: bool,
}

impl InterruptButton {
    /// Create new interrupt button for active-low configuration (pullup resistor)
    /// 
    /// This configuration is typical for buttons connected between GPIO and GND,
    /// with an internal or external pullup resistor. In this configuration:
    /// - Button released: GPIO reads HIGH (pulled up)
    /// - Button pressed: GPIO reads LOW (pulled to GND)
    /// 
    /// # Parameters
    /// 
    /// * `debounce_time_ms` - Debounce delay in milliseconds (typically 10-50ms)
    /// 
    /// # Examples
    /// 
    /// ```rust,no_run
    /// let button = InterruptButton::new_active_low(20); // 20ms debounce
    /// ```
    pub fn new_active_low(debounce_time_ms: u32) -> Self {
        Self {
            stable_state: ButtonState::Released,
            pending_state: ButtonState::Released,
            active_low: true,
            debounce_time_ms,
            debounce_pending: false,
        }
    }

    /// Create new interrupt button for active-high configuration (pulldown resistor)
    /// 
    /// This configuration is typical for buttons connected between GPIO and VCC,
    /// with an internal or external pulldown resistor. In this configuration:
    /// - Button released: GPIO reads LOW (pulled down)
    /// - Button pressed: GPIO reads HIGH (pulled to VCC)
    /// 
    /// # Parameters
    /// 
    /// * `debounce_time_ms` - Debounce delay in milliseconds (typically 10-50ms)
    /// 
    /// # Examples
    /// 
    /// ```rust,no_run
    /// let button = InterruptButton::new_active_high(20); // 20ms debounce
    /// ```
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
    /// 
    /// This method handles the active-high vs active-low logic transparently,
    /// always returning logical button states regardless of hardware configuration.
    fn pin_to_button_state(&self, pin_high: bool) -> ButtonState {
        if self.active_low {
            // Active-low: HIGH = released, LOW = pressed
            if pin_high {
                ButtonState::Released
            } else {
                ButtonState::Pressed
            }
        } else {
            // Active-high: HIGH = pressed, LOW = released
            if pin_high {
                ButtonState::Pressed
            } else {
                ButtonState::Released
            }
        }
    }

    /// Handle GPIO interrupt in RTIC interrupt context
    /// 
    /// This method should be called from your EXTI interrupt handler within an RTIC
    /// resource lock. It processes the interrupt and determines if a debounce task
    /// should be spawned.
    /// 
    /// # Parameters
    /// 
    /// * `pin_high` - Current GPIO pin state (true = HIGH, false = LOW)
    /// 
    /// # Returns
    /// 
    /// * `true` - A debounce task should be spawned
    /// * `false` - Interrupt should be ignored (already debouncing or no state change)
    /// 
    /// # RTIC Usage Pattern
    /// 
    /// ```rust,no_run
    /// #[task(binds = EXTI2, shared = [button_handler, button_pin])]
    /// fn button_interrupt(mut ctx: button_interrupt::Context) {
    ///     let pin_state = ctx.shared.button_pin.lock(|pin| pin.is_high());
    ///     let should_debounce = ctx.shared.button_handler.lock(|handler| {
    ///         handler.handle_interrupt(pin_state)
    ///     });
    ///     if should_debounce {
    ///         debounce_task::spawn(pin_state).ok();
    ///     }
    /// }
    /// ```
    pub fn handle_interrupt(&mut self, pin_high: bool) -> bool {
        let new_state = self.pin_to_button_state(pin_high);
        
        // Only start debounce if state changed and we're not already debouncing
        if new_state != self.stable_state && !self.debounce_pending {
            self.pending_state = new_state;
            self.debounce_pending = true;
            true // Should spawn debounce task
        } else {
            false // Ignore spurious interrupts or already debouncing
        }
    }

    /// Handle debounce completion in RTIC async task context
    /// 
    /// This method should be called from your RTIC debounce task after the debounce
    /// delay period has elapsed. It verifies the button state is still stable and
    /// generates the final button event.
    /// 
    /// # Parameters
    /// 
    /// * `current_pin_high` - Current GPIO pin state after debounce delay
    /// 
    /// # Returns
    /// 
    /// * `Some(ButtonEvent)` - Stable button event that should trigger an action
    /// * `None` - Button state was not stable during debounce period
    /// 
    /// # RTIC Usage Pattern
    /// 
    /// ```rust,no_run
    /// #[task(shared = [button_handler, button_pin])]
    /// async fn debounce_task(mut ctx: debounce_task::Context, interrupt_pin_state: bool) {
    ///     let debounce_ms = ctx.shared.button_handler.lock(|h| h.debounce_time_ms());
    ///     Mono::delay(debounce_ms.millis()).await;
    ///     
    ///     let current_state = ctx.shared.button_pin.lock(|pin| pin.is_high());
    ///     let event = ctx.shared.button_handler.lock(|handler| {
    ///         handler.handle_debounce_complete(current_state)
    ///     });
    ///     
    ///     if let Some(event) = event {
    ///         action_task::spawn(event).ok();
    ///     }
    /// }
    /// ```
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

    /// Get the configured debounce time in milliseconds
    /// 
    /// This is useful for RTIC tasks that need to know how long to delay
    /// for the debounce period.
    pub fn debounce_time_ms(&self) -> u32 {
        self.debounce_time_ms
    }

    /// Check if a debounce operation is currently in progress
    /// 
    /// This can be useful for diagnostics or ensuring proper state machine operation.
    pub fn is_debounce_pending(&self) -> bool {
        self.debounce_pending
    }

    /// Get the current stable button state
    /// 
    /// This returns the last confirmed stable button state, which may be different
    /// from the current pin state if a debounce operation is in progress.
    pub fn stable_state(&self) -> ButtonState {
        self.stable_state
    }
}

/// High-level RTIC button interrupt handler
/// 
/// This function encapsulates the button interrupt logic and should be called
/// from your EXTI interrupt handler within an RTIC resource lock. It provides
/// a clean, hardware-agnostic interface for processing button interrupts.
/// 
/// # Parameters
/// 
/// * `button_handler` - Mutable reference to the button handler (from RTIC shared resource)
/// * `pin_high` - Current GPIO pin state (hardware-agnostic boolean)
/// 
/// # Returns
/// 
/// * `Some(pin_state)` - Debounce task should be spawned with this pin state
/// * `None` - Interrupt should be ignored
/// 
/// # Real-Time Characteristics
/// 
/// This function executes in O(1) time and is suitable for interrupt context.
/// It performs minimal work to maintain real-time responsiveness.
pub fn handle_button_interrupt(
    button_handler: &mut InterruptButton,
    pin_high: bool,
) -> Option<bool> {
    if button_handler.handle_interrupt(pin_high) {
        Some(pin_high) // Return pin state for debounce task
    } else {
        None
    }
}

/// High-level RTIC button debounce handler
/// 
/// This function should be called from your RTIC debounce task after the
/// debounce delay period. It verifies button state stability and generates
/// the final button event if the state is confirmed stable.
/// 
/// # Parameters
/// 
/// * `button_handler` - Mutable reference to the button handler (from RTIC shared resource)
/// * `current_pin_high` - Current GPIO pin state after debounce delay
/// * `_interrupt_pin_state` - Original pin state from interrupt (unused but kept for API consistency)
/// 
/// # Returns
/// 
/// * `Some(ButtonEvent)` - Confirmed button event ready for action processing
/// * `None` - Button state was not stable during debounce
/// 
/// # Real-Time Characteristics
/// 
/// This function executes in O(1) time and is suitable for RTIC task context.
pub fn handle_button_debounce(
    button_handler: &mut InterruptButton,
    current_pin_high: bool,
    _interrupt_pin_state: bool,
) -> Option<ButtonEvent> {
    button_handler.handle_debounce_complete(current_pin_high)
}

/// Default button event handler for simple toggle applications
/// 
/// This provides a reasonable default behavior for simple applications
/// that want to toggle something on button press.
pub struct DefaultButtonHandler;

impl DefaultButtonHandler {
    fn handle_event(&self, event: ButtonEvent) -> ButtonAction {
        match event {
            ButtonEvent::Press => ButtonAction::Toggle,
            ButtonEvent::Release => ButtonAction::None,
        }
    }
}

/// High-level RTIC button event handler
/// 
/// This function converts button events into application-specific actions.
/// It provides a clean separation between button logic and application logic
/// and should be called from your RTIC action task.
/// 
/// # Parameters
/// 
/// * `event` - Confirmed button event from the debounce process
/// 
/// # Returns
/// 
/// * `ButtonAction` - Application-specific action to be executed
/// 
/// # Real-Time Characteristics
/// 
/// This function executes in O(1) time and is suitable for RTIC task context.
pub fn handle_button_event(event: ButtonEvent) -> ButtonAction {
    let default_handler = DefaultButtonHandler;
    default_handler.handle_event(event)
}

