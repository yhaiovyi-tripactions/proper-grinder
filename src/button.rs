//! Embassy Async Button Handler
//!
//! This module provides a production-ready, async button handler specifically
//! designed for Embassy async/await applications. It uses Embassy's EXTI (External Interrupt)
//! support for efficient async GPIO operations.
//!
//! # Key Features
//! 
//! - **🚫 Zero Polling** - Pure async/await architecture with Embassy's EXTI support
//! - **⚡ Embassy Native** - Designed for Embassy's async executor and task patterns
//! - **🔄 Software Debouncing** - Eliminates mechanical switch bounce with Embassy Timer
//! - **🎯 Event-Driven** - Uses Embassy Channels for clean async communication
//! - **🧩 Hardware Agnostic** - Works with any Embassy-compatible GPIO pin
//! - **⚙️ Dual Configuration** - Supports both active-high and active-low button configurations
//!
//! # Embassy Integration Pattern
//!
//! This module follows Embassy's async pattern:
//! 1. **Button Task** → waits for pin edges with `wait_for_any_edge()` → sends to channel
//! 2. **Debounce Logic** → uses `Timer::after()` for delay → validates state
//! 3. **Event Processing** → receives from channel → processes button events
//!
//! # Usage Example
//!
//! ```rust,no_run
//! use embassy_sync::channel::Channel;
//! use embassy_sync::blocking_mutex::raw::NoopRawMutex;
//! use button::{ButtonEvent, ButtonAction, button_task, handle_button_event};
//!
//! // Create a channel for button events
//! static BUTTON_CHANNEL: Channel<NoopRawMutex, ButtonEvent, 4> = Channel::new();
//!
//! #[embassy_executor::main]
//! async fn main(spawner: Spawner) {
//!     let p = embassy_stm32::init(Default::default());
//!     let button = Input::new(p.PB2, Pull::Up);
//!     let button_exti = ExtiInput::new(button, p.EXTI2);
//!     
//!     // Spawn button task
//!     spawner.spawn(button_task(button_exti, BUTTON_CHANNEL.sender())).unwrap();
//!     
//!     // Handle button events
//!     loop {
//!         let event = BUTTON_CHANNEL.receive().await;
//!         let action = handle_button_event(event);
//!         // Process action...
//!     }
//! }
//! ```

use embassy_time::{Duration, Timer};
use embassy_stm32::exti::ExtiInput;
use embassy_sync::channel::Sender;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// Button events after successful debouncing
/// 
/// These events represent stable, confirmed button state transitions
/// that have passed the debounce filter using Embassy's Timer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
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

/// Actions that can result from button events
/// 
/// This enum provides common action types that applications might want to take
/// in response to button events. Applications can extend this or use custom logic.
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

/// Embassy async button configuration
/// 
/// This struct encapsulates button configuration parameters for Embassy tasks.
/// All timing uses Embassy's Duration type for consistency with async operations.
#[derive(Clone, Copy, Debug)]
pub struct ButtonConfig {
    /// Debounce delay duration
    /// 
    /// **Range**: 10-50ms (typical for mechanical switches)
    /// **Default**: 20ms - Good balance between responsiveness and noise rejection
    pub debounce_delay: Duration,
    
    /// Button logic configuration
    /// 
    /// **Active High**: Button reads HIGH when pressed (pulldown resistor)
    /// **Active Low**: Button reads LOW when pressed (pullup resistor)
    pub active_high: bool,
}

impl Default for ButtonConfig {
    fn default() -> Self {
        Self {
            debounce_delay: Duration::from_millis(20),
            active_high: true,
        }
    }
}

/// Embassy async button task
/// 
/// This task handles button input using Embassy's async EXTI support.
/// It performs debouncing and sends validated button events to a channel.
/// 
/// # Parameters
/// 
/// * `button_exti` - Embassy ExtiInput configured for the button
/// * `sender` - Channel sender for sending button events
/// * `config` - Button configuration (debounce timing, active level)
/// 
/// # Async Behavior
/// 
/// This task:
/// 1. Waits for GPIO edge interrupts using `wait_for_any_edge()`
/// 2. Performs software debouncing with `Timer::after()`
/// 3. Validates stable state transitions
/// 4. Sends confirmed events via Embassy channel
/// 
/// # Example
/// 
/// ```rust,no_run
/// #[embassy_executor::task]
/// async fn my_button_task() {
///     let button = Input::new(p.PB2, Pull::Up);
///     let button_exti = ExtiInput::new(button, p.EXTI2);
///     let config = ButtonConfig::active_low(Duration::from_millis(20));
///     button_task(button_exti, channel_sender, config).await;
/// }
/// ```
#[embassy_executor::task]
pub async fn button_task(
    mut button_exti: ExtiInput<'static>,
    sender: Sender<'static, CriticalSectionRawMutex, ButtonEvent, 4>,
    config: ButtonConfig,
) {
    let mut last_state = read_button_state(&button_exti, config.active_high);
    
    loop {
        // Wait for any edge (rising or falling)
        button_exti.wait_for_any_edge().await;
        
        // Read immediate state after interrupt
        let interrupt_state = read_button_state(&button_exti, config.active_high);
        
        // Skip if no actual state change (noise/glitch)
        if interrupt_state == last_state {
            continue;
        }
        
        // Wait for debounce period
        Timer::after(config.debounce_delay).await;
        
        // Re-read state after debounce delay
        let stable_state = read_button_state(&button_exti, config.active_high);
        
        // Validate that state is still stable
        if stable_state == interrupt_state && stable_state != last_state {
            // State is stable and different - generate event
            let event = match stable_state {
                ButtonState::Pressed => ButtonEvent::Press,
                ButtonState::Released => ButtonEvent::Release,
            };
            
            // Send event through channel (non-blocking)
            let _ = sender.try_send(event);
            last_state = stable_state;
        }
        // If state changed during debounce, ignore this transition
    }
}

/// Internal button state representation
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ButtonState {
    Pressed,
    Released,
}

/// Read current button state from ExtiInput pin
/// 
/// This function handles the active-high vs active-low logic transparently,
/// always returning logical button states regardless of hardware configuration.
fn read_button_state(pin: &ExtiInput<'static>, active_high: bool) -> ButtonState {
    let pin_high = pin.is_high();
    
    if active_high {
        // Active-high: HIGH = pressed, LOW = released
        if pin_high {
            ButtonState::Pressed
        } else {
            ButtonState::Released
        }
    } else {
        // Active-low: HIGH = released, LOW = pressed
        if pin_high {
            ButtonState::Released
        } else {
            ButtonState::Pressed
        }
    }
}

/// High-level button event handler for application logic
/// 
/// This function converts button events into application-specific actions.
/// It provides a clean separation between button logic and application logic.
/// Applications can customize this function or implement their own event handling.
/// 
/// # Parameters
/// 
/// * `event` - Confirmed button event from the debounce process
/// 
/// # Returns
/// 
/// * `ButtonAction` - Application-specific action to be executed
/// 
/// # Customization
/// 
/// Applications can customize button behavior by:
/// 1. Modifying this function directly
/// 2. Implementing custom event handling logic
/// 3. Using pattern matching on ButtonEvent in application code
/// 
/// # Example Custom Handler
/// 
/// ```rust,no_run
/// fn custom_button_handler(event: ButtonEvent) -> CustomAction {
///     match event {
///         ButtonEvent::Press => CustomAction::StartSequence,
///         ButtonEvent::Release => CustomAction::StopSequence,
///     }
/// }
/// ```
pub fn handle_button_event(event: ButtonEvent) -> ButtonAction {
    match event {
        ButtonEvent::Press => ButtonAction::Toggle,
        ButtonEvent::Release => ButtonAction::None,
    }
}

