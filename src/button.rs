//! Embassy Async Button Handler
//!
//! This module provides a simple, interrupt-driven button handler for Embassy async/await applications.
//! It uses Embassy's EXTI (External Interrupt) support for efficient, zero-polling button handling.
//!
//! # Key Features
//! 
//! - **🚫 Zero Polling** - Pure interrupt-driven architecture with Embassy's EXTI support
//! - **⚡ Embassy Native** - Designed for Embassy's async executor and task patterns
//! - **🔄 Software Debouncing** - Eliminates mechanical switch bounce with Embassy Timer
//! - **🎯 Signal-Based** - Uses Embassy Signal for simple async communication
//! - **🧩 Hardware Agnostic** - Works with any Embassy-compatible EXTI pin
//! - **⚙️ Configurable** - Supports different debounce timings and active levels
//!
//! # Usage Example
//!
//! ```rust,no_run
//! use embassy_sync::signal::Signal;
//! use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
//! use button::button_task;
//!
//! // Create a signal for button presses
//! static BUTTON_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
//!
//! #[embassy_executor::main]
//! async fn main(spawner: Spawner) {
//!     let p = embassy_stm32::init(Default::default());
//!     let button_exti = ExtiInput::new(p.PB2, p.EXTI2, Pull::Up);
//!     
//!     // Spawn button task
//!     spawner.spawn(button_task(button_exti, &BUTTON_SIGNAL, ButtonConfig::active_low())).unwrap();
//!     
//!     // Wait for button presses
//!     loop {
//!         BUTTON_SIGNAL.wait().await;
//!         defmt::info!("Button pressed!");
//!     }
//! }
//! ```

use embassy_time::{Duration, Timer};
use embassy_stm32::exti::ExtiInput;
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// Button configuration for Embassy async button handling
/// 
/// This struct encapsulates button configuration parameters for Embassy tasks.
/// All timing uses Embassy's Duration type for consistency with async operations.
#[derive(Clone, Copy, Debug)]
pub struct ButtonConfig {
    /// Debounce delay duration
    /// 
    /// **Range**: 10-50ms (typical for mechanical switches)
    /// **Default**: 50ms - Conservative debouncing for reliable operation
    pub debounce_delay: Duration,
    
    /// Button logic configuration
    /// 
    /// **Active High**: Button reads HIGH when pressed (pulldown resistor)
    /// **Active Low**: Button reads LOW when pressed (pullup resistor)
    pub active_low: bool,
}

impl ButtonConfig {
    /// Create configuration for active-low button (most common)
    /// 
    /// This is the typical configuration for buttons with pullup resistors
    /// where pressing the button connects the pin to ground.
    pub fn active_low() -> Self {
        Self {
            debounce_delay: Duration::from_millis(50),
            active_low: true,
        }
    }
    
    /// Create configuration for active-high button
    /// 
    /// This configuration is for buttons with pulldown resistors
    /// where pressing the button connects the pin to VCC.
    pub fn active_high() -> Self {
        Self {
            debounce_delay: Duration::from_millis(50),
            active_low: false,
        }
    }
    
    /// Create configuration with custom debounce timing
    /// 
    /// Use this to customize debounce timing for specific switch types.
    pub fn with_debounce(debounce_ms: u64, active_low: bool) -> Self {
        Self {
            debounce_delay: Duration::from_millis(debounce_ms),
            active_low,
        }
    }
}

impl Default for ButtonConfig {
    fn default() -> Self {
        Self::active_low()
    }
}

/// Embassy async button task
/// 
/// This task handles button input using Embassy's interrupt-driven EXTI support.
/// It performs debouncing and signals button presses via Embassy Signal.
/// 
/// # Parameters
/// 
/// * `button_exti` - Embassy ExtiInput configured for the button pin
/// * `signal` - Embassy Signal to notify on button presses
/// * `config` - Button configuration (debounce timing, active level)
/// 
/// # Interrupt-Driven Behavior
/// 
/// This task:
/// 1. Waits for GPIO edge interrupts (falling/rising based on config)
/// 2. Performs software debouncing with `Timer::after()`
/// 3. Signals button press event via Embassy Signal
/// 4. Waits for button release with debouncing
/// 
/// # Example
/// 
/// ```rust,no_run
/// // Spawn the button task
/// spawner.spawn(button_task(button_exti, &BUTTON_SIGNAL, ButtonConfig::active_low())).unwrap();
/// ```
#[embassy_executor::task]
pub async fn button_task(
    mut button_exti: ExtiInput<'static>,
    signal: &'static Signal<CriticalSectionRawMutex, ()>,
    config: ButtonConfig,
) {
    defmt::info!("Button task started (active_low: {})", config.active_low);
    
    loop {
        if config.active_low {
            // Active LOW: wait for falling edge (button press)
            button_exti.wait_for_falling_edge().await;
            defmt::info!("Button PRESSED!");
            
            // Debounce delay
            Timer::after(config.debounce_delay).await;
            
            // Signal the press
            signal.signal(());
            
            // Wait for rising edge (button release)
            button_exti.wait_for_rising_edge().await;
            defmt::info!("Button RELEASED!");
            
            // Debounce delay before next press
            Timer::after(config.debounce_delay).await;
        } else {
            // Active HIGH: wait for rising edge (button press)
            button_exti.wait_for_rising_edge().await;
            defmt::info!("Button PRESSED!");
            
            // Debounce delay
            Timer::after(config.debounce_delay).await;
            
            // Signal the press
            signal.signal(());
            
            // Wait for falling edge (button release)
            button_exti.wait_for_falling_edge().await;
            defmt::info!("Button RELEASED!");
            
            // Debounce delay before next press
            Timer::after(config.debounce_delay).await;
        }
    }
}

