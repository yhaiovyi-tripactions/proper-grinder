//! Hardware Configuration Module for STM32F411 Black Pill
//!
//! This module centralizes all hardware-specific configuration parameters for the
//! STM32F411CEU6 microcontroller running at 100MHz. It provides compile-time constants
//! for timing parameters, pin assignments, interrupt mappings, and clock configuration.
//!
//! # Hardware Target
//! - **MCU**: STM32F411CEU6 (Black Pill development board)
//! - **Clock**: 100MHz system clock from internal HSI or external crystal
//! - **GPIO**: PC13 (LED, active LOW), PB2 (Button, active HIGH with pullup)
//! - **Interrupts**: EXTI2 for button input with rising/falling edge detection
//!
//! # Design Principles
//! - All configuration is compile-time constant for predictable real-time behavior
//! - Hardware-specific but easily portable to other STM32F4xx variants
//! - Type-safe pin definitions with proper GPIO modes
//! - RTIC-compatible timing constants for systick monotonic integration
//! - Comprehensive documentation with validation ranges and constraints

use stm32f4xx_hal::gpio::{Input, Output, PushPull, PB2, PC13};

//=============================================================================
// CLOCK CONFIGURATION
//=============================================================================

/// System clock configuration constants
pub mod clocks {
    /// System clock frequency in Hz (100MHz for STM32F411)
    /// 
    /// **Range**: 16MHz (HSI) to 100MHz (max for STM32F411)
    /// **Real-time Impact**: Higher frequencies improve task scheduling precision
    /// **Power Trade-off**: Higher frequencies increase power consumption
    pub const SYSCLK_HZ: u32 = 100_000_000;
    
    /// APB1 peripheral clock frequency in Hz (50MHz max for STM32F411)
    /// 
    /// **Range**: AHB ÷ 1,2,4,8 or 16 (max 50MHz for STM32F411)
    /// **Usage**: Timers 2-5, USART2/3, I2C, SPI2/3
    pub const APB1_HZ: u32 = SYSCLK_HZ / 2; // 50MHz
    
    /// APB2 peripheral clock frequency in Hz (100MHz max for STM32F411)
    /// 
    /// **Range**: AHB ÷ 1,2,4,8 or 16 (max 100MHz for STM32F411)  
    /// **Usage**: Timers 1,9-11, USART1/6, ADC, SPI1/4/5
    pub const APB2_HZ: u32 = SYSCLK_HZ; // 100MHz
}

//=============================================================================
// PIN CONFIGURATION
//=============================================================================

/// GPIO pin type aliases for type safety and readability
/// 
/// # Pin Assignment Summary
/// 
/// | Pin | Function | Mode | Logic | Interrupt |
/// |-----|----------|------|--------|-----------|
/// | PC13| LED      | Output | Active LOW | None |
/// | PB2 | Button   | Input | Active HIGH | EXTI2 |
pub mod pins {
    use super::*;
    
    /// LED pin type (PC13, active LOW, push-pull output)
    /// 
    /// **Hardware**: Built-in LED on STM32F411 Black Pill board
    /// **Logic**: LOW = LED ON, HIGH = LED OFF (active LOW)
    /// **Drive**: Push-pull output mode for clean switching
    pub type LedPin = PC13<Output<PushPull>>;
    
    /// Button pin type (PB2, active HIGH, input with pullup)
    /// 
    /// **Hardware**: External tactile switch connected to PB2
    /// **Logic**: HIGH = pressed, LOW = released (with internal pullup)
    /// **Interrupt**: EXTI2 line for hardware interrupt generation
    pub type ButtonPin = PB2<Input>;
}

/// Re-export pin types for convenient access
pub use pins::{LedPin, ButtonPin};

//=============================================================================
// INTERRUPT CONFIGURATION  
//=============================================================================

/// Hardware interrupt configuration and priority assignments
/// 
/// # EXTI Interrupt Mapping for RTIC
/// 
/// **Current Configuration**: PB2 → EXTI2  
/// **RTIC Task Binding**: `#[task(binds = EXTI2, ...)]`
/// 
/// ## Pin-to-EXTI Mapping Table
/// 
/// | Pin   | EXTI Line | RTIC Binding      |
/// |-------|-----------|-------------------|
/// | PB0   | EXTI0     | `binds = EXTI0`   |
/// | PB1   | EXTI1     | `binds = EXTI1`   |
/// | PB2   | EXTI2     | `binds = EXTI2`   | ← Current
/// | PB3   | EXTI3     | `binds = EXTI3`   |
/// | PB4   | EXTI4     | `binds = EXTI4`   |
/// | PB5-9 | EXTI9_5   | `binds = EXTI9_5` |
/// | PB10-15| EXTI15_10| `binds = EXTI15_10`|
/// 
/// ## Usage Pattern
/// 
/// 1. **Runtime (Dynamic)**: Use `pin.interrupt()` for NVIC setup
/// 2. **Compile-time (Static)**: Use matching `binds =` in RTIC task
/// 
/// ```rust,no_run
/// // Runtime: Works with any pin
/// unsafe { cortex_m::peripheral::NVIC::unmask(button_pin.interrupt()); }
/// 
/// // Compile-time: Must match pin choice manually
/// #[task(binds = EXTI2, shared = [button_pin, button_handler])]
/// fn button_interrupt(ctx: Context) { /* ... */ }
/// ```
pub mod interrupts {
    /// RTIC task priorities for real-time scheduling
    /// 
    /// **Range**: 1-15 (higher number = higher priority)
    /// **Guidelines**: 
    /// - Interrupt handlers: High priority (10-15)
    /// - Fast async tasks: Medium priority (5-9)  
    /// - Background tasks: Low priority (1-4)
    pub mod priorities {
        /// Button interrupt handler priority (high for responsiveness)
        /// 
        /// **Value**: 12/15 - High priority for immediate button response
        /// **Rationale**: Button input is user-interactive and should be responsive
        pub const BUTTON_INTERRUPT: u8 = 12;
        
        /// Button debounce task priority (medium for proper sequencing)
        /// 
        /// **Value**: 8/15 - Medium priority after interrupt processing
        /// **Rationale**: Debouncing needs to complete before action tasks
        pub const BUTTON_DEBOUNCE: u8 = 8;
        
        /// Button action task priority (medium for state changes)
        /// 
        /// **Value**: 6/15 - Medium priority for application state updates
        /// **Rationale**: LED toggle actions should be processed promptly
        pub const BUTTON_ACTION: u8 = 6;
        
        /// LED blink task priority (low for background operation)
        /// 
        /// **Value**: 2/15 - Low priority background task
        /// **Rationale**: LED blinking is visual feedback, not time-critical
        pub const LED_BLINK: u8 = 2;
    }
}

//=============================================================================
// TIMING CONFIGURATION
//=============================================================================

/// Timing constants optimized for RTIC systick monotonic (1ms ticks)
pub mod timing {
    /// Button debounce delay in milliseconds
    /// 
    /// **Range**: 10-50ms (typical for mechanical switches)
    /// **Value**: 20ms - Good balance between responsiveness and noise rejection
    /// **Hardware**: Eliminates mechanical switch bounce during press/release
    /// **Real-time**: Must be shorter than typical user button press duration
    pub const BUTTON_DEBOUNCE_MS: u32 = 20;
    
    /// LED blink timing constants for visual feedback
    pub mod led {
        /// LED ON duration during blink cycle in milliseconds
        /// 
        /// **Range**: 50-500ms (visible but not annoying)  
        /// **Value**: 100ms - Clear visual indication without excessive power usage
        /// **Usage**: Duration LED stays ON during each blink cycle
        pub const BLINK_ON_TIME_MS: u32 = 100;
        
        /// LED OFF duration during blink cycle in milliseconds
        /// 
        /// **Range**: 50-500ms (should match or exceed ON time)
        /// **Value**: 100ms - Creates 50% duty cycle for balanced blinking
        /// **Usage**: Duration LED stays OFF during each blink cycle  
        pub const BLINK_OFF_TIME_MS: u32 = 100;
        
        /// LED state update interval when not blinking in milliseconds
        /// 
        /// **Range**: 10-100ms (frequent enough for responsive state changes)
        /// **Value**: 50ms - Balance between responsiveness and CPU efficiency
        /// **Usage**: How often to check if blinking should start/stop
        pub const IDLE_UPDATE_MS: u32 = 50;
    }
    
    /// Re-export LED timing constants for backward compatibility
    pub use led::{
        BLINK_ON_TIME_MS as LED_BLINK_ON_TIME_MS,
        BLINK_OFF_TIME_MS as LED_BLINK_OFF_TIME_MS, 
        IDLE_UPDATE_MS as LED_IDLE_UPDATE_MS,
    };
}

//=============================================================================
// HARDWARE ABSTRACTION FUNCTIONS
//=============================================================================

/// Initialize all GPIO pins with proper configuration
/// 
/// This function configures GPIO pins according to the hardware requirements
/// and returns properly typed pin objects for use in RTIC tasks.
/// 
/// # Parameters
/// 
/// * `gpiob` - GPIO Port B peripheral parts from HAL
/// * `gpioc` - GPIO Port C peripheral parts from HAL
/// 
/// # Returns
/// 
/// * `(LedPin, ButtonPin)` - Configured LED and button pins ready for use
/// 
/// # Hardware Configuration Applied
/// 
/// - **LED Pin (PC13)**: Push-pull output, active LOW logic
/// - **Button Pin (PB2)**: Input with internal pullup, active HIGH logic
/// 
/// # Getting Interrupt Line from Pin
/// 
/// To get the EXTI interrupt line number from any pin that supports interrupts,
/// use the `interrupt()` method from the `ExtiPin` trait:
/// 
/// ```rust,no_run
/// let button_interrupt = button_pin.interrupt(); // Returns pac::Interrupt::EXTI2 for PB2
/// unsafe { cortex_m::peripheral::NVIC::unmask(button_interrupt); }
/// ```
/// 
/// This eliminates the need for hardcoded interrupt constants and makes the code
/// more portable when changing pin assignments.
/// 
/// # Example
/// 
/// ```rust,no_run
/// let gpiob = dp.GPIOB.split();
/// let gpioc = dp.GPIOC.split();
/// let (led, button) = init_pins(gpiob, gpioc);
/// let interrupt_line = button.interrupt(); // Dynamic interrupt discovery
/// ```
pub fn init_pins(
    gpiob: stm32f4xx_hal::gpio::gpiob::Parts,
    gpioc: stm32f4xx_hal::gpio::gpioc::Parts,
) -> (LedPin, ButtonPin) {
    // LED on PC13 (active LOW) - Built-in LED on Black Pill board
    let led = gpioc.pc13.into_push_pull_output();
    
    // Button on PB2 (active HIGH with pullup) - External tactile switch
    let button = gpiob.pb2.into_pull_up_input();
    
    (led, button)
}

//=============================================================================
// HARDWARE VALIDATION AND CONSTRAINTS
//=============================================================================

/// Compile-time hardware configuration validation
/// 
/// These const assertions verify that configuration parameters are within
/// valid ranges for the STM32F411 hardware platform.
#[allow(unused)]
mod validation {
    use super::*;
    
    // Clock frequency validation
    const _: () = assert!(clocks::SYSCLK_HZ >= 16_000_000, "SYSCLK too low (min 16MHz HSI)");
    const _: () = assert!(clocks::SYSCLK_HZ <= 100_000_000, "SYSCLK too high (max 100MHz for STM32F411)");
    const _: () = assert!(clocks::APB1_HZ <= 50_000_000, "APB1 too high (max 50MHz for STM32F411)");
    const _: () = assert!(clocks::APB2_HZ <= 100_000_000, "APB2 too high (max 100MHz for STM32F411)");
    
    // Timing parameter validation  
    const _: () = assert!(timing::BUTTON_DEBOUNCE_MS >= 5, "Button debounce too short (min 5ms)");
    const _: () = assert!(timing::BUTTON_DEBOUNCE_MS <= 100, "Button debounce too long (max 100ms)");
    const _: () = assert!(timing::led::BLINK_ON_TIME_MS >= 10, "LED blink ON time too short");
    const _: () = assert!(timing::led::BLINK_OFF_TIME_MS >= 10, "LED blink OFF time too short");
    const _: () = assert!(timing::led::IDLE_UPDATE_MS >= 10, "LED idle update too frequent");
    
    // Interrupt priority validation
    const _: () = assert!(interrupts::priorities::BUTTON_INTERRUPT <= 15, "Invalid interrupt priority");
    const _: () = assert!(interrupts::priorities::BUTTON_DEBOUNCE <= 15, "Invalid task priority");
    const _: () = assert!(interrupts::priorities::BUTTON_ACTION <= 15, "Invalid task priority");
    const _: () = assert!(interrupts::priorities::LED_BLINK <= 15, "Invalid task priority");
} 