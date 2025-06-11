//! Hardware Configuration Module for STM32F411 Black Pill
//!
//! This module centralizes all hardware-specific configuration parameters for the
//! STM32F411CEU6 microcontroller running at 100MHz. It provides compile-time constants
//! for timing parameters, pin assignments, and clock configuration optimized for Embassy.
//!
//! # Hardware Target
//! - **MCU**: STM32F411CEU6 (Black Pill development board)
//! - **Clock**: 100MHz system clock from internal HSI or external crystal
//! - **GPIO**: PC13 (LED, active LOW), PB2 (Button, active HIGH with pullup)
//! - **Framework**: Embassy async/await with built-in interrupt handling
//!
//! # Design Principles
//! - All configuration is compile-time constant for predictable behavior
//! - Clock settings provide 100MHz operation for optimal performance  
//! - Pin assignments match hardware layout for easy physical verification
//! - Embassy timing constants align with real-time requirements
//! - Configuration is hardware-specific but platform-agnostic for easy porting
//! - **Interrupt handling**: Embassy automatically determines EXTI interrupt lines
//!   from pin configuration. Use `pin.interrupt()` at runtime to get the
//!   interrupt source for NVIC configuration.
//!
//! # Examples
//! ```rust,no_run
//! // Hardware configuration defines pins
//! let led_pin = Output::new(p.PC13, Level::High, Speed::Low);
//! let button_pin = Input::new(p.PB2, Pull::Up);
//! 
//! // Embassy automatically determines interrupt line from pin
//! let interrupt_source = button_pin.interrupt(); // Returns EXTI2 for PB2
//! 
//! // Use in NVIC setup
//! unsafe { cortex_m::peripheral::NVIC::unmask(interrupt_source); }
//! ```

use embassy_stm32::gpio::{Level, Speed, Pull};

//=============================================================================
// CLOCK CONFIGURATION
//=============================================================================

/// System clock configuration constants optimized for Embassy
pub mod clocks {
    /// System clock frequency in Hz (100MHz for STM32F411)
    /// 
    /// **Range**: 16MHz (HSI) to 100MHz (max for STM32F411)
    /// **Embassy Impact**: Higher frequencies improve async task scheduling precision
    /// **Power Trade-off**: Higher frequencies increase power consumption
    pub const SYSCLK_HZ: u32 = 100_000_000;
    
    /// APB1 peripheral clock frequency in Hz (50MHz max for STM32F411)
    /// 
    /// **Range**: AHB ÷ 1,2,4,8 or 16 (max 50MHz for STM32F411)
    /// **Usage**: Timers 2-5, USART2/3, I2C, SPI2/3, Embassy time driver
    pub const APB1_HZ: u32 = SYSCLK_HZ / 2; // 50MHz
    
    /// APB2 peripheral clock frequency in Hz (100MHz max for STM32F411)
    /// 
    /// **Range**: AHB ÷ 1,2,4,8 or 16 (max 100MHz for STM32F411)  
    /// **Usage**: Timers 1,9-11, USART1/6, ADC, SPI1/4/5, Embassy EXTI
    pub const APB2_HZ: u32 = SYSCLK_HZ; // 100MHz
}

//=============================================================================
// PIN CONFIGURATION
//=============================================================================

/// GPIO pin type aliases for type safety and Embassy compatibility
/// 
/// # Pin Assignment Summary
/// 
/// | Pin | Function | Mode | Logic | Embassy Driver |
/// |-----|----------|------|--------|----------------|
/// | PC13| LED      | Output | Active LOW | GPIO Output |
/// | PB2 | Button   | Input | Active HIGH | GPIO Input + EXTI |
pub mod pins {
    use super::*;
    
    /// LED pin configuration (PC13, active LOW, push-pull output)
    /// 
    /// **Hardware**: Built-in LED on STM32F411 Black Pill board
    /// **Logic**: LOW = LED ON, HIGH = LED OFF (active LOW)
    /// **Embassy**: Compatible with embassy_stm32::gpio::Output
    pub const LED_INITIAL_LEVEL: Level = Level::High; // LED off initially
    pub const LED_SPEED: Speed = Speed::Low; // Low speed sufficient for LED
    
    /// Button pin configuration (PB2, active LOW with pullup)
    /// 
    /// **Hardware**: External tactile switch connected to PB2
    /// **Logic**: LOW = pressed (switch to GND), HIGH = released (pullup)
    /// **Embassy**: Compatible with embassy_stm32::gpio::Input + EXTI
    pub const BUTTON_PULL: Pull = Pull::Up; // Internal pullup
}

//=============================================================================
// EMBASSY CONFIGURATION
//=============================================================================

/// Embassy-specific configuration parameters
pub mod embassy {
    /// Embassy executor configuration
    pub mod executor {
        /// Maximum number of Embassy tasks that can be spawned
        /// 
        /// **Range**: 1-16 (typical for small embedded applications)
        /// **Value**: 8 - Sufficient for LED, button, and future expansion
        /// **Memory**: Each task slot uses ~64 bytes of RAM
        pub const MAX_TASKS: usize = 8;
        
        /// Embassy task stack size in bytes
        /// 
        /// **Range**: 512-4096 bytes (depends on task complexity)
        /// **Value**: 1024 bytes - Adequate for async GPIO operations
        /// **Memory**: Per-task stack allocation
        pub const TASK_STACK_SIZE: usize = 1024;
    }
    
    /// Embassy time driver configuration
    pub mod time {
        /// Time driver tick frequency in Hz
        /// 
        /// **Range**: 1000-1000000 Hz (1ms to 1μs resolution)
        /// **Value**: 32768 Hz - Good balance of precision and power efficiency
        /// **Usage**: Embassy Timer::after(), Ticker::every() operations
        pub const TICK_HZ: u32 = 32_768;
        
        /// Maximum Embassy timer delay in milliseconds
        /// 
        /// **Range**: 100-60000 ms (limited by tick counter overflow)
        /// **Value**: 30000 ms - 30 second maximum delay for safety
        /// **Usage**: Prevents accidental infinite delays in async code
        pub const MAX_DELAY_MS: u64 = 30_000;
    }
}

//=============================================================================
// TIMING CONFIGURATION
//=============================================================================

/// Timing constants optimized for Embassy async operations
pub mod timing {
    /// Button debounce timing (mechanical switch bounce filtering)
    /// 
    /// **Value**: 20ms - Optimal balance between responsiveness and noise rejection
    /// **Range**: 10-50ms typical for mechanical switches
    /// **Usage**: `Timer::after(Duration::from_millis(BUTTON_DEBOUNCE_MS)).await`
    pub const BUTTON_DEBOUNCE_MS: u64 = 20;

    /// LED blink timing constants
    /// 
    /// These constants define the LED blink pattern timing optimized for
    /// Embassy's async Timer operations and visual feedback requirements.
    
    /// LED ON duration during blink cycle
    /// 
    /// **Value**: 100ms - Clear visual indication without being too fast
    /// **Usage**: `Timer::after(Duration::from_millis(LED_BLINK_ON_MS)).await`
    pub const LED_BLINK_ON_MS: u64 = 100;
    
    /// LED OFF duration during blink cycle  
    /// 
    /// **Value**: 400ms - Creates distinctive blink pattern (1:4 ratio)
    /// **Usage**: `Timer::after(Duration::from_millis(LED_BLINK_OFF_MS)).await`
    pub const LED_BLINK_OFF_MS: u64 = 400;
}

//=============================================================================
// HARDWARE VALIDATION AND CONSTRAINTS
//=============================================================================

/// Compile-time hardware configuration validation
/// 
/// These const assertions verify that configuration parameters are within
/// valid ranges for the STM32F411 hardware platform and Embassy framework.
#[allow(unused)]
mod validation {
    use super::*;
    
    // Clock frequency validation
    const _: () = assert!(clocks::SYSCLK_HZ >= 16_000_000, "SYSCLK too low (min 16MHz HSI)");
    const _: () = assert!(clocks::SYSCLK_HZ <= 100_000_000, "SYSCLK too high (max 100MHz for STM32F411)");
    const _: () = assert!(clocks::APB1_HZ <= 50_000_000, "APB1 too high (max 50MHz for STM32F411)");
    const _: () = assert!(clocks::APB2_HZ <= 100_000_000, "APB2 too high (max 100MHz for STM32F411)");
    
    // Embassy executor validation
    const _: () = assert!(embassy::executor::MAX_TASKS >= 2, "Need at least 2 tasks (LED + button)");
    const _: () = assert!(embassy::executor::MAX_TASKS <= 32, "Too many tasks (memory constraint)");
    const _: () = assert!(embassy::executor::TASK_STACK_SIZE >= 512, "Task stack too small");
    const _: () = assert!(embassy::executor::TASK_STACK_SIZE <= 8192, "Task stack too large");
    
    // Timing parameter validation
    const _: () = assert!(embassy::time::TICK_HZ >= 1000, "Tick frequency too low (min 1kHz)");
    const _: () = assert!(embassy::time::TICK_HZ <= 1_000_000, "Tick frequency too high (max 1MHz)");
    const _: () = assert!(embassy::time::MAX_DELAY_MS >= 1000, "Max delay too short (min 1s)");
    const _: () = assert!(embassy::time::MAX_DELAY_MS <= 60_000, "Max delay too long (max 60s)");
} 