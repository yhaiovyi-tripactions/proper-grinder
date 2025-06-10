//! Pin Configuration Module
//!
//! Centralizes all GPIO pin definitions for the STM32F411 project.

use stm32f4xx_hal::gpio::{Input, Output, PushPull, PB2, PC13};

/// Pin type aliases for better readability
pub type LedPin = PC13<Output<PushPull>>;
pub type ButtonPin = PB2<Input>;

/// Initialize all pins and return them individually
pub fn init_pins(
    gpiob: stm32f4xx_hal::gpio::gpiob::Parts,
    gpioc: stm32f4xx_hal::gpio::gpioc::Parts,
) -> (LedPin, ButtonPin) {
    // LED on PC13 (active LOW)
    let led = gpioc.pc13.into_push_pull_output();
    
    // Button on PB2 (active HIGH with pullup)  
    let button = gpiob.pb2.into_pull_up_input();
    
    (led, button)
}

/// Timing constants
pub mod timing {
    pub const BUTTON_DEBOUNCE_MS: u32 = 20;
    pub const LED_BLINK_ON_TIME_MS: u32 = 100;
    pub const LED_BLINK_OFF_TIME_MS: u32 = 100;
    pub const LED_IDLE_UPDATE_MS: u32 = 50;
} 