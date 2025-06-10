//! STM32F411 LED Blinker with Interrupt-Driven Button Control
//!
//! This firmware demonstrates:
//! - RTIC 2.1 async tasks
//! - Interrupt-driven button handling (EXTI + software debouncing)
//! - LED control via button toggle
//! - STM32F411CEU6 running at 100MHz
//! - Centralized pin configuration
//! - Event-driven architecture with minimal polling
//!
//! Hardware:
//! - LED on PC13 (active LOW)
//! - Button on PB2 (active HIGH with pullup, EXTI interrupt)

#![no_std]
#![no_main]

use panic_halt as _;
use rtic_monotonics::systick::prelude::*;

// Create the monotonic timer
systick_monotonic!(Mono, 1_000);

// Import modules
mod button;
mod pin_config;

use button::{ButtonEvent, InterruptButton, ButtonAction, handle_button_interrupt, handle_button_debounce, handle_button_event};
use pin_config::{timing::*, init_pins, LedPin};

use stm32f4xx_hal::{
    gpio::{Edge, ExtiPin},
    prelude::*,
};

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM2, TIM3])]
mod app {
    use super::*;
    use rtic_monotonics::Monotonic;
    use stm32f4xx_hal::{
        gpio::{Edge, ExtiPin},
        prelude::*,
    };

    #[shared]
    struct Shared {
        /// LED blinking state (on/off)
        led_on: bool,
        /// Button pin with EXTI capability (shared between interrupt and debounce task)
        button_pin: pin_config::ButtonPin,
        /// Button handler for debouncing logic
        button_handler: InterruptButton,
    }

    #[local]
    struct Local {
        /// LED pin (PC13, active LOW)
        led: LedPin,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device;

        // Clock setup - 100MHz operation
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(100_000_000.Hz()).freeze();
        Mono::start(ctx.core.SYST, clocks.sysclk().raw());

        // GPIO setup
        let gpioc = dp.GPIOC.split();
        let gpiob = dp.GPIOB.split();

        // Initialize pins using pin config
        let (led, mut button_pin) = init_pins(gpiob, gpioc);

        // Set up EXTI interrupt for button (PB2 = EXTI2) on both edges
        let mut syscfg = dp.SYSCFG.constrain();
        let mut exti = dp.EXTI;
        
        button_pin.make_interrupt_source(&mut syscfg);
        button_pin.enable_interrupt(&mut exti);
        button_pin.trigger_on_edge(&mut exti, Edge::RisingFalling);

        // Initialize button handler
        let button_handler = InterruptButton::new_active_high(BUTTON_DEBOUNCE_MS);

        // Start only the LED task (button tasks are spawned by interrupts)
        blink_task::spawn().ok();

        (
            Shared { 
                led_on: false,
                button_pin,
                button_handler,
            }, 
            Local { 
                led, 
            }
        )
    }

    /// EXTI interrupt handler for button (PB2 -> EXTI2)
    /// Uses button module to handle all button logic
    #[task(binds = EXTI2, shared = [button_pin, button_handler])]
    fn button_interrupt(mut ctx: button_interrupt::Context) {
        // Clear the interrupt flag and read pin state
        let pin_high = ctx.shared.button_pin.lock(|pin| {
            pin.clear_interrupt_pending_bit();
            pin.is_high()
        });

        // Use button module to handle interrupt logic
        let should_debounce = ctx.shared.button_handler.lock(|handler| {
            handle_button_interrupt(handler, pin_high)
        });

        if let Some(pin_state) = should_debounce {
            button_debounce_task::spawn(pin_state).ok();
        }
    }

    /// Button debouncing task - uses button module for debounce logic
    #[task(shared = [button_pin, button_handler])]
    async fn button_debounce_task(mut ctx: button_debounce_task::Context, interrupt_pin_state: bool) {
        // Wait for debounce period
        let debounce_ms = ctx.shared.button_handler.lock(|handler| {
            handler.debounce_time_ms()
        });
        
        Mono::delay(debounce_ms.millis()).await;
        
        // Re-read current pin state and use button module for debounce logic
        let current_pin_state = ctx.shared.button_pin.lock(|pin| pin.is_high());
        
        let event = ctx.shared.button_handler.lock(|handler| {
            handle_button_debounce(handler, current_pin_state, interrupt_pin_state)
        });

        if let Some(event) = event {
            button_task::spawn(event).ok();
        }
    }

    /// LED blinking task - blinks when enabled, stays off when disabled
    #[task(local = [led], shared = [led_on])]
    async fn blink_task(mut ctx: blink_task::Context) {
        loop {
            let should_blink = ctx.shared.led_on.lock(|led_on| *led_on);

            if should_blink {
                // Blink pattern: configurable timing
                ctx.local.led.set_low(); // LED on (active LOW)
                Mono::delay(LED_BLINK_ON_TIME_MS.millis()).await;
                ctx.local.led.set_high(); // LED off (active LOW)
                Mono::delay(LED_BLINK_OFF_TIME_MS.millis()).await;
            } else {
                // Keep LED off when not blinking
                ctx.local.led.set_high(); // LED off (active LOW)
                Mono::delay(LED_IDLE_UPDATE_MS.millis()).await;
            }
        }
    }

    /// Button handling task - uses button module for event logic
    #[task(shared = [led_on])]
    async fn button_task(mut ctx: button_task::Context, event: ButtonEvent) {
        // Use button module to determine action
        let action = handle_button_event(event);
        
        // Execute the action
        match action {
            ButtonAction::ToggleLed => {
                ctx.shared.led_on.lock(|led_on| {
                    *led_on = !*led_on;
                });
            }
            ButtonAction::None => {
                // No action needed
            }
        }
    }
}
