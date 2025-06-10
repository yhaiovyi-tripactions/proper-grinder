//! STM32F411 LED Blinker with Button Control
//!
//! This firmware demonstrates:
//! - RTIC 2.1 async tasks
//! - Software button debouncing with generic timer support
//! - LED control via button toggle
//! - STM32F411CEU6 running at 100MHz
//! - Centralized pin configuration
//!
//! Hardware:
//! - LED on PC13 (active LOW)
//! - Button on PB2 (active HIGH with pullup)

#![no_std]
#![no_main]

use panic_halt as _;
use rtic_monotonics::systick::prelude::*;

// Create the monotonic timer
systick_monotonic!(Mono, 1_000);

// Import modules
mod button;
mod pin_config;

use button::{ButtonEvent, DebouncedButton};
use pin_config::{timing::*, init_pins, LedPin};

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM2, TIM3])]
mod app {
    use super::*;
    use rtic_monotonics::Monotonic;
    use stm32f4xx_hal::{
        gpio::Input,
        prelude::*,
    };

    #[shared]
    struct Shared {
        /// LED blinking state (on/off)
        led_on: bool,
    }

    #[local]
    struct Local {
        /// LED pin (PC13, active LOW)
        led: LedPin,
        /// Debounced button handler
        button: DebouncedButton<pin_config::ButtonPin>,
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
        let (led, button_pin) = init_pins(gpiob, gpioc);
        let button = DebouncedButton::new_active_high(button_pin, BUTTON_DEBOUNCE_MS);

        // Start async tasks
        blink_task::spawn().ok();
        button_task::spawn().ok();

        (Shared { led_on: false }, Local { led, button })
    }

    /// LED blinking task - blinks when enabled, stays off when disabled
    #[task(local = [led], shared = [led_on])]
    async fn blink_task(mut ctx: blink_task::Context) {
        loop {
            let should_blink = ctx.shared.led_on.lock(|led_on| *led_on);

            if should_blink {
                // Blink pattern: 100ms on, 100ms off
                ctx.local.led.set_low(); // LED on (active LOW)
                Mono::delay(100.millis()).await;
                ctx.local.led.set_high(); // LED off (active LOW)
                Mono::delay(100.millis()).await;
            } else {
                // Keep LED off when not blinking
                ctx.local.led.set_high(); // LED off (active LOW)
                Mono::delay(50.millis()).await;
            }
        }
    }

    /// Button handling task - toggles LED blinking on button press
    #[task(local = [button], shared = [led_on])]
    async fn button_task(mut ctx: button_task::Context) {
        loop {
            // Check for button events with debouncing
            if let Some(event) = ctx.local.button.check_event::<Mono>().await {
                match event {
                    ButtonEvent::Press => {
                        // Toggle LED blinking state
                        ctx.shared.led_on.lock(|led_on| {
                            *led_on = !*led_on;
                        });
                    }
                    ButtonEvent::Release => {
                        // Button release events can be handled here if needed
                    }
                }
            }

            // Poll button every 10ms (this was in the working version)
            Mono::delay(10.millis()).await;
        }
    }
}
