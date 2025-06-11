//! STM32F411 Embassy Async LED Blinker with Button Control
//!
//! This firmware demonstrates Embassy async/await patterns:
//! - Embassy async tasks and executor
//! - Async interrupt-driven button handling (EXTI + software debouncing)  
//! - LED control via button toggle using Embassy Signal
//! - STM32F411CEU6 running at 16MHz with Embassy time driver (32.768kHz tick)
//! - Modular button handling with extracted button.rs module
//! - Event-driven architecture with Embassy async primitives
//!
//! Hardware:
//! - LED on PC13 (active LOW)
//! - Button on PB2 (active LOW with pullup, Embassy EXTI interrupt)

#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;

mod button;

use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Output, Level, Speed, Pull};
use embassy_time::{Duration, Timer};
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_futures::select::{select, Either};

use button::{button_task, ButtonConfig};

// Signal to communicate button presses between tasks
static BUTTON_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    
    defmt::info!("Embassy STM32F411 Button-controlled LED Demo Started");

    // Initialize LED on PC13 (active LOW) - start OFF
    let led = Output::new(p.PC13, Level::High, Speed::Low);
    
    // Initialize button on PB2 with pullup (active LOW)
    let button_exti = ExtiInput::new(p.PB2, p.EXTI2, Pull::Up);

    defmt::info!("GPIO initialized - LED: PC13, Button: PB2");

    // Spawn tasks - button task now comes from button.rs module
    spawner.spawn(led_control_task(led)).unwrap();
    spawner.spawn(button_task(button_exti, &BUTTON_SIGNAL, ButtonConfig::active_low())).unwrap();

    // Main task heartbeat
    let mut counter = 0u32;
    loop {
        Timer::after(Duration::from_secs(5)).await;
        counter += 1;
        defmt::info!("System heartbeat #{}", counter);
    }
}

#[embassy_executor::task]
async fn led_control_task(mut led: Output<'static>) {
    defmt::info!("LED control task started");
    
    let mut blinking = false;
    
    loop {
        if blinking {
            // Toggle LED state
            if led.is_set_high() {
                // LED is OFF, turn it ON
                led.set_low();  // LED on (active LOW)
                defmt::info!("LED ON");
            } else {
                // LED is ON, turn it OFF  
                led.set_high(); // LED off (active LOW)
                defmt::info!("LED OFF");
            }
            
            // Check for button press during blink delay
            match select(
                Timer::after(Duration::from_millis(500)),
                BUTTON_SIGNAL.wait()
            ).await {
                Either::First(_) => {
                    // Timer completed, continue blinking
                }
                Either::Second(_) => {
                    // Button pressed, stop blinking
                    blinking = false;
                    led.set_high(); // Ensure LED is OFF when stopped
                    defmt::info!("LED blinking STOPPED");
                }
            }
        } else {
            // Not blinking, ensure LED is OFF and wait for button press
            led.set_high(); // Ensure LED is OFF
            BUTTON_SIGNAL.wait().await;
            blinking = true;
            defmt::info!("LED blinking STARTED");
            // LED will start blinking from OFF state (current state)
        }
    }
}
