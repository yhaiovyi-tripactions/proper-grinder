//! STM32F411 Embassy Async LED Blinker with Button Control
//!
//! This firmware demonstrates Embassy async/await patterns:
//! - Embassy async tasks and executor
//! - Async interrupt-driven button handling (EXTI + software debouncing)  
//! - LED control via button toggle using Embassy channels
//! - STM32F411CEU6 running at 100MHz with Embassy time driver
//! - Centralized hardware configuration
//! - Event-driven architecture with Embassy async primitives
//!
//! Hardware:
//! - LED on PC13 (active LOW)
//! - Button on PB2 (active LOW with pullup, Embassy EXTI interrupt)

#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Output, Level, Speed, Pull};
use embassy_time::{Duration, Timer};
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_futures::select::{select, Either};

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

    // Spawn tasks
    spawner.spawn(led_control_task(led)).unwrap();
    spawner.spawn(button_task(button_exti)).unwrap();

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
    let mut led_state = false;
    
    loop {
        if blinking {
            // Blink the LED
            led_state = !led_state;
            if led_state {
                led.set_low();  // LED on (active LOW)
                defmt::info!("LED ON");
            } else {
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
                    led.set_high(); // Turn off LED
                    defmt::info!("LED blinking STOPPED");
                }
            }
        } else {
            // Not blinking, just wait for button press
            BUTTON_SIGNAL.wait().await;
            blinking = true;
            defmt::info!("LED blinking STARTED");
        }
    }
}

#[embassy_executor::task]
async fn button_task(mut button_exti: ExtiInput<'static>) {
    defmt::info!("Button task started");
    
    loop {
        // Wait for button press (falling edge, since button is active LOW)
        button_exti.wait_for_falling_edge().await;
        defmt::info!("Button PRESSED!");
        
        // Simple debounce delay
        Timer::after(Duration::from_millis(50)).await;
        
        // Signal the LED task
        BUTTON_SIGNAL.signal(());
        
        // Wait for button release (rising edge)
        button_exti.wait_for_rising_edge().await;
        defmt::info!("Button RELEASED!");
        
        // Debounce delay before next press
        Timer::after(Duration::from_millis(50)).await;
    }
}
