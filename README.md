# STM32F411 LED Blinker with Button Control

A clean, production-ready embedded Rust firmware for STM32F411CEU6 microcontrollers using RTIC 2.1.

## Features

- **RTIC 2.1** async/await tasks
- **Generic button debounce module** - works with any GPIO pin and timer
- **100MHz operation** - STM32F411CEU6 at maximum stable frequency
- **Future-proof architecture** - easy to port to other microcontrollers

## Hardware Setup

- **MCU**: STM32F411CEU6 "BlackPill" board
- **LED**: PC13 (active LOW, built-in LED)
- **Button**: PB2 (active HIGH with internal pullup)
- **Programmer**: STLink V2-1 or compatible

## Quick Start

1. **Install dependencies**:
   ```bash
   cargo install probe-rs
   ```

2. **Build and flash**:
   ```bash
   cargo run --release
   ```

3. **Usage**:
   - Press button to toggle LED blinking on/off
   - 20ms software debouncing eliminates switch bounce
   - 100ms blink rate when active

## Code Structure

```
src/
├── main.rs           # RTIC application and platform setup
├── button.rs         # Generic button debounce module
├── memory.x          # STM32F411 memory layout
└── .cargo/config.toml # Build configuration
```

## Generic Button Module

The button debounce module is fully generic and can be used with any microcontroller:

```rust
// Works with any GPIO pin and any RTIC monotonic timer
let mut button = DebouncedButton::new_active_high(pin, 20);

if let Some(event) = button.check_event::<Mono>().await {
    match event {
        ButtonEvent::Press => { /* handle press */ }
        ButtonEvent::Release => { /* handle release */ }
    }
}
```

## Adding New Platforms

The code uses a future-proof architecture. To add support for new MCUs:

1. Add feature flag in `Cargo.toml`
2. Add conditional dependencies
3. Create platform-specific setup code (see comments in `main.rs`)

The button module and core logic remain unchanged across platforms.

## Build Configuration

- **Target**: `thumbv7em-none-eabihf` (ARM Cortex-M4F)
- **Optimization**: Release builds with LTO enabled
- **Debugger**: Probe-rs with STM32F411CEUx chip support 