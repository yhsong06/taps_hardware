## Firmware Overview
STM32-based firmware for TAPS data acquisition system.

### MCU
- STM32H753 (or 정확한 모델)

### Peripherals
- I2S: Digital microphone input (8 kHz)
- SPI: Throat accelerometer (DMA, circular)
- DMA: Double buffering
- SD / UART / Bluetooth (선택)

### Build Environment
- STM32CubeIDE v1.xx
- STM32CubeMX v6.xx

### How to Build
1. Open STM32CubeIDE
2. Import project
3. Regenerate code from .ioc
4. Build & flash
