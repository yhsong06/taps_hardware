# Firmware

STM32-based firmware used for the TAPS data acquisition device.

This firmware acquires synchronized signals from a throat-mounted IMU
(IIM42652) and an acoustic microphone (CMM-4030D-261), converts the data
into hexadecimal format, and transmits it via UART for dataset collection.

---

## Hardware Configuration

- **MCU**: STM32 (see `.ioc` file for exact configuration)
- **IMU**: IIM42652 (throat-mounted vibration sensor)
- **Microphone**: CMM-4030D-261 (acoustic microphone)

---

## Functionality

- Sampling of IMU and microphone signals
- Conversion of sensor data to HEX format
- UART-based data transmission to host PC
- Used exclusively for synchronized data recording (no on-device processing)

---

## Build Environment

- STM32CubeIDE v1.xx
- STM32CubeMX v6.xx

---

## How to Build

1. Open STM32CubeIDE
2. Import the project
3. Regenerate code from the `.ioc` file
4. Build and flash the firmware

---

## Notes

Due to repository size limitations, additional materials (e.g., extended
documentation or test utilities) are not included.
For further information, please contact: **yhsong@postech.ac.kr**

---

## Related Publication

This firmware is used in the data acquisition device described in:

```bibtex
@misc{kim2025tapsthroatacousticpaired,
  title={TAPS: Throat and Acoustic Paired Speech Dataset for Deep Learning-Based Speech Enhancement},
  author={Yunsik Kim and Yonghun Song and Yoonyoung Chung},
  year={2025},
  eprint={2502.11478},
  archivePrefix={arXiv},
  primaryClass={cs.SD},
  url={https://arxiv.org/abs/2502.11478}
}
