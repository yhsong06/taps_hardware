# TAPS Hardware

This repository provides the **open-source hardware design files and firmware**
used for the **TAPS (Throat and Acoustic Paired Speech) dataset**.

The TAPS system is a multimodal wearable data acquisition platform designed to
simultaneously record throat-mounted vibration signals and acoustic microphone
signals for robust speech capture in noisy environments.

---

## Repository Structure
taps_hardware/
├─ hardware/
│ ├─ main_pcb/ # MCU + acoustic microphone PCB (Altium Designer)
│ └─ neck_pcb/ # Throat-mounted IMU sensor PCB (Altium Designer)
├─ firmware/ # STM32-based data acquisition firmware
└─ README.md


- **hardware/**: PCB design files created using *Altium Designer*
- **firmware/**: STM32 firmware for synchronized sensor data acquisition and UART transmission

---

## Additional Resources

- 🌐 **Project website**:  
  https://taps.postech.ac.kr  
  (Detailed description of the TAPS dataset, device, and experiments)

- 📄 **Research publication page**:  
  https://arxiv.org/abs/2502.11478

For further technical details or additional materials not included in this
repository due to size limitations, please contact:  
**yhsong@postech.ac.kr**

---

## Citation

If you use this hardware design, firmware, or the TAPS dataset in your research,
please cite the following paper:

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
