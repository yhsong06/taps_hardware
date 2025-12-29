# Hardware

This directory contains the hardware design files for the TAPS data acquisition device.

The hardware was designed using **Altium Designer** and consists of two PCB projects:

- **main_pcb**: Includes the MCU and the acoustic microphone (CMM-4030D-261)
- **neck_pcb**: Includes the throat-mounted IMU sensor (IIM42652)

Each folder contains the corresponding Altium project files and schematic documents.

---

## Notes

Due to repository size limitations, additional materials (e.g., extended documentation, manufacturing outputs, or mechanical design files) are not included.  
For further information, please contact: **yhsong@postech.ac.kr**

---

## Related Publication

This hardware is used in the data acquisition device described in:

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
