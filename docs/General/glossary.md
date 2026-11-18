# Glossary {#glossary_page}

Terms used in this pack. CMSIS terms such as CMSIS-Pack, SVD, and CMSIS-Driver are defined in the [CMSIS 6 introduction](https://arm-software.github.io/CMSIS_6/main/General/index.html).

| Term | Meaning in this pack |
|---|---|
| ATOC | Application Table of Contents. The secure-enclave image that describes which application images live in MRAM. A debug session needs an ATOC that contains the M55 debug stubs. |
| aiPM | Alif power-management architecture. It powers a block only while that block is in use. Device descriptions list the power modes GO, READY, IDLE, STANDBY, and STOP. |
| AppKit | A kit with camera, microphones, and a display, used for AI/ML demos. |
| Conductor | Alif tool that configures pins and clocks. |
| DevKit | A kit that brings out pins for power and performance measurement. |
| Ethos-U55 / Ethos-U85 | Arm microNPUs. E1 through E7 use Ethos-U55. E4, E6, and E8 add Ethos-U85. |
| M55_HE | The High-Efficiency Cortex-M55. Always-on core. Pack clock is 160 MHz. Processor name `M55_HE`, define `RTSS_HE`. |
| M55_HP | The High-Performance Cortex-M55. Wakes for heavy work on dual-core parts. Pack clock is 400 MHz. Processor name `M55_HP`, define `RTSS_HP`. |
| MRAM | On-chip non-volatile memory that holds the application. The flash algorithm programs it. |
| PRG USB | The onboard UART-to-USB connector SETOOLS uses to talk to the secure enclave. |
| RTSS | Real-time subsystem. The Cortex-M55 side of the device. `RTSS_HP` and `RTSS_HE` select which core a translation unit is built for. |
| SETOOLS | Alif Security Toolkit. It generates and writes the ATOC. It is not inside this pack. |
| SVD | CMSIS System View Description. The debugger uses it to name peripheral registers. |
| UART4 | The application console used by the Blinky and DualCore examples. Baud rate is 115200. |
