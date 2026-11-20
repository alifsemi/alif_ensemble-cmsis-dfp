# Devices {#devices_page}

Ensemble is a family of Arm-based 32-bit microcontrollers and fusion processors for battery-powered edge AI. Alif groups the public lineup as Edge AI (E1C, E1, E3, E5, E7), Edge AI + Gen AI (E4, E6, E8), and Edge AI + Wireless (B1). This pack describes the Edge AI and Edge AI + Gen AI series. B1 appears only as a SETOOLS part string in the existing overview. It has no device entry in this pack.

Family figures published on [alifsemi.com](https://alifsemi.com/) and the [Ensemble family page](https://alifsemi.com/products/ensemble/) are more than 19 MB of integrated memory, more than 3670 DMIPS of combined compute, and more than 250 GigaOPS of AI performance. Those are family ranges. The table below uses the clocks and memory maps in `AlifSemiconductor.Ensemble.pdsc`.

Shared features from the Ensemble family description:

- Real-time cores: up to two Cortex-M55 CPUs, up to 400 MHz, with Helium.
- Application cores on the fusion-processor series: up to two Cortex-A32 CPUs, up to 800 MHz, with NEON. This pack does not build software for those cores.
- AI acceleration: Ethos-U55, and on E4, E6, and E8 also Ethos-U85.
- Power: aiPM domain control. The pack lists GO, READY, IDLE, STANDBY, and STOP.
- Security: a full secure enclave, secure boot, lifecycle management, and a root of trust.
- Memory: on-chip MRAM and SRAM. Octal SPI can extend RAM or non-volatile memory, with execute-in-place.
- Interfaces: Ethernet, USB, SDIO, I3C, CAN-FD, UART, SPI, I2C, 12-bit SAR ADC, 24-bit sigma-delta ADC, 12-bit DAC, and comparators.
- Graphics and imaging: MIPI-DSI, parallel display, MIPI-CSI-2, parallel camera, and on many parts a 2D GPU.
- Audio: I2S and PDM microphones.

| Series | Cortex-M55 in this pack | NPU in this pack | Representative on-chip memory | Alif positions it for |
|---|---|---|---|---|
| E1C | M55_HE at 160 MHz | Ethos-U55, 128 MACs, on `AE1C1F4051920PH0` | Pack map `0x0018C000` of MRAM and 2 MB SRAM. SETOOLS calls the same kit "1.86 MRAM / 2.0 SRAM" | Entry edge AI. DevKit and StartKit are in the pack |
| E1 | M55_HE at 160 MHz | Ethos-U55, 128 MACs, on `AE101F4071542LH` | 1.5 MB MRAM and 4.25 MB SRAM | Wearables, hearables, fitness trackers |
| E3 | M55_HP at 400 MHz and M55_HE at 160 MHz | Ethos-U55. MAC count depends on the SKU | MRAM 1.5 MB or 5.5 MB. SRAM 3.75 MB to 13.5 MB | Security cameras, person and gesture detection |
| E5 | M55_HP at 400 MHz and M55_HE at 160 MHz | Ethos-U55, 256 MACs on HP and 128 MACs on HE | 5.5 MB MRAM. SRAM 8.25 MB or 13.5 MB | Health and medical devices that also run Linux on Cortex-A32 |
| E7 | M55_HP at 400 MHz and M55_HE at 160 MHz | Ethos-U55, 256 MACs on HP and 128 MACs on HE | 5.5 MB MRAM and 13.5 MB SRAM | Smart retail and high-end edge control. Two Cortex-A32 cores at 800 MHz |
| E4, E6, E8 | M55_HP at 400 MHz and M55_HE at 160 MHz | Ethos-U55 (256 / 128 MACs) and Ethos-U85 (256 MACs) | 5.5 MB MRAM and 9.75 MB SRAM | Edge AI + Gen AI. E8 is the series with kit support in this tree |

The High-Efficiency core is the always-on core. On dual-core parts the High-Performance core wakes for the heavy work and returns to sleep.

## E1C

One Cortex-M55 High-Efficiency core at 160 MHz, a secure enclave, USB, one cryptographic Octal SPI, SDIO, CAN-FD, a low-power parallel camera, I2S and PDM audio, a 12-bit SAR ADC, and a 24-bit sigma-delta ADC. The documented part `AE1C1F4051920PH0` adds a 2D GPU and MIPI-DSI, in a 194-ball FBGA. DevKit-E1C and StartKit-E1C mount that part.

The SETOOLS name is `E1C (AE1C1F4051920PH) - 1.86 MRAM / 2.0 SRAM`. The linker map in the pack is `0x0018C000` bytes of MRAM. Publish one figure only after checking the E1C datasheet. Until that check, this page keeps both numbers.

[E1C datasheet](https://alifsemi.com/download/ADTS0008) · [E1C hardware reference manual](https://alifsemi.com/download/AHRM0008) · [Software reference manual](https://alifsemi.com/download/ASRM0001)

## E1

A single real-time core in a small package, aimed at wearables. The public E1 description includes a 2-lane MIPI display or 24-bit RGB, a parallel camera for low-resolution machine vision, up to 14 microphones, and up to 120 GPIO pins at 1.8 V plus Flex I/O at 1.8 V or 3.3 V. The pack part `AE101F4071542LH` is a 194-ball FBGA with Ethos-U55, CAN-FD, a 2D GPU, and MIPI-DSI.

[E1 datasheet](https://alifsemi.com/download/ADTS0008) · [E1 hardware reference manual](https://alifsemi.com/download/AHRM0008) · [Software reference manual](https://alifsemi.com/download/ASRM0001)

## E3

E3 adds the 400 MHz High-Performance Cortex-M55 beside the High-Efficiency core, and can add a second microNPU. The public description lists up to 120 GPIO pins, up to 22 microphones, an 8-bit or 16-bit parallel camera, and 2-lane MIPI-CSI. The pack lists several orderables, from 1.5 MB MRAM / 3.75 MB SRAM up to 5.5 MB MRAM / 13.5 MB SRAM. Release 2.2.0 added further E3 SKUs.

[E3 datasheet](https://alifsemi.com/download/ADTS0007) · [E3 errata](https://alifsemi.com/download/AERR0007) · [E3 hardware reference manual](https://alifsemi.com/download/AHRM0007) · [Software reference manual](https://alifsemi.com/download/ASRM0001)

## E5

A fusion processor: one Cortex-A32 application core up to 800 MHz, plus both Cortex-M55 cores, sharing peripherals and memory. Alif positions it for medical equipment that needs networking and a high-level OS next to real-time control. This pack's processor views are `M55_HP` and `M55_HE` only.

[E5 datasheet](https://alifsemi.com/download/ADTS0006) · [E5 hardware reference manual](https://alifsemi.com/download/AHRM0006) · [Software reference manual](https://alifsemi.com/download/ASRM0001)

## E7

Two Cortex-A32 cores at 800 MHz, plus both Cortex-M55 cores. Public interfaces include 10/100 Ethernet with IEEE 1588, two Octal SPI ports, MIPI-DSI or 24-bit RGB, and up to three image sensors. The pack memory map is 5.5 MB MRAM and 13.5 MB SRAM. The E7 DevKit and the E7 AI/ML AppKit mount `AE722F80F55D5LS`.

[E7 datasheet](https://alifsemi.com/download/ADTS0005) · [E7 hardware reference manual](https://alifsemi.com/download/AHRM0005) · [Software reference manual](https://alifsemi.com/download/ASRM0001)

## E4, E6, and E8

These are the Edge AI + Gen AI devices on [alifsemi.com](https://alifsemi.com/). In this pack they share the dual Cortex-M55 clocks (400 MHz and 160 MHz), 5.5 MB MRAM, 9.75 MB SRAM, Ethos-U55 on both real-time cores, and an Ethos-U85. E8 is the series with board layers and examples in this tree. E8 layers are split into Ethos-U55 files (`Board_HP.clayer.yml`, `Board_HE.clayer.yml`) and Ethos-U85 files (`Board_HP-U85.clayer.yml`, `Board_HE-U85.clayer.yml`).

The E8 DevKit mounts `AE822FA0E5597LS0`. The E8 AI/ML AppKit mounts `AE822FA0E5597BS0`.

## Conductor and the secure enclave

Pin and clock configuration is done with the [Conductor tool](https://alifsemi.com/whitepaper/alif-semiconductors-conductor-tool/). The secure enclave is programmed with SETOOLS, which builds the ATOC. Details are on @ref setools_page.
