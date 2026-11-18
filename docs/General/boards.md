# Boards {#boards_page}

DevKits bring out pins for power and performance measurement. AppKits add the camera, microphones, and display used for AI/ML demos. E1C also has a smaller StartKit. Photos below are the images shipped in `docs/images/` and referenced by the pack.

| Board | Pack name | Mounted device | Examples in this tree |
|---|---|---|---|
| E8 AI/ML AppKit | `AppKit-E8-AIML` | `AE822FA0E5597BS0` | Board layers. CI does not build an AppKit-E8 example |
| E8 DevKit | `DevKit-E8` rev A1 | `AE822FA0E5597LS0` | Blinky_HP, Blinky_HE, DualCore, Hello_World |
| E7 AI/ML AppKit | `AppKit-E7-AIML` rev D1 | `AE722F80F55D5LS` | Blinky on HP, Blinky on HE, DualCore |
| E7 DevKit | `DevKit-E7` rev D1 | `AE722F80F55D5LS` | Blinky_HP, Blinky_HE, Dual_Core, plus µVision Blinky and Hello World |
| E1C DevKit | `DevKit-E1C` rev C | `AE1C1F4051920PH0` | Board description is in the pack. No example folder in this tree |
| E1C StartKit | `StartKit-E1C` rev B | `AE1C1F4051920PH0` | Low-cost kit. No example folder in this tree |

Product pages: [E8 DevKit](https://alifsemi.com/support/kits/ensemble-e8devkit/), [E8 AppKit](https://alifsemi.com/support/kits/ensemble-e8appkit/), [E7 DevKit](https://alifsemi.com/support/kits/ensemble-e7devkit/), [E7 AppKit](https://alifsemi.com/support/kits/ensemble-e7appkit/), [E1C DevKit](https://alifsemi.com/support/kits/ensemble-e1cdevkit/), [E1C StartKit](https://alifsemi.com/support/kits/ensemble-e1cstartkit/).

## E8 AI/ML AppKit

Camera, motion, and sound sensors, plus a display. The pack lists a Wi-Fi/BLE module, 64 MB Hex SPI HyperRAM, 128 MB Octal SPI flash, a microSD holder, headphone and line-in jacks, ICM-42670-P and BMI323 sensors, I2S and PDM microphones, and a MIPI camera. This repository does not include a photo of the E8 AppKit.

### Board layers

| Layer | Description |
|---|---|
| `Board_HP.clayer.yml` | M55 High-Performance core with Ethos-U55 |
| `Board_HE.clayer.yml` | M55 High-Efficiency core with Ethos-U55 |
| `Board_HP-U85.clayer.yml` | M55 High-Performance core with Ethos-U85 |
| `Board_HE-U85.clayer.yml` | M55 High-Efficiency core with Ethos-U85 |

### Interfaces

| API | Connection |
|---|---|
| `CMSIS_ETH` | Ethernet on the RJ45 connector |
| `CMSIS_USART` | USART4 on the PRG USB connector |
| `CMSIS_USB_Device` | USB on the MCU USB connector |
| `CMSIS_VIO` | RGB LED and joystick |
| `CMSIS_VSTREAM_VIDEO_IN` | MIPI camera |
| `CMSIS_VSTREAM_VIDEO_OUT` | GLCD display |
| STDIN, STDOUT, STDERR | USART4 COM port |

## E7 AI/ML AppKit

![E7 AI/ML AppKit](AK-E7-AIML_Gen-2_small.jpg)

![E7 AI/ML AppKit, bottom side](e7-appkit-bottom.jpg)

On-board peripherals include a WE310F5 Wi-Fi/BLE module, 64 MB Octal SPI HyperRAM, 64 MB Octal SPI flash, a 4.3-inch capacitive TFT (480 x 800), and an onsemi MT9M114 1.26 MP camera on 2-lane MIPI CSI.

### Examples

| Example | Tool | Description |
|---|---|---|
| Blinky, M55_HP | CMSIS-Toolbox | Blinky on the High-Performance core. Prints `"Blinky example"` |
| Blinky, M55_HE | CMSIS-Toolbox | Blinky on the High-Efficiency core |
| DualCore | CMSIS-Toolbox | Both cores. Prints `"DualCore example"` |

### Board layers

| Layer | Description |
|---|---|
| `Board_HP.clayer.yml` | M55 High-Performance core |
| `Board_HE.clayer.yml` | M55 High-Efficiency core |

### Interfaces

| API | Connection |
|---|---|
| `CMSIS_VIO` | RGB LED and joystick |
| `CMSIS_USB_Device` | USB on the SoC USB connector |
| `CMSIS_VSTREAM_AUDIO_IN` | I2S microphones |
| `CMSIS_VSTREAM_VIDEO_IN` | MIPI camera |
| `CMSIS_VSTREAM_VIDEO_OUT` | GLCD display |
| STDIN, STDOUT, STDERR | USART COM port |

SETOOLS uses J15-A and J15-B in position SE. Application `printf` uses the same jumpers in position U4.

## E8 DevKit

![E8 DevKit](DK-E8_small.jpg)

Revision A1. On-board J-Link, 64 MB HyperRAM, 128 MB Octal SPI flash, 10/100 Ethernet, USB device and host, audio jacks, and a CR2032 holder for VBAT retention.

### Examples

| Example | Tool | Description |
|---|---|---|
| Blinky_HP | CMSIS-Toolbox | Blinky on the High-Performance core |
| Blinky_HE | CMSIS-Toolbox | Blinky on the High-Efficiency core |
| DualCore | CMSIS-Toolbox | Both cores |
| Hello_World | CMSIS-Toolbox | Hello World on the High-Efficiency core |

Layers match the E8 AppKit table, including the Ethos-U85 variants.

### Interfaces

| API | Connection |
|---|---|
| `CMSIS_ETH` | Ethernet on the RJ45 connector |
| `CMSIS_USB_Device` | USB on the MCU USB connector |
| `CMSIS_VIO` | RGB LED and joystick |
| `CMSIS_VSTREAM_AUDIO_IN` | I2S microphones |
| `CMSIS_VSTREAM_VIDEO_IN` | MIPI camera |
| `CMSIS_VSTREAM_VIDEO_OUT` | GLCD display |
| STDIN, STDOUT, STDERR | USART COM port |

SETOOLS uses SW4 in position SE. Application `printf` uses SW4 in position U4.

## E7 DevKit

![E7 DevKit](DK-E7_Gen-2_small.jpg)

Revision D1. Pin headers for digital I/O, analog I/O, a parallel camera or display, and mikroBUS. 64 MB HyperRAM, 64 MB Octal SPI flash, microSD, 10/100 Ethernet, USB device and host, CAN-FD, ICM-42670-P and BMI323 sensors, I2S and PDM microphones, and a MIPI display connector. External debug probes connect to J16 (20-pin) and J17 (10-pin).

### Examples

| Example | Tool | Description |
|---|---|---|
| Blinky_M55_HE | µVision | Blinky on the High-Efficiency core |
| Blinky_M55_HE_TCM | µVision | Same example, executing from TCM |
| Blinky_HP | CMSIS-Toolbox | Blinky on the High-Performance core |
| Blinky_HE | CMSIS-Toolbox | Blinky on the High-Efficiency core |
| Dual_Core | CMSIS-Toolbox | Both cores |
| HelloWorld_M55_HP | µVision | Hello World on the High-Performance core |

### Board layers

| Layer | Description |
|---|---|
| `Board_HP.clayer.yml` | M55 High-Performance core |
| `Board_HE.clayer.yml` | M55 High-Efficiency core |

### Interfaces

| API | Connection |
|---|---|
| `CMSIS_ETH` | Ethernet on the RJ45 connector |
| `CMSIS_USB_Device` | USB on the SoC USB connector |
| `CMSIS_VIO` | RGB LED and joystick |
| `CMSIS_VSTREAM_AUDIO_IN` | I2S microphones |
| STDIN, STDOUT, STDERR | USART COM port |

Application `printf` uses J26-A and J26-B in position 4 (UART4).

## E1C DevKit

![E1C DevKit](DK-E1C_small.jpg)

Revision C. Mounts `AE1C1F4051920PH0`. On-board J-Link over USB-C, 64 MB HyperRAM, 128 MB Octal SPI flash, microSD, USB device and host, audio jacks, ICM-42670-P and BMI323, PDM microphones, a joystick, one RGB LED, a 1-lane MIPI-DSI connector, and a CAN header. There is no Ethernet jack.

Compatible devices in the pack: `AE1C1F4051920HH0`, `AE1C1F1041010PH0`, `AE1C1F1041010HH0`, `AE1C1F1040505PH0`, `AE1C1F1040505HH0`.

Release 2.3.0 moves the SD card-detect GPIO from P3_3 to P6_5 on this kit.

## E1C StartKit

![E1C StartKit](SK-E1C_small.jpg)

Revision B. USB-C device and on-board J-Link, PDM microphones, user and reset buttons, mikroBUS, and an Arducam connector. It does not include the DevKit HyperRAM, Octal SPI flash, display connector, or audio jacks.
