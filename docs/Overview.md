# Pack Content

The `Alif Semiconductor Ensemble Device Family Pack` contains Device and Board support for:

- [Alif Semiconductor Ensemble E1C/E1/E3/E5/E7/E8](https://alifsemi.com/products/ensemble/) AI/ML microcontroller series with up to two Arm Cortex-M55 processors and two Arm Ethos-U55/U85 microNPUs.
- [Alif Semiconductor Ensemble E8 DevKit](https://alifsemi.com/support/kits/) single-board computer with access to all signals and pins for power/performance profiling.
- [Alif Semiconductor Ensemble E7 DevKit - Gen 2](https://alifsemi.com/support/kits/) single-board computer with access to all signals and pins for power/performance profiling.
- [Alif Semiconductor Ensemble E7 AI/ML AppKit - Gen 2](https://alifsemi.com/support/kits/) single-board computer with camera/image, motion, and sound sensors, and a display.

This Pack contains:

- Support for Arm Compiler 6 (AC6) and GCC.
- Flash algorithms for the on-chip Flash memory.
- Debug configuration information and SVD files for peripheral awareness.
- SoC peripheral interfaces and CMSIS-Driver for CAN, GPIO, I2C, SAI, SPI, and USART.
- Templates and example projects for uVision IDE and CMSIS-Toolbox (VS Code).

## Device Support (DFP)

The device is configured using the [Alif Semiconductor Conductor Tool](https://alifsemi.com/whitepaper/alif-semiconductors-conductor-tool/).
The device's [secure enclave](https://alifsemi.com/understanding-the-ensemble-difference-built-in-secure-enclave-provides-strong-cyber-protection-to-edge-ai-devices/)
is configured using the [Alif Secure Toolkit (SETOOLS)](https://swrm.alifsemi.com/Content/3.4%20SETOOLS.htm?TocPath=Secure%20Enclave%20Subsystem%7C_____4),
which generates and downloads an Application Table of Contents (ATOC) for the device.

> _**NOTE**_<br>
> Before downloading examples, you must program the ATOC to the device. [Usage](#usage) below contains more information.

### Related packs

The following packs are required for this DFP:

```yml
    - pack: ARM::CMSIS@^6.2.0               # Arm CMSIS pack 6.2.0 or higher
    - pack: ARM::CMSIS-Compiler@2.1.0       # Arm Compiler pack 2.1.0 or higher (for STDIN/OUT)
```

## E8 DevKit (BSP)

The E8 DevKit Board Support (BSP) part provides the following examples and layers:

Example                 | Tool           | Description
:-----------------------|:---------------|:--------------------------
Blinky_HP               | CMSIS-Toolbox  | Blinky example configured for M55 High Performance code
Blinky_HE               | CMSIS-Toolbox  | Blinky example configured for M55 High Efficiency code

Layer Type: Board       | Tool           | Description
:-----------------------|:---------------|:--------------------------
`Board_HP.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Performance core
`Board_HE.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Efficiency core

The Board layers enable device-agnostic [Reference Applications](https://www.keil.arm.com/refapps/) and implement these API interfaces:

Provided API Interface    | Description
:-------------------------|:------------------------------------------------------------------------------
 CMSIS_ETH                | CMSIS-Driver Ethernet connected to RJ45 connector
 CMSIS_USB_Device         | CMSIS-Driver USB connected to MCU USB connector
 CMSIS_VIO                | CMSIS-Driver VIO connected to RGB LED and Joystick
 CMSIS_VSTREAM_AUDIO_IN   | CMSIS-Driver for Audio input stream via I2S Microphones
 CMSIS_VSTREAM_VIDEO_IN   | CMSIS-Driver for Video input stream via MIPI Camera
 CMSIS_VSTREAM_VIDEO_OUT  | CMSIS-Driver for Video output stream via GLCD Display
 STDIN, STDOUT, STDERR    | Standard I/O connected to USART COM port

## E7 DevKit (BSP)

The E7 DevKit Board Support (BSP) part provides the following examples and layers:

Example                  | Tool           | Description
:------------------------|:---------------|:--------------------------
Blinky_M55_HE            | uVision IDE    | Blinky example running on M55 High Efficiency core
Blinky_M55_HE_TCM        | uVision IDE    | Blinky example running on M55 High Efficiency core from TCM
Blinky_HP                | CMSIS-Toolbox  | Blinky example configured for M55 High Performance code
Blinky_HE                | CMSIS-Toolbox  | Blinky example configured for M55 High Efficiency code
DualCore_HE_HP           | CMSIS-Toolbox  | Dual Core example running on M55 High Efficiency and High Performance core
HelloWorld_M55_HP        | uVision IDE    | Hello World example running on M55 High Performance core

Layer Type: Board       | Tool           | Description
:-----------------------|:---------------|:--------------------------
`Board_HP.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Performance core
`Board_HE.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Efficiency core

The Board layers enable device-agnostic [Reference Applications](https://www.keil.arm.com/refapps/) and implement these API interfaces:

Provided API Interface    | Description
:-------------------------|:------------------------------------------------------------------------------
 CMSIS_ETH                | CMSIS-Driver Ethernet connected to RJ45 connector
 CMSIS_USB_Device         | CMSIS-Driver USB connected to SoC USB connector
 CMSIS_VIO                | CMSIS-Driver VIO connected to RGB LED and Joystick
 CMSIS_VSTREAM_AUDIO_IN   | CMSIS-Driver for Audio input stream via I2S Microphones
 STDIN, STDOUT, STDERR    | Standard I/O connected to USART COM port

## E7 AppKit (BSP)

The E7 AppKit BSP part provides the following:

Example                 | Tool           | Description
:-----------------------|:---------------|:--------------------------
Blinky_HP               | CMSIS-Toolbox  | Blinky example running on M55 High Performance core
Blinky_HE               | CMSIS-Toolbox  | Blinky example running on M55 High Efficiency core
DualCore_HE_HP          | CMSIS-Toolbox  | Dual Core example running on M55 High Efficiency and High Performance core

Layer Type: Board       | Tool           | Description
:-----------------------|:---------------|:--------------------------
`Board_HP.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Performance core
`Board_HE.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Efficiency core

The Board layers enable device-agnostic [Reference Applications](https://open-cmsis-pack.github.io/cmsis-toolbox/ReferenceApplications/) and implement these API interfaces:

Provided API Interface    | Description
:-------------------------|:------------------------------------------------------------------------------
 CMSIS_VIO                | CMSIS-Driver VIO connected to RGB LED and Joystick
 CMSIS_USB_Device         | CMSIS-Driver USB connected to SOC USB connector
 CMSIS_VSTREAM_AUDIO_IN   | CMSIS-Driver for Audio input stream via I2S Microphones
 CMSIS_VSTREAM_VIDEO_IN   | CMSIS-Driver for Video input stream via MIPI Camera
 CMSIS_VSTREAM_VIDEO_OUT  | CMSIS-Driver for Video output stream via GLCD Display
 STDIN, STDOUT, STDERR    | Standard I/O connected to USART COM port

## CMSIS Solution Templates

For starting new applications in VS Code, select these templates when using the CMSIS View menu command *Create a Solution*:

- Alif CMSIS: A bare-metal application for an Alif Ensemble device.
- Alif Zephyr: A Zephyr application (using West) for an Alif Ensemble device.

## Usage

The host PC runs [SETOOLS](https://swrm.alifsemi.com/Content/3.4%20SETOOLS.htm?TocPath=Secure%20Enclave%20Subsystem%7C_____4)
(Alif Security Toolkit) that connects to the Alif Semiconductor development board via onboard UART-to-USB interface adapter (PRG_USB).
The IDE and debugger connect to development board via debug probes (J-Link, CMSIS DAP, ULINKpro).

### SETOOLS

SETOOLS is mandatory for generating and flashing the processor Application Table of Contents (ATOC). The first time you load
an image or change the processor setup, an ATOC image containing debug stubs must be generated and flashed onto the device prior
to starting a debug session (otherwise, the debug connection will fail). After this is done once, the application can be built,
flashed, and debugged without reprogramming the ATOC.

> Download SETOOLS from Alif Semiconductor [Software & Tools](https://alifsemi.com/support/software-tools/ensemble/) page (requires registration).

#### Configure Target Device

Latest SETOOLS require target device configuration before the ATOC can be programmed to the device.
See Section 4 of the latest [Alif Security Toolkit Quick Start Guide](https://alifsemi.com/download/AQSG0002) for details on
different ways to do this.

For automatic configuration using the `tools-config` executable, just connect Alif Semiconductor development board
to the host PC and execute

```sh
tools-config -a
```

Alternatively, to specify target device manually, see the examples below:

 To select the correct target device use the `tools-config` executable

Kit        | Command
:----------|:-------------------------
E7 AppKit  | `tools-config -p "E7 (AE722F80F55D5LS) - 5.5 MRAM / 13.5 SRAM" -r B4`
E7 DevKit  | `tools-config -p "E7 (AE722F80F55D5LS) - 5.5 MRAM / 13.5 SRAM" -r B4`
E1C DevKit | `tools-config -p “E1C (AE1C1F4051920PH) - 1.86 MRAM / 2.0 SRAM” -r A7`
B1 DevKit  | `tools-config -p “B1 (AB1C1F4M51820PH) - 1.8 MRAM / 2.0 SRAM” -r A0`
E8 DevKit  | `tools-config -p "E8 (AE822FA0E5597LS0) - 5.5 MRAM / 9.75 SRAM" -r A1`

### Tasks for VS Code

For VS Code, the CMSIS-Toolbox examples include a `tasks.json` file that generates and programs the ATOC using SETOOLS
with the following _tasks_:

Tasks                                                                  | Description
:----------------------------------------------------------------------|:-------------------------
Alif: Install M55_HE or M55_HP debug stubs (single core configuration) | Setup for HP and HE single core examples
Alif: Install M55_HE and M55_HP debug stubs (dual core configuration)  | Setup for dual-core examples

To run these _tasks_:

1. Set the variable `alif.setools.root` in the global `settings.json`. Use `Ctrl+,` to open the settings dialog, then
   switch to text mode and enter (this is required only once):

    ```json
    {
    "alif.setools.root": "C:\\Alif\\SETOOLS",     // SETOOLS installation path on your PC
    :
    ```

2. Use **Terminal - Run Task** from the VS Code menu to execute the SETOOLS configuration _task_.
   On Windows, ensure that the default terminal is `Git Bash` or `PowerShell`.

Example output:

```
 *  Executing task: cp './.alif/M55_HE_mram_cfg.json' 'C:\Alif/build/config/M55_HE_mram_cfg.json'; cp './.alif/M55_HP_mram_cfg.json' 'C:\Alif/build/config/M55_HP_mram_cfg.json'; cp './.alif/M55_HP_mram_stub.bin' 'C:\Alif/build/images/M55_HP_mram_stub.bin'; cp './.alif/M55_HE_mram_stub.bin' 'C:\Alif/build/images/M55_HE_mram_stub.bin'; cd 'C:\Alif'; ./app-gen-toc -f 'build/config/M55_HE_mram_cfg.json'; ./app-write-mram -p -d; 

Generating APP Package with:
Device Part# E7 (AE722F80F55D5LS) - 5.5 MRAM / 13.5 SRAM - Rev: B2
- System MRAM Base Address: 0x80580000
- APP MRAM Base Address: 0x80000000
- APP MRAM Size: 5767168
- Configuration file: build/config/M55_HE_mram_cfg.json
- Output file: build/AppTocPackage.bin

Generating Device Configuration for: app-device-config.json
Calculating APP area...
Creating Content Certificates...
2025-07-02 16:26:43,856 - Content Certificate Generation Utility started (Logging to ../build/logs/SBContent.log)
  :
Creating APP TOC Package...
Creating Signature...
Binary File:  ../build/images/M55_HE_mram_stub.bin
2025-07-02 16:26:44,813 - Content Certificate Generation Utility started (Logging to ../build/logs/SBContent.log)
Content Certificate File:  build/images/M55_HE_mram_stub.bin.crt
Signature File:  build/images/M55_HE_mram_stub.bin.sign
Adding ATOC...
APP TOC Package size: 13552 bytes
Creating Signature...
Binary File:  ../build/AppTocPackage.bin
2025-07-02 16:26:45,057 - Content Certificate Generation Utility started (Logging to ../build/logs/SBContent.log)
Content Certificate File:  build/AppTocPackage.bin.crt
Signature File:  build/AppTocPackage.bin.sign
Done!
Writing MRAM with parameters:
Device Part# E7 (AE722F80F55D5LS) - 5.5 MRAM / 13.5 SRAM - Rev: B2
- Available MRAM: 5767168 bytes
[INFO] Burning: ../build/images/M55_HE_mram_stub.bin 0x80000000 ../build/AppTocPackage.bin 0x8057cb10
[INFO] baud rate  55000
[INFO] dynamic baud rate change  Enabled
COM ports detected = 2
-> COM3
-> COM6
Enter port name:COM6
[INFO] COM6 open Serial port success
Maintenance Mode = Enabled 
Authenticate Image:  False
Download Image
build\images\M55_HE_mram_stub.bin[####################]100%: 3040/3040 bytes
Done
       0.07 seconds

Authenticate Image:  False
Download Image
build\AppTocPackage.bin         [####################]100%: 13552/13552 bytes
Done
       0.32 seconds
```

## Links

- [Product page](https://alifsemi.com/products/ensemble/)
- [Alif GitHub Repos](https://github.com/alifsemi)
- [Support](https://alifsemi.com/support/)
- [User forum](https://community.arm.com/support-forums/f/keil-forum)
