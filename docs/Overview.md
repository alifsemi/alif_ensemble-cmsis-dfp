# Pack Content

The `Alif Semiconductor Ensemble Device Family Pack` contains Device and Board support for:

- [Alif Semiconductor Ensemble E1C/E1/E3/E5/E7/E8](https://alifsemi.com/products/ensemble/) AI/ML microcontroller series with up two Arm Cortex-M55 and two Arm Ethos-U55 microNPUs.
- [Alif Semiconductor Ensemble E7 DevKit - Gen 2](https://alifsemi.com/support/kits/ensemble-devkit/) single board computer with access to all signals and pins for power/performance profiling.
- [Alif Semiconductor Ensemble E7 AI/ML AppKit - Gen 2](https://alifsemi.com/support/kits/ai-ml-appkit/) single board computer with camera/image, motion, sound sensors and display.

This Pack contains:

- Support for Arm Compiler 6 (AC6) and GCC.
- Flash algorithms for the on-chip Flash memory.
- Debug configuration information and SVD files for peripheral awareness.
- SoC Peripheral interfaces and CMSIS-Driver for CAN, GPIO, I2C, SAI, SPI, USART.
- Template and example projects for uVision IDE and CMSIS-Toolbox (VS Code).

## Device Support (DFP)

The device is configured using the [Alif Semiconductor Conductor Tool](https://alifsemi.com/whitepaper/alif-semiconductors-conductor-tool/). The [secure enclave](https://alifsemi.com/understanding-the-ensemble-difference-built-in-secure-enclave-provides-strong-cyber-protection-to-edge-ai-devices/) of the device is configured using the [Alif Secure Toolkit (SETOOLS)](https://swrm.alifsemi.com/Content/3.4%20SETOOLS.htm?TocPath=Secure%20Enclave%20Subsystem%7C_____4) that generates and downloads an Application Table of Content (ATOC) for the device.

> _**NOTE**_<br>
> Before downloading examples it is required to program the ATOC to the device. [Usage](#usage) below contains  more information.

### Related packs

The following packs are required for this DFP:

```yml
    - pack: ARM::CMSIS@^6.2.0               # Arm CMSIS pack 6.2.0 or higher
    - pack: ARM::CMSIS-Compiler@2.1.0       # Arm Compiler pack 2.1.0 or higher (for STDIN/OUT)
```

## E7 DevKit (BSP)

The E7 DevKit Board Support (BSP) part provides the following examples and layers:

Example                  | Tool           | Description
:------------------------|:---------------|:--------------------------
Blinky_M55_HE            | uVision IDE    | Blinky example running on M55 High Efficiency core
Blinky_M55_HE_TCM        | uVision IDE    | Blinky example running on M55 High Efficiency core from TCM
Csolution Project        | CMSIS-Toolbox  | Blinky examples configured for M55 High Performance or High Efficiency code
DualCore_HE_HP           | CMSIS-Toolbox  | Dual Core example running on M55 High Efficiency and High Performance core
HelloWorld_M55_HP        | uVision IDE    | Hello World example running on M55 High Performance core
Zephyr_HP                | CMSIS-Toolbox  | Zephyr setup for CMSIS-Debugger on M55 High Performance core
Zephyr_HE                | CMSIS-Toolbox  | Zephyr setup for CMSIS-Debugger on M55 High Efficiency core
Zephyr_HE_HP             | CMSIS-Toolbox  | Zephyr setup for CMSIS-Debugger on M55 High Efficiency and High Performance core

Layer Type: Board       | Tool           | Description
:-----------------------|:---------------|:--------------------------
`Board_HP.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Performance core
`Board_HE.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Efficient core

The Board Layers enable device-agnostic [Reference Applications](https://www.keil.arm.com/refapps/) and implement these API interfaces:

Provided API Interface    | Description
:-------------------------|:------------------------------------------------------------------------------
 CMSIS_ETH                | CMSIS-Driver Ethernet connected to RJ45 connector
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
Zephyr_HP               | CMSIS-Toolbox  | Zephyr setup for CMSIS-Debugger on M55 High Performance core
Zephyr_HE               | CMSIS-Toolbox  | Zephyr setup for CMSIS-Debugger on M55 High Efficiency core
Zephyr_HE_HP            | CMSIS-Toolbox  | Zephyr setup for CMSIS-Debugger on M55 High Efficiency and High Performance core

Layer Type: Board       | Tool           | Description
:-----------------------|:---------------|:--------------------------
`Board_HP.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Performance core
`Board_HE.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Efficient core

The Board Layers enable device-agnostic [Reference Applications](https://open-cmsis-pack.github.io/cmsis-toolbox/ReferenceApplications/) and implement these API interfaces:

Provided API Interface    | Description
:-------------------------|:------------------------------------------------------------------------------
 CMSIS_VIO                | CMSIS-Driver VIO connected to RGB LED and Joystick
 CMSIS_VSTREAM_AUDIO_IN   | CMSIS-Driver for Audio input stream via I2S Microphones
 CMSIS_VSTREAM_VIDEO_IN   | CMSIS-Driver for Video input stream via MIPI Camera
 CMSIS_VSTREAM_VIDEO_OUT  | CMSIS-Driver for Video output stream via GLCD Display
 STDIN, STDOUT, STDERR    | Standard I/O connected to USART COM port

## Usage

The host PC runs the [SETOOLS](https://swrm.alifsemi.com/Content/3.4%20SETOOLS.htm?TocPath=Secure%20Enclave%20Subsystem%7C_____4), IDE and debugger connects to Alif Semiconductor development boards via debug probes (J-Link, CMSIS DAP, ULINKpro). Optional, for the SETOOLS  an onboard UART-to-USB
interface adapter (PRG_USB) is available.

> _**NOTE**_<br>
> SETOOLS are mandatory for generating and flashing processor the Application Table of Content (ATOC). The first time you load an image or change the processor setup, a ATOC image containing debug stubs must be generated and flashed onto the device prior to starting a debug session (otherwise, the debug connection will fail). After this is done once, the application can be built, flashed, and debugged without reprogramming the ATOC.
>
> Download SETOOLS from Alif Semiconductor [Software & Tools](https://alifsemi.com/support/software-tools/ensemble/) page (requires registration).

The CMSIS-Toolbox examples include for VS Code a `tasks.json` file that generates and programs the ATOC using the SETOOLS using the following _tasks_:

Tasks                                                                  | Description
:----------------------------------------------------------------------|:-------------------------
Alif: Install M55_HE or M55_HP debug stubs (single core configuration) | Setup for HP and HE single core examples
Alif: Install M55_HE and M55_HP debug stubs (dual core configuration)  | Setup for dualcore examples

To run these _tasks_:

1. Set the variable `alif.setools.root` in the global `settings.json`. Use `Ctrl+,` to open the settings dialog, then switch to text mode and enter (this is required only once):

    ```json
    {
    "alif.setools.root": "C:\\Alif\\SETOOLS",     // SETOOLS installation path on your PC
    :
    ```

2. Use from the VS Code menu **Terminal - Run Task** to execute the SETOOLS configuration _task_.
   For Windows ensure that the Terminal default is `Git Bash` or `PowerShell`.

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
<!--
- [User forum]()
 -->
