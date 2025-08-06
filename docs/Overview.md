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

The device is configured using the [Alif Semiconductor Conductor Tool](https://alifsemi.com/whitepaper/alif-semiconductors-conductor-tool/).

### Related packs

The following packs are required for this DFP:

```yml
    - pack: ARM::CMSIS@^6.2.0               # Arm CMSIS pack 6.2.0 or higher
    - pack: ARM::CMSIS-Compiler@2.1.0       # Arm Compiler pack 2.1.0 or higher (for STDIN/OUT)
```

## E7 DevKit (BSP)

The E7 DevKit BSP part provides the following:

Example                  | Tool           | Description
:------------------------|:---------------|:--------------------------
Blinky_M55_HE            | uVision IDE    | Blinky example running on M55 High Efficiency core
Blinky_M55_HE_TCM        | uVision IDE    | Blinky example running on M55 High Efficiency core from TCM
Csolution Project        | CMSIS-Toolbox  | Blinky examples configured for M55 High Performance or High Efficiency code
Dual Core (M55_HE/M55_HP)| CMSIS-Toolbox  | Dual Core example running on M55 High Efficiency and High Performance core
HelloWorld_M55_HP        | uVision IDE    | Hello World example running on M55 High Performance core

Layer Type: Board       | Tool           | Description
:-----------------------|:---------------|:--------------------------
`Board_HP.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Performance core
`Board_HE.clayer.yml`   | CMSIS-Toolbox  | Board layer for M55 High Efficient core

The Board Layers enable device-agnostic Reference Applications and implement these API interfaces:

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
Blinky (M55_HP)         | CMSIS-Toolbox  | Blinky example running on M55 High Performance core
Blinky (M55_HE)         | CMSIS-Toolbox  | Blinky example running on M55 High Efficiency core
DualCore (M55_HE/M55_HP)| CMSIS-Toolbox  | Dual Core example running on M55 High Efficiency and High Performance core

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

## Host PC Setup Requirements

The host PC system will be used to run the IDE tools and will interface to the Alif Semiconductor development boards through
the debug probes (J-Link, CMSIS DAP, ULINKpro) and also, for Alif Security Toolkit (SETOOLS), through an onboard UART-to-USB
interface adapter (PRG_USB) connected to an USB port on the host PC.

SETOOLS are mandatory for generating and flashing processor Application Table of Content (ATOC) image. The first time you load
an image or change processors, a processor ATOC image containing debug stubs must be generated and flashed onto the device prior
to starting a debug session. Otherwise, the JTAG connection will fail. After this is done once, the next time the application
can be built and debugged without reprogramming the ATOC again.

VS Code examples provide tasks that can be used to generate and program initial ATOC with debug stubs to the target device.

For latest version of SETOOLS and installation guidance, go to [Software & Tools](https://alifsemi.com/support/software-tools/ensemble/).

> A registration to [Alif Semiconductor](https://alifsemi.com/) website is required for downloading software and documentation from
> [Support](https://alifsemi.com/support/).

<!-- Todo: Additional usage information.
## Usage
-->

<!-- Todo: Useful links with documentation/help/forums.
## Links
- [Product page]()
- [GitHub Repo]()
- [Support]() 
- [User forum]()
 -->
