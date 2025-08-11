# Board: AlifSemiconductor DevKit

## Board Layer for M55 High Performance Core

Device: AE722F80F55D5LS:M55_HP

This setup is configured using **Conductor Tool**, an interactive tool provided by Alif Semiconductor for device configuration.
Refer to ["Alif Conductor Tool Manual"](https://conductor.alifsemi.com/Alif_HTML_DCT_User_Help/Content/Help%20Manual.htm) for additional information.

### System Configuration

| System Component        | Setting
|:------------------------|:----------------------------------------
| Heap                    | 96 kB (configured in M55_HP linker file)
| Stack (MSP)             |  8 kB (configured in M55_HP linker file)

### STDIO mapping

**STDIO** is routed to Virtual COM port via **UART4** peripheral

> Note:
> For STDOUT (printf output) configure J26-A & J26-B to position 4 (UART4)

### CMSIS-Driver mapping

| CMSIS-Driver          | Peripheral | Board connector/component     | Connection
|:----------------------|:-----------|:------------------------------|:----------------------
| Driver_ETH_MAC0       | ETH        | Ethernet RJ45 connector (J28) | CMSIS_ETH
| Driver_I2C0           | I2C0       | MikroBUS Header               | MIKROBUS_I2C
| Driver_SPI0           | SPI0       | MikroBUS Header               | MIKROBUS_SPI
| Driver_USART4         | UART4      | PRG USB connector (J3)        | STDIN, STDOUT, STDERR
| CMSIS-Driver VIO      | GPIO       | Buttons, RGB LED, Joystick    | CMSIS_VIO
| Driver_vStreamAudioIn | SAI3       | I2S Microphone (U13, U15)     | CMSIS_VSTREAM_AUDIO_IN

### CMSIS-Driver Virtual I/O mapping

| CMSIS-Driver VIO | Board component
|:-----------------|:----------------------------
|vioBUTTON0        | Joystick Select Button
|vioJOYup          | Joystick Up
|vioJOYdown        | Joystick Down
|vioJOYleft        | Joystick Left
|vioJOYright       | Joystick Right
|vioJOYselect      | Joystick Select Button
|vioLED0           | RGB LED Red
|vioLED1           | RGB LED Green
|vioLED2           | RGB LED Blue

### CMSIS-Driver vStream configuration

| Driver                 | Stream Format Description
|:-----------------------|:----------------------------------------------------
| Driver_vStreamAudioIn  | 16-bit PCM audio,      16000 samples/second

## SETOOLS

Before using layers on the board it is required to program the ATOC of the device
using the Alif SETOOLS. The required `.vscode\tasks.json` commands are part of the
Blinky examples. It is therefore recommended to start with such an example.

Refer to the section [Usage](https://www.keil.arm.com/packs/ensemble-alifsemiconductor)
in the overview page of the Alif Semiconductor Ensemble DFP/BSP for information on how
to setup these tools.

In VS Code use the menu command **Terminal - Run Tasks** and execute:

- "Alif: Install M55_HE or M55_HP debug stubs (single core configuration)"

> Note: For Windows ensure that the Terminal default is `Git Bash` or `PowerShell`.
