# Dual Core Project

The **Dual Core** project is an example project that shows how to configure dual core application.

## Project Structure

The DualCore project is generated using the [CMSIS-Toolbox](https://open-cmsis-pack.github.io/cmsis-toolbox/build-overview)
and is defined in [`csolution`](https://open-cmsis-pack.github.io/cmsis-toolbox/YML-Input-Format) format:

- [`DualCore.csolution.yml`](./DualCore.csolution.yml) lists the required packs, defines, hardware targets, and the build-types (along with the compiler).
- [`M55_HP.cproject.yml`](./M55_HP.cproject.yml) defines the source files and the software components for the M55_HP core.
- [`M55_HE.cproject.yml`](./M55_HE.cproject.yml) defines the source files and the software components for the M55_HE core.

## JLink

When using the JLink debug adapter ensure that JLink version 8.44 or higher is installed.

## SETOOLS

Before using examples on the board it is required to program the ATOC of the device
using the Alif SETOOLS.

Refer to the section [Usage](https://www.keil.arm.com/packs/ensemble-alifsemiconductor)
in the overview page of the Alif Semiconductor Ensemble DFP/BSP for information on how
to setup these tools.

In VS Code use the menu command **Terminal - Run Tasks** and execute:

- "Alif: Install M55_HE and M55_HP debug stubs (dual core configuration)"

> Note: For Windows ensure that the Terminal default is `Git Bash` or `PowerShell`.

## Operation

M55_HP (High Performance) and M55_HE (High Efficiency) are two independent Cortex-M55 cores that share common peripherals
and memory resources. Each core is executing its own instance of CMSIS-RTOS2.

- M55 High Performance core:
  - configures peripherals (GPIO, STDOUT)
  - outputs "DualCore example" to STDOUT (UART4, baudrate 115200bps)
  - toggles vioLED0 in 100ms interval
  - reads vioBUTTON0 state to switch vioLED0 toggling frequency between 100ms and 500ms

- M55 High Efficiency core:
  - toggles vioLED1 in 500ms interval

### CMSIS-Driver Virtual I/O mapping

| CMSIS-Driver VIO  | Physical resource
|:------------------|:----------------------
| vioBUTTON0        | Joystick Select Button
| vioLED0           | RGB LED Red
| vioLED1           | RGB LED Green

> Note:
> For STDOUT (printf output) configure J26-A & J26-B to position 4 (UART4)
