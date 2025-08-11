# Blinky project

The **Blinky** project is a simple example that can be used to verify the
basic tool setup.

It is compliant to the Cortex Microcontroller Software Interface Standard (CMSIS)
and uses the CMSIS-RTOS2 API interface for RTOS functionality. The CMSIS-RTOS2 API
is available with various real-time operating systems, for example RTX5 or FreeRTOS.

## JLink

When using the JLink debug adapter ensure that JLink version 8.44 or higher is installed.

## SETOOLS

Before using examples on the board it is required to program the ATOC of the device
using the Alif SETOOLS.

Refer to the section [Usage](https://www.keil.arm.com/packs/ensemble-alifsemiconductor)
in the overview page of the Alif Semiconductor Ensemble DFP/BSP for information on how
to setup these tools.

In VS Code use the menu command **Terminal - Run Tasks** and execute:

- "Alif: Install M55_HE or M55_HP debug stubs (single core configuration)"

> Note:
>
> - For Windows ensure that the Terminal default is `Git Bash` or `PowerShell`.
> - Configure J15-A & J15-B to position SE (Secure UART) to enable SETOOLS communication with the device.

## Operation

- At start:
  - outputs "Blinky example" to STDOUT (UART4, baudrate 115200bps)
  - blinks vioLED0 in 1 sec interval.
- The vioBUTTON0 changes the blink frequency and start/stops vioLED1.

### CMSIS-Driver Virtual I/O mapping

| CMSIS-Driver VIO  | Physical resource
|:------------------|:----------------------
| vioBUTTON0        | Joystick Select Button
| vioLED0           | RGB LED Red
| vioLED1           | RGB LED Green

> Note:
> For STDOUT (printf output) configure J15-A & J15-B to position U4 (UART4)
