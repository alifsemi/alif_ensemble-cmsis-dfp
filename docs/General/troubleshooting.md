# Troubleshooting {#trouble_page}

## The debugger does not connect

Program the ATOC before the first debug session. In VS Code run **Terminal > Run Task** and choose the single-core or dual-core stub task. The task is described on @ref setools_page.

Confirm `alif.setools.root` points at the SETOOLS install directory. Confirm the kit is the part selected by `tools-config`. An E7 configuration will not program an E8 or E1C board.

Use J-Link 8.84 or later when the debug adapter is J-Link. The E8 and E1C kits have an on-board J-Link. The E7 DevKit uses the Cortex debug connectors J16 and J17.

![E7 DevKit connector J16, with the debug ribbon fitted](e7-devkit-debug.jpg)

![E7 AI/ML AppKit debug connection](e7-appkit-debug.jpg)

## SETOOLS does not see the device

SETOOLS uses the secure UART, not the application console.

| Kit | Switch for SETOOLS | Switch for application `printf` |
|---|---|---|
| E8 DevKit | SW4 in position SE (Secure UART) | SW4 in position U4 (UART4) |
| E7 AI/ML AppKit | J15-A and J15-B in position SE | J15-A and J15-B in position U4 (UART4) |
| E7 DevKit | Secure-UART position used by the ATOC task | J26-A and J26-B in position 4 (UART4) for STDOUT |

![E7 DevKit top side, used to locate the UART jumpers](e7-devkit-top.jpg)

The sample SETOOLS log asks for a COM port when more than one port is present. Pick the PRG USB port, not the debug probe.

## The LED blinks and the console is silent

The examples print on UART4 at 115200 baud. The UART switch must be in the U4 position after the ATOC has been programmed. A terminal left on the secure-UART port will not show `"Blinky example"` or `"DualCore example"`.

## The wrong core is running

Single-core examples need the single-core ATOC task. Dual-core examples need the task that installs both M55_HE and M55_HP stubs. Rebuild after changing that configuration.

Select the processor view that matches the project. `M55_HP` is compiled with `RTSS_HP`. `M55_HE` is compiled with `RTSS_HE`. A layer named `Board_HP-U85` is the E8 High-Performance layer for Ethos-U85, not the Ethos-U55 layer.

## The project does not build after a pack upgrade

Release 2.0.0 and later are not source-compatible with 1.3.4 or older. Follow @ref migrate_page and [AAPN0038](https://alifsemi.com/download/AAPN0038).

A project that still uses CMSIS 5 core headers fails against this pack. CMSIS 6.0.0 changed the Cortex-M headers. Install `ARM::CMSIS` 6.2.0 or later. The [CMSIS 6 migration note](https://arm-software.github.io/CMSIS_6/main/General/index.html) describes that header change.
