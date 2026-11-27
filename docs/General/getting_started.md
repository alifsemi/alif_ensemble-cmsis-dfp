# Getting started {#gs_page}

![E8 DevKit](DK-E8_small.jpg)

The complete install and the first `cbuild` command are on @ref setup_page. This page is the shorter map of the same flow.

Install CMSIS-Toolbox 2.12 or later, this pack, `ARM::CMSIS` 6.2.0 or later, and `ARM::CMSIS-Compiler` 2.1.0 or later. Install `ARM::CMSIS-Ethos-U` 1.0.0 or later when the application uses an Ethos-U. Install SETOOLS and a debug probe (J-Link 8.84 or later, CMSIS-DAP, or ULINKpro) before the first download. The compile itself does not need the probe.

## Create a new application

In VS Code, open the command palette and run **CMSIS: Create a Solution**. The templates live in `Boards/Templates`.

| Template | Result |
|---|---|
| Alif CMSIS | Bare-metal application for an Ensemble device |
| Alif Zephyr | Zephyr application built with West |

Choose the board, then the mounted part. For the E8 DevKit that is `AE822FA0E5597LS0`. For the E7 DevKit and the E7 AI/ML AppKit that is `AE722F80F55D5LS`.

## Use a CMSIS Reference Application

The board layers support device-independent [CMSIS Reference Applications](https://open-cmsis-pack.github.io/cmsis-toolbox/ReferenceApplications/).

After that flow creates a solution, copy these directories from `Boards/Templates/CMSIS` to the root of the generated solution:

    .alif/
    .vscode.d/

`.alif` holds the M55 MRAM debug stubs and the SETOOLS configuration. `.vscode.d` holds the VS Code tasks that generate and program the ATOC. Both directories are hidden. Include them in the copy, then run **Refresh** in CMSIS View. A VS Code restart is not required.

## Prepare, build, and debug

1. Connect the kit. SETOOLS uses the onboard UART-to-USB port. The IDE uses the debug probe.
2. Put the secure-UART switch in the SE position. On the E8 DevKit that is SW4. On the E7 AI/ML AppKit that is J15-A and J15-B. See @ref trouble_page.
3. Configure the target and program the ATOC. See @ref setools_page. Do this once before the first debug session, and again after any processor-configuration change.
4. Move the UART switch to the application console (UART4) if you want `printf` output.
5. Build in CMSIS View, or follow @ref setup_page. From an example directory the command is:

       cbuild ./<project>.csolution.yml --packs --toolchain GCC

   Use `AC6` instead of `GCC` for Arm Compiler 6. The example solutions request `AlifSemiconductor::Ensemble@^2.2.1-0`. Register the local `AlifSemiconductor.Ensemble.pdsc` before building.

6. Start the debug session. If the ATOC has not been programmed, the debug connection fails.

## First programs to run

| Goal | Example | Expected result |
|---|---|---|
| Prove the compile | E8 DevKit Blinky_HP | `cbuild` produces an ELF. The E8 Blinky README does not record the UART text |
| Prove the console and the LED | E7 AppKit Blinky on M55_HP | `Blinky example` on UART4 at 115200 baud, red LED toggling once per second |
| Prove both cores | DualCore on E7 or E8 | `DualCore example` on UART4, red LED from M55_HP, green LED from M55_HE |
