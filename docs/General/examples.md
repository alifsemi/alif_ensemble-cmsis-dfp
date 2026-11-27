# Examples {#examples_page}

Build an example from its directory with the same command the pack CI uses. Install the tools first. See @ref setup_page.

    cbuild ./<project>.csolution.yml --packs --toolchain GCC

Replace `GCC` with `AC6` for Arm Compiler 6. Program the ATOC before debugging. See @ref setools_page. J-Link users need version 8.84 or later.

Every solution in the table below requests `AlifSemiconductor::Ensemble@^2.2.1-0`. Register the local pdsc so the build uses this tree.

The CI workflow `.github/workflows/Test-Examples.yml` lists the projects marked **In CI matrix**. A row in that file means the workflow tries to compile the project. It does not record that the latest run passed, and it does not flash a board. See @ref validate_page.

## Blinky

The E7 AppKit Blinky is a CMSIS-RTOS2 application. Its project file selects `CMSIS:RTOS2:Keil RTX5&Source`. The expected UART and LED behavior below is from `Boards/AppKit-e7-aiml/Examples/M55_HP/Blinky/README.md`.

The E8 DevKit and E7 DevKit Blinky projects do not select an RTOS component. Their READMEs describe how to create the solution. They do not record the UART string. Do not copy the AppKit sentence onto those boards until the hardware log confirms it.

The steps in the next table are only the E7 AppKit Blinky README.

| Step | What happens |
|---|---|
| ATOC | Single-core task: "Alif: Install M55_HE or M55_HP debug stubs" |
| Console | UART4, 115200 baud, after the switch is moved to U4 |
| At start | Prints `Blinky example`. Toggles `vioLED0` once per second |
| Button | `vioBUTTON0` changes the blink rate and starts or stops `vioLED1` |

| VIO signal | Pin function |
|---|---|
| `vioBUTTON0` | Joystick select |
| `vioLED0` | RGB LED, red |
| `vioLED1` | RGB LED, green |

The same VIO names are used in the E8 DualCore README.

| Project | Board | RTOS component | In CI matrix |
|---|---|---|---|
| `Boards/AppKit-e7-aiml/Examples/M55_HP/Blinky` | E7 AI/ML AppKit | Keil RTX5 | GCC and AC6 |
| `Boards/AppKit-e7-aiml/Examples/M55_HE/Blinky` | E7 AI/ML AppKit | Keil RTX5 | GCC and AC6 |
| `Boards/DevKit-e7/Examples/Blinky_HP` | E7 DevKit, part `AE722F80F55D5LS` | Not selected | GCC and AC6 |
| `Boards/DevKit-e7/Examples/Blinky_HE` | E7 DevKit | Not selected | GCC and AC6 |
| `Boards/DevKit-e8/Examples/Blinky_HP` | E8 DevKit, part `AE822FA0E5597LS0` | Not selected | GCC and AC6 |
| `Boards/DevKit-e8/Examples/Blinky_HE` | E8 DevKit | Not selected | GCC and AC6 |

## DualCore

Source of the expected result: `Boards/DevKit-e8/Examples/DualCore/README.md` and `Boards/DevKit-e7/Examples/Dual_Core/README.md`.

Each core runs its own CMSIS-RTOS2 instance. They share peripherals and memory.

| Core | Behavior |
|---|---|
| M55_HP | Configures GPIO and STDOUT. Prints `DualCore example`. Toggles `vioLED0` every 100 ms. `vioBUTTON0` switches that period between 100 ms and 500 ms |
| M55_HE | Toggles `vioLED1` every 500 ms |

| Project | Board | ATOC task | In CI matrix |
|---|---|---|---|
| `Boards/DevKit-e8/Examples/DualCore` | E8 DevKit. STDOUT switch is SW4 = U4 | Both stubs | GCC and AC6 |
| `Boards/DevKit-e7/Examples/Dual_Core` | E7 DevKit. STDOUT jumpers J26-A and J26-B = 4 | Both stubs | GCC and AC6 |
| `Boards/AppKit-e7-aiml/Examples/M55_HP_HE/DualCore` | E7 AI/ML AppKit. STDOUT jumpers J15 = U4 | Both stubs | GCC and AC6 |

## Hello World

`Boards/DevKit-e8/Examples/Hello_World` is a CMSIS-Toolbox project for M55_HE on `AE822FA0E5597LS0`. Its README describes how to create the solution in VS Code. It does not record the console text, so this page does not invent one. The E7 DevKit also lists a µVision Hello World on M55_HP. Neither project is in the CI matrix. Program the single-core ATOC before debugging.

## µVision projects on the E7 DevKit

The pack overview lists these µVision examples in addition to the CMSIS-Toolbox builds:

| Example | Core |
|---|---|
| Blinky_M55_HE | M55 High-Efficiency, running from MRAM |
| Blinky_M55_HE_TCM | M55 High-Efficiency, running from TCM |
| HelloWorld_M55_HP | M55 High-Performance |

## Creating a project from the template

The short READMEs under several example folders describe the VS Code flow:

1. Command palette, **CMSIS: Create a Solution**.
2. Board: Alif Semiconductor, then DevKit-E8 or DevKit-E7.
3. Device: `AE822FA0E5597LS0` on E8, or `AE722F80F55D5LS` on E7.
4. Template: local CSolution example.
5. Choose a folder and create.

Those READMEs name older pack versions (2.0.0, 2.0.2, 2.1.0). The pack to install for this book is 2.3.0.
