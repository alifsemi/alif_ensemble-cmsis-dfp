# Setup and first build {#setup_page}

This is the path that produces a Blinky image for the E8 DevKit. The same commands build the other examples in @ref examples_page. Drivers are not separate programs. After this build works, enable a driver by adding its component, as described on @ref drivers_page.

![E8 DevKit, the board used for the first build](DK-E8_small.jpg)

## 1. Install the tools

| Tool | Version used by this pack | Purpose |
|---|---|---|
| CMSIS-Toolbox | 2.12 or later. The E8 Blinky solution is marked `created-for: CMSIS-Toolbox@2.12.0` | Provides `cbuild` and `cpackget` |
| CMSIS Solution extension | Current marketplace release | **CMSIS: Create a Solution** in VS Code |
| GCC or Arm Compiler 6 | Either. The CI builds both | Compiler selected with `--toolchain GCC` or `--toolchain AC6` |
| J-Link | 8.84 or later, when the probe is a J-Link | Debug. The E8 DevKit has an on-board probe |
| SETOOLS | Current Alif release, from the [Software & Tools](https://alifsemi.com/support/software-tools/ensemble/) page | Programs the ATOC. Required before the first debug session, not before the compile |

CMSIS-Toolbox is the Arm command-line package described with the [CMSIS 6 tools](https://arm-software.github.io/CMSIS_6/main/General/index.html). Install it so `cbuild` and `cpackget` are on the PATH. Confirm with:

    cbuild --version

## 2. Register this pack

From the root of this repository, the same registration the CI uses is:

    cpackget init https://www.keil.com/pack/index.pidx
    cpackget update-index
    cpackget add AlifSemiconductor.Ensemble.pdsc

`cpackget add` with no version argument registers the local `.pdsc`. Also install the Arm packs named by the example solutions:

    cpackget add ARM::CMSIS@^6.2.0
    cpackget add ARM::CMSIS-Compiler@^2.1.0

Add an RTOS pack only for an example that lists one. The E7 AppKit Blinky lists `CMSIS:RTOS2:Keil RTX5&Source`. The E8 DevKit Blinky does not.

`ARM::CMSIS-Ethos-U@^1.0.0` is required for an Ethos-U application. Blinky does not select it.

## 3. Which pack version the examples accept

Every Blinky and DualCore solution in this tree contains:

    - pack: AlifSemiconductor::Ensemble@^2.2.1-0

That range is 2.2.1 or newer, below 3.0.0. Register the `.pdsc` from this tree before building, so `cbuild` resolves the pack you have open. The documentation release described in this book is 2.3.0. If the resolver reports that the pack version does not match, the solution file is still asking for `^2.2.1-0`.

## 4. Build Blinky

    cd Boards/DevKit-e8/Examples/Blinky_HP
    cbuild ./Blinky.csolution.yml --packs --toolchain GCC

Use `--toolchain AC6` for Arm Compiler 6. `cbuild` writes the ELF, BIN, and HEX files for the `M55_HP` project. This compiles the device startup, GPIO, pin configuration, and `CMSIS Driver:USART` because those components are listed in `M55_HP/M55_HP.cproject.yml`. It does not compile CAN, I2C, SPI, or the camera driver.

## 5. Program the ATOC and check the board

Build can finish before the board is programmed. To run the image:

1. Connect the E8 DevKit. Put SW4 in the SE position.
2. Run the VS Code task **Alif: Install M55_HE or M55_HP debug stubs (single core configuration)**. See @ref setools_page.
3. Move SW4 to U4.
4. Start the debug session and open a terminal at 115200 baud on UART4.

The E7 AppKit Blinky prints `Blinky example` and toggles the red LED once per second. Use that sentence for the E7 AppKit project. The E8 Blinky README does not record the UART string, so confirm the console on the hardware before writing it into a test log.

## 6. Build another example

Repeat step 4 in that example's directory. The project file name differs:

| Example | Directory | Solution file |
|---|---|---|
| E8 Blinky, High-Performance | `Boards/DevKit-e8/Examples/Blinky_HP` | `Blinky.csolution.yml` |
| E8 Blinky, High-Efficiency | `Boards/DevKit-e8/Examples/Blinky_HE` | `Blinky.csolution.yml` |
| E8 DualCore | `Boards/DevKit-e8/Examples/DualCore` | `DualCore.csolution.yml` |
| E7 DevKit Blinky, High-Performance | `Boards/DevKit-e7/Examples/Blinky_HP` | `Blinky_HP.csolution.yml` |
| E7 AppKit Blinky, High-Performance | `Boards/AppKit-e7-aiml/Examples/M55_HP/Blinky` | `Blinky.csolution.yml` |

DualCore needs the ATOC task that installs both cores. See @ref trouble_page if the debugger does not connect.
