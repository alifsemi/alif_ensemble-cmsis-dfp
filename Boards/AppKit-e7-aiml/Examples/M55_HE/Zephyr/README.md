# Zephyr project

This example project demonstrates Zephyr debugging using Arm Keil Studio extensions for VS Code.

The debug launch.json file is adapted to load the elf image built by the Zephyr West tool. It can
be then debugged using the CMSIS-debugger extension.

## Setup the Zephyr environment

Follow these steps to build the Zephyr image to use with this example project:

1. Install dependencies to your PC: python3, python3-pip, wget, cmake, ninja
2. Install west and pyelftools by executing the following command:
   - `python3 -m pip install west pyelftools`

3. Add the paths to the mentioned executables to your PATH environment variable: `python3`, `pip`, `wget`, `cmake`, and `ninja`
4. To reuse the Arm GNU compiler toolchain installed by vcpkg in Keil Studio VS Code, add system environment variables:
   - `ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb` and
   - `GNUARMEMB_TOOLCHAIN_PATH=path/to/.../.vcpkg/artifacts/2139c4c6/compilers.arm.arm.none.eabi.gcc/14.2.1`

     *(replace path/to/.../ with the actual path)*

More details can be found in the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).

## Build the example project

1. Create a new directory e.g. called `sdk-alif` (outside of this project directory)
2. Go into sdk-alif directory and run the command:
   - `west init -m https://github.com/alifsemi/sdk-alif.git --mr main`

     and then the command:

   - `west update`

3. Now the entire Alif SDK together with Zephyr is downloaded from the repo and initialized locally in the sdk-alif directory
4. To build the Hello World example, go to the `zephyr` subdirectory inside the sdk-alif directory and run the following command:
   - `west build -p auto -b alif_e7_dk_rtss_he samples/hello_world -DCONFIG_DEBUG=y`.

     This builds the application image for execution from MRAM by the HE core of Alif DevKit-E7.

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

## Load and debug the example project

The build output image file is called `zephyr.elf` and is located in the default build output directory:

- `sdk-alif/zephyr/build/zephyr`

Note down the absolute path to the generated executable image e.g. `path/to/.../sdk-alif/zephyr/build/zephyr/zephyr.elf`

This project already contains launch configuration in `launch.json` but in general, to configure debugging, one should follow the
[Create a launch configuration](https://open-cmsis-pack.github.io/vscode-cmsis-debugger/configure.html) instructions and add the
CMSIS Debugger: pyOCD configuration to the `launch.json` file.

Existing launch configuration must be edited to point to the generated Zephyr image file. Edit the `"program"` line in `launch.json`:

- `"program": "/path/to/.../sdk-alif/zephyr/build/zephyr/zephyr.elf",`

J-Link debugger is connected to the J2 USB connector (PRG USB). Connect a USB-Micro cable to the J3 port and start
the debug session in VS Code. Open a serial port connection to the board, run the application, and the Hello World
message will be printed via the serial port.

> NOTE: When using the J-Link debugger ensure that J-Link version 8.44 or higher is installed.
