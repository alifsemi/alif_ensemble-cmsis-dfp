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
4. To build the Message Handling Unit (MHU) example, go to the `zephyr` subdirectory inside the sdk-alif directory. To build
   the application image for the M55_HE core run the following command:
   - `west build -d ../build_M55_HE/ -p auto -b alif_e7_dk_rtss_he ../alif/samples/drivers/ipm/ipm_arm_mhuv2/ -DCONFIG_DEBUG=y -DCONFIG_DEBUG_THREAD_INFO=y -DSE_SERVICES=OFF -DRTSS_HP_MHU0=ON`

     and then for the M55_HP core:

   - `west build -d ../build_M55_HP/ -p auto -b alif_e7_dk_rtss_hp ../alif/samples/drivers/ipm/ipm_arm_mhuv2/ -DCONFIG_DEBUG=y -DCONFIG_DEBUG_THREAD_INFO=y -DSE_SERVICES=OFF -DRTSS_HE_MHU0=ON`

     This builds two application images for execution from MRAM by the HE and HP core of Alif DevKit-E7.

## SETOOLS

Before using examples on the board it is required to program the ATOC of the device
using the Alif SETOOLS.

Refer to the section [Usage](https://www.keil.arm.com/packs/ensemble-alifsemiconductor)
in the overview page of the Alif Semiconductor Ensemble DFP/BSP for information on how
to setup these tools.

In VS Code use the menu command **Terminal - Run Tasks** and execute:

- "Alif: Install M55_HE and M55_HP debug stubs (dual core configuration)"

> Note: For Windows ensure that the Terminal default is `Git Bash` or `PowerShell`.

## Load and debug the example project

The build output image file is called `zephyr.elf` and is located in the specified build output directory:

- `sdk-alif/build_M55_HE/rtss_he/zephyr`

Note down the absolute path to the generated executable image e.g. `path/to/.../sdk-alif/build_M55_HE/rtss_he/zephyr/zephyr.elf`

This project already contains launch configuration in `launch.json` but in general, to configure debugging, one should follow the
[Create a launch configuration](https://open-cmsis-pack.github.io/vscode-cmsis-debugger/configure.html) instructions and add the
CMSIS Debugger: pyOCD configuration to the `launch.json` file.

Existing launch configuration must be edited to point to the generated Zephyr image file. Edit the `"program"` line in `launch.json`:

- `"program": "/path/to/.../sdk-alif/build_M55_HE/rtss_he/zephyr/zephyr.elf",`
- `"program": "/path/to/.../sdk-alif/build_M55_HP/rtss_hp/zephyr/zephyr.elf",`

Now connect a CMSIS-DAP debugger such as ULINKplus to the J17 debug port of the Alif E7 DK board. Connect a USB-Micro cable to the J3 port,
start the debug session in VS Code, and open a serial port connection to the board. Run the application, and the Hello World printf output
will appear via the serial port.

> NOTE: When using the J-Link debugger ensure that J-Link version 8.44 or higher is installed.
