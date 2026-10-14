# Zephyr Template for Alif

The following template can be used to create applications with Zephyr for [Alif Essemble](https://www.keil.arm.com/packs/ensemble-alifsemiconductor) devices.

## Steps to create an application

- Select from CMSIS View the menu command: Create a Solution
- Choose a Target Board that is based on an Alif Semicondutor Essemble device.
- Select a folder for the CMSIS Solution project.
- Install Zephyr.
- Add to the file `Zephry.csolution.yml` the references to the Zephry application.
- Use from CMSIS View the menu command: Manage Solution Settings, select Zephry applications, and configure a Debug Adapter.
- Use from CMSIS View the menu command: Build solution.
- Configure the device using the [Alif SETools](https://www.keil.arm.com/packs/ensemble-alifsemiconductor).
- Use from CMSIS View the menu command: Load & Run or Load & Debug application.

## Installation of Zephyr

Refer to [github.com/Arm-Examples/cmsis-zephyr](https://github.com/Arm-Examples/cmsis-zephyr).

## Examples

For examples, refer to [github.com/Open-CMSIS-Pack/Zephyr-Alif](https://github.com/Open-CMSIS-Pack/Zephyr-Alif).
