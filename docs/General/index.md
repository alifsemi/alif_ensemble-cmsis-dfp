# Alif Ensemble CMSIS-DFP

The **Alif Semiconductor Ensemble Device Family Pack** (`AlifSemiconductor::Ensemble`) is the CMSIS-Pack for the Ensemble family of AI/ML microcontrollers and fusion processors. It gives compilers and debuggers one device description, startup and peripheral headers, CMSIS-Drivers, flash algorithms, SVD debug views, and board layers for the supported kits.

This book follows the same chapter order as the [CMSIS 6 introduction](https://arm-software.github.io/CMSIS_6/main/General/index.html): what the delivery contains, why it is useful, how development is organized, coding rules, validation, migration, and license. The extra chapters are the ones a device pack needs: devices, boards, SETOOLS, examples, and a glossary.

Ensemble devices scale from a single Arm Cortex-M55 to multi-core fusion processors. Higher series add a second Cortex-M55, up to two Arm Cortex-A32 application cores, and Arm Ethos-U55 or Ethos-U85 neural processing units. Every series includes an isolated secure enclave. Alif [aiPM](https://alifsemi.com/) technology powers only the blocks the application is using. The public lineup is on the [Ensemble family page](https://alifsemi.com/products/ensemble/) and [alifsemi.com](https://alifsemi.com/).

This pack builds and debugs the Cortex-M55 real-time subsystem (`M55_HP` and `M55_HE`). Cortex-A32 cores on E5 and E7 are part of the silicon. They are not a Linux board-support package in this delivery.

## What you can do with this pack

- Start a bare-metal or Zephyr application from a template in VS Code.
- Run the same CMSIS Reference Application on E7 and E8 kits by selecting a board layer.
- Program on-chip MRAM and debug with a peripheral register view.
- Call CMSIS-Drivers for CAN, GPIO, I2C, SAI, SPI, and USART.
- Load the secure-enclave Application Table of Contents (ATOC) before the first debug session.

Ready-to-use AI/ML applications that use Ethos-U are published on [github.com/Arm-Examples](https://github.com/Arm-Examples).

## Contents

- @subpage pack_page "Pack contents"
- @subpage setup_page "Setup and first build"
- @subpage gs_page "Getting started"
- @subpage devices_page "Devices"
- @subpage boards_page "Boards"
- @subpage setools_page "SETOOLS and ATOC"
- @subpage drivers_page "Drivers"
- @subpage examples_page "Examples"
- @subpage migrate_page "Migration"
- @subpage history_page "Revision history"
- @subpage rules_page "Documentation rules"
- @subpage validate_page "Validation"
- @subpage glossary_page "Glossary"
- @subpage trouble_page "Troubleshooting"
- @subpage license_page "License and links"

## Where the project lives

The pack is developed in [alifsemi/alif_ensemble-cmsis-dfp](https://github.com/alifsemi/alif_ensemble-cmsis-dfp). Support is [support@alifsemi.com](mailto:support@alifsemi.com). CMSIS-Core, CMSIS-Compiler, and the Ethos-U pack are Arm components under the [CMSIS 6](https://arm-software.github.io/CMSIS_6/main/General/index.html) project and are installed beside this pack.
