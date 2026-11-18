# Migration {#migrate_page}

Release 2.0.0 and later are not backward compatible with 1.3.4 or older. Existing projects should follow the [CMSIS DFP 2.0 Migration Guide (AAPN0038)](https://alifsemi.com/download/AAPN0038).

2.0.0 restructured the pack. It added E1C and E8 device support, CSolution examples, board-support libraries for the E7 and E8 DevKits, and new GPIO, OSPI, and secure-enclave service behavior. A project that still includes 1.x paths, the old `board_config` secure-enclave init call, or a pre-2.0 retarget header will not build unchanged.

CMSIS-Core 6.2.0 is the baseline this pack is built against. In CMSIS 6.0.0 the Cortex-M core headers changed and are not source-compatible with CMSIS 5. Use the CMSIS 6 pack. The note is on the [CMSIS 6 introduction](https://arm-software.github.io/CMSIS_6/main/General/index.html), under "Migration from CMSIS v5".

## Checklist

1. Read AAPN0038 and apply it to the existing project.
2. Install `ARM::CMSIS` 6.2.0 or later, `ARM::CMSIS-Ethos-U` 1.0.0 or later, and `ARM::CMSIS-Compiler` 2.1.0.
3. Recreate the solution from the Alif CMSIS or Alif Zephyr template, or retarget the existing `csolution` to the 2.x board layer.
4. Program a new ATOC before the first debug session on 2.x. See @ref setools_page.
5. Rebuild with `cbuild` and GCC or AC6. Review warnings in I2C, SPI, UART, GPIO, and the secure-enclave service as source changes.

## What changed for a 1.x project

| Area | 2.x behavior |
|---|---|
| Pack layout | Restructured in 2.0.0. Include paths from 1.3.4 do not apply |
| Devices | E1C and E8 added in 2.0.0. E4, E5, and E3 SKUs extended in 2.2.0 |
| GPIO | New GPIO instances. Flex I/O uses a secure-enclave service call |
| Retarget | `retarget_config.h` follows the 2.0.0 config file |
| Secure enclave | SE service init was removed from `board_config`. Host services track SE firmware (1.07 in 2.0.0, 1.09 in 2.1.0, 1.10 in 2.2.0) |
| Examples | CSolution projects replaced the old example layout |
