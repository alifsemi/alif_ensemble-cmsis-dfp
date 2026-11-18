# Pack contents {#pack_page}

Pack name: `AlifSemiconductor::Ensemble`  
Documented release: **2.3.0** (16 November 2026)  
Repository: [alifsemi/alif_ensemble-cmsis-dfp](https://github.com/alifsemi/alif_ensemble-cmsis-dfp)

The rows follow the CMSIS component table: what it is, where the sources live, and which guide page describes it.

| Content | What it provides | Sources | Guide |
|---|---|---|---|
| Device headers and startup | Cortex-M55 startup, `alif.h`, processor views `RTSS_HP` and `RTSS_HE` | `Device/` | @ref devices_page |
| SVD | Peripheral-aware debug view per part number | `Debug/SVD/` | @ref devices_page |
| Flash algorithms | On-chip MRAM, and external IS25WX256 where fitted | `Flash/algorithms/` | @ref devices_page |
| CMSIS-Drivers | CAN, GPIO, I2C, SAI, SPI, USART, plus SD, OSPI, camera, and ISP | `drivers/include/` | @ref drivers_page |
| Secure enclave services | Host services that configure the device and build the ATOC | `se_services/` | @ref setools_page |
| Board layers | `Board_HP.clayer.yml` and `Board_HE.clayer.yml`. E8 also has Ethos-U85 layers | `Boards/` | @ref boards_page |
| Examples | Blinky, Hello World, and dual-core projects | `Boards/*/Examples` | @ref examples_page |
| Templates | Bare-metal CMSIS solution and Zephyr solution | `Boards/Templates` | @ref gs_page |

## Required packs

Install these beside this DFP. Versions are the minimum named in `docs/Overview.md`.

| Pack | Minimum | Used for |
|---|---|---|
| `ARM::CMSIS` | 6.2.0 | CMSIS-Core for Cortex-M55 |
| `ARM::CMSIS-Ethos-U` | 1.0.0 | Ethos-U driver API |
| `ARM::CMSIS-Compiler` | 2.1.0 | STDIN, STDOUT, and STDERR retarget |

CMSIS-DSP, CMSIS-NN, and CMSIS-RTOS2 are separate Arm packs. Blinky and DualCore use the CMSIS-RTOS2 API, so install an RTOS pack (RTX5 or FreeRTOS) when you build those examples.

## What this pack does not contain

- Linux or Cortex-A32 board support for E5 and E7.
- SETOOLS. Download it from the Alif [Software & Tools](https://alifsemi.com/support/software-tools/ensemble/) page. Registration is required.
- The CMSIS-Toolbox and the CMSIS Solution VS Code extension. Those are Arm tools described in the [CMSIS 6 tools list](https://arm-software.github.io/CMSIS_6/main/General/index.html).
