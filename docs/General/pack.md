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
| Secure enclave services | Target-side services that talk to the secure enclave over MHU. This library does not build the ATOC | `se_services/` | @ref setools_page |
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

CMSIS-DSP and CMSIS-NN are separate Arm packs. CMSIS-RTOS2 is the RTOS API inside `ARM::CMSIS`. It is not a kernel and it is not a separate pack.

The E7 and E8 DevKit Blinky projects are bare-metal. They do not select an RTOS. The E7 AppKit Blinky projects and the DualCore projects select `CMSIS:RTOS2:Keil RTX5&Source` and require `ARM::CMSIS-RTX@^5.9.0`. Installing FreeRTOS does not satisfy that selection.

## What this pack does not contain

- Linux or Cortex-A32 board support for E5 and E7.
- SETOOLS, including the PC-side `app-gen-toc` tool that builds the ATOC. Download it from the Alif [Software & Tools](https://alifsemi.com/support/software-tools/ensemble/) page. Registration is required. The VS Code tasks call that tool. See @ref setools_page.
- The CMSIS-Toolbox and the CMSIS Solution VS Code extension. Those are Arm tools described in the [CMSIS 6 tools list](https://arm-software.github.io/CMSIS_6/main/General/index.html).
