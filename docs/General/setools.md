# SETOOLS and ATOC {#setools_page}

The host PC runs [SETOOLS](https://swrm.alifsemi.com/Content/3.4%20SETOOLS.htm) and talks to the kit through the onboard UART-to-USB adapter (PRG USB). The IDE uses a separate debug probe.

SETOOLS builds and programs the Application Table of Contents. Program an ATOC that contains the M55 debug stubs before the first debug session, and again whenever the processor setup changes. After that, rebuild and debug the application without writing the ATOC again.

Download SETOOLS from the Alif [Software & Tools](https://alifsemi.com/support/software-tools/ensemble/) page. Registration is required. Section 4 of the [Alif Security Toolkit Quick Start Guide](https://alifsemi.com/download/AQSG0002) describes target configuration.

## Configure the target

With the kit connected, automatic configuration is:

    tools-config -a

To select the part yourself, the commands already recorded in `docs/Overview.md` are:

| Kit | Command |
|---|---|
| E7 AppKit | `tools-config -p "E7 (AE722F80F55D5LS) - 5.5 MRAM / 13.5 SRAM" -r B4` |
| E7 DevKit | `tools-config -p "E7 (AE722F80F55D5LS) - 5.5 MRAM / 13.5 SRAM" -r B4` |
| E1C DevKit | `tools-config -p "E1C (AE1C1F4051920PH) - 1.86 MRAM / 2.0 SRAM" -r A7` |
| B1 DevKit | `tools-config -p "B1 (AB1C1F4M51820PH) - 1.8 MRAM / 2.0 SRAM" -r A0` |
| E8 AppKit | `tools-config -p "E8 (AE822FA0E5597BS0) - 5.5 MRAM / 9.75 SRAM" -r A1` |
| E8 DevKit | `tools-config -p "E8 (AE822FA0E5597LS0) - 5.5 MRAM / 9.75 SRAM" -r A1` |

B1 is the Edge AI + Wireless series on [alifsemi.com](https://alifsemi.com/). The command is the SETOOLS selection from the current overview. This pack has no B1 device or board entry.

Put the UART switch in the SE position before this step. See @ref trouble_page.

## VS Code tasks

CMSIS-Toolbox examples ship a `tasks.json` that calls SETOOLS.

| Task | Use it for |
|---|---|
| Alif: Install M55_HE or M55_HP debug stubs (single core configuration) | Blinky and Hello World |
| Alif: Install M55_HE and M55_HP debug stubs (dual core configuration) | DualCore |

Set the install path once. Open Settings (`Ctrl+,`), switch to the JSON view, and add the path on that PC:

    {
        "alif.setools.root": "C:\\Alif\\SETOOLS"
    }

Then run **Terminal > Run Task** and choose the single-core or dual-core task.

The task copies the MRAM stub binaries and the JSON configuration into the SETOOLS build tree, runs `app-gen-toc`, and runs `app-write-mram`. A successful run prints the part number, the APP MRAM base and size, and a completed download of the stub and `AppTocPackage.bin`. The long sample log in `docs/Overview.md` was captured on an E7, revision B2, from a Windows host on 2025-07-02. E8 and E1C print a different part line. Linux hosts print different paths.
