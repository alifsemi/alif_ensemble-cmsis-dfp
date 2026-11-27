# Revision history {#history_page}

The pack version is the `<release>` entry in `AlifSemiconductor.Ensemble.pdsc`. Older 1.x notes stay in that file. This page is the user-visible 2.x history.

| Version | Date | What a user should notice |
|---|---|---|
| 2.3.0 | 2026-11-16 | E1C SD card-detect moves from P3_3 to P6_5. New bare-metal camera, viewfinder, and ISP video demos. CRC, I2C, I3C, CAN, PDM, UART, camera, ISP, and OSPI driver updates. SVD files updated. Arm Compiler 6 builds add `-funwind-tables` so GDB can unwind frames |
| 2.2.1 | 2026-08-26 | ISP library update. Fixes in SD, I2C, I3C, ISP, the MRAM FreeRTOS application, and PDM board pins |
| 2.2.0 | 2026-06-30 | Timer, UART, I3C, ISP, I2C, GPIO, and RTC driver updates. SE firmware 1.10. More E5, E4, and E3 part numbers. OV5640 and OV5675 cameras. JPEG driver. SD driver refactor. User MRAM base is `APP_MRAM_USER_BASE` |
| 2.1.0 | 2025-12-19 | AppKit-E8 board support. CH201 time-of-flight. MX66 flash and APS512 RAM. SE firmware 1.09. csolution template updates |
| 2.0.4 | 2025-11-04 | Clang support. SD and CAN-FD fixes. AppKit-E8-AIML board entry. SPI abort handling |
| 2.0.3 | 2025-10-10 | ISP driver and library. Standard camera. HM0360 suspend and resume. Blinky for DevKit-E7 |
| 2.0.2 | 2025-08-11 | OSPI1 pinmux fix. Retarget header aligned with 2.0.0. TrustZone configuration build fix |
| 2.0.1 | 2025-08-07 | AppKit-E7-AIML and DevKit-E7 layers and examples |
| 2.0.0 | 2025-08-04 | Pack restructure. E1C and E8 support. CSolution examples. Not compatible with 1.3.4 or older. See @ref migrate_page |
| 1.3.4 | 2025-05-09 | Last 1.x release. Move to 2.x only through AAPN0038 |
