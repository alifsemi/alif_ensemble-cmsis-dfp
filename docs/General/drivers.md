# Drivers {#drivers_page}

A driver is compiled only when the application selects it. There is no separate makefile. Add one line under `components:` in the `cproject.yml`, or use a board layer that already lists it, then run the same `cbuild` command as on @ref setup_page.

    components:
      - component: CMSIS Driver:I2C

The **Files** tab of this HTML book lists the functions that have a Doxygen comment. I2C, UART, CRC, CDC, PDM, and ADC have function comments. CAN, GPIO, SPI, and I2S still have a file banner only, so their function list is short. The comment form is on @ref rules_page.

## CMSIS-Driver components

The component name is the string `cbuild` expects. "Selected by" is a file in this tree that already contains that line. Where the cell says none, the pack ships the driver and no example or board layer in this tree selects it. Do not describe a demo that is not in the tree.

| Driver | Component line | Selected by |
|---|---|---|
| USART | `CMSIS Driver:USART` | `Boards/DevKit-e8/Examples/Blinky_HP/M55_HP/M55_HP.cproject.yml` |
| GPIO | `CMSIS Driver:GPIO` | E8 `Board_HP.clayer.yml`, and the E7 AppKit Blinky project |
| I2C | `CMSIS Driver:I2C` | E7 and E8 board layers, both cores |
| Low-power I2C | `CMSIS Driver:I2C:LPI2C` | None in this tree |
| SPI | `CMSIS Driver:SPI` | E7 DevKit layers and E8 layers |
| SAI / I2S | `CMSIS Driver:SAI` | E8 `Board_HE.clayer.yml`, E7 DevKit layers, E7 AppKit layers |
| CAN | `CMSIS Driver:CAN` | None in this tree |
| SD card | `CMSIS Driver:MCI` | None in this tree |
| Ethernet | `CMSIS Driver:Ethernet MAC` and `CMSIS Driver:Ethernet PHY:Custom` | E8 board layers |
| USB device | `CMSIS Driver:USB Device` | E8 board layers |
| Board LED and button | `CMSIS Driver:VIO:Board&DevKit-E8` (variant matches the kit) | Blinky and the board layers |

Application code includes the CMSIS-Driver headers. Those headers declare `ARM_DRIVER_I2C`, `ARM_DRIVER_USART`, and the other driver types. They come from the `ARM::CMSIS` pack, under `CMSIS/Driver/Include/`. They are not in this repository. The bare-metal demos follow that include: `Boards/Templates/Baremetal/demo_i2c.c` includes `Driver_I2C.h`, and `demo_uart4.c` includes `Driver_USART.h`.

| Interface | Application header |
|---|---|
| CAN | `Driver_CAN.h` |
| GPIO | `Driver_GPIO.h` |
| I2C | `Driver_I2C.h` |
| SAI / I2S | `Driver_SAI.h` |
| SPI | `Driver_SPI.h` |
| USART | `Driver_USART.h` |
| SD card | `Driver_MCI.h` |
| Ethernet MAC | `Driver_ETH_MAC.h` |
| Ethernet PHY | `Driver_ETH_PHY.h` |
| USB device | `Driver_USBD.h` |

The files under `drivers/include/` are the implementation headers for this pack. They do not declare the CMSIS driver types. Read them when you are changing the driver. Do not include them in place of the application header.

| Interface | Implementation header |
|---|---|
| CAN | [canfd.h](https://github.com/alifsemi/alif_ensemble-cmsis-dfp/blob/main/drivers/include/canfd.h) |
| GPIO | [gpio.h](https://github.com/alifsemi/alif_ensemble-cmsis-dfp/blob/main/drivers/include/gpio.h) |
| I2C | [i2c.h](https://github.com/alifsemi/alif_ensemble-cmsis-dfp/blob/main/drivers/include/i2c.h), low-power in `lpi2c.h` |
| SAI / I2S | [i2s.h](https://github.com/alifsemi/alif_ensemble-cmsis-dfp/blob/main/drivers/include/i2s.h), microphones in `pdm.h` |
| SPI | [spi.h](https://github.com/alifsemi/alif_ensemble-cmsis-dfp/blob/main/drivers/include/spi.h) |
| USART | [uart.h](https://github.com/alifsemi/alif_ensemble-cmsis-dfp/blob/main/drivers/include/uart.h) |

## Device peripheral components

Camera, OSPI, JPEG, CRC, I3C, and PDM are `Device:SOC Peripherals` components. They follow the same rule: add the line, then run `cbuild`.

| Block | Component line | Selected by |
|---|---|---|
| Camera parallel interface | `Device:SOC Peripherals:CPI` | E7 and E8 board layers |
| OSPI | `Device:SOC Peripherals:OSPI` | None in this tree |
| JPEG | `Device:SOC Peripherals:JPEG` | None in this tree |
| CRC | `Device:SOC Peripherals:CRC` | None in this tree |
| I3C | `Device:SOC Peripherals:I3C` | None in this tree |
| PDM microphones | `Device:SOC Peripherals:PDM` | None in this tree |

Release 2.3.0 updates CRC, I2C, I3C, CAN, PDM, UART, camera, ISP, and OSPI. Release 2.2.0 added the JPEG driver and OV5640 / OV5675 camera support. The E1C SD card-detect pin in that release is P6_5. Those notes describe the source change. They are not a build command.

`sys_ctrl_*.h` headers program clocks and resets. They are not the application component you add to a `cproject.yml`.
