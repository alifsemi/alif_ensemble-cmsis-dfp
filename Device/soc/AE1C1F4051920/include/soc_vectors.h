/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SOC_VECTORS_H
#define SOC_VECTORS_H

#include "vectors.h"

#define IRQ67_Handler  REFCLK_CNTBASE0_IRQHandler
#define IRQ68_Handler  REFCLK_CNTBASE1_IRQHandler
#define IRQ69_Handler  REFCLK_CNTBASE2_IRQHandler
#define IRQ70_Handler  REFCLK_CNTBASE3_IRQHandler
#define IRQ71_Handler  S32K_CNTBASE0_IRQHandler
#define IRQ72_Handler  S32K_CNTBASE1_IRQHandler
#define IRQ73_Handler  SOC_ETR_IRQHandler
#define IRQ74_Handler  SOC_CATU_IRQHandler

#define IRQ96_Handler  OSPI0_IRQHandler

#define IRQ98_Handler  AES0_IRQHandler

#define IRQ100_Handler BOD_IRQHandler
#define IRQ101_Handler USB_IRQHandler
#define IRQ102_Handler SDMMC_IRQHandler
#define IRQ103_Handler SDMMC_WAKEUP_IRQHandler
#define IRQ104_Handler CANFD0_IRQHandler
#define IRQ105_Handler CANFD1_IRQHandler

#define IRQ121_Handler PPU0_IRQHandler
#define IRQ122_Handler PPU1_IRQHandler

#define IRQ124_Handler UART0_IRQHandler
#define IRQ125_Handler UART1_IRQHandler
#define IRQ126_Handler UART2_IRQHandler
#define IRQ127_Handler UART3_IRQHandler
#define IRQ128_Handler UART4_IRQHandler
#define IRQ129_Handler UART5_IRQHandler

#define IRQ132_Handler I2C0_IRQHandler
#define IRQ133_Handler I2C1_IRQHandler

#define IRQ136_Handler I3C_IRQHandler
#define IRQ137_Handler SPI0_IRQHandler
#define IRQ138_Handler SPI1_IRQHandler
#define IRQ139_Handler SPI2_IRQHandler

#define IRQ141_Handler I2S0_IRQHandler
#define IRQ142_Handler I2S1_IRQHandler

#define IRQ151_Handler ADC120_DONE0_IRQHandler
#define IRQ152_Handler ADC121_DONE0_IRQHandler

#define IRQ154_Handler ADC120_DONE1_IRQHandler
#define IRQ155_Handler ADC121_DONE1_IRQHandler

#define IRQ157_Handler ADC120_CMPA_IRQHandler
#define IRQ158_Handler ADC121_CMPA_IRQHandler

#define IRQ160_Handler ADC120_CMPB_IRQHandler
#define IRQ161_Handler ADC121_CMPB_IRQHandler

#define IRQ163_Handler ADC24_DONE0_IRQHandler
#define IRQ164_Handler ADC24_DONE1_IRQHandler
#define IRQ165_Handler ADC24_CMPA_IRQHandler
#define IRQ166_Handler ADC24_CMPB_IRQHandler
#define IRQ167_Handler CMP0_IRQHandler
#define IRQ168_Handler CMP1_IRQHandler

#define IRQ171_Handler LPGPIO_IRQ0Handler
#define IRQ172_Handler LPGPIO_IRQ1Handler

#define IRQ179_Handler GPIO0_IRQ0Handler
#define IRQ180_Handler GPIO0_IRQ1Handler
#define IRQ181_Handler GPIO0_IRQ2Handler
#define IRQ182_Handler GPIO0_IRQ3Handler
#define IRQ183_Handler GPIO0_IRQ4Handler
#define IRQ184_Handler GPIO0_IRQ5Handler
#define IRQ185_Handler GPIO0_IRQ6Handler
#define IRQ186_Handler GPIO0_IRQ7Handler
#define IRQ187_Handler GPIO1_IRQ0Handler
#define IRQ188_Handler GPIO1_IRQ1Handler
#define IRQ189_Handler GPIO1_IRQ2Handler
#define IRQ190_Handler GPIO1_IRQ3Handler
#define IRQ191_Handler GPIO1_IRQ4Handler
#define IRQ192_Handler GPIO1_IRQ5Handler
#define IRQ193_Handler GPIO1_IRQ6Handler
#define IRQ194_Handler GPIO1_IRQ7Handler
#define IRQ195_Handler GPIO2_IRQ0Handler
#define IRQ196_Handler GPIO2_IRQ1Handler
#define IRQ197_Handler GPIO2_IRQ2Handler
#define IRQ198_Handler GPIO2_IRQ3Handler
#define IRQ199_Handler GPIO2_IRQ4Handler
#define IRQ200_Handler GPIO2_IRQ5Handler
#define IRQ201_Handler GPIO2_IRQ6Handler
#define IRQ202_Handler GPIO2_IRQ7Handler
#define IRQ203_Handler GPIO3_IRQ0Handler
#define IRQ204_Handler GPIO3_IRQ1Handler
#define IRQ205_Handler GPIO3_IRQ2Handler
#define IRQ206_Handler GPIO3_IRQ3Handler
#define IRQ207_Handler GPIO3_IRQ4Handler
#define IRQ208_Handler GPIO3_IRQ5Handler
#define IRQ209_Handler GPIO3_IRQ6Handler
#define IRQ210_Handler GPIO3_IRQ7Handler
#define IRQ211_Handler GPIO4_IRQ0Handler
#define IRQ212_Handler GPIO4_IRQ1Handler
#define IRQ213_Handler GPIO4_IRQ2Handler
#define IRQ214_Handler GPIO4_IRQ3Handler
#define IRQ215_Handler GPIO4_IRQ4Handler
#define IRQ216_Handler GPIO4_IRQ5Handler
#define IRQ217_Handler GPIO4_IRQ6Handler
#define IRQ218_Handler GPIO4_IRQ7Handler
#define IRQ219_Handler GPIO5_IRQ0Handler
#define IRQ220_Handler GPIO5_IRQ1Handler
#define IRQ221_Handler GPIO5_IRQ2Handler
#define IRQ222_Handler GPIO5_IRQ3Handler
#define IRQ223_Handler GPIO5_IRQ4Handler
#define IRQ224_Handler GPIO5_IRQ5Handler
#define IRQ225_Handler GPIO5_IRQ6Handler
#define IRQ226_Handler GPIO5_IRQ7Handler
#define IRQ227_Handler GPIO6_IRQ0Handler
#define IRQ228_Handler GPIO6_IRQ1Handler
#define IRQ229_Handler GPIO6_IRQ2Handler
#define IRQ230_Handler GPIO6_IRQ3Handler
#define IRQ231_Handler GPIO6_IRQ4Handler
#define IRQ232_Handler GPIO6_IRQ5Handler
#define IRQ233_Handler GPIO6_IRQ6Handler
#define IRQ234_Handler GPIO6_IRQ7Handler
#define IRQ235_Handler GPIO7_IRQ0Handler
#define IRQ236_Handler GPIO7_IRQ1Handler
#define IRQ237_Handler GPIO7_IRQ2Handler
#define IRQ238_Handler GPIO7_IRQ3Handler
#define IRQ239_Handler GPIO7_IRQ4Handler
#define IRQ240_Handler GPIO7_IRQ5Handler
#define IRQ241_Handler GPIO7_IRQ6Handler
#define IRQ242_Handler GPIO7_IRQ7Handler
#define IRQ243_Handler GPIO8_IRQ0Handler
#define IRQ244_Handler GPIO8_IRQ1Handler
#define IRQ245_Handler GPIO8_IRQ2Handler
#define IRQ246_Handler GPIO8_IRQ3Handler
#define IRQ247_Handler GPIO8_IRQ4Handler
#define IRQ248_Handler GPIO8_IRQ5Handler
#define IRQ249_Handler GPIO8_IRQ6Handler
#define IRQ250_Handler GPIO8_IRQ7Handler
#define IRQ251_Handler GPIO9_IRQ0Handler
#define IRQ252_Handler GPIO9_IRQ1Handler
#define IRQ253_Handler GPIO9_IRQ2Handler

#define IRQ332_Handler GPU2D_IRQHandler
#define IRQ333_Handler CDC_SCANLINE0_IRQHandler

#define IRQ335_Handler CDC_FIFO_WARNING0_IRQHandler

#define IRQ337_Handler CDC_FIFO_UNDERRUN0_IRQHandler

#define IRQ339_Handler CDC_BUS_ERROR0_IRQHandler

#define IRQ341_Handler CDC_REG_RELOAD0_IRQHandler

#define IRQ343_Handler DSI_IRQHandler

#define IRQ377_Handler UTIMER_IRQ0Handler
#define IRQ378_Handler UTIMER_IRQ1Handler
#define IRQ379_Handler UTIMER_IRQ2Handler
#define IRQ380_Handler UTIMER_IRQ3Handler
#define IRQ381_Handler UTIMER_IRQ4Handler
#define IRQ382_Handler UTIMER_IRQ5Handler
#define IRQ383_Handler UTIMER_IRQ6Handler
#define IRQ384_Handler UTIMER_IRQ7Handler
#define IRQ385_Handler UTIMER_IRQ8Handler
#define IRQ386_Handler UTIMER_IRQ9Handler
#define IRQ387_Handler UTIMER_IRQ10Handler
#define IRQ388_Handler UTIMER_IRQ11Handler
#define IRQ389_Handler UTIMER_IRQ12Handler
#define IRQ390_Handler UTIMER_IRQ13Handler
#define IRQ391_Handler UTIMER_IRQ14Handler
#define IRQ392_Handler UTIMER_IRQ15Handler
#define IRQ393_Handler UTIMER_IRQ16Handler
#define IRQ394_Handler UTIMER_IRQ17Handler
#define IRQ395_Handler UTIMER_IRQ18Handler
#define IRQ396_Handler UTIMER_IRQ19Handler
#define IRQ397_Handler UTIMER_IRQ20Handler
#define IRQ398_Handler UTIMER_IRQ21Handler
#define IRQ399_Handler UTIMER_IRQ22Handler
#define IRQ400_Handler UTIMER_IRQ23Handler
#define IRQ401_Handler UTIMER_IRQ24Handler
#define IRQ402_Handler UTIMER_IRQ25Handler
#define IRQ403_Handler UTIMER_IRQ26Handler
#define IRQ404_Handler UTIMER_IRQ27Handler
#define IRQ405_Handler UTIMER_IRQ28Handler
#define IRQ406_Handler UTIMER_IRQ29Handler
#define IRQ407_Handler UTIMER_IRQ30Handler
#define IRQ408_Handler UTIMER_IRQ31Handler

#endif /* SOC_VECTORS_H */
