/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**
 * RTSS HE core has both private and shared irqs (connected via irqrtr).
 * The first 64 irqs are private.
 *
 * This file only has the definitions of default Exception handlers
 * and the privates ones. The shared irq vectors are defined in soc_vector.c
 * and is attached to this vector table in the linker script such that it is
 * continous.
 */

#include "soc.h"
#include "system.h"
#include "soc_vectors.h"
#include "sys_utils.h"

#define __WEAK_FAULT __attribute__((weak, alias("Fault_Handler")))
#define __WEAK_ISR   __attribute__((weak, alias("Default_Handler")))

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

static void Fault_Handler(void)
{
    for (;;) {
    }
}

static void Default_Handler(void)
{
    for (;;) {
    }
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic pop
#endif

__NO_RETURN void  Reset_Handler(void);
__WEAK_ISR void   NMI_Handler(void);
__WEAK_FAULT void HardFault_Handler(void);
__WEAK_FAULT void MemManage_Handler(void);
__WEAK_FAULT void BusFault_Handler(void);
__WEAK_FAULT void UsageFault_Handler(void);
__WEAK_FAULT void SecureFault_Handler(void);
__WEAK_ISR void   SVC_Handler(void);
__WEAK_ISR void   DebugMon_Handler(void);
__WEAK_ISR void   PendSV_Handler(void);
__WEAK_ISR void   SysTick_Handler(void);

__WEAK_ISR void IRQ0_Handler(void);
__WEAK_ISR void IRQ1_Handler(void);
__WEAK_ISR void IRQ2_Handler(void);
__WEAK_ISR void IRQ3_Handler(void);
__WEAK_ISR void IRQ4_Handler(void);
__WEAK_ISR void IRQ5_Handler(void);
__WEAK_ISR void IRQ6_Handler(void);
__WEAK_ISR void IRQ7_Handler(void);
__WEAK_ISR void IRQ8_Handler(void);
__WEAK_ISR void IRQ9_Handler(void);
__WEAK_ISR void IRQ10_Handler(void);
__WEAK_ISR void IRQ11_Handler(void);
__WEAK_ISR void IRQ12_Handler(void);
__WEAK_ISR void IRQ13_Handler(void);
__WEAK_ISR void IRQ14_Handler(void);
__WEAK_ISR void IRQ15_Handler(void);
__WEAK_ISR void IRQ16_Handler(void);
__WEAK_ISR void IRQ17_Handler(void);
__WEAK_ISR void IRQ18_Handler(void);
__WEAK_ISR void IRQ19_Handler(void);
__WEAK_ISR void IRQ20_Handler(void);
__WEAK_ISR void IRQ21_Handler(void);
__WEAK_ISR void IRQ22_Handler(void);
__WEAK_ISR void IRQ23_Handler(void);
__WEAK_ISR void IRQ24_Handler(void);
__WEAK_ISR void IRQ25_Handler(void);
__WEAK_ISR void IRQ26_Handler(void);
__WEAK_ISR void IRQ27_Handler(void);
__WEAK_ISR void IRQ28_Handler(void);
__WEAK_ISR void IRQ29_Handler(void);
__WEAK_ISR void IRQ30_Handler(void);
__WEAK_ISR void IRQ31_Handler(void);
__WEAK_ISR void IRQ32_Handler(void);
__WEAK_ISR void IRQ33_Handler(void);
__WEAK_ISR void IRQ34_Handler(void);
__WEAK_ISR void IRQ35_Handler(void);
__WEAK_ISR void IRQ36_Handler(void);
__WEAK_ISR void IRQ37_Handler(void);
__WEAK_ISR void IRQ38_Handler(void);
__WEAK_ISR void IRQ39_Handler(void);
__WEAK_ISR void IRQ40_Handler(void);
__WEAK_ISR void IRQ41_Handler(void);
__WEAK_ISR void IRQ42_Handler(void);
__WEAK_ISR void IRQ43_Handler(void);
__WEAK_ISR void IRQ44_Handler(void);
__WEAK_ISR void IRQ45_Handler(void);
__WEAK_ISR void IRQ46_Handler(void);
__WEAK_ISR void IRQ47_Handler(void);
__WEAK_ISR void IRQ48_Handler(void);
__WEAK_ISR void IRQ49_Handler(void);
__WEAK_ISR void IRQ50_Handler(void);
__WEAK_ISR void IRQ51_Handler(void);
__WEAK_ISR void IRQ52_Handler(void);
__WEAK_ISR void IRQ53_Handler(void);
__WEAK_ISR void IRQ54_Handler(void);
__WEAK_ISR void IRQ55_Handler(void);
__WEAK_ISR void IRQ56_Handler(void);
__WEAK_ISR void IRQ57_Handler(void);
__WEAK_ISR void IRQ58_Handler(void);
__WEAK_ISR void IRQ59_Handler(void);
__WEAK_ISR void IRQ60_Handler(void);
__WEAK_ISR void IRQ61_Handler(void);
__WEAK_ISR void IRQ62_Handler(void);
__WEAK_ISR void IRQ63_Handler(void);
__WEAK_ISR void IRQ64_Handler(void);
__WEAK_ISR void IRQ65_Handler(void);
__WEAK_ISR void IRQ66_Handler(void);
__WEAK_ISR void IRQ67_Handler(void);
__WEAK_ISR void IRQ68_Handler(void);
__WEAK_ISR void IRQ69_Handler(void);
__WEAK_ISR void IRQ70_Handler(void);
__WEAK_ISR void IRQ71_Handler(void);
__WEAK_ISR void IRQ72_Handler(void);
__WEAK_ISR void IRQ73_Handler(void);
__WEAK_ISR void IRQ74_Handler(void);
__WEAK_ISR void IRQ75_Handler(void);
__WEAK_ISR void IRQ76_Handler(void);
__WEAK_ISR void IRQ77_Handler(void);
__WEAK_ISR void IRQ78_Handler(void);
__WEAK_ISR void IRQ79_Handler(void);
__WEAK_ISR void IRQ80_Handler(void);
__WEAK_ISR void IRQ81_Handler(void);
__WEAK_ISR void IRQ82_Handler(void);
__WEAK_ISR void IRQ83_Handler(void);
__WEAK_ISR void IRQ84_Handler(void);
__WEAK_ISR void IRQ85_Handler(void);
__WEAK_ISR void IRQ86_Handler(void);
__WEAK_ISR void IRQ87_Handler(void);
__WEAK_ISR void IRQ88_Handler(void);
__WEAK_ISR void IRQ89_Handler(void);
__WEAK_ISR void IRQ90_Handler(void);
__WEAK_ISR void IRQ91_Handler(void);
__WEAK_ISR void IRQ92_Handler(void);
__WEAK_ISR void IRQ93_Handler(void);
__WEAK_ISR void IRQ94_Handler(void);
__WEAK_ISR void IRQ95_Handler(void);
__WEAK_ISR void IRQ96_Handler(void);
__WEAK_ISR void IRQ97_Handler(void);
__WEAK_ISR void IRQ98_Handler(void);
__WEAK_ISR void IRQ99_Handler(void);
__WEAK_ISR void IRQ100_Handler(void);
__WEAK_ISR void IRQ101_Handler(void);
__WEAK_ISR void IRQ102_Handler(void);
__WEAK_ISR void IRQ103_Handler(void);
__WEAK_ISR void IRQ104_Handler(void);
__WEAK_ISR void IRQ105_Handler(void);
__WEAK_ISR void IRQ106_Handler(void);
__WEAK_ISR void IRQ107_Handler(void);
__WEAK_ISR void IRQ108_Handler(void);
__WEAK_ISR void IRQ109_Handler(void);
__WEAK_ISR void IRQ110_Handler(void);
__WEAK_ISR void IRQ111_Handler(void);
__WEAK_ISR void IRQ112_Handler(void);
__WEAK_ISR void IRQ113_Handler(void);
__WEAK_ISR void IRQ114_Handler(void);
__WEAK_ISR void IRQ115_Handler(void);
__WEAK_ISR void IRQ116_Handler(void);
__WEAK_ISR void IRQ117_Handler(void);
__WEAK_ISR void IRQ118_Handler(void);
__WEAK_ISR void IRQ119_Handler(void);
__WEAK_ISR void IRQ120_Handler(void);
__WEAK_ISR void IRQ121_Handler(void);
__WEAK_ISR void IRQ122_Handler(void);
__WEAK_ISR void IRQ123_Handler(void);
__WEAK_ISR void IRQ124_Handler(void);
__WEAK_ISR void IRQ125_Handler(void);
__WEAK_ISR void IRQ126_Handler(void);
__WEAK_ISR void IRQ127_Handler(void);
__WEAK_ISR void IRQ128_Handler(void);
__WEAK_ISR void IRQ129_Handler(void);
__WEAK_ISR void IRQ130_Handler(void);
__WEAK_ISR void IRQ131_Handler(void);
__WEAK_ISR void IRQ132_Handler(void);
__WEAK_ISR void IRQ133_Handler(void);
__WEAK_ISR void IRQ134_Handler(void);
__WEAK_ISR void IRQ135_Handler(void);
__WEAK_ISR void IRQ136_Handler(void);
__WEAK_ISR void IRQ137_Handler(void);
__WEAK_ISR void IRQ138_Handler(void);
__WEAK_ISR void IRQ139_Handler(void);
__WEAK_ISR void IRQ140_Handler(void);
__WEAK_ISR void IRQ141_Handler(void);
__WEAK_ISR void IRQ142_Handler(void);
__WEAK_ISR void IRQ143_Handler(void);
__WEAK_ISR void IRQ144_Handler(void);
__WEAK_ISR void IRQ145_Handler(void);
__WEAK_ISR void IRQ146_Handler(void);
__WEAK_ISR void IRQ147_Handler(void);
__WEAK_ISR void IRQ148_Handler(void);
__WEAK_ISR void IRQ149_Handler(void);
__WEAK_ISR void IRQ150_Handler(void);
__WEAK_ISR void IRQ151_Handler(void);
__WEAK_ISR void IRQ152_Handler(void);
__WEAK_ISR void IRQ153_Handler(void);
__WEAK_ISR void IRQ154_Handler(void);
__WEAK_ISR void IRQ155_Handler(void);
__WEAK_ISR void IRQ156_Handler(void);
__WEAK_ISR void IRQ157_Handler(void);
__WEAK_ISR void IRQ158_Handler(void);
__WEAK_ISR void IRQ159_Handler(void);
__WEAK_ISR void IRQ160_Handler(void);
__WEAK_ISR void IRQ161_Handler(void);
__WEAK_ISR void IRQ162_Handler(void);
__WEAK_ISR void IRQ163_Handler(void);
__WEAK_ISR void IRQ164_Handler(void);
__WEAK_ISR void IRQ165_Handler(void);
__WEAK_ISR void IRQ166_Handler(void);
__WEAK_ISR void IRQ167_Handler(void);
__WEAK_ISR void IRQ168_Handler(void);
__WEAK_ISR void IRQ169_Handler(void);
__WEAK_ISR void IRQ170_Handler(void);
__WEAK_ISR void IRQ171_Handler(void);
__WEAK_ISR void IRQ172_Handler(void);
__WEAK_ISR void IRQ173_Handler(void);
__WEAK_ISR void IRQ174_Handler(void);
__WEAK_ISR void IRQ175_Handler(void);
__WEAK_ISR void IRQ176_Handler(void);
__WEAK_ISR void IRQ177_Handler(void);
__WEAK_ISR void IRQ178_Handler(void);
__WEAK_ISR void IRQ179_Handler(void);
__WEAK_ISR void IRQ180_Handler(void);
__WEAK_ISR void IRQ181_Handler(void);
__WEAK_ISR void IRQ182_Handler(void);
__WEAK_ISR void IRQ183_Handler(void);
__WEAK_ISR void IRQ184_Handler(void);
__WEAK_ISR void IRQ185_Handler(void);
__WEAK_ISR void IRQ186_Handler(void);
__WEAK_ISR void IRQ187_Handler(void);
__WEAK_ISR void IRQ188_Handler(void);
__WEAK_ISR void IRQ189_Handler(void);
__WEAK_ISR void IRQ190_Handler(void);
__WEAK_ISR void IRQ191_Handler(void);
__WEAK_ISR void IRQ192_Handler(void);
__WEAK_ISR void IRQ193_Handler(void);
__WEAK_ISR void IRQ194_Handler(void);
__WEAK_ISR void IRQ195_Handler(void);
__WEAK_ISR void IRQ196_Handler(void);
__WEAK_ISR void IRQ197_Handler(void);
__WEAK_ISR void IRQ198_Handler(void);
__WEAK_ISR void IRQ199_Handler(void);
__WEAK_ISR void IRQ200_Handler(void);
__WEAK_ISR void IRQ201_Handler(void);
__WEAK_ISR void IRQ202_Handler(void);
__WEAK_ISR void IRQ203_Handler(void);
__WEAK_ISR void IRQ204_Handler(void);
__WEAK_ISR void IRQ205_Handler(void);
__WEAK_ISR void IRQ206_Handler(void);
__WEAK_ISR void IRQ207_Handler(void);
__WEAK_ISR void IRQ208_Handler(void);
__WEAK_ISR void IRQ209_Handler(void);
__WEAK_ISR void IRQ210_Handler(void);
__WEAK_ISR void IRQ211_Handler(void);
__WEAK_ISR void IRQ212_Handler(void);
__WEAK_ISR void IRQ213_Handler(void);
__WEAK_ISR void IRQ214_Handler(void);
__WEAK_ISR void IRQ215_Handler(void);
__WEAK_ISR void IRQ216_Handler(void);
__WEAK_ISR void IRQ217_Handler(void);
__WEAK_ISR void IRQ218_Handler(void);
__WEAK_ISR void IRQ219_Handler(void);
__WEAK_ISR void IRQ220_Handler(void);
__WEAK_ISR void IRQ221_Handler(void);
__WEAK_ISR void IRQ222_Handler(void);
__WEAK_ISR void IRQ223_Handler(void);
__WEAK_ISR void IRQ224_Handler(void);
__WEAK_ISR void IRQ225_Handler(void);
__WEAK_ISR void IRQ226_Handler(void);
__WEAK_ISR void IRQ227_Handler(void);
__WEAK_ISR void IRQ228_Handler(void);
__WEAK_ISR void IRQ229_Handler(void);
__WEAK_ISR void IRQ230_Handler(void);
__WEAK_ISR void IRQ231_Handler(void);
__WEAK_ISR void IRQ232_Handler(void);
__WEAK_ISR void IRQ233_Handler(void);
__WEAK_ISR void IRQ234_Handler(void);
__WEAK_ISR void IRQ235_Handler(void);
__WEAK_ISR void IRQ236_Handler(void);
__WEAK_ISR void IRQ237_Handler(void);
__WEAK_ISR void IRQ238_Handler(void);
__WEAK_ISR void IRQ239_Handler(void);
__WEAK_ISR void IRQ240_Handler(void);
__WEAK_ISR void IRQ241_Handler(void);
__WEAK_ISR void IRQ242_Handler(void);
__WEAK_ISR void IRQ243_Handler(void);
__WEAK_ISR void IRQ244_Handler(void);
__WEAK_ISR void IRQ245_Handler(void);
__WEAK_ISR void IRQ246_Handler(void);
__WEAK_ISR void IRQ247_Handler(void);
__WEAK_ISR void IRQ248_Handler(void);
__WEAK_ISR void IRQ249_Handler(void);
__WEAK_ISR void IRQ250_Handler(void);
__WEAK_ISR void IRQ251_Handler(void);
__WEAK_ISR void IRQ252_Handler(void);
__WEAK_ISR void IRQ253_Handler(void);
__WEAK_ISR void IRQ254_Handler(void);
__WEAK_ISR void IRQ255_Handler(void);
__WEAK_ISR void IRQ256_Handler(void);
__WEAK_ISR void IRQ257_Handler(void);
__WEAK_ISR void IRQ258_Handler(void);
__WEAK_ISR void IRQ259_Handler(void);
__WEAK_ISR void IRQ260_Handler(void);
__WEAK_ISR void IRQ261_Handler(void);
__WEAK_ISR void IRQ262_Handler(void);
__WEAK_ISR void IRQ263_Handler(void);
__WEAK_ISR void IRQ264_Handler(void);
__WEAK_ISR void IRQ265_Handler(void);
__WEAK_ISR void IRQ266_Handler(void);
__WEAK_ISR void IRQ267_Handler(void);
__WEAK_ISR void IRQ268_Handler(void);
__WEAK_ISR void IRQ269_Handler(void);
__WEAK_ISR void IRQ270_Handler(void);
__WEAK_ISR void IRQ271_Handler(void);
__WEAK_ISR void IRQ272_Handler(void);
__WEAK_ISR void IRQ273_Handler(void);
__WEAK_ISR void IRQ274_Handler(void);
__WEAK_ISR void IRQ275_Handler(void);
__WEAK_ISR void IRQ276_Handler(void);
__WEAK_ISR void IRQ277_Handler(void);
__WEAK_ISR void IRQ278_Handler(void);
__WEAK_ISR void IRQ279_Handler(void);
__WEAK_ISR void IRQ280_Handler(void);
__WEAK_ISR void IRQ281_Handler(void);
__WEAK_ISR void IRQ282_Handler(void);
__WEAK_ISR void IRQ283_Handler(void);
__WEAK_ISR void IRQ284_Handler(void);
__WEAK_ISR void IRQ285_Handler(void);
__WEAK_ISR void IRQ286_Handler(void);
__WEAK_ISR void IRQ287_Handler(void);
__WEAK_ISR void IRQ288_Handler(void);
__WEAK_ISR void IRQ289_Handler(void);
__WEAK_ISR void IRQ290_Handler(void);
__WEAK_ISR void IRQ291_Handler(void);
__WEAK_ISR void IRQ292_Handler(void);
__WEAK_ISR void IRQ293_Handler(void);
__WEAK_ISR void IRQ294_Handler(void);
__WEAK_ISR void IRQ295_Handler(void);
__WEAK_ISR void IRQ296_Handler(void);
__WEAK_ISR void IRQ297_Handler(void);
__WEAK_ISR void IRQ298_Handler(void);
__WEAK_ISR void IRQ299_Handler(void);
__WEAK_ISR void IRQ300_Handler(void);
__WEAK_ISR void IRQ301_Handler(void);
__WEAK_ISR void IRQ302_Handler(void);
__WEAK_ISR void IRQ303_Handler(void);
__WEAK_ISR void IRQ304_Handler(void);
__WEAK_ISR void IRQ305_Handler(void);
__WEAK_ISR void IRQ306_Handler(void);
__WEAK_ISR void IRQ307_Handler(void);
__WEAK_ISR void IRQ308_Handler(void);
__WEAK_ISR void IRQ309_Handler(void);
__WEAK_ISR void IRQ310_Handler(void);
__WEAK_ISR void IRQ311_Handler(void);
__WEAK_ISR void IRQ312_Handler(void);
__WEAK_ISR void IRQ313_Handler(void);
__WEAK_ISR void IRQ314_Handler(void);
__WEAK_ISR void IRQ315_Handler(void);
__WEAK_ISR void IRQ316_Handler(void);
__WEAK_ISR void IRQ317_Handler(void);
__WEAK_ISR void IRQ318_Handler(void);
__WEAK_ISR void IRQ319_Handler(void);
__WEAK_ISR void IRQ320_Handler(void);
__WEAK_ISR void IRQ321_Handler(void);
__WEAK_ISR void IRQ322_Handler(void);
__WEAK_ISR void IRQ323_Handler(void);
__WEAK_ISR void IRQ324_Handler(void);
__WEAK_ISR void IRQ325_Handler(void);
__WEAK_ISR void IRQ326_Handler(void);
__WEAK_ISR void IRQ327_Handler(void);
__WEAK_ISR void IRQ328_Handler(void);
__WEAK_ISR void IRQ329_Handler(void);
__WEAK_ISR void IRQ330_Handler(void);
__WEAK_ISR void IRQ331_Handler(void);
__WEAK_ISR void IRQ332_Handler(void);
__WEAK_ISR void IRQ333_Handler(void);
__WEAK_ISR void IRQ334_Handler(void);
__WEAK_ISR void IRQ335_Handler(void);
__WEAK_ISR void IRQ336_Handler(void);
__WEAK_ISR void IRQ337_Handler(void);
__WEAK_ISR void IRQ338_Handler(void);
__WEAK_ISR void IRQ339_Handler(void);
__WEAK_ISR void IRQ340_Handler(void);
__WEAK_ISR void IRQ341_Handler(void);
__WEAK_ISR void IRQ342_Handler(void);
__WEAK_ISR void IRQ343_Handler(void);
__WEAK_ISR void IRQ344_Handler(void);
__WEAK_ISR void IRQ345_Handler(void);
__WEAK_ISR void IRQ346_Handler(void);
__WEAK_ISR void IRQ347_Handler(void);
__WEAK_ISR void IRQ348_Handler(void);
__WEAK_ISR void IRQ349_Handler(void);
__WEAK_ISR void IRQ350_Handler(void);
__WEAK_ISR void IRQ351_Handler(void);
__WEAK_ISR void IRQ352_Handler(void);
__WEAK_ISR void IRQ353_Handler(void);
__WEAK_ISR void IRQ354_Handler(void);
__WEAK_ISR void IRQ355_Handler(void);
__WEAK_ISR void IRQ356_Handler(void);
__WEAK_ISR void IRQ357_Handler(void);
__WEAK_ISR void IRQ358_Handler(void);
__WEAK_ISR void IRQ359_Handler(void);
__WEAK_ISR void IRQ360_Handler(void);
__WEAK_ISR void IRQ361_Handler(void);
__WEAK_ISR void IRQ362_Handler(void);
__WEAK_ISR void IRQ363_Handler(void);
__WEAK_ISR void IRQ364_Handler(void);
__WEAK_ISR void IRQ365_Handler(void);
__WEAK_ISR void IRQ366_Handler(void);
__WEAK_ISR void IRQ367_Handler(void);
__WEAK_ISR void IRQ368_Handler(void);
__WEAK_ISR void IRQ369_Handler(void);
__WEAK_ISR void IRQ370_Handler(void);
__WEAK_ISR void IRQ371_Handler(void);
__WEAK_ISR void IRQ372_Handler(void);
__WEAK_ISR void IRQ373_Handler(void);
__WEAK_ISR void IRQ374_Handler(void);
__WEAK_ISR void IRQ375_Handler(void);
__WEAK_ISR void IRQ376_Handler(void);
__WEAK_ISR void IRQ377_Handler(void);
__WEAK_ISR void IRQ378_Handler(void);
__WEAK_ISR void IRQ379_Handler(void);
__WEAK_ISR void IRQ380_Handler(void);
__WEAK_ISR void IRQ381_Handler(void);
__WEAK_ISR void IRQ382_Handler(void);
__WEAK_ISR void IRQ383_Handler(void);
__WEAK_ISR void IRQ384_Handler(void);
__WEAK_ISR void IRQ385_Handler(void);
__WEAK_ISR void IRQ386_Handler(void);
__WEAK_ISR void IRQ387_Handler(void);
__WEAK_ISR void IRQ388_Handler(void);
__WEAK_ISR void IRQ389_Handler(void);
__WEAK_ISR void IRQ390_Handler(void);
__WEAK_ISR void IRQ391_Handler(void);
__WEAK_ISR void IRQ392_Handler(void);
__WEAK_ISR void IRQ393_Handler(void);
__WEAK_ISR void IRQ394_Handler(void);
__WEAK_ISR void IRQ395_Handler(void);
__WEAK_ISR void IRQ396_Handler(void);
__WEAK_ISR void IRQ397_Handler(void);
__WEAK_ISR void IRQ398_Handler(void);
__WEAK_ISR void IRQ399_Handler(void);
__WEAK_ISR void IRQ400_Handler(void);
__WEAK_ISR void IRQ401_Handler(void);
__WEAK_ISR void IRQ402_Handler(void);
__WEAK_ISR void IRQ403_Handler(void);
__WEAK_ISR void IRQ404_Handler(void);
__WEAK_ISR void IRQ405_Handler(void);
__WEAK_ISR void IRQ406_Handler(void);
__WEAK_ISR void IRQ407_Handler(void);
__WEAK_ISR void IRQ408_Handler(void);
__WEAK_ISR void IRQ409_Handler(void);
__WEAK_ISR void IRQ410_Handler(void);
__WEAK_ISR void IRQ411_Handler(void);
__WEAK_ISR void IRQ412_Handler(void);
__WEAK_ISR void IRQ413_Handler(void);
__WEAK_ISR void IRQ414_Handler(void);
__WEAK_ISR void IRQ415_Handler(void);
__WEAK_ISR void IRQ416_Handler(void);
__WEAK_ISR void IRQ417_Handler(void);
__WEAK_ISR void IRQ418_Handler(void);
__WEAK_ISR void IRQ419_Handler(void);
__WEAK_ISR void IRQ420_Handler(void);
__WEAK_ISR void IRQ421_Handler(void);
__WEAK_ISR void IRQ422_Handler(void);
__WEAK_ISR void IRQ423_Handler(void);
__WEAK_ISR void IRQ424_Handler(void);
__WEAK_ISR void IRQ425_Handler(void);
__WEAK_ISR void IRQ426_Handler(void);
__WEAK_ISR void IRQ427_Handler(void);
__WEAK_ISR void IRQ428_Handler(void);
__WEAK_ISR void IRQ429_Handler(void);
__WEAK_ISR void IRQ430_Handler(void);
__WEAK_ISR void IRQ431_Handler(void);
__WEAK_ISR void IRQ432_Handler(void);
__WEAK_ISR void IRQ433_Handler(void);
__WEAK_ISR void IRQ434_Handler(void);
__WEAK_ISR void IRQ435_Handler(void);
__WEAK_ISR void IRQ436_Handler(void);
__WEAK_ISR void IRQ437_Handler(void);
__WEAK_ISR void IRQ438_Handler(void);
__WEAK_ISR void IRQ439_Handler(void);
__WEAK_ISR void IRQ440_Handler(void);
__WEAK_ISR void IRQ441_Handler(void);
__WEAK_ISR void IRQ442_Handler(void);
__WEAK_ISR void IRQ443_Handler(void);
__WEAK_ISR void IRQ444_Handler(void);
__WEAK_ISR void IRQ445_Handler(void);
__WEAK_ISR void IRQ446_Handler(void);
__WEAK_ISR void IRQ447_Handler(void);
__WEAK_ISR void IRQ448_Handler(void);
__WEAK_ISR void IRQ449_Handler(void);
__WEAK_ISR void IRQ450_Handler(void);
__WEAK_ISR void IRQ451_Handler(void);
__WEAK_ISR void IRQ452_Handler(void);
__WEAK_ISR void IRQ453_Handler(void);
__WEAK_ISR void IRQ454_Handler(void);
__WEAK_ISR void IRQ455_Handler(void);
__WEAK_ISR void IRQ456_Handler(void);
__WEAK_ISR void IRQ457_Handler(void);
__WEAK_ISR void IRQ458_Handler(void);
__WEAK_ISR void IRQ459_Handler(void);
__WEAK_ISR void IRQ460_Handler(void);
__WEAK_ISR void IRQ461_Handler(void);
__WEAK_ISR void IRQ462_Handler(void);
__WEAK_ISR void IRQ463_Handler(void);
__WEAK_ISR void IRQ464_Handler(void);
__WEAK_ISR void IRQ465_Handler(void);
__WEAK_ISR void IRQ466_Handler(void);
__WEAK_ISR void IRQ467_Handler(void);
__WEAK_ISR void IRQ468_Handler(void);
__WEAK_ISR void IRQ469_Handler(void);
__WEAK_ISR void IRQ470_Handler(void);
__WEAK_ISR void IRQ471_Handler(void);
__WEAK_ISR void IRQ472_Handler(void);
__WEAK_ISR void IRQ473_Handler(void);
__WEAK_ISR void IRQ474_Handler(void);
__WEAK_ISR void IRQ475_Handler(void);
__WEAK_ISR void IRQ476_Handler(void);
__WEAK_ISR void IRQ477_Handler(void);
__WEAK_ISR void IRQ478_Handler(void);
__WEAK_ISR void IRQ479_Handler(void);

extern uint32_t __INITIAL_SP;

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

extern const VECTOR_TABLE_Type __VECTOR_TABLE[];
const VECTOR_TABLE_Type        __VECTOR_TABLE[496] __VECTOR_TABLE_ATTRIBUTE = {
    (VECTOR_TABLE_Type) &__INITIAL_SP,  //     Initial Stack Pointer
    Reset_Handler,                      // -15 Reset Handler
    NMI_Handler,                        // -14 NMI Handler
    HardFault_Handler,                  // -13 Hard Fault Handler
    MemManage_Handler,                  // -12 MPU Fault Handler
    BusFault_Handler,                   // -11 Bus Fault Handler
    UsageFault_Handler,                 // -10 Usage Fault Handler
    SecureFault_Handler,                //  -9 Secure Fault Handler
    0,                                  //     Reserved
    0,                                  //     Reserved
    0,                                  //     Reserved
    SVC_Handler,                        //  -5 SVCall Handler
    DebugMon_Handler,                   //  -4 Debug Monitor Handler
    0,                                  //     Reserved
    PendSV_Handler,                     //  -2 PendSV Handler
    SysTick_Handler,                    //  -1 SysTick Handler

    IRQ0_Handler,
    IRQ1_Handler,
    IRQ2_Handler,
    IRQ3_Handler,
    IRQ4_Handler,
    IRQ5_Handler,
    IRQ6_Handler,
    IRQ7_Handler,
    IRQ8_Handler,
    IRQ9_Handler,
    IRQ10_Handler,
    IRQ11_Handler,
    IRQ12_Handler,
    IRQ13_Handler,
    IRQ14_Handler,
    IRQ15_Handler,
    IRQ16_Handler,
    IRQ17_Handler,
    IRQ18_Handler,
    IRQ19_Handler,
    IRQ20_Handler,
    IRQ21_Handler,
    IRQ22_Handler,
    IRQ23_Handler,
    IRQ24_Handler,
    IRQ25_Handler,
    IRQ26_Handler,
    IRQ27_Handler,
    IRQ28_Handler,
    IRQ29_Handler,
    IRQ30_Handler,
    IRQ31_Handler,
    IRQ32_Handler,
    IRQ33_Handler,
    IRQ34_Handler,
    IRQ35_Handler,
    IRQ36_Handler,
    IRQ37_Handler,
    IRQ38_Handler,
    IRQ39_Handler,
    IRQ40_Handler,
    IRQ41_Handler,
    IRQ42_Handler,
    IRQ43_Handler,
    IRQ44_Handler,
    IRQ45_Handler,
    IRQ46_Handler,
    IRQ47_Handler,
    IRQ48_Handler,
    IRQ49_Handler,
    IRQ50_Handler,
    IRQ51_Handler,
    IRQ52_Handler,
    IRQ53_Handler,
    IRQ54_Handler,
    IRQ55_Handler,
    IRQ56_Handler,
    IRQ57_Handler,
    IRQ58_Handler,
    IRQ59_Handler,
    IRQ60_Handler,
    IRQ61_Handler,
    IRQ62_Handler,
    IRQ63_Handler,
    IRQ64_Handler,
    IRQ65_Handler,
    IRQ66_Handler,
    IRQ67_Handler,
    IRQ68_Handler,
    IRQ69_Handler,
    IRQ70_Handler,
    IRQ71_Handler,
    IRQ72_Handler,
    IRQ73_Handler,
    IRQ74_Handler,
    IRQ75_Handler,
    IRQ76_Handler,
    IRQ77_Handler,
    IRQ78_Handler,
    IRQ79_Handler,
    IRQ80_Handler,
    IRQ81_Handler,
    IRQ82_Handler,
    IRQ83_Handler,
    IRQ84_Handler,
    IRQ85_Handler,
    IRQ86_Handler,
    IRQ87_Handler,
    IRQ88_Handler,
    IRQ89_Handler,
    IRQ90_Handler,
    IRQ91_Handler,
    IRQ92_Handler,
    IRQ93_Handler,
    IRQ94_Handler,
    IRQ95_Handler,
    IRQ96_Handler,
    IRQ97_Handler,
    IRQ98_Handler,
    IRQ99_Handler,
    IRQ100_Handler,
    IRQ101_Handler,
    IRQ102_Handler,
    IRQ103_Handler,
    IRQ104_Handler,
    IRQ105_Handler,
    IRQ106_Handler,
    IRQ107_Handler,
    IRQ108_Handler,
    IRQ109_Handler,
    IRQ110_Handler,
    IRQ111_Handler,
    IRQ112_Handler,
    IRQ113_Handler,
    IRQ114_Handler,
    IRQ115_Handler,
    IRQ116_Handler,
    IRQ117_Handler,
    IRQ118_Handler,
    IRQ119_Handler,
    IRQ120_Handler,
    IRQ121_Handler,
    IRQ122_Handler,
    IRQ123_Handler,
    IRQ124_Handler,
    IRQ125_Handler,
    IRQ126_Handler,
    IRQ127_Handler,
    IRQ128_Handler,
    IRQ129_Handler,
    IRQ130_Handler,
    IRQ131_Handler,
    IRQ132_Handler,
    IRQ133_Handler,
    IRQ134_Handler,
    IRQ135_Handler,
    IRQ136_Handler,
    IRQ137_Handler,
    IRQ138_Handler,
    IRQ139_Handler,
    IRQ140_Handler,
    IRQ141_Handler,
    IRQ142_Handler,
    IRQ143_Handler,
    IRQ144_Handler,
    IRQ145_Handler,
    IRQ146_Handler,
    IRQ147_Handler,
    IRQ148_Handler,
    IRQ149_Handler,
    IRQ150_Handler,
    IRQ151_Handler,
    IRQ152_Handler,
    IRQ153_Handler,
    IRQ154_Handler,
    IRQ155_Handler,
    IRQ156_Handler,
    IRQ157_Handler,
    IRQ158_Handler,
    IRQ159_Handler,
    IRQ160_Handler,
    IRQ161_Handler,
    IRQ162_Handler,
    IRQ163_Handler,
    IRQ164_Handler,
    IRQ165_Handler,
    IRQ166_Handler,
    IRQ167_Handler,
    IRQ168_Handler,
    IRQ169_Handler,
    IRQ170_Handler,
    IRQ171_Handler,
    IRQ172_Handler,
    IRQ173_Handler,
    IRQ174_Handler,
    IRQ175_Handler,
    IRQ176_Handler,
    IRQ177_Handler,
    IRQ178_Handler,
    IRQ179_Handler,
    IRQ180_Handler,
    IRQ181_Handler,
    IRQ182_Handler,
    IRQ183_Handler,
    IRQ184_Handler,
    IRQ185_Handler,
    IRQ186_Handler,
    IRQ187_Handler,
    IRQ188_Handler,
    IRQ189_Handler,
    IRQ190_Handler,
    IRQ191_Handler,
    IRQ192_Handler,
    IRQ193_Handler,
    IRQ194_Handler,
    IRQ195_Handler,
    IRQ196_Handler,
    IRQ197_Handler,
    IRQ198_Handler,
    IRQ199_Handler,
    IRQ200_Handler,
    IRQ201_Handler,
    IRQ202_Handler,
    IRQ203_Handler,
    IRQ204_Handler,
    IRQ205_Handler,
    IRQ206_Handler,
    IRQ207_Handler,
    IRQ208_Handler,
    IRQ209_Handler,
    IRQ210_Handler,
    IRQ211_Handler,
    IRQ212_Handler,
    IRQ213_Handler,
    IRQ214_Handler,
    IRQ215_Handler,
    IRQ216_Handler,
    IRQ217_Handler,
    IRQ218_Handler,
    IRQ219_Handler,
    IRQ220_Handler,
    IRQ221_Handler,
    IRQ222_Handler,
    IRQ223_Handler,
    IRQ224_Handler,
    IRQ225_Handler,
    IRQ226_Handler,
    IRQ227_Handler,
    IRQ228_Handler,
    IRQ229_Handler,
    IRQ230_Handler,
    IRQ231_Handler,
    IRQ232_Handler,
    IRQ233_Handler,
    IRQ234_Handler,
    IRQ235_Handler,
    IRQ236_Handler,
    IRQ237_Handler,
    IRQ238_Handler,
    IRQ239_Handler,
    IRQ240_Handler,
    IRQ241_Handler,
    IRQ242_Handler,
    IRQ243_Handler,
    IRQ244_Handler,
    IRQ245_Handler,
    IRQ246_Handler,
    IRQ247_Handler,
    IRQ248_Handler,
    IRQ249_Handler,
    IRQ250_Handler,
    IRQ251_Handler,
    IRQ252_Handler,
    IRQ253_Handler,
    IRQ254_Handler,
    IRQ255_Handler,
    IRQ256_Handler,
    IRQ257_Handler,
    IRQ258_Handler,
    IRQ259_Handler,
    IRQ260_Handler,
    IRQ261_Handler,
    IRQ262_Handler,
    IRQ263_Handler,
    IRQ264_Handler,
    IRQ265_Handler,
    IRQ266_Handler,
    IRQ267_Handler,
    IRQ268_Handler,
    IRQ269_Handler,
    IRQ270_Handler,
    IRQ271_Handler,
    IRQ272_Handler,
    IRQ273_Handler,
    IRQ274_Handler,
    IRQ275_Handler,
    IRQ276_Handler,
    IRQ277_Handler,
    IRQ278_Handler,
    IRQ279_Handler,
    IRQ280_Handler,
    IRQ281_Handler,
    IRQ282_Handler,
    IRQ283_Handler,
    IRQ284_Handler,
    IRQ285_Handler,
    IRQ286_Handler,
    IRQ287_Handler,
    IRQ288_Handler,
    IRQ289_Handler,
    IRQ290_Handler,
    IRQ291_Handler,
    IRQ292_Handler,
    IRQ293_Handler,
    IRQ294_Handler,
    IRQ295_Handler,
    IRQ296_Handler,
    IRQ297_Handler,
    IRQ298_Handler,
    IRQ299_Handler,
    IRQ300_Handler,
    IRQ301_Handler,
    IRQ302_Handler,
    IRQ303_Handler,
    IRQ304_Handler,
    IRQ305_Handler,
    IRQ306_Handler,
    IRQ307_Handler,
    IRQ308_Handler,
    IRQ309_Handler,
    IRQ310_Handler,
    IRQ311_Handler,
    IRQ312_Handler,
    IRQ313_Handler,
    IRQ314_Handler,
    IRQ315_Handler,
    IRQ316_Handler,
    IRQ317_Handler,
    IRQ318_Handler,
    IRQ319_Handler,
    IRQ320_Handler,
    IRQ321_Handler,
    IRQ322_Handler,
    IRQ323_Handler,
    IRQ324_Handler,
    IRQ325_Handler,
    IRQ326_Handler,
    IRQ327_Handler,
    IRQ328_Handler,
    IRQ329_Handler,
    IRQ330_Handler,
    IRQ331_Handler,
    IRQ332_Handler,
    IRQ333_Handler,
    IRQ334_Handler,
    IRQ335_Handler,
    IRQ336_Handler,
    IRQ337_Handler,
    IRQ338_Handler,
    IRQ339_Handler,
    IRQ340_Handler,
    IRQ341_Handler,
    IRQ342_Handler,
    IRQ343_Handler,
    IRQ344_Handler,
    IRQ345_Handler,
    IRQ346_Handler,
    IRQ347_Handler,
    IRQ348_Handler,
    IRQ349_Handler,
    IRQ350_Handler,
    IRQ351_Handler,
    IRQ352_Handler,
    IRQ353_Handler,
    IRQ354_Handler,
    IRQ355_Handler,
    IRQ356_Handler,
    IRQ357_Handler,
    IRQ358_Handler,
    IRQ359_Handler,
    IRQ360_Handler,
    IRQ361_Handler,
    IRQ362_Handler,
    IRQ363_Handler,
    IRQ364_Handler,
    IRQ365_Handler,
    IRQ366_Handler,
    IRQ367_Handler,
    IRQ368_Handler,
    IRQ369_Handler,
    IRQ370_Handler,
    IRQ371_Handler,
    IRQ372_Handler,
    IRQ373_Handler,
    IRQ374_Handler,
    IRQ375_Handler,
    IRQ376_Handler,
    IRQ377_Handler,
    IRQ378_Handler,
    IRQ379_Handler,
    IRQ380_Handler,
    IRQ381_Handler,
    IRQ382_Handler,
    IRQ383_Handler,
    IRQ384_Handler,
    IRQ385_Handler,
    IRQ386_Handler,
    IRQ387_Handler,
    IRQ388_Handler,
    IRQ389_Handler,
    IRQ390_Handler,
    IRQ391_Handler,
    IRQ392_Handler,
    IRQ393_Handler,
    IRQ394_Handler,
    IRQ395_Handler,
    IRQ396_Handler,
    IRQ397_Handler,
    IRQ398_Handler,
    IRQ399_Handler,
    IRQ400_Handler,
    IRQ401_Handler,
    IRQ402_Handler,
    IRQ403_Handler,
    IRQ404_Handler,
    IRQ405_Handler,
    IRQ406_Handler,
    IRQ407_Handler,
    IRQ408_Handler,
    IRQ409_Handler,
    IRQ410_Handler,
    IRQ411_Handler,
    IRQ412_Handler,
    IRQ413_Handler,
    IRQ414_Handler,
    IRQ415_Handler,
    IRQ416_Handler,
    IRQ417_Handler,
    IRQ418_Handler,
    IRQ419_Handler,
    IRQ420_Handler,
    IRQ421_Handler,
    IRQ422_Handler,
    IRQ423_Handler,
    IRQ424_Handler,
    IRQ425_Handler,
    IRQ426_Handler,
    IRQ427_Handler,
    IRQ428_Handler,
    IRQ429_Handler,
    IRQ430_Handler,
    IRQ431_Handler,
    IRQ432_Handler,
    IRQ433_Handler,
    IRQ434_Handler,
    IRQ435_Handler,
    IRQ436_Handler,
    IRQ437_Handler,
    IRQ438_Handler,
    IRQ439_Handler,
    IRQ440_Handler,
    IRQ441_Handler,
    IRQ442_Handler,
    IRQ443_Handler,
    IRQ444_Handler,
    IRQ445_Handler,
    IRQ446_Handler,
    IRQ447_Handler,
    IRQ448_Handler,
    IRQ449_Handler,
    IRQ450_Handler,
    IRQ451_Handler,
    IRQ452_Handler,
    IRQ453_Handler,
    IRQ454_Handler,
    IRQ455_Handler,
    IRQ456_Handler,
    IRQ457_Handler,
    IRQ458_Handler,
    IRQ459_Handler,
    IRQ460_Handler,
    IRQ461_Handler,
    IRQ462_Handler,
    IRQ463_Handler,
    IRQ464_Handler,
    IRQ465_Handler,
    IRQ466_Handler,
    IRQ467_Handler,
    IRQ468_Handler,
    IRQ469_Handler,
    IRQ470_Handler,
    IRQ471_Handler,
    IRQ472_Handler,
    IRQ473_Handler,
    IRQ474_Handler,
    IRQ475_Handler,
    IRQ476_Handler,
    IRQ477_Handler,
    IRQ478_Handler,
    IRQ479_Handler};

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
