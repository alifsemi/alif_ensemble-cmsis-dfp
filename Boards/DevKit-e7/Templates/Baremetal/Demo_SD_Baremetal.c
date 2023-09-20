/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**************************************************************************//**
 * @file     Demo_SD_Baremetal.c
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    Baremeetal sd driver test Application.
 * @bug      None.
 * @Note     None
 ******************************************************************************/
/* System Includes */
#include "stdio.h"

/* include for SD Driver */
#include "sd.h"
/* include for Pin Mux config */
#include "pinconf.h"

#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


#define BAREMETAL_SD_TEST_RAW_SECTOR 0x2000     //start reading and writing raw data from partition sector
volatile unsigned char sdbuffer[512*4] __attribute__((section("sd_dma_buf"))) __attribute__((aligned(32)));

const diskio_t  *p_SD_Driver = &SD_Driver;

/**
  \fn           BareMetalSDTest(uint32_t startSec, uint32_t EndSector)
  \brief        Baremetal SD driver Test Function
  \param[in]    starSecr - Test Read/Write start sector number
  \param[in]    EndSector - Test Read/Write End sector number
  \return       none
*/
void BareMetalSDTest(uint32_t startSec, uint32_t EndSector){

    int j;
    uint32_t *p = (uint32_t *)sdbuffer;

    /* SD Clock and Board Pin mux Configurations */
    pinconf_set(PORT_7, PIN_0, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE); //cmd
    pinconf_set(PORT_7, PIN_1, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE); //clk
    pinconf_set(PORT_5, PIN_0, PINMUX_ALTERNATE_FUNCTION_7, PADCTRL_READ_ENABLE); //d0
#ifdef SDMMC_4BIT_MODE
    pinconf_set(PORT_5, PIN_1, PINMUX_ALTERNATE_FUNCTION_7, PADCTRL_READ_ENABLE); //d1
    pinconf_set(PORT_5, PIN_2, PINMUX_ALTERNATE_FUNCTION_7, PADCTRL_READ_ENABLE); //d2
    pinconf_set(PORT_5, PIN_3, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE); //d3
#endif
#ifdef SDMMC_8BIT_MODE
    pinconf_set(PORT_5, PIN_4, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE); //d4
    pinconf_set(PORT_5, PIN_5, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_READ_ENABLE); //d5
    pinconf_set(PORT_5, PIN_6, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_READ_ENABLE); //d6
    pinconf_set(PORT_5, PIN_7, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_READ_ENABLE); //d7
#endif



    if(p_SD_Driver->disk_initialize(1) != SD_DRV_STATUS_OK){
        printf("SD initialization failed...\n");
        goto error;
    }

    /* read and print sector data from start to end */
    while(startSec < EndSector){

        if(p_SD_Driver->disk_read(startSec, 1, sdbuffer) != SD_DRV_STATUS_OK)
            continue;

        printf("Sector %d\n",startSec);
        j = 0;

        while(j<128){
            printf("%08x %08x %08x %08x\n",p[j+0], p[j+1], p[j+2], p[j+3]);
            j += 4;
        }

        if(p_SD_Driver->disk_write(startSec, 1, sdbuffer) != SD_DRV_STATUS_OK)
            printf("Unable to write Back sector: %d\n",startSec);
        startSec++;
    }

error:
    return;

}

int main()
{
    #if defined(RTE_Compiler_IO_STDOUT_User)
    int32_t ret;
    ret = stdout_init();
    if(ret != ARM_DRIVER_OK)
    {
        while(1)
        {
        }
    }
    #endif
    /* Enter the Baremetal demo Application.  */
    BareMetalSDTest(BAREMETAL_SD_TEST_RAW_SECTOR, BAREMETAL_SD_TEST_RAW_SECTOR+0x200);

    return 0;
}
