/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     issi_flash.c
 * @version  V1.0.0
 * @brief    Flash driver to set up flash in XIP mode
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#include "issi_flash_private.h"
#include "ospi_drv.h"
#include "ospi_xip_user.h"

static ospi_flash_cfg_t ospi_flash_config;

/**
  \fn         static void ospi_flash_reset(ospi_flash_cfg_t *ospi_cfg)
  \brief      This function resets the ISSI NOR Flash to the default state
  \param[in]  ospi_cfg : OSPI configuration structure
  \return     none
 */
static void issi_flash_reset(ospi_flash_cfg_t *ospi_cfg)
{
    ospi_setup_write(ospi_cfg, ADDR_LENGTH_0_BITS);
    ospi_send(ospi_cfg, ISSI_RESET_ENABLE);
    ospi_send(ospi_cfg, ISSI_RESET_MEMORY);
}

/**
  \fn         static void issi_write_enable(ospi_flash_cfg_t *ospi_cfg)
  \brief      Write enable for ISSI Flash before any write / program / erase
  \param[in]  ospi_cfg : OSPI configuration structure
  \return     none
 */
static void issi_write_enable(ospi_flash_cfg_t *ospi_cfg)
{
    /* Write WEL bit in OctalSPI mode */
    ospi_setup_write(ospi_cfg, ADDR_LENGTH_0_BITS);
    ospi_send(ospi_cfg, ISSI_WRITE_ENABLE);
}

/**
  \fn         static uint8_t issi_decode_id(ospi_flash_cfg_t *ospi_cfg, uint8_t *buffer)
  \brief      Decode ID if read in SDR mode
  \param[in]  ospi_cfg : OSPI configuration structure
  \param[in]  buffer : ID read in SDR mode
  \return     ID of NOR flash
 */
static uint8_t issi_decode_id(ospi_flash_cfg_t *ospi_cfg, uint8_t *buffer)
{
    uint8_t iter, id = 0;

    for (iter = 0 ; iter < 8; iter++)
    {
        /* Since SPI controller supports octal mode only, so 1 byte of data sent by flash will be distributed over 8 byte data read */
        if (*buffer & 0x2)
        {
            id |= 1;
        }
        if (iter < 7)
        {
            id <<= 1;
        }
        buffer++;
    }

    ospi_cfg->device_id = id;

    return id;
}

/**
  \fn         static uint8_t opsi_flash_ReadID_DDR(ospi_flash_cfg_t *ospi_cfg)
  \brief      This function reads the Device ID , if the ISSI NOR Flash boots up in DDR mode
  \param[in]  ospi_cfg : OSPI configuration structure
  \return     Device ID of NOR Flash
 */
static uint8_t issi_flash_ReadID_DDR(ospi_flash_cfg_t *ospi_cfg)
{
    uint8_t buffer;
    ospi_cfg->ddr_en = 1;

    ospi_setup_read(ospi_cfg, ADDR_LENGTH_0_BITS, 1, 8);
    ospi_recv(ospi_cfg, ISSI_READ_ID, &buffer);

    return buffer;
}

/**
  \fn        uint8_t ospi_flash_ReadID(ospi_flash_cfg_t *ospi_cfg)
  \brief     This function reads the Device ID , if the ISSI NOR Flash boots up in SDR mode
  \param[in] ospi_cfg : OSPI configuration structure
  \return    Device ID of NOR Flash
 */
static uint8_t issi_flash_ReadID(ospi_flash_cfg_t *ospi_cfg)
{
    uint8_t buffer[8];

    ospi_setup_read(ospi_cfg, ADDR_LENGTH_0_BITS, 8, 0);
    ospi_recv(ospi_cfg, ISSI_READ_ID, buffer);

    return issi_decode_id(ospi_cfg, buffer);
}

/**
  \fn      static void issi_flash_set_configuration_register_SDR(ospi_flash_cfg_t *ospi_cfg, uint8_t cmd, uint8_t address, uint8_t value)
  \brief   This function sets the configuration register of the ISSI NOR Flash in SDR mode
  \param[in] ospi_cfg : OSPI configuration structure
  \param[in] cmd : Command
  \param[in] address : Address of register
  \param[in] value : Value to be set
  \return    none
 */
static void issi_flash_set_configuration_register_SDR(ospi_flash_cfg_t *ospi_cfg, uint8_t cmd, uint8_t address, uint8_t value)
{
    issi_write_enable(ospi_cfg);
    ospi_setup_write_sdr(ospi_cfg, ADDR_LENGTH_24_BITS);
    ospi_push(ospi_cfg, cmd);
    ospi_push(ospi_cfg, 0x00);
    ospi_push(ospi_cfg, 0x00);
    ospi_push(ospi_cfg, address);
    ospi_send(ospi_cfg, value);
}

/**
  \fn      static void issi_flash_set_configuration_register_DDR(ospi_flash_cfg_t *ospi_cfg, uint8_t cmd, uint8_t address, uint8_t value)
  \brief   This function sets the configuration register of the ISSI NOR Flash in DDR mode
  \param[in] ospi_cfg : OSPI configuration structure
  \param[in] cmd : Command
  \param[in] address : Address of register
  \param[in] value : Value to be set
  \return    none
 */
static void issi_flash_set_configuration_register_DDR(ospi_flash_cfg_t *ospi_cfg, uint8_t cmd, uint32_t address, uint8_t value)
{
    issi_write_enable(ospi_cfg);
    ospi_setup_write(ospi_cfg, ADDR_LENGTH_32_BITS);
    ospi_push(ospi_cfg, cmd);
    ospi_push(ospi_cfg, address);
    ospi_push(ospi_cfg, value);
    ospi_send(ospi_cfg, value);
}

/**
  \fn      static int32_t issi_flash_read_configuration_register_ddr(ospi_flash_cfg_t *ospi_cfg, uint32_t reg_type, uint32_t address)
  \brief   This function reads the configuration register of the ISSI NOR Flash in DDR mode
  \param[in]  ospi_cfg : OSPI configuration structure
  \param[in]  cmd : Command
  \param[in]  address : Address of register
  \return     Value of register
 */
static uint32_t issi_flash_read_configuration_register_ddr(ospi_flash_cfg_t *ospi_cfg, uint32_t reg_type, uint32_t address)
{
    uint8_t val;

    ospi_setup_read(ospi_cfg, ADDR_LENGTH_32_BITS, 1, 8);

    if (reg_type == VOLATILE_CONFIG_REG)
    {
        ospi_push(ospi_cfg, ISSI_READ_VOLATILE_CONFIG_REG);
    }
    else if (reg_type == NONVOLATILE_CONFIG_REG)
    {
        ospi_push(ospi_cfg, ISSI_READ_NONVOLATILE_CONFIG_REG);
    }

    ospi_recv(ospi_cfg, address, &val);

    return val ;
}

/**
  \fn      static int issi_flash_probe (ospi_flash_cfg_t * ospi_cfg)
  \brief   Probe for an ISSI Flash device connected on the specified OSPI instance
  \param[in]  ospi_cfg : OSPI Flash Configuration
  \return     Success or Fail
 */
static int issi_flash_probe (ospi_flash_cfg_t *ospi_cfg)
{
    /* Initialize SPI in Single mode 1-1-1 and read Flash ID */
    if (issi_flash_ReadID(ospi_cfg) == DEVICE_ID_ISSI_FLASH_IS25WX256)
    {
        /* Set wrap configuration to 32 bytes */
        issi_flash_set_configuration_register_SDR(ospi_cfg, ISSI_WRITE_VOLATILE_CONFIG_REG, 0x07, WRAP_32_BYTE);

        /* Switch the flash to Octal DDR mode */
        issi_flash_set_configuration_register_SDR(ospi_cfg, ISSI_WRITE_VOLATILE_CONFIG_REG, 0x00, OCTAL_DDR_DQS);

        return 0;
    }

    return -1;
}

/**
  \fn         static int flash_xip_init(ospi_flash_cfg_t *ospi_cfg)
  \brief      This function initializes the flash in XIP mode
  \param[in]  ospi_cfg : OSPI configuration structure
  \return     Success or Fail
 */
static int flash_xip_init(ospi_flash_cfg_t *ospi_cfg)
{
    ospi_xip_enter(ospi_cfg, ISSI_DDR_OCTAL_IO_FAST_READ, ISSI_DDR_OCTAL_IO_FAST_READ);
    return 0;
}

/**
  \fn         int setup_flash_xip(void)
  \brief      This function initializes the NOR Flash and OSPI and enters the XIP mode
  \param[in]  none
  \return     Success or Fail
 */
int setup_flash_xip(void)
{
    ospi_flash_cfg_t *ospi_cfg = &ospi_flash_config;

#if OSPI_XIP_INSTANCE == OSPI0
    ospi_cfg->regs = (ssi_regs_t *) OSPI0_BASE;
    ospi_cfg->aes_base = (aes_regs_t *) AES0_BASE;
    ospi_cfg->xip_base = (volatile void *) OSPI0_XIP_BASE;
#else
    ospi_cfg->regs = (ssi_regs_t *) OSPI1_BASE;
    ospi_cfg->aes_regs = (aes_regs_t *) AES1_BASE;
    ospi_cfg->xip_base = (volatile void *) OSPI1_XIP_BASE;
#endif

    ospi_cfg->ser = 1;
    ospi_cfg->addrlen = ADDR_LENGTH_32_BITS;
    ospi_cfg->ospi_clock = OSPI_CLOCK;
    ospi_cfg->ddr_en = 0;
    ospi_cfg->wait_cycles = DEFAULT_WAIT_CYCLES_ISSI;

    ospi_init(ospi_cfg);

    if (issi_flash_probe(ospi_cfg))
    {
        return -1;
    }

    ospi_cfg->ddr_en = 1;

    if (flash_xip_init(ospi_cfg))
    {
        return -1;
    }

    return 0;
}
