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
 * @file     DPHY_init.c
 * @author   Prasanna Ravi
 * @email    prasanna.ravi@alifsemi.com
 * @version  V1.0.0
 * @date     24-Feb-2022
 * @brief    Driver for MIPI DPHY.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "sys_ctrl_dphy.h"
#include "dsi.h"
#include "Driver_Common.h"
#include "DPHY_init.h"
#include "dphy.h"
#include "DPHY_Loopback_test.h"
#include "sys_ctrl_dsi.h"
#include "DPHY_Private.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "power.h"

#if (RTE_MIPI_CSI2)
#include "csi.h"
#include "sys_ctrl_csi.h"
#endif

#if (RTE_MIPI_CSI2) || (RTE_MIPI_DSI)
/*DPHY initialize status global variables*/
static volatile uint32_t csi2_init_status = 0;
static volatile uint32_t dsi_init_status = 0;

/*MIPI CSI2 and DSI register base*/
static DSI_Type * const dsi_reg_base = ((DSI_Type*)DSI_BASE);
#if(RTE_MIPI_CSI2)
static CSI_Type * const csi_reg_base = ((CSI_Type*)CSI_BASE);
#endif

/*hsfreqrange and osc_freq_target range*/
static const DPHY_FREQ_RANGE frequency_range[] =
{
    { 80, 0x00, 0x1B6 }, { 90, 0x10, 0x1B6 }, { 100, 0x20, 0x1B6 },
    { 110, 0x30, 0x1B6 }, { 120, 0x01, 0x1B6 }, { 130, 0x11, 0x1B6 },
    { 140, 0x21, 0x1B6 }, { 150, 0x31, 0x1B6 }, { 160, 0x02, 0x1B6 },
    { 170, 0x12, 0x1B6 }, { 180, 0x22, 0x1B6 }, { 190, 0x32, 0x1B6 },
    { 205, 0x03, 0x1B6 }, { 220, 0x13, 0x1B6 }, { 235, 0x23, 0x1B6 },
    { 250, 0x33, 0x1B6 }, { 275, 0x04, 0x1B6 }, { 300, 0x14, 0x1B6 },
    { 325, 0x25, 0x1B6 }, { 350, 0x35, 0x1B6 }, { 400, 0x05, 0x1B6 },
    { 450, 0x16, 0x1B6 }, { 500, 0x26, 0x1B6 }, { 550, 0x37, 0x1B6 },
    { 600, 0x07, 0x1B6 }, { 650, 0x18, 0x1B6 }, { 700, 0x28, 0x1B6 },
    { 750, 0x39, 0x1B6 }, { 800, 0x09, 0x1B6 }, { 850, 0x19, 0x1B6 },
    { 900, 0x29, 0x1B6 }, { 950, 0x3A, 0x1B6 }, { 1000, 0x0A, 0x1B6 },
    { 1050, 0x1A, 0x1B6 }, { 1100, 0x2A, 0x1B6 }, { 1150, 0x3B, 0x1B6 },
    { 1200, 0x0B, 0x1B6 }, { 1250, 0x1B, 0x1B6 }, { 1300, 0x2B, 0x1B6 },
    { 1350, 0x3C, 0x1B6 }, { 1400, 0x0C, 0x1B6 }, { 1450, 0x1C, 0x1B6 },
    { 1500, 0x2C, 0x1B6 }, { 1550, 0x3D, 0x10F }, { 1600, 0x0D, 0x118 },
    { 1650, 0x1D, 0x121 }, { 1700, 0x2E, 0x12A }, { 1750, 0x3E, 0x132 },
    { 1800, 0x0E, 0x13B }, { 1850, 0x1E, 0x144 }, { 1900, 0x2F, 0x14D },
    { 1950, 0x3F, 0x155 }, { 2000, 0x0F, 0x15E }, { 2050, 0x40, 0x167 },
    { 2100, 0x41, 0x170 }, { 2150, 0x42, 0x178 }, { 2200, 0x43, 0x181 },
    { 2250, 0x44, 0x18A }, { 2300, 0x45, 0x193 }, { 2350, 0x46, 0x19B },
    { 2400, 0x47, 0x1A4 }, { 2450, 0x48, 0x1AD }, { 2500, 0x49, 0x1B6 }
};

/*vco_cntrl range*/
static const DPHY_PLL_VCO_CTRL vco_ctrl_range[] =
{
    { 1170, 0x03 }, { 975, 0x07 }, { 853.125, 0x08 }, { 706.875, 0x08 },
    { 585, 0x0B }, { 487.5, 0x0F }, { 426.56, 0x10 }, { 353.4, 0x10 },
    { 292.5, 0x13 }, { 243.75, 0x17 }, { 213.3, 0x18 }, { 176.72, 0x18 },
    { 146.25, 0x1B }, { 121.88, 0x1F }, { 106.64, 0x20 }, { 88.36, 0x20 },
    { 73.13, 0x23}, { 60.93, 0x27 }, { 53.32, 0x28 }, { 44.18, 0x28 },
    { 40, 0x2B}
};

/*Output division factor range*/
static const DPHY_PLL_OUTPUT_DIVISION_FACTOR pll_p_factor[] =
{
    { 1000, 2 }, { 500, 4 }, { 250, 8 }, { 125, 16 }, { 62.5, 32 }, { 40, 64 }
};

/**
  \fn          static uint8_t MIPI_DPHY_Read (uint16_t address, DPHY_Mode mode)
  \brief       Test and control interface protocol to read DPHY registers.
  \param[in]   address index on DPHY register.
  \param[in]   mode is to select the DPHY mode(CSI2/DSI).
  \return      ret register value.
*/
static uint8_t MIPI_DPHY_Read (uint16_t address, DPHY_MODE_CFG mode)
{
    uint8_t ret = 0;
    uint32_t read_reg = 0;
    volatile uint32_t *test_ctrl0 = NULL;
    volatile uint32_t *test_ctrl1 = NULL;

#if (RTE_MIPI_CSI2)
    if(mode == DPHY_MODE_CFG_DSI)
    {
        test_ctrl0 = &dsi_reg_base->DSI_PHY_TST_CTRL0;
        test_ctrl1 = &dsi_reg_base->DSI_PHY_TST_CTRL1;
    }
    else
    {
        test_ctrl0 = &csi_reg_base->CSI_PHY_TEST_CTRL0;
        test_ctrl1 = &csi_reg_base->CSI_PHY_TEST_CTRL1;
    }
#else
    test_ctrl0 = &dsi_reg_base->DSI_PHY_TST_CTRL0;
    test_ctrl1 = &dsi_reg_base->DSI_PHY_TST_CTRL1;
#endif

    /*Ensure that t(r)x_testclk and t(r)x_testen is set to low*/
    CLEAR_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    CLEAR_BIT(*test_ctrl1, PHY_TESTEN_Msk);
    /*Set t(r)x_testen to high. */
    SET_BIT(*test_ctrl1, PHY_TESTEN_Msk);
    /*Set t(r)x_testen to high. */
    SET_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    /*Place 0x00 in t(r)x_testdin.*/
    read_reg = READ_REG(*test_ctrl1);
    read_reg &= ~(PHY_TESTDIN_Msk);
    WRITE_REG(*test_ctrl1, read_reg);
    /*Set t(r)x_testclk to low (with the falling edge on t(r)x_testclk,
    the t(r)x_testdin signal content is latched internally).*/
    CLEAR_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    /*Set t(r)x_testen to low*/
    CLEAR_BIT(*test_ctrl1, PHY_TESTEN_Msk);
    /*Place the 8-bit word corresponding to the testcode MSBs in t(r)x_testdin.*/
    read_reg = READ_REG(*test_ctrl1);
    read_reg &= ~(PHY_TESTDIN_Msk);
    read_reg |= _VAL2FLD(PHY_TESTDIN, (address >> 8));
    WRITE_REG(*test_ctrl1, read_reg);
    /*Set t(r)x_testclk to high.*/
    SET_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    /*Set t(r)x_testclk to low*/
    CLEAR_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    /*Set t(r)x_testen to high*/
    SET_BIT(*test_ctrl1, PHY_TESTEN_Msk);
    /*Set t(r)x_testclk to high.*/
    SET_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    /*Place the 8-bit word test data in t(r)x_testdin.*/
    read_reg = READ_REG(*test_ctrl1);
    read_reg &= ~(PHY_TESTDIN_Msk);
    read_reg |= _VAL2FLD(PHY_TESTDIN, address);
    WRITE_REG(*test_ctrl1, read_reg);
    /*Set t(r)x_testclk to low (with the falling edge on t(r)x_testclk,
    the t(r)x_testdin signal content is latched internally).*/
    CLEAR_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    read_reg = READ_REG(*test_ctrl1);
    read_reg &= (PHY_TESTDOUT_Msk);
    ret = _FLD2VAL(PHY_TESTDOUT, read_reg);
    /*Set t(r)x_testen to low.*/
    CLEAR_BIT(*test_ctrl1, PHY_TESTEN_Msk);

    return ret;
}

/**
  \fn          static void MIPI_CSI2_DPHY_Write (uint16_t address, uint8_t data)
  \brief       Test and control interface protocol to Write DPHY registers.
  \param[in]   address index on DPHY register.
  \param[in]   data register value.
  \param[in]   mode is to  select the DPHY mode(CSI2/DSI).
*/
static void MIPI_DPHY_Write (uint16_t address, uint8_t data, DPHY_MODE_CFG mode)
{
    uint32_t read_reg = 0;
    volatile uint32_t *test_ctrl0 = NULL;
    volatile uint32_t *test_ctrl1 = NULL;

#if (RTE_MIPI_CSI2)
    if(mode == DPHY_MODE_CFG_DSI)
    {
        test_ctrl0 = &dsi_reg_base->DSI_PHY_TST_CTRL0;
        test_ctrl1 = &dsi_reg_base->DSI_PHY_TST_CTRL1;
    }
    else
    {
        test_ctrl0 = &csi_reg_base->CSI_PHY_TEST_CTRL0;
        test_ctrl1 = &csi_reg_base->CSI_PHY_TEST_CTRL1;
    }
#else
    test_ctrl0 = &dsi_reg_base->DSI_PHY_TST_CTRL0;
    test_ctrl1 = &dsi_reg_base->DSI_PHY_TST_CTRL1;
#endif

    /*Ensure that t(r)x_testclk and t(r)x_testen is set to low*/
    CLEAR_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    CLEAR_BIT(*test_ctrl1, PHY_TESTEN_Msk);
    /*Set t(r)x_testen to high. */
    SET_BIT(*test_ctrl1, PHY_TESTEN_Msk);
    /*Set t(r)x_testen to high. */
    SET_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    /*Place 0x00 in t(r)x_testdin.*/
    read_reg = READ_REG(*test_ctrl1);
    read_reg &= ~(PHY_TESTDIN_Msk);
    WRITE_REG(*test_ctrl1, read_reg);
    /*Set t(r)x_testclk to low (with the falling edge on t(r)x_testclk,
    the t(r)x_testdin signal content is latched internally).*/
    CLEAR_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    /*Set t(r)x_testen to low*/
    CLEAR_BIT(*test_ctrl1, PHY_TESTEN_Msk);
    /*Place the 8-bit word corresponding to the testcode MSBs in t(r)x_testdin.*/
    read_reg = READ_REG(*test_ctrl1);
    read_reg &= ~(PHY_TESTDIN_Msk);
    read_reg |= _VAL2FLD(PHY_TESTDIN, (address >> 8));
    WRITE_REG(*test_ctrl1, read_reg);
    /*Set t(r)x_testclk to high.*/
    SET_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    /*Set t(r)x_testclk to low*/
    CLEAR_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    /*Set t(r)x_testen to high*/
    SET_BIT(*test_ctrl1, PHY_TESTEN_Msk);
    /*Set t(r)x_testclk to high.*/
    SET_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    /*Place the 8-bit word test data in t(r)x_testdin.*/
    read_reg = READ_REG(*test_ctrl1);
    read_reg &= ~(PHY_TESTDIN_Msk);
    read_reg |= _VAL2FLD(PHY_TESTDIN, address);
    WRITE_REG(*test_ctrl1, read_reg);
    /*Set t(r)x_testclk to low (with the falling edge on t(r)x_testclk,
    the t(r)x_testdin signal content is latched internally).*/
    CLEAR_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    /*Set t(r)x_testen to low.*/
    CLEAR_BIT(*test_ctrl1, PHY_TESTEN_Msk);
    /*Place the 8-bit word corresponding to the page offset in t(r)x_testdin.*/
    read_reg = READ_REG(*test_ctrl1);
    read_reg &= ~(PHY_TESTDIN_Msk);
    read_reg |= _VAL2FLD(PHY_TESTDIN, data);
    WRITE_REG(*test_ctrl1, read_reg);
    /*Set t(r)x_testclk to high (test data is programmed internally).*/
    SET_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
    CLEAR_BIT(*test_ctrl0, PHY_TESTCLK_Msk);
}
#endif

#if (RTE_MIPI_CSI2)
/**
  \fn          static void MIPI_CSI2_DPHY_Shutdown (uint8_t state)
  \brief       PHY shutdown line control callback function.
  \param[in]   state ENABLE/DISABLE the line.
*/
static void MIPI_CSI2_DPHY_Shutdown (uint8_t state)
{
    if(state == ENABLE)
    {
        csi_enable_dphy_shutdown_line((CSI_Type *)CSI_BASE);
    }
    else
    {
        csi_disable_dphy_shutdown_line((CSI_Type *)CSI_BASE);
    }
}

/**
  \fn          static void MIPI_CSI2_DPHY_Testclr (uint8_t state)
  \brief       PHY testclr line control callback function.
  \param[in]   state ENABLE/DISABLE the line.
*/
static void MIPI_CSI2_DPHY_Testclr (uint8_t state)
{
    if(state == ENABLE)
    {
        csi_enable_dphy_testclr_line((CSI_Type *)CSI_BASE);
    }
    else
    {
        csi_disable_dphy_testclr_line((CSI_Type *)CSI_BASE);
    }
}

/**
  \fn          static void MIPI_CSI2_DPHY_Rst (uint8_t state)
  \brief       PHY reset line control callback function.
  \param[in]   state ENABLE/DISABLE the line.
*/
static void MIPI_CSI2_DPHY_Rst (uint8_t state)
{
    if(state == ENABLE)
    {
        csi_enable_dphy_reset_line((CSI_Type *)CSI_BASE);
    }
    else
    {
        csi_disable_dphy_reset_line((CSI_Type *)CSI_BASE);
    }
}

/**
  \fn          static uint8_t MIPI_CSI2_DPHY_Stopstate (void)
  \brief       status of stopstate from PHY
  \return      ret status of stopstate.
*/
static DPHY_STOPSTATE MIPI_CSI2_DPHY_Stopstate (void)
{
    uint8_t ret = 0;

    if(csi_get_lane_stopstate_status((CSI_Type *)CSI_BASE, CSI_LANE_CLOCK) == CSI_LANE_STOPSTATE_ON)
    {
        ret |= DPHY_STOPSTATE_LANE0;
    }

    if(csi_get_lane_stopstate_status((CSI_Type *)CSI_BASE, CSI_LANE_0) == CSI_LANE_STOPSTATE_ON)
    {
        ret |= DPHY_STOPSTATE_LANE1;
    }

    if(csi_get_lane_stopstate_status((CSI_Type *)CSI_BASE, CSI_LANE_1) == CSI_LANE_STOPSTATE_ON)
    {
        ret |= DPHY_STOPSTATE_CLOCK;
    }


    return ret;
}

/**
  \fn          uint8_t DPHY_CSI2_Read_Mask (uint16_t address,
                                            uint8_t  pos,
                                            uint8_t  width)
  \brief       Read Mask CSI2 DPHY registers.
  \param[in]   address is register index.
  \param[in]   pos  is start bit position.
  \param[in]   width is number bits to read.
  \return      return received data from DPHY register.
*/
static uint8_t DPHY_CSI2_Read_Mask (uint16_t address,
                                    uint8_t  pos,
                                    uint8_t  width)
{
    return (MIPI_DPHY_Read(address, DPHY_MODE_CFG_CSI2) >> pos) & ((1 << width) - 1);
}

/**
  \fn          void DPHY_CSI2_Write_Mask (uint16_t address,
                                          uint8_t  data,
                                          uint8_t  pos,
                                          uint8_t  width)
  \brief       write Mask CSI2 DPHY registers.
  \param[in]   address is register index
  \param[in]   data is value to be write to the DPHY register.
  \param[in]   pos  is start bit position.
  \param[in]   width is number bits to write.
*/
static void DPHY_CSI2_Write_Mask (uint16_t address,
                                  uint8_t  data,
                                  uint8_t  pos,
                                  uint8_t  width)
{
    uint8_t reg_data = 0;
    uint8_t mask = (1U << width) - 1;

    reg_data = MIPI_DPHY_Read(address, DPHY_MODE_CFG_CSI2);
    reg_data &= ~(mask << pos);
    reg_data |= (data & mask) << pos;
    MIPI_DPHY_Write(address, reg_data, DPHY_MODE_CFG_CSI2);
}
#endif


#if (RTE_MIPI_CSI2) || (RTE_MIPI_DSI)
/**
  \fn          static void MIPI_DSI_DPHY_Shutdown (uint8_t state)
  \brief       PHY shutdown line control callback function.
  \param[in]   state ENABLE/DISABLE the line.
  */
static void MIPI_DSI_DPHY_Shutdown (uint8_t state)
{
    if(state == ENABLE)
    {
        dsi_phy_shutdown_enable((DSI_Type *)DSI_BASE);
    }
    else
    {
        dsi_phy_shutdown_disable((DSI_Type *)DSI_BASE);
    }
}

/**
  \fn          static void MIPI_DSI_DPHY_Rst (uint8_t state)
  \brief       PHY reset line control callback function.
  \param[in]   state ENABLE/DISABLE the line.
  */
static void MIPI_DSI_DPHY_Rst (uint8_t state)
{
    if(state == ENABLE)
    {
        dsi_phy_reset_enable((DSI_Type *)DSI_BASE);
    }
    else
    {
        dsi_phy_reset_disable((DSI_Type *)DSI_BASE);
    }
}

/**
  \fn          static void MIPI_DSI_DPHY_Enableclk (uint8_t state)
  \brief       PHY enable clock line control callback function.
  \param[in]   state ENABLE/DISABLE the line.
  */
static void MIPI_DSI_DPHY_Enableclk (uint8_t state)
{
    if(state == ENABLE)
    {
        dsi_phy_enable_clock((DSI_Type *)DSI_BASE);
    }
    else
    {
        dsi_phy_disable_clock((DSI_Type *)DSI_BASE);
    }
}

/**
  \fn          static void MIPI_DSI_DPHY_Testclr (uint8_t state)
  \brief       PHY testclr line control callback function.
  \param[in]   state ENABLE/DISABLE the line.
  */
static void MIPI_DSI_DPHY_Testclr (uint8_t state)
{
    if(state == ENABLE)
    {
        dsi_phy_testclr_enable((DSI_Type *)DSI_BASE);
    }
    else
    {
        dsi_phy_testclr_disable((DSI_Type *)DSI_BASE);
    }

}

/**
  \fn          static DSI_PLL_STATUS MIPI_DSI_DPHY_PLL_Lock (void)
  \brief       PHY testclr line control callback function.
  \return      return status of the PLL lock.
  */
static DSI_PLL_STATUS MIPI_DSI_DPHY_PLL_Lock (void)
{
    return dsi_get_phy_lock_status((DSI_Type *)DSI_BASE);
}

/**
  \fn          static uint8_t MIPI_DSI_DPHY_Stopstate (void)
  \brief       status of stopstate from PHY
  \return      return status of stopstate.
  */
static DPHY_STOPSTATE MIPI_DSI_DPHY_Stopstate (void)
{
    uint8_t ret = 0;

    if(dsi_get_lane_stopstate_status((DSI_Type *)DSI_BASE, DSI_LANE_CLOCK) == DSI_LANE_STOPSTATE_ON)
    {
        ret |= DPHY_STOPSTATE_LANE0;
    }

    if(dsi_get_lane_stopstate_status((DSI_Type *)DSI_BASE, DSI_LANE_0) == DSI_LANE_STOPSTATE_ON)
    {
        ret |= DPHY_STOPSTATE_LANE1;
    }

    if(dsi_get_lane_stopstate_status((DSI_Type *)DSI_BASE, DSI_LANE_1) == DSI_LANE_STOPSTATE_ON)
    {
        ret |= DPHY_STOPSTATE_CLOCK;
    }

    return ret;

}

/**
  \fn          uint8_t DPHY_DSI_Read_Mask (uint16_t address,
                                           uint8_t  pos,
                                           uint8_t  width)
  \brief       Read Mask DSI DPHY registers.
  \param[in]   address is register index.
  \param[in]   pos  is start bit position.
  \param[in]   width is number bits to read.
  \return      return received data from DPHY register.
*/
static uint8_t DPHY_DSI_Read_Mask (uint16_t address,
                                   uint8_t  pos,
                                   uint8_t  width)
{
    return (MIPI_DPHY_Read(address, DPHY_MODE_CFG_DSI) >> pos) & ((1 << width) - 1);
}


/**
  \fn          void DPHY_DSI_Write_Mask (uint16_t address,
                                         uint8_t  data,
                                         uint8_t  pos,
                                         uint8_t  width)
  \brief       write Mask DSI DPHY registers.
  \param[in]   address is register index
  \param[in]   data is value to be write to the DPHY register.
  \param[in]   pos  is start bit position.
  \param[in]   width is number bits to write.
*/
static void DPHY_DSI_Write_Mask (uint16_t address,
                                 uint8_t  data,
                                 uint8_t  pos,
                                 uint8_t  width)
{
    uint8_t reg_data = 0;
    uint8_t mask = (1U << width) - 1;

    reg_data = MIPI_DPHY_Read(address, DPHY_MODE_CFG_DSI);
    reg_data &= ~(mask << pos);
    reg_data |= (data & mask) << pos;
    MIPI_DPHY_Write(address,reg_data, DPHY_MODE_CFG_DSI);
}

/**
  \fn          void DPHY_PowerEnable (void)
  \brief       Enable DPHY Interface Power.
*/
static void DPHY_PowerEnable (void)
{
    enable_cgu_clk100m();

    enable_cgu_clk38p4m();

    enable_mipi_dphy_power();

    disable_mipi_dphy_isolation();

    enable_dphy_pll_reference_clock();

    enable_txdphy_configure_clock();

    enable_dsi_periph_clk();

#if RTE_MIPI_CSI2
    enable_csi_periph_clk();

    enable_rxdphy_configure_clock();
#endif

}

/**
  \fn          void DPHY_PowerDisable (void)
  \brief       Disable DPHY Interface Power.
*/
static void DPHY_PowerDisable (void)
{

#if RTE_MIPI_CSI2
    disable_csi_periph_clk();

    disable_rxdphy_configure_clock();
#endif

    disable_dsi_periph_clk();

    disable_txdphy_configure_clock();

    disable_dphy_pll_reference_clock();

    disable_mipi_dphy_power();

    enable_mipi_dphy_isolation();

    disable_cgu_clk38p4m();

    disable_cgu_clk100m();

}

/**
  \fn          int32_t DPHY_ConfigurePLL(uint32_t clock_frequency)
  \brief       configuring MIPI TX DPHY PLL.
  \param[in]   clock_frequency DPHY clock frequency.
  \return      \ref execution_status
*/
static int32_t DPHY_ConfigurePLL(uint32_t clock_frequency)
{
    float frequency_in_mhz = clock_frequency/1000000.0f;
    uint32_t pll_m = 0;
    uint8_t pll_p = 0;
    uint8_t vco_ctrl = 0;
    uint8_t range = 0;

#if (RTE_MIPI_DSI)
    uint8_t pll_n = RTE_MIPI_DSI_PLL_INPUT_DIV_FACTOR_N;
#else
    uint8_t pll_n = DPHY_DEFAULT_PLL_INPUT_DIV_FACTOR_N;
#endif

    if(((DPHY_FCLKIN_MHZ/pll_n) > 24) || ((DPHY_FCLKIN_MHZ/pll_n) < 8))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    for( range = 0; (range < ARRAY_SIZE(vco_ctrl_range) - 1) &&
        ((frequency_in_mhz) < vco_ctrl_range[range].frequency_mhz);
        ++range);

    vco_ctrl = vco_ctrl_range[range].vco_ctrl;

    for( range = 0; (range < ARRAY_SIZE(pll_p_factor) - 1) &&
    ((frequency_in_mhz) <= pll_p_factor[range].frequency_mhz);
    ++range);

    pll_p = pll_p_factor[range].p;

    pll_m = ((frequency_in_mhz * pll_n * pll_p * 2) / DPHY_FCLKIN_MHZ);

    enable_dphy_pll_shadow_control();

    set_dphy_pll_clksel(DPHY_PLL_CLKSEL_CLOCK_GENERAT);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_28, (uint8_t)pll_m, 0, 8);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_29, (uint8_t)(pll_m >> 8), 0, 2);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_30, 0x1, 0, 1);

    /*According to HW specification setting Input Frequency Division Ratio by N - 1*/
    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_27, (pll_n - 1), 3, 4);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_27, 0x1, 7, 1);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_30, vco_ctrl, 1, 6);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_30, 0x1, 7, 1);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_1, DPHY_CPBIAS_CNTRL, 0, 7);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_5, DPHY_GMP_CNTRL, 0, 2);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_5, DPHY_INT_CNTRL, 2, 6);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_17, DPHY_PROP_CNTRL, 0, 6);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_17, 0x1, 7, 1);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_17, 0x1, 6, 1);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_CB_3, DPHY_CB_VREF_MPLL_REG_REL_RW, 0, 3);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t DPHY_MasterSetup (uint32_t clock_frequency, uint8_t n_lanes, DPHY_CLK_MODE clk_mode)
  \brief       MIPI DPHY Tx startup sequence.
  \param[in]   clock_frequency DPHY clock frequency.
  \param[in]   n_lanes number of lanes.
  \param[in]   clk_mode select DPHY clock mode \ref DPHY_CLK_MODE.
  \return      \ref execution_status
*/
static int32_t DPHY_MasterSetup (uint32_t clock_frequency, uint8_t n_lanes, DPHY_CLK_MODE clk_mode)
{
    uint32_t bitrate_mbps = (clock_frequency * 2)/1000000;
    uint8_t hsfreqrange = 0;
    uint8_t cfgclkfreqrange = 0;
    uint8_t range = 0;
    uint8_t stopstate_check = 0;
    uint32_t lp_count = 0;

    for(range = 0; (range < ARRAY_SIZE(frequency_range) - 1) &&
        ((bitrate_mbps) > frequency_range[range].bitrate_in_mbps);
        ++range);

    hsfreqrange = frequency_range[range].hsfreqrange;

    dsi_set_active_lanes((DSI_Type *)DSI_BASE, n_lanes - 1);


    MIPI_DSI_DPHY_Rst(DISABLE);

    MIPI_DSI_DPHY_Shutdown(DISABLE);

    set_tx_dphy_txrx(DPHY_MODE_MASTER);

    set_tx_dphy_testport_select(DPHY_TESTPORT_SELECT_RX);
    MIPI_DSI_DPHY_Testclr(ENABLE);
    set_tx_dphy_testport_select(DPHY_TESTPORT_SELECT_TX);
    MIPI_DSI_DPHY_Testclr(ENABLE);

    sys_busy_loop_us(1);

    set_tx_dphy_testport_select(DPHY_TESTPORT_SELECT_RX);
    MIPI_DSI_DPHY_Testclr(DISABLE);
    set_tx_dphy_testport_select(DPHY_TESTPORT_SELECT_TX);
    MIPI_DSI_DPHY_Testclr(DISABLE);

    set_tx_dphy_hsfreqrange(hsfreqrange);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_13, 0x3, 0, 2);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_CB_1, 0x2, 0, 2);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_CB_0, 0x2, 5, 2);

    if(bitrate_mbps < 450)
        DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_CB_2, 0x1, 4, 1);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_CLK_TERMLOWCAP, 0x2, 0, 2);

    if(bitrate_mbps <= 1000)
    {
        DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_SLEW_5,
                               (uint8_t)DPHY_LESS_THEN_1GBPS_SR_OSC_FREQ_TARGET,
                               0, 8);
        DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_SLEW_6,
                               (uint8_t)(DPHY_LESS_THEN_1GBPS_SR_OSC_FREQ_TARGET >> 8),
                               0, 4);
        DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_SLEW_7, 0x1, 4, 1);
        DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_SLEW_7, 0x1, 0, 1);
    }
    else if ((bitrate_mbps > 1000) && (bitrate_mbps <= 1500))
    {
        DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_SLEW_5,
                               (uint8_t)DPHY_MORE_THEN_1GBPS_SR_OSC_FREQ_TARGET,
                               0, 8);
        DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_SLEW_6,
                               (uint8_t)(DPHY_MORE_THEN_1GBPS_SR_OSC_FREQ_TARGET >> 8),
                               0, 4);
    }

    cfgclkfreqrange = (DPHY_FCFG_CLOCK_MHZ - 17) * 4;

    set_tx_dphy_cfgclkfreqrange(cfgclkfreqrange);

    if(DPHY_ConfigurePLL(clock_frequency) != ARM_DRIVER_OK)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    unset_tx_dphy_basedir((1U << n_lanes) - 1);


    unset_tx_dphy_forcerxmode((1U << n_lanes) - 1);


    sys_busy_loop_us(1);

    MIPI_DSI_DPHY_Enableclk(ENABLE);

    sys_busy_loop_us(1);

    MIPI_DSI_DPHY_Shutdown(ENABLE);

    sys_busy_loop_us(1);

    MIPI_DSI_DPHY_Rst(ENABLE);


    while(MIPI_DSI_DPHY_PLL_Lock() != (DSI_PLL_STATUS) DPHY_PLL_STATUS_PLL_LOCK)
    {
        if(lp_count++ < 1000000)
        {
            sys_busy_loop_us(1);
        }
        else
        {
            return ARM_DRIVER_ERROR;
        }
    }


    if(clk_mode == DPHY_CLK_MODE_NON_CONTINUOUS)
    {
        stopstate_check |= DPHY_STOPSTATE_CLOCK;
    }

    stopstate_check |= (n_lanes == 1 ? (DPHY_STOPSTATE_LANE0) :
                                             (DPHY_STOPSTATE_LANE0) | (DPHY_STOPSTATE_LANE1) );



    lp_count = 0;
    while(MIPI_DSI_DPHY_Stopstate() != stopstate_check)
    {
        if(lp_count++ < 1000000)
        {
            sys_busy_loop_us(1);
        }
        else
        {
            return ARM_DRIVER_ERROR;
        }
    }

    return ARM_DRIVER_OK;

}
#endif

#if (RTE_MIPI_DSI)
/**
  \fn          int32_t DSI_DPHY_Initialize (uint32_t frequency)
  \brief       Initialize MIPI DSI DPHY Interface.
  \param[in]   frequency to configure DPHY PLL.
  \return      \ref execution_status
  */
int32_t DSI_DPHY_Initialize (uint32_t frequency)
{
    int32_t ret = ARM_DRIVER_OK;

    if(dsi_init_status == DPHY_INIT_STATUS_INITIALIZED)
    {
        return ARM_DRIVER_OK;
    }

    if(csi2_init_status == DPHY_INIT_STATUS_UNINITIALIZED)
    {
        DPHY_PowerEnable();
    }

    ret = DPHY_MasterSetup(frequency, RTE_MIPI_DSI_N_LANES, RTE_MIPI_DSI_NON_CONTINUOUS_CLOCK_MODE);
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    dsi_init_status = DPHY_INIT_STATUS_INITIALIZED;

    return ret;
}

/**
  \fn          int32_t DSI_DPHY_Uninitialize (void)
  \brief       Uninitialize MIPI DSI DPHY Interface.
  \return      \ref execution_status
  */
int32_t DSI_DPHY_Uninitialize (void)
{
    if(dsi_init_status == DPHY_INIT_STATUS_UNINITIALIZED)
    {
        return ARM_DRIVER_OK;
    }

    if(csi2_init_status == DPHY_INIT_STATUS_UNINITIALIZED)
    {
        DPHY_PowerDisable();
    }

    dsi_init_status = DPHY_INIT_STATUS_UNINITIALIZED;

    return ARM_DRIVER_OK;
}

#endif

#if (RTE_MIPI_CSI2)
/**
  \fn          int32_t DPHY_SlaveSetup (uint32_t clock_frequency, uint8_t n_lanes, DPHY_CLK_MODE hsclk_mode)
  \brief       MIPI DPHY Rx startup sequence.
  \param[in]   clock_frequency DPHY clock frequency.
  \param[in]   n_lanes number of lanes.
  \param[in]   clk_mode select DPHY clock mode \ref DPHY_CLK_MODE.
  \return      \ref execution_status
*/
static int32_t DPHY_SlaveSetup (uint32_t clock_frequency, uint8_t n_lanes, DPHY_CLK_MODE clk_mode)
{
    uint32_t bitrate = (clock_frequency * 2);
    uint8_t hsfreqrange = 0;
    uint8_t cfgclkfreqrange = 0;
    uint32_t osc_freq_target = 0;
    uint8_t range = 0;
    uint8_t stopstate_check =0;
    uint32_t lp_count = 0;

    for(range = 0; (range < ARRAY_SIZE(frequency_range) - 1) &&
        ((bitrate/1000000) > frequency_range[range].bitrate_in_mbps);
        ++range);

    hsfreqrange = frequency_range[range].hsfreqrange;
    osc_freq_target = frequency_range[range].osc_freq_target;

    MIPI_CSI2_DPHY_Rst(DISABLE);

    MIPI_CSI2_DPHY_Shutdown(DISABLE);

    set_rx_dphy_txrx(DPHY_MODE_SLAVE);

    set_rx_dphy_testport_select(DPHY_TESTPORT_SELECT_RX);
    MIPI_CSI2_DPHY_Testclr(ENABLE);
    set_rx_dphy_testport_select(DPHY_TESTPORT_SELECT_TX);
    MIPI_CSI2_DPHY_Testclr(ENABLE);

    sys_busy_loop_us(1);

    set_rx_dphy_testport_select(DPHY_TESTPORT_SELECT_RX);
    MIPI_CSI2_DPHY_Testclr(DISABLE);
    set_rx_dphy_testport_select(DPHY_TESTPORT_SELECT_TX);
    MIPI_CSI2_DPHY_Testclr(DISABLE);

    set_rx_dphy_hsfreqrange(hsfreqrange);

    DPHY_CSI2_Write_Mask(dphy4txtester_DIG_RDWR_TX_PLL_13, 0x3, 0, 2);

    DPHY_CSI2_Write_Mask(dphy4txtester_DIG_RDWR_TX_CB_1, 0x2, 0, 2);

    DPHY_CSI2_Write_Mask(dphy4txtester_DIG_RDWR_TX_CB_0, 0x2, 5, 2);

    set_rx_dphy_testport_select(DPHY_TESTPORT_SELECT_RX);

    DPHY_CSI2_Write_Mask(dphy4rxtester_DIG_RDWR_RX_CLKLANE_LANE_6, 0x1, 7, 1);

    if((bitrate/1000000) == 80)
    {
        DPHY_CSI2_Write_Mask(dphy4rxtester_DIG_RD_RX_SYS_1, 0x85, 0, 8);
    }

    DPHY_CSI2_Write_Mask(dphy4rxtester_DIG_RDWR_RX_RX_STARTUP_OVR_2, (uint8_t)osc_freq_target, 0, 8);

    DPHY_CSI2_Write_Mask(dphy4rxtester_DIG_RDWR_RX_RX_STARTUP_OVR_3, (uint8_t)(osc_freq_target >> 8), 0, 4);

    DPHY_CSI2_Write_Mask(dphy4rxtester_DIG_RDWR_RX_RX_STARTUP_OVR_4, 0x1, 0, 1);

    cfgclkfreqrange = (DPHY_FCFG_CLOCK_MHZ - 17) * 4;

    set_rx_dphy_cfgclkfreqrange(cfgclkfreqrange);

    set_rx_dphy_basedir((1U << n_lanes) - 1);

    set_rx_dphy_forcerxmode((1U << n_lanes) - 1);

    sys_busy_loop_us(1);

    MIPI_CSI2_DPHY_Shutdown(ENABLE);

    sys_busy_loop_us(1);

    MIPI_CSI2_DPHY_Rst(ENABLE);


    if(clk_mode == DPHY_CLK_MODE_NON_CONTINUOUS)
    {
        stopstate_check |= DPHY_STOPSTATE_CLOCK;
    }

    stopstate_check |= (n_lanes == 1 ? (DPHY_STOPSTATE_LANE0) :
                                             (DPHY_STOPSTATE_LANE0) | (DPHY_STOPSTATE_LANE1) );

    while(MIPI_CSI2_DPHY_Stopstate() != stopstate_check)
    {
        if(lp_count++ < 1000000)
        {
            sys_busy_loop_us(1);
        }
        else
        {
            return ARM_DRIVER_ERROR;
        }
    }

    unset_rx_dphy_forcerxmode((1U << n_lanes) - 1);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t CSI2_DPHY_Initialize (uint32_t frequency, uint8_t n_lanes, DPHY_CLK_MODE clk_mode)
  \brief       Initialize MIPI CSI2 DPHY Interface.
  \param[in]   frequency to configure DPHY PLL.
  \param[in]   n_lanes number of lanes.
  \param[in]   clk_mode select DPHY clock mode \ref DPHY_CLK_MODE.
  \return      \ref execution_status
  */
int32_t CSI2_DPHY_Initialize (uint32_t frequency, uint8_t n_lanes, DPHY_CLK_MODE clk_mode)
{
    int32_t ret = ARM_DRIVER_OK;
    uint32_t pixclk, htotal, vtotal;
    uint32_t dsi_freq;

    if(csi2_init_status == DPHY_INIT_STATUS_INITIALIZED)
    {
        return ARM_DRIVER_OK;

    }

    if(dsi_init_status == DPHY_INIT_STATUS_UNINITIALIZED)
    {
        DPHY_PowerEnable();
    }

    if(dsi_init_status == DPHY_INIT_STATUS_UNINITIALIZED)
    {
#if (RTE_MIPI_DSI)

        /* LCD Manufacturer provides the Frame timing values
         *     HTOTAL = WIDTH + HSYNC + HFP + HBP
         *     VTOTAL = HEIGHT + VSYNC + VFP + VBP
         * Calculate the pixel clock for DPI controller
         *     PIXCLK = FPS x HTOTAL x VTOTAL
         * Calculate the pixel clock divider
         *     PIXCLK_DIV = CDC200_PIXCLK_SOURCE / PIXCLK
         */
        htotal = (RTE_PANEL_HSYNC_TIME
                  + RTE_PANEL_HBP_TIME
                  + RTE_PANEL_HFP_TIME
                  + RTE_PANEL_HACTIVE_TIME);

        vtotal = (RTE_PANEL_VSYNC_LINE
                  + RTE_PANEL_VBP_LINE
                  + RTE_PANEL_VFP_LINE
                  + RTE_PANEL_VACTIVE_LINE);

        pixclk = (htotal * vtotal * RTE_CDC200_DPI_FPS);

        /* SCALE = LANEBYTECLK / PIXCLK
         * MIPI data rate must be exactly equal, not greater than, for 1.5 scale to work
         * MIPI data rate + 33% allows for scaling times 2
         *    24 x 1.333 / 16 = 2
         * LANEBYTECLK = PIXCLK * SCALE
         * lanebyteclk frequency is 1/4th of the DPHY frequency
         * PLL frequency = LANEBYTECLK * 4
         *               = PIXCLK * SCALE * 4
         */
        dsi_freq = pixclk * 4 * 2;

#if (RTE_MIPI_DSI_ILI9806E_PANEL)
        /*Checking LCD Panel supports MIPI DSI DPHY data rate*/
        if((dsi_freq * 2) > (RTE_ILI9806E_PANEL_MAX_BITRATE_MBPS * 1000000))
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
#endif
        ret = DSI_DPHY_Initialize(dsi_freq);
        if(ret != ARM_DRIVER_OK)
        {
            return ret;
        }
#else
        ret = DPHY_MasterSetup(DPHY_MINIMUM_FREQUENCY, n_lanes, clk_mode);
        if(ret != ARM_DRIVER_OK)
        {
            return ret;
        }
#endif
    }

    ret = DPHY_SlaveSetup(frequency, n_lanes, clk_mode);
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    csi2_init_status = DPHY_INIT_STATUS_INITIALIZED;

    return ret;
}

/**
  \fn          int32_t CSI2_DPHY_Uninitialize (void)
  \brief       Uninitialize MIPI CSI2 DPHY Interface.
  \return      \ref execution_status
  */
int32_t CSI2_DPHY_Uninitialize (void)
{
    if(csi2_init_status == DPHY_INIT_STATUS_UNINITIALIZED)
    {
        return ARM_DRIVER_OK;
    }

    if(dsi_init_status  == DPHY_INIT_STATUS_UNINITIALIZED)
    {
        DPHY_PowerDisable();
    }

    csi2_init_status = DPHY_INIT_STATUS_UNINITIALIZED;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t DPHY_ADC_Probing_Procedure (void)
  \brief       ADC Probing Procedure.
  \return      \ref execution_status
  */
static int32_t DPHY_ADC_Probing_Procedure (void)
{
    uint32_t lp_count = 0;

    set_tx_dphy_testport_select(DPHY_TESTPORT_SELECT_TX);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_CB_2, 0, 0, 2);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_LANE0_LANE_0, 1, 0 , 1);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_DAC_0, 0, 0 ,1);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_DAC_0, 1, 1 ,1);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_DAC_0, 0, 1 ,1);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_DAC_0, 1, 0 ,1);

    while(! (DPHY_DSI_Read_Mask(dphy4txtester_DIG_RD_TX_DAC_0, 0, 1)))
    {
        if(lp_count++ < 1000000)
        {
            sys_busy_loop_us(1);
        }
        else
        {
            return ARM_DRIVER_ERROR;
        }
    }

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_DAC_0, 0, 0 ,1);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_DAC_0, 0, 1 ,1);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PHY_2_PHY_BIST_Test(void)
  \brief       PHY2PHY High Speed BIST test.
  \return      \ref execution_status
  */
static int32_t PHY_2_PHY_BIST_Test(void)
{
    set_rx_dphy_testport_select(DPHY_TESTPORT_SELECT_TX);

    DPHY_CSI2_Write_Mask(dphy4txtester_DIG_RDWR_TX_SYS_3, 0x1, 3, 1);

    set_tx_dphy_testport_select(DPHY_TESTPORT_SELECT_TX);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_SYS_3, 0x1, 3, 1);

    set_rx_dphy_testport_select(DPHY_TESTPORT_SELECT_RX);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_LANE0_LANE_0, 0x1, 7, 1);

    DPHY_DSI_Write_Mask(dphy4txtester_DIG_RDWR_TX_LANE1_LANE_0, 0x1, 7, 1);

    set_tx_dphy_testport_select(DPHY_TESTPORT_SELECT_RX);

    DPHY_DSI_Write_Mask(dphy4rxtester_DIG_RDWR_RX_BIST_3, 0XF, 0, 8);

    set_tx_dphy_testport_select(DPHY_TESTPORT_SELECT_TX);

    enable_tx_dphy_bist_on();

    if(get_tx_dphy_bist_ok() == DPHY_BIST_OK_STATUS_NOT_SET)
    {
        return ARM_DRIVER_ERROR;
    }


    set_rx_dphy_testport_select(DPHY_TESTPORT_SELECT_RX);

    DPHY_CSI2_Write_Mask(dphy4rxtester_DIG_RDWR_RX_LANE0_LANE_9, 0x3, 5, 2);

    DPHY_CSI2_Write_Mask(dphy4rxtester_DIG_RDWR_RX_LANE1_LANE_9, 0x3, 5, 2);

    sys_busy_loop_us(10);

    DPHY_CSI2_Write_Mask(dphy4rxtester_DIG_RDWR_RX_LANE0_LANE_9, 0x1, 7, 1);

    DPHY_CSI2_Write_Mask(dphy4rxtester_DIG_RDWR_RX_LANE0_LANE_12, 0x1, 7, 1);

    DPHY_CSI2_Write_Mask(dphy4rxtester_DIG_RDWR_RX_LANE1_LANE_9, 0x1, 7, 1);

    DPHY_CSI2_Write_Mask(dphy4rxtester_DIG_RDWR_RX_LANE1_LANE_12, 0x1, 7, 1);

    if(DPHY_CSI2_Read_Mask(dphy4rxtester_DIG_RD_RX_LANE0_LANE_7, 0, 8) != 0)
    {
        return ARM_DRIVER_ERROR;
    }

    if(DPHY_CSI2_Read_Mask(dphy4rxtester_DIG_RD_RX_LANE0_LANE_8, 0, 8) != 0)
    {
        return ARM_DRIVER_ERROR;
    }

    if(DPHY_CSI2_Read_Mask(dphy4rxtester_DIG_RD_RX_LANE1_LANE_7, 0, 8) != 0)
    {
        return ARM_DRIVER_ERROR;
    }

    if(DPHY_CSI2_Read_Mask(dphy4rxtester_DIG_RD_RX_LANE1_LANE_8, 0, 8) != 0)
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t DPHY_External_Loopback_Test (uint32_t frequency, uint32_t loopback_test_run_time_us)
  \brief       External Loopback test.
  \param[in]   frequency to configure DPHY PLL.
  \param[in]   time in microseconds for which loopback test should run.
  \return      return test status TPASS or TFAIL.
  */
int32_t DPHY_External_Loopback_Test (uint32_t frequency, uint32_t loopback_test_run_time_us)
{
    int32_t  ret = ARM_DRIVER_OK;
    uint32_t lp_count = 0;

    /* Enable DPHY Power */
    DPHY_PowerEnable();

    /* Configuring Master */
    ret = DPHY_MasterSetup(frequency, 2, DPHY_CLK_MODE_NON_CONTINUOUS);
    if(ret != ARM_DRIVER_OK)
    {
        goto error_Power_disable;
    }

    /* Configuring ADC */
    ret = DPHY_ADC_Probing_Procedure();
    if(ret != ARM_DRIVER_OK)
    {
        goto error_Power_disable;
    }

    /* Configuring Slave*/
    ret = DPHY_SlaveSetup(frequency, 2, DPHY_CLK_MODE_NON_CONTINUOUS);
    if(ret != ARM_DRIVER_OK)
    {
        goto error_Power_disable;
    }

    /*Run PHY2PHY High speed BIST test*/
    ret = PHY_2_PHY_BIST_Test();
    if(ret != ARM_DRIVER_OK)
    {
        goto error_Power_disable;
    }

    while(lp_count++ < loopback_test_run_time_us)
    {
        sys_busy_loop_us(1);
    }

error_Power_disable:
    /* Disable DPHY Power */
    DPHY_PowerDisable();

    /*Test Failed*/
    if(ret != ARM_DRIVER_OK)
    {
        return TFAIL;
    }

    /*Test Passed*/
    return TPASS;
}

#endif
