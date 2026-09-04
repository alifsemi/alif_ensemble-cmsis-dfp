/**
 * @file services_host_clocks.c
 *
 * @brief Clocks services service source file
 * @ingroup host_services
 * @ingroup services-host-clocks
 * @par
 *
 * Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 *  I N C L U D E   F I L E S
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "services_lib_api.h"
#include "services_lib_protocol.h"
#include "services_lib_ids.h"

/**
 * @fn   uint32_t SERVICES_clocks_select_osc_source(uint32_t services_handle,
 *                                                  oscillator_source_t source,
 *                                                  oscillator_target_t target,
 *                                                  uint32_t * error_code)
 * @brief Select RC or XTAL as Oscillator clock source
 * @param services_handle
 * @param source            RC or XTAL
 * @param target            SYSCLK (HF), PERIPHCLK (HF), S32K (LF)
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_clocks_select_osc_source(uint32_t services_handle, oscillator_source_t source,
                                           oscillator_target_t target, uint32_t *error_code)
{
    clk_select_clock_source_svc_t *p_svc =
        (clk_select_clock_source_svc_t *) SERVICES_prepare_packet_buffer(
            sizeof(clk_select_clock_source_svc_t));

    p_svc->send_clock_source = source;
    p_svc->send_clock_target = target;

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_CLOCK_SELECT_OSC_SOURCE, DEFAULT_TIMEOUT);

    *error_code = p_svc->resp_error_code;
    return ret;
}

/**
 * @fn   uint32_t SERVICES_clocks_select_pll_source(uint32_t services_handle,
 *                                                  oscillator_source_t source,
 *                                                  oscillator_target_t target,
 *                                                  uint32_t * error_code)
 * @brief Select Oscillator or PLL clock source for various target clocks
 * @param services_handle
 * @param source            Oscillator or PLL
 * @param target            SYSREFCLK, SYSCLK, ExtSus0, ExtSys1
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_clocks_select_pll_source(uint32_t services_handle, pll_source_t source,
                                           pll_target_t target, uint32_t *error_code)
{
    clk_select_clock_source_svc_t *p_svc =
        (clk_select_clock_source_svc_t *) SERVICES_prepare_packet_buffer(
            sizeof(clk_select_clock_source_svc_t));

    p_svc->send_clock_source = source;
    p_svc->send_clock_target = target;

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_CLOCK_SELECT_PLL_SOURCE, DEFAULT_TIMEOUT);

    *error_code = p_svc->resp_error_code;
    return ret;
}

/**
 * @fn   uint32_t SERVICES_clocks_enable_clock(uint32_t services_handle,
 *                                             clock_enable_t clock,
 *                                             bool enable,
 *                                             uint32_t * error_code)
 * @brief Select Oscialltor or PLL clock source for various target clocks
 * @param services_handle
 * @param clock             Clock to enable or disable
 * @param enable            Enable/Disable flag
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_clocks_enable_clock(uint32_t services_handle, clock_enable_t clock, bool enable,
                                      uint32_t *error_code)
{
    clk_set_enable_svc_t *p_svc =
        (clk_set_enable_svc_t *) SERVICES_prepare_packet_buffer(sizeof(clk_set_enable_svc_t));

    p_svc->send_clock_type = clock;
    p_svc->send_enable     = enable;

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_CLOCK_SET_ENABLE, DEFAULT_TIMEOUT);

    *error_code = p_svc->resp_error_code;
    return ret;
}

/**
 * @fn  uint32_t SERVICES_clocks_set_ES0_frequency(uint32_t services_handle,
 *                                                 clock_frequency_t frequency,
 *                                                 uint32_t * error_code)
 * @brief Set the clock frequency for External System 0 (M55-HP)
 * @param services_handle
 * @param frequency         Clock frequency
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_clocks_set_ES0_frequency(uint32_t services_handle, clock_frequency_t frequency,
                                           uint32_t *error_code)
{
    clk_m55_set_frequency_svc_t *p_svc =
        (clk_m55_set_frequency_svc_t *) SERVICES_prepare_packet_buffer(
            sizeof(clk_m55_set_frequency_svc_t));

    p_svc->send_frequency = frequency;

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_CLOCK_ES0_SET_FREQ, DEFAULT_TIMEOUT);

    *error_code = p_svc->resp_error_code;
    return ret;
}

/**
 * @fn  uint32_t SERVICES_clocks_set_ES1_frequency(uint32_t services_handle,
 *                                                 clock_frequency_t frequency,
 *                                                 uint32_t * error_code)
 * @brief Set the clock frequency for External System 1 (M55-HE)
 * @param services_handle
 * @param frequency         Clock frequency
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_clocks_set_ES1_frequency(uint32_t services_handle, clock_frequency_t frequency,
                                           uint32_t *error_code)
{
    clk_m55_set_frequency_svc_t *p_svc =
        (clk_m55_set_frequency_svc_t *) SERVICES_prepare_packet_buffer(
            sizeof(clk_m55_set_frequency_svc_t));

    p_svc->send_frequency = frequency;

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_CLOCK_ES1_SET_FREQ, DEFAULT_TIMEOUT);

    *error_code = p_svc->resp_error_code;

    return ret;
}

/**
 * @fn  uint32_t SERVICES_clocks_select_a32_source(uint32_t services_handle,
 *                                                 a32_source_t source,
 *                                                 uint32_t * error_code)
 * @brief Select a source clock for the A32 cores
 * @param services_handle
 * @param source            Clock source
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_clocks_select_a32_source(uint32_t services_handle, a32_source_t source,
                                           uint32_t *error_code)
{
    clk_select_sys_clk_source_svc_t *p_svc =
        (clk_select_sys_clk_source_svc_t *) SERVICES_prepare_packet_buffer(
            sizeof(clk_select_sys_clk_source_svc_t));

    p_svc->send_source = source;

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_CLOCK_SELECT_A32_SOURCE, DEFAULT_TIMEOUT);

    *error_code = p_svc->resp_error_code;

    return ret;
}

/**
 * @fn  uint32_t SERVICES_clocks_select_aclk_source(uint32_t services_handle,
 *                                                  aclk_source_t source,
 *                                                  uint32_t * error_code)
 * @brief Select a source clock for system buses (AXI, AHB, APB)
 * @param services_handle
 * @param source            Clock source
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_clocks_select_aclk_source(uint32_t services_handle, aclk_source_t source,
                                            uint32_t *error_code)
{
    clk_select_sys_clk_source_svc_t *p_svc =
        (clk_select_sys_clk_source_svc_t *) SERVICES_prepare_packet_buffer(
            sizeof(clk_select_sys_clk_source_svc_t));

    p_svc->send_source = source;

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_CLOCK_SELECT_ACLK_SOURCE, DEFAULT_TIMEOUT);

    *error_code = p_svc->resp_error_code;

    return ret;
}

/**
 * @fn  uint32_t SERVICES_clocks_set_divider(uint32_t services_handle,
 *                                           clock_divider_t divider,
 *                                           uint32_t value,
 *                                           uint32_t * error_code)
 * @brief Set the value of a divider
 * @param services_handle
 * @param divider           Which divider
 * @param value             Divider value
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_clocks_set_divider(uint32_t services_handle, clock_divider_t divider,
                                     uint32_t value, uint32_t *error_code)
{
    clk_set_clk_divider_svc_t *p_svc = (clk_set_clk_divider_svc_t *) SERVICES_prepare_packet_buffer(
        sizeof(clk_set_clk_divider_svc_t));

    p_svc->send_divider = divider;
    p_svc->send_value   = value;

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_CLOCK_SET_DIVIDER, DEFAULT_TIMEOUT);

    *error_code = p_svc->resp_error_code;

    return ret;
}

/**
 * @fn  uint32_t SERVICES_pll_initialize(uint32_t services_handle,
 *                                       uint32_t * error_code)
 * @brief High-level PLL initialize - startup HF XTAL and PLL, and switch to PLL
 * @param services_handle
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_pll_initialize(uint32_t services_handle, uint32_t *error_code)
{
    generic_svc_t *p_svc = (generic_svc_t *) SERVICES_prepare_packet_buffer(sizeof(generic_svc_t));

    uint32_t ret = SERVICES_send_request(services_handle, SERVICE_PLL_INITIALIZE, DEFAULT_TIMEOUT);

    *error_code  = p_svc->resp_error_code;

    return ret;
}

/**
 * @fn  uint32_t SERVICES_pll_deinit(uint32_t services_handle,
 *                                   uint32_t * error_code)
 * @brief High-level PLL deinit - switch to RC clocks, stop PLL and HF XTAL
 * @param services_handle
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_pll_deinit(uint32_t services_handle, uint32_t *error_code)
{
    generic_svc_t *p_svc = (generic_svc_t *) SERVICES_prepare_packet_buffer(sizeof(generic_svc_t));

    uint32_t ret = SERVICES_send_request(services_handle, SERVICE_PLL_DEINIT, DEFAULT_TIMEOUT);

    *error_code  = p_svc->resp_error_code;
    return ret;
}

/**
 * @fn  uint32_t SERVICES_pll_xtal_start(uint32_t services_handle,
 *                                       bool faststart,
 *                                       bool boost,
 *                                       uint32_t delay_count,
 *                                       uint32_t * error_code)
 * @brief Start the HF XTAL
 * @param services_handle
 * @param faststart         Enable faststart mode
 * @param boost             Enable boost mode
 * @param delay_count       Wait time
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_pll_xtal_start(uint32_t services_handle, bool faststart, bool boost,
                                 uint32_t delay_count, uint32_t *error_code)
{
    pll_xtal_start_svc_t *p_svc =
        (pll_xtal_start_svc_t *) SERVICES_prepare_packet_buffer(sizeof(pll_xtal_start_svc_t));

    p_svc->send_faststart   = faststart;
    p_svc->send_boost       = boost;
    p_svc->send_delay_count = delay_count;

    uint32_t ret = SERVICES_send_request(services_handle, SERVICE_PLL_XTAL_START, DEFAULT_TIMEOUT);

    *error_code  = p_svc->resp_error_code;
    return ret;
}

/**
 * @fn  uint32_t SERVICES_pll_xtal_stop(uint32_t services_handle,
 *                                      uint32_t * error_code)
 * @brief Stop the HF XTAL
 * @param services_handle
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_pll_xtal_stop(uint32_t services_handle, uint32_t *error_code)
{
    generic_svc_t *p_svc = (generic_svc_t *) SERVICES_prepare_packet_buffer(sizeof(generic_svc_t));

    uint32_t ret = SERVICES_send_request(services_handle, SERVICE_PLL_XTAL_STOP, DEFAULT_TIMEOUT);

    *error_code  = p_svc->resp_error_code;
    return ret;
}

/**
 * @fn  uint32_t SERVICES_pll_xtal_is_started(uint32_t services_handle,
 *                                            bool * is_started,
 *                                            uint32_t * error_code)
 * @brief Check if the HF XTAL is started
 * @param services_handle
 * @param is_started
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_pll_xtal_is_started(uint32_t services_handle, bool *is_started,
                                      uint32_t *error_code)
{
    generic_svc_t *p_svc = (generic_svc_t *) SERVICES_prepare_packet_buffer(sizeof(generic_svc_t));

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_PLL_XTAL_IS_STARTED, DEFAULT_TIMEOUT);

    *is_started = p_svc->resp_error_code != 0x0;
    *error_code = p_svc->resp_error_code;
    return ret;
}

/**
 * @fn  uint32_t SERVICES_pll_clkpll_start(uint32_t services_handle,
 *                                         bool faststart,
 *                                         uint32_t delay_count,
 *                                         uint32_t * error_code)
 * @brief Start the PLL
 * @param services_handle
 * @param faststart         Enable faststart mode
 * @param delay_count       Wait time
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_pll_clkpll_start(uint32_t services_handle, bool faststart, uint32_t delay_count,
                                   uint32_t *error_code)
{
    pll_clkpll_start_svc_t *p_svc =
        (pll_clkpll_start_svc_t *) SERVICES_prepare_packet_buffer(sizeof(pll_clkpll_start_svc_t));

    p_svc->send_faststart   = faststart;
    p_svc->send_delay_count = delay_count;

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_PLL_CLKPLL_START, DEFAULT_TIMEOUT);

    *error_code = p_svc->resp_error_code;
    return ret;
}

/**
 * @fn  uint32_t SERVICES_pll_clkpll_stop(uint32_t services_handle,
 *                                        uint32_t * error_code)
 * @brief Stop the PLL
 * @param services_handle
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_pll_clkpll_stop(uint32_t services_handle, uint32_t *error_code)
{
    generic_svc_t *p_svc = (generic_svc_t *) SERVICES_prepare_packet_buffer(sizeof(generic_svc_t));

    uint32_t ret = SERVICES_send_request(services_handle, SERVICE_PLL_CLKPLL_STOP, DEFAULT_TIMEOUT);

    *error_code  = p_svc->resp_error_code;
    return ret;
}

/**
 * @fn  uint32_t SERVICES_pll_clkpll_is_locked(uint32_t services_handle,
 *                                             bool * is_locked,
 *                                             uint32_t * error_code)
 * @brief Check if the PLL is started and locked
 * @param services_handle
 * @param is_locked         Which divider
 * @param error_code        Service error code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_pll_clkpll_is_locked(uint32_t services_handle, bool *is_locked,
                                       uint32_t *error_code)
{
    generic_svc_t *p_svc = (generic_svc_t *) SERVICES_prepare_packet_buffer(sizeof(generic_svc_t));

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_PLL_CLKPLL_IS_LOCKED, DEFAULT_TIMEOUT);

    *is_locked  = p_svc->resp_error_code != 0x0;
    *error_code = p_svc->resp_error_code;
    return ret;
}

/**
 * @fn uint32_t SERVICES_clocks_setting_get(uint32_t, clock_setting_t, uint32_t*, uint32_t*)
 * @brief                   Get a clock setting from the 'clock_setting_t' enumeration
 * @param services_handle
 * @param setting_type
 * @param value
 * @param error_code
 * @return                  Transport layer error code
 * @ingroup services-host-clocks
 */
uint32_t SERVICES_clocks_setting_get(uint32_t services_handle, clock_setting_t setting_type,
                                     uint32_t *value, uint32_t *error_code)
{
    clock_setting_svc_t *p_svc =
        (clock_setting_svc_t *) SERVICES_prepare_packet_buffer(sizeof(clock_setting_svc_t));

    p_svc->send_setting_type = setting_type;

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_CLOCK_SETTING_GET_REQ_ID, DEFAULT_TIMEOUT);

    if (SERVICES_REQ_SUCCESS == ret) {
        *value = p_svc->value;
    }

    *error_code = p_svc->resp_error_code;
    return ret;
}

/**
 * @brief
 * @param services_handle
 * @param clk_settings
 * @param error_code
 * @return
 */
uint32_t SERVICES_clocks_get_data(uint32_t services_handle,
				  clock_get_t *clk_settings,
				  uint32_t *error_code)
{
	clk_get_clocks_svc_t *p_svc =
		(clk_get_clocks_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(clk_get_clocks_svc_t));

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_CLOCK_GET_CLOCKS, DEFAULT_TIMEOUT);

  clk_settings->status = p_svc->status;
  clk_settings->se_frequency_mhz = p_svc->se_frequency_mhz;
  clk_settings->cgu_osc_ctrl = p_svc->cgu_osc_ctrl;
  clk_settings->cgu_pll_sel = p_svc->cgu_pll_sel;
  clk_settings->cgu_clk_ena = p_svc->cgu_clk_ena;
  clk_settings->cgu_escclk_sel = p_svc->cgu_escclk_sel;
  clk_settings->systop_clk_div = p_svc->systop_clk_div;
  clk_settings->hostcpuclk_ctrl = p_svc->hostcpuclk_ctrl;
  clk_settings->hostcpuclk_div0 = p_svc->hostcpuclk_div0;
  clk_settings->hostcpuclk_div1 = p_svc->hostcpuclk_div1;
  clk_settings->aclk_ctrl = p_svc->aclk_ctrl;
  clk_settings->aclk_div0 = p_svc->aclk_div0;

  /* Extended fields to match returns from ISP */
  clk_settings->cgu_pll_lock_ctrl = p_svc->cgu_pll_lock_ctrl;
  clk_settings->misc_reg1 = p_svc->misc_reg1;
  clk_settings->xo_reg1 = p_svc->xo_reg1;
  clk_settings->pd4_clk_sel = p_svc->pd4_clk_sel;
  clk_settings->pd4_clk_pll = p_svc->pd4_clk_pll;
  clk_settings->misc_ctrl = p_svc->misc_ctrl;
  clk_settings->dcdc_reg1 = p_svc->dcdc_reg1;
  clk_settings->dcdc_reg2 = p_svc->dcdc_reg2;
  clk_settings->vbat_ana_reg1 = p_svc->vbat_ana_reg1;
  clk_settings->vbat_ana_reg2 = p_svc->vbat_ana_reg2;
  clk_settings->lf_oscillator_source = p_svc->lf_oscillator_source;
  clk_settings->lf_frequency_hz = p_svc->lf_frequency_hz;

  *error_code = p_svc->resp_error_code;
  return ret;
}
/**
 * @brief Function to set aclk entry delay and force enable/disable
 *
 * @param services_handle
 * @param aclk_entry_delay
 * @param aclk_force_en
 * @param error_code
 * @return
 */
uint32_t SERVICES_clocks_set_aclk(uint32_t services_handle, uint32_t *aclk_entry_delay,
                                  uint32_t *aclk_force_en, uint32_t *error_code)
{
    set_aclk_svc_t *p_svc =
        (set_aclk_svc_t *) SERVICES_prepare_packet_buffer(sizeof(set_aclk_svc_t));

    p_svc->send_aclk_entry_delay = *aclk_entry_delay;
    p_svc->send_aclk_force_en    = *aclk_force_en;

    uint32_t ret =
        SERVICES_send_request(services_handle, SERVICE_CLOCK_SET_ACLK_REQ_ID, DEFAULT_TIMEOUT);

    *error_code = p_svc->resp_error_code;
    return ret;
}

/**
 * @brief Function to set vbat clk for fast or slow mode
 *
 * Slow mode uses 32k clock for all vbat access. Fast clock uses 10M clock.
 * If fast clock is used RTC access is restricted. if RTC is used vbat clock
 * needs to be changed to slow.
 *
 * @param services_handle
 * @param vbat_fast_clk_en
 * @param error_code
 * @return
 */
uint32_t SERVICES_clocks_set_vbat_clk(uint32_t services_handle,
					      uint32_t *vbat_fast_clk_en,
					      uint32_t *error_code)
{
	set_vbat_clk_svc_t *p_svc =
		(set_vbat_clk_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(set_vbat_clk_svc_t));

  p_svc->send_vbat_fast_clk_en = *vbat_fast_clk_en;

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_CLOCK_SET_VBAT_CLK_REQ_ID, DEFAULT_TIMEOUT);

  *error_code = p_svc->resp_error_code;
  return ret;
}
