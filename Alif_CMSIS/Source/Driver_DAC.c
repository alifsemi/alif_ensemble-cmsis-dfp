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
 * @file     Driver_DAC.c
 * @author   Nisarga A M
 * @email    nisarga.am@alifsemi.com
 * @version  V1.0.0
 * @date     22-Feb-2022
 * @brief    DAC(Digital to Analog Converter) driver definitions.DAC0 connected
 *           to P2_2 and DAC1 connected to P2_3.
 ******************************************************************************/

/* Project Includes */
#include <Driver_DAC_Private.h>
#include "analog_config.h"

#if !(RTE_DAC0 || RTE_DAC1)
#error "DAC is not enabled in the RTE_device.h"
#endif

#if (defined(RTE_Drivers_DAC0) && !RTE_DAC0)
#error "DAC0 not configured in RTE_Device.h!"
#endif

#if (defined(RTE_Drivers_DAC1) && !RTE_DAC1)
#error "DAC1 not configured in RTE_Device.h!"
#endif

#define ARM_DAC_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /*  Driver version */

/*Driver version*/
static const ARM_DRIVER_VERSION DriverVersion = {
        ARM_DAC_API_VERSION,
        ARM_DAC_DRV_VERSION
};

/*Driver Capabilities   */
static const ARM_DAC_CAPABILITIES DriverCapabilities = {
    1,/* 12 bit DAC resolution */
    0 /* Reserved ( must be ZERO) */
};

/**
 @fn           void analog_config(void)
 @brief        Analog configuration register includes Vbat and comparator
 @param[in]    none
 @return       none
 */
static void Analog_Config(void)
{
    /* Analog configuration Vbat register2 */
    analog_config_vbat_reg2();

    /* Analog configuration comparator register2 */
    analog_config_cmp_reg2();
}

/**
 @fn           ARM_DRIVER_VERSION DAC_GetVersion(void)
 @brief        get DAC version
 @param        none
 @return       driver version
 */
static ARM_DRIVER_VERSION DAC_GetVersion(void)
{
    return DriverVersion;
}

/**
 @fn           ARM_RTC_CAPABILITIES DAC_GetCapabilities(void)
 @brief        get DAC Capabilites
 @param        none
 @return       driver Capabilites
 */
static ARM_DAC_CAPABILITIES DAC_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 @fn           int32_t DAC_Initialize(DAC_RESOURCES *DAC)
 @brief        Initialize the DAC
 @param[in]    DAC : Pointer to DAC resources
 @return       ARM_DRIVER_ERROR_PARAMETER : if dac parameter is invalid
               ARM_DRIVER_OK              : if dac successfully initialized
 */
static int32_t DAC_Initialize(DAC_RESOURCES *DAC)
{
    int32_t ret = ARM_DRIVER_OK;

    /* Setting the flag */
    DAC->flags |= DAC_FLAG_DRV_INIT_DONE;

    return ret;
}

/**
 @fn           int32_t DAC_Uninitialize(DAC_RESOURCES *DAC)
 @brief        Un-Initialize the DAC
 @param[in]    DAC  : Pointer to DAC resources
 @return       ARM_DRIVER_OK : if dac successfully initialized
 */
static int32_t DAC_Uninitialize(DAC_RESOURCES *DAC)
{
      int32_t ret = ARM_DRIVER_OK;

      /* Reset the flag */
      DAC->flags = 0U;

     return ret;
}

/**
 @fn           int32_t DAC_PowerControl(ARM_POWER_STATE state,
                                        DAC_RESOURCES *DAC)
 @brief        CMSIS-DRIVER DAC power control
 @param[in]    state : Power state
 @param[in]    DAC   : Pointer to DAC resources
 @return       ARM_DRIVER_ERROR_PARAMETER  : if initialize is not done
               ARM_DRIVER_OK               : if power done successful
 */
static int32_t DAC_PowerControl(ARM_POWER_STATE state,
                                DAC_RESOURCES *DAC)
{
    switch(state)
    {
          case ARM_POWER_OFF:

               /* Reset the power flag */
               DAC->flags &= ~(DAC_FLAG_DRV_POWER_DONE);

               /* Clear the DAC configuration */
               dac_clear_config(DAC->regs);

               disable_dac_periph_clk(DAC->instance);

               disable_adc_cmp_periph_clk();

               break;

          case ARM_POWER_FULL:
               if(!(DAC->flags & DAC_FLAG_DRV_INIT_DONE))
               {
                    /* error:Driver is not initialized */
                    return ARM_DRIVER_ERROR;
               }

               if((DAC->flags & DAC_FLAG_DRV_POWER_DONE))
               {
                    return ARM_DRIVER_OK;
               }

               /* Set the power flag enabled */
               DAC->flags |= DAC_FLAG_DRV_POWER_DONE;

               enable_adc_cmp_periph_clk();

               enable_dac_periph_clk(DAC->instance);

               /* Initialization for Analog configuration register */
               Analog_Config();

               /* DAC Disable */
               dac_disable(DAC->regs);

               /* DAC reset released */
               dac_reset_deassert(DAC->regs);

               /* Initialize DAC configuration */
               dac_set_config(DAC->regs, DAC->input_mux_val, DAC->bypass_val);

              break;

          case ARM_POWER_LOW:
          default:
              return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
 @fn           int32_t DAC_Control(DAC_RESOURCES *DAC, uint32_t control, uint32_t arg)
 @brief        CMSIS-Driver dac control.
               Control DAC Interface.
 @param[in]    DAC     : Pointer to DAC resources
 @param[in]    control : Operation \ref Driver_DAC.h : DAC control codes
 @param[in]    arg     : Argument of operation (optional)
 @return       ARM_DRIVER_ERROR_PARAMETER  : if dac parameter is invalid
               ARM_DRIVER_OK               : if dac successfully initialized
 */
static int32_t DAC_Control(DAC_RESOURCES *DAC, uint32_t control, uint32_t arg)
{
    ARG_UNUSED(arg);
    int32_t ret = ARM_DRIVER_OK;

    switch (control)
    {
        case ARM_DAC_RESET:

            /* DAC reset asserted */
            dac_reset_assert(DAC->regs);
            break;

        default:
            ret = ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
    }
    return ret;
}

/**
 * @fn         DAC_Start (DAC_RESOURCES *DAC)
 * @brief      CMSIS-Driver DAC Start
 *             Enable the DAC.
 @param[in]    DAC  : Pointer to DAC resources
 @return       ARM_DRIVER_OK  : if dac successfully uninitialized or already not initialized
 */
static int32_t DAC_Start (DAC_RESOURCES *DAC)
{
    int32_t ret = ARM_DRIVER_OK;

    if(!(DAC->flags & DAC_FLAG_DRV_POWER_DONE))
    {
         /* error:Driver is not initialized */
         return ARM_DRIVER_ERROR;
    }

    if((DAC->flags & DAC_FLAG_DRV_STARTED))
    {
         return ARM_DRIVER_OK;
    }

    DAC->flags |= DAC_FLAG_DRV_STARTED;

    /* Enable the DAC */
    dac_enable(DAC->regs);

    return ret;
}

/**
 *@fn          DAC_Stop (DAC_RESOURCES *DAC)
 *@brief       CMSIS-Driver DAC Stop
               Disable the DAC
 *@param[in]   DAC     : Pointer to DAC resources
 *@return      ARM_DRIVER_OK : if function return successfully
 */
static int32_t DAC_Stop (DAC_RESOURCES *DAC)
{
    int32_t ret = ARM_DRIVER_OK;

    /* Disable the DAC */
    dac_disable(DAC->regs);

    DAC->flags &= ~(DAC_FLAG_DRV_STARTED);

    return ret;
}

/**
 @fn           DAC_SetInput(DAC_RESOURCES *DAC, uint32_t value)
 @brief        CMSIS-Driver to set the DAC input.
 @param[in]    Input : Operation
 @param[in]    value  : DAC input
 @param[in]    DAC  : Pointer to dac resources
 @return       ARM_DRIVER_ERROR_PARAMETER : if parameter are invalid
               ARM_DRIVER_OK              : if the function return successful
 */
static int32_t DAC_SetInput(DAC_RESOURCES *DAC, uint32_t value)
{
    int32_t ret = ARM_DRIVER_OK;

    if(!(DAC->flags & DAC_FLAG_DRV_STARTED))
    {
         /* error:Driver is not started */
         return ARM_DRIVER_ERROR;
    }
    /* error if input is out of the range */
    if(value > DAC_MAX_INPUT)
    {
        ret = ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Set dac input */
    dac_input(DAC->regs, value);

    return ret;
}

/* DAC0 driver instance */
#if(RTE_DAC0)

/* DAC configuration */
static DAC_RESOURCES DAC0 = {
        .regs           = (DAC120_Type *)DAC120_BASE,
        .flags          = 0,
        .input_mux_val  = (RTE_DAC1_INPUT_BYP_MUX_EN),
        .bypass_val     = (RTE_DAC1_BYP_VAL),
        .instance       = DAC_INSTANCE_0
};

/* Function Name: DAC0_Initialize */
static int32_t DAC0_Initialize(void)
{
    return (DAC_Initialize(&DAC0));
}

/* Function Name: DAC0_uninitialize */
static int32_t DAC0_Uninitialize(void)
{
    return (DAC_Uninitialize(&DAC0));
}

/* Function Name: DAC0_PowerControl */
static int32_t DAC0_PowerControl(ARM_POWER_STATE state)
{
  return (DAC_PowerControl(state, &DAC0));
}

/* Function Name: DAC0_Control */
static int32_t DAC0_Control(uint32_t control, uint32_t arg)
{
    return (DAC_Control(&DAC0, control, arg));
}

/* Function Name: DAC0_Start */
static int32_t DAC0_Start(void)
{
    return (DAC_Start(&DAC0));
}

/* Function Name: DAC0_Stop */
static int32_t DAC0_Stop(void)
{
    return (DAC_Stop(&DAC0));
}

/* Function Name: DAC0_SetInput */
static int32_t DAC0_SetInput(uint32_t value)
{
    return (DAC_SetInput(&DAC0, value));
}

extern ARM_DRIVER_DAC Driver_DAC0;
ARM_DRIVER_DAC Driver_DAC0 =
{
        DAC_GetVersion,
        DAC_GetCapabilities,
        DAC0_Initialize,
        DAC0_Uninitialize,
        DAC0_PowerControl,
        DAC0_Control,
        DAC0_Start,
        DAC0_Stop,
        DAC0_SetInput
};

#endif /*RTE_DAC0 */

/* DAC1 driver instance */
#if(RTE_DAC1)

/* DAC1 configuration */
static DAC_RESOURCES DAC1 = {
        .regs           = (DAC120_Type *)DAC121_BASE,
        .flags          = 0,
        .input_mux_val  = (RTE_DAC1_INPUT_BYP_MUX_EN),
        .bypass_val     = (RTE_DAC1_BYP_VAL),
        .instance       = DAC_INSTANCE_1
};

/* Function Name: DAC1_Initialize */
static int32_t DAC1_Initialize(void)
{
    return (DAC_Initialize(&DAC1));
}

/* Function Name: DAC1_uninitialize */
static int32_t DAC1_Uninitialize(void)
{
    return (DAC_Uninitialize(&DAC1));
}

/* Function Name: DAC1_PowerControl */
static int32_t DAC1_PowerControl(ARM_POWER_STATE state)
{
  return (DAC_PowerControl(state, &DAC1));
}

/* Function Name: DAC1_Control */
static int32_t DAC1_Control(uint32_t control, uint32_t arg)
{
    return (DAC_Control(&DAC1, control, arg));
}

/* Function Name: DAC1_Start */
static int32_t DAC1_Start(void)
{
    return (DAC_Start(&DAC1));
}

/* Function Name: DAC1_Stop */
static int32_t DAC1_Stop(void)
{
    return (DAC_Stop(&DAC1));
}

/* Function Name: DAC1_SetInput */
static int32_t DAC1_SetInput(uint32_t value)
{
    return (DAC_SetInput(&DAC1, value));
}

extern ARM_DRIVER_DAC Driver_DAC1;
ARM_DRIVER_DAC Driver_DAC1 =
{
        DAC_GetVersion,
        DAC_GetCapabilities,
        DAC1_Initialize,
        DAC1_Uninitialize,
        DAC1_PowerControl,
        DAC1_Control,
        DAC1_Start,
        DAC1_Stop,
        DAC1_SetInput
};

#endif /* RTE_DAC1 */
