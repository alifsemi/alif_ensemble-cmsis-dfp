/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     Driver_IO_Private.h
 * @author   Girish BN, Manoj A Murudi
 * @email    girish.bn@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     29-March-2023
 * @brief    Alif Header file for GPIO.
 * @bug      None.
 * @Note	 None
 ******************************************************************************/

#ifndef DRIVER_IO_PRIVATE_H_
#define DRIVER_IO_PRIVATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_IO.h"
#include "gpio.h"

#if SOC_FEAT_LPGPIO_HAS_PIN2_7
#define LPGPIO_MAX_PINS 8
#else
#define LPGPIO_MAX_PINS 2
#endif

#define GPIO_PORT_MAX_PIN_NUMBER      0x8U /* Number of pins in each port */

#define ARM_GPIO_BIT_IRQ_POLARITY_Pos 0U  ///< bits - 0
#define ARM_GPIO_BIT_IRQ_POLARITY_Msk (1U << ARM_GPIO_BIT_IRQ_POLARITY_Pos)
#define ARM_GPIO_BIT_IRQ_POLARITY(x)                                                               \
    (((x) & ARM_GPIO_BIT_IRQ_POLARITY_Msk) >> ARM_GPIO_BIT_IRQ_POLARITY_Pos)

#define ARM_GPIO_BIT_IRQ_BOTH_EDGE_Pos 1U  ///< bits - 1
#define ARM_GPIO_BIT_IRQ_BOTH_EDGE_Msk (1U << ARM_GPIO_BIT_IRQ_BOTH_EDGE_Pos)
#define ARM_GPIO_BIT_IRQ_BOTH_EDGE(x)                                                              \
    (((x) & ARM_GPIO_BIT_IRQ_BOTH_EDGE_Msk) >> ARM_GPIO_BIT_IRQ_BOTH_EDGE_Pos)

#define ARM_GPIO_BIT_IRQ_SENSITIVE_Pos 2U  ///< bits - 2
#define ARM_GPIO_BIT_IRQ_SENSITIVE_Msk (1U << ARM_GPIO_BIT_IRQ_SENSITIVE_Pos)
#define ARM_GPIO_BIT_IRQ_SENSITIVE(x)                                                              \
    (((x) & ARM_GPIO_BIT_IRQ_SENSITIVE_Msk) >> ARM_GPIO_BIT_IRQ_SENSITIVE_Pos)

/**
 * enum GPIO_INSTANCE.
 * GPIO instances.
 */
typedef enum _GPIO_INSTANCE {
    GPIO0_INSTANCE,
    GPIO1_INSTANCE,
    GPIO2_INSTANCE,
    GPIO3_INSTANCE,
    GPIO4_INSTANCE,
    GPIO5_INSTANCE,
    GPIO6_INSTANCE,
    GPIO7_INSTANCE,
    GPIO8_INSTANCE,
    GPIO9_INSTANCE,
    GPIO10_INSTANCE,
    GPIO11_INSTANCE,
    GPIO12_INSTANCE,
    GPIO13_INSTANCE,
    GPIO14_INSTANCE,
    GPIO16_INSTANCE,
    GPIO17_INSTANCE,
    LPGPIO_INSTANCE
} GPIO_INSTANCE;

/**
 * enum GPIO_CONTROL_MODE.
 * GPIO control mode.
 */
typedef enum _GPIO_CONTROL_MODE {
    GPIO_SOFTWARE_CONTROL_MODE,
    GPIO_HARDWARE_CONTROL_MODE
} GPIO_CONTROL_MODE;

/**
 * enum GPIO_INTERRUPT_TYPE.
 * GPIO interrupt type.
 */
typedef enum _GPIO_INTERRUPT_TYPE {
    GPIO_INTERRUPT_TYPE_INDIVIDUAL,
    GPIO_INTERRUPT_TYPE_COMBINED
} GPIO_INTERRUPT_TYPE;

typedef struct _GPIO_DRV_STATE {
    uint32_t initialized: 1;  /* Driver Initialized */
    uint32_t powered    : 1;  /* Driver powered */
    uint32_t reserved   : 30; /* Reserved */
} GPIO_DRV_STATE;

/**
 * @brief GPIO Resources
 */
typedef struct _GPIO_RESOURCES {
    GPIO_Type             *reg_base;     /**< GPIO PORT Base Address>**/
    IRQn_Type              IRQ_base_num; /**< GPIO PORT IRQ base Num>**/
    uint16_t               db_clkdiv; /**< GPIO PORT debounce clk divisor: only for GPIO 0-14 >**/
    GPIO_DRV_STATE         state[GPIO_PORT_MAX_PIN_NUMBER];        /**< GPIO PORT status flag >**/
    GPIO_INTERRUPT_TYPE    IRQ_type;                               /**< GPIO IRQ type         >**/
    uint8_t                IRQ_priority[GPIO_PORT_MAX_PIN_NUMBER]; /**< GPIO PIN IRQ priority >**/
    uint8_t                max_pin;                                /**< GPIO MAX PIN Numbers >**/
    uint8_t                ref_count;       /**< GPIO PORT reference count >**/
    GPIO_INSTANCE          gpio_id;         /**< GPIO instance >*/
    GPIO_CONTROL_MODE      control_mode;    /**< GPIO control mode >*/
    bool                   gpio_bit_man_en; /**< GPIO PORT Bit Manipulation feature >*/
    ARM_GPIO_SignalEvent_t cb_event[GPIO_PORT_MAX_PIN_NUMBER]; /**< GPIO Call back function >*/
} GPIO_RESOURCES;

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_IO_PRIVATE_H_ */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
