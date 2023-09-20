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
 * @file     utimer.h
 * @author   Girish BN, Manoj A Murudi
 * @email    girish.bn@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     02-April-2023
 * @brief    Low Level header file for UTIMER.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef UTIMER_H_
#define UTIMER_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "stdbool.h"

/*****************  Bit definition for TIMER_RegInfo:cntr_start_src_0 register  ******************/
#define CNTR_SRC0_TRIG0_RISING                      ((uint32_t)0x00000001)
#define CNTR_SRC0_TRIG0_FALLING                     ((uint32_t)0x00000002)
#define CNTR_SRC0_TRIG1_RISING                      ((uint32_t)0x00000004)
#define CNTR_SRC0_TRIG1_FALLING                     ((uint32_t)0x00000008)
#define CNTR_SRC0_TRIG2_RISING                      ((uint32_t)0x00000010)
#define CNTR_SRC0_TRIG2_FALLING                     ((uint32_t)0x00000020)
#define CNTR_SRC0_TRIG3_RISING                      ((uint32_t)0x00000040)
#define CNTR_SRC0_TRIG3_FALLING                     ((uint32_t)0x00000080)
#define CNTR_SRC0_TRIG4_RISING                      ((uint32_t)0x00000100)
#define CNTR_SRC0_TRIG4_FALLING                     ((uint32_t)0x00000200)
#define CNTR_SRC0_TRIG5_RISING                      ((uint32_t)0x00000400)
#define CNTR_SRC0_TRIG5_FALLING                     ((uint32_t)0x00000800)
#define CNTR_SRC0_TRIG6_RISING                      ((uint32_t)0x00001000)
#define CNTR_SRC0_TRIG6_FALLING                     ((uint32_t)0x00002000)
#define CNTR_SRC0_TRIG7_RISING                      ((uint32_t)0x00004000)
#define CNTR_SRC0_TRIG7_FALLING                     ((uint32_t)0x00008000)
#define CNTR_SRC0_TRIG8_RISING                      ((uint32_t)0x00010000)
#define CNTR_SRC0_TRIG8_FALLING                     ((uint32_t)0x00020000)
#define CNTR_SRC0_TRIG9_RISING                      ((uint32_t)0x00040000)
#define CNTR_SRC0_TRIG9_FALLING                     ((uint32_t)0x00080000)
#define CNTR_SRC0_TRIG10_RISING                     ((uint32_t)0x00100000)
#define CNTR_SRC0_TRIG10_FALLING                    ((uint32_t)0x00200000)
#define CNTR_SRC0_TRIG11_RISING                     ((uint32_t)0x00400000)
#define CNTR_SRC0_TRIG11_FALLING                    ((uint32_t)0x00800000)
#define CNTR_SRC0_TRIG12_RISING                     ((uint32_t)0x01000000)
#define CNTR_SRC0_TRIG12_FALLING                    ((uint32_t)0x02000000)
#define CNTR_SRC0_TRIG13_RISING                     ((uint32_t)0x04000000)
#define CNTR_SRC0_TRIG13_FALLING                    ((uint32_t)0x08000000)
#define CNTR_SRC0_TRIG14_RISING                     ((uint32_t)0x10000000)
#define CNTR_SRC0_TRIG14_FALLING                    ((uint32_t)0x20000000)
#define CNTR_SRC0_TRIG15_RISING                     ((uint32_t)0x40000000)
#define CNTR_SRC0_TRIG15_FALLING                    ((uint32_t)0x80000000)

/*****************  Bit definition for TIMER_RegInfo:cntr_start_src_1 register  ******************/
#define CNTR_SRC1_DRIVE_A_RISING_B_0                ((uint32_t)0x00000001)
#define CNTR_SRC1_DRIVE_A_RISING_B_1                ((uint32_t)0x00000002)
#define CNTR_SRC1_DRIVE_A_FALLING_B_0               ((uint32_t)0x00000004)
#define CNTR_SRC1_DRIVE_A_FALLING_B_1               ((uint32_t)0x00000008)
#define CNTR_SRC1_DRIVE_B_RISING_A_0                ((uint32_t)0x00000010)
#define CNTR_SRC1_DRIVE_B_RISING_A_1                ((uint32_t)0x00000020)
#define CNTR_SRC1_DRIVE_B_FALLING_A_0               ((uint32_t)0x00000040)
#define CNTR_SRC1_DRIVE_B_FALLING_A_1               ((uint32_t)0x00000080)
#define CNTR_SRC1_PGM_EN                            ((uint32_t)0x80000000)

/*****************  Bit definition for TIMER_RegInfo:cntr_pause_src register  ******************/
#define CNTR_PAUSE_SRC_0_HIGH_EN                    ((uint32_t)0x00000001)  //todo: remove pause triggers 2&3
#define CNTR_PAUSE_SRC_0_LOW_EN                     ((uint32_t)0x00000002)
#define CNTR_PAUSE_SRC_1_HIGH_EN                    ((uint32_t)0x00000004)
#define CNTR_PAUSE_SRC_1_LOW_EN                     ((uint32_t)0x00000008)
#define CNTR_PAUSE_SRC_2_HIGH_EN                    ((uint32_t)0x00000010)
#define CNTR_PAUSE_SRC_2_LOW_EN                     ((uint32_t)0x00000020)
#define CNTR_PAUSE_SRC_3_HIGH_EN                    ((uint32_t)0x00000040)
#define CNTR_PAUSE_SRC_3_LOW_EN                     ((uint32_t)0x00000080)

/*****************  Bit definition for TIMER_RegInfo:cntr_ctrl register  ******************/
#define CNTR_CTRL_EN                                ((uint32_t)0x00000001)
#define CNTR_CTRL_START                             ((uint32_t)0x00000002)
#define CNTR_CTRL_SAWTOOTH_ONE_SHOT                 ((uint32_t)0x00000004)
#define CNTR_CTRL_TRIANGLE_BUF_TROUGH               ((uint32_t)0x00000010)
#define CNTR_CTRL_TRIANGLE_BUF_TROUGH_CREST         ((uint32_t)0x00000014)
#define CNTR_CTRL_TRIANGLE_ONE_SHOT                 ((uint32_t)0x00000018)
#define CNTR_CTRL_DIR_DOWN                          ((uint32_t)0x00000100)

/*****************  Bit definition for TIMER_RegInfo:filter_ctrl register  ******************/
#define CNTR_CTRL_EN                                ((uint32_t)0x00000001)

/*****************  Bit definition for TIMER_RegInfo:compare_ctrl register  ******************/
#define COMPARE_CTRL_DRV_MATCH_0                    (uint32_t) (0x00000001)
#define COMPARE_CTRL_DRV_MATCH_1                    (uint32_t) (0x00000002)
#define COMPARE_CTRL_DRV_MATCH                      (uint32_t) (COMPARE_CTRL_DRV_MATCH_1|COMPARE_CTRL_DRV_MATCH_0)
#define COMPARE_CTRL_DRV_CYCLE_END_0                (uint32_t) (0x00000004)
#define COMPARE_CTRL_DRV_CYCLE_END_1                (uint32_t) (0x00000008)
#define COMPARE_CTRL_DRV_CYCLE_END                  (uint32_t) (COMPARE_CTRL_DRV_CYCLE_END_1|COMPARE_CTRL_DRV_CYCLE_END_0)
#define COMPARE_CTRL_DRV_START_VAL                  (uint32_t) (0x00000010)
#define COMPARE_CTRL_DRV_STOP_VAL                   (uint32_t) (0x00000040)
#define COMPARE_CTRL_DRV_START_STOP_LEVEL           (uint32_t) (0x00000080)
#define COMPARE_CTRL_DRV_DRIVER_EN                  (uint32_t) (0x00000100)
#define COMPARE_CTRL_DRV_DISABLE_VAL                (uint32_t) (0x00000200)
#define COMPARE_CTRL_DRV_COMPARE_EN                 (uint32_t) (0x00000800)
#define COMPARE_CTRL_DRV_COMPARE_TRIG_EN            (uint32_t) (0x00001000)
#define COMPARE_CTRL_DRV_DMA_CLEAR_EN               (uint32_t) (0x00002000)

/*****************  Bit definition for TIMER_RegInfo:buf_op_ctrl register  ******************/
#define BUF_OP_CTRL_CAPTURE_BUF_EN                  ((uint32_t)0x00000001)
#define BUF_OP_CTRL_CNTR_BUF_EN                     ((uint32_t)0x00000002)
#define BUF_OP_CTRL_COMPARE_BUF_EN                  ((uint32_t)0x00000004)
#define BUF_OP_CTRL_CAPTURE_A_BUF_OP_BIT0           ((uint32_t)0x00010000)
#define BUF_OP_CTRL_CAPTURE_A_BUF_OP_BIT1           ((uint32_t)0x00020000)
#define BUF_OP_CTRL_CAPTURE_A_BUF_OP                ((uint32_t)(BUF_OP_CTRL_CAPTURE_A_BUF_OP_BIT0|BUF_OP_CTRL_CAPTURE_A_BUF_OP_BIT1))
#define BUF_OP_CTRL_CAPTURE_B_BUF_OP_BIT0           ((uint32_t)0x00040000)
#define BUF_OP_CTRL_CAPTURE_B_BUF_OP_BIT1           ((uint32_t)0x00080000)
#define BUF_OP_CTRL_CAPTURE_B_BUF_OP                ((uint32_t)(BUF_OP_CTRL_CAPTURE_B_BUF_OP_BIT0|BUF_OP_CTRL_CAPTURE_B_BUF_OP_BIT1))
#define BUF_OP_CTRL_CNTR_BUF_OP_BIT0                ((uint32_t)0x00100000)
#define BUF_OP_CTRL_CNTR_BUF_OP_BIT1                ((uint32_t)0x00200000)
#define BUF_OP_CTRL_CNTR_BUF_OP                     ((uint32_t)(BUF_OP_CTRL_CNTR_BUF_OP_BIT0|BUF_OP_CTRL_CNTR_BUF_OP_BIT1))
#define BUF_OP_CTRL_FORCE_COMPARE_BUF_OP            ((uint32_t)0x00400000)
#define BUF_OP_CTRL_COMPARE_A_BUF_EVENT_BIT0        ((uint32_t)0x01000000)
#define BUF_OP_CTRL_COMPARE_A_BUF_EVENT_BIT1        ((uint32_t)0x02000000)
#define BUF_OP_CTRL_COMPARE_A_BUF_EVENT             ((uint32_t)(BUF_OP_CTRL_COMPARE_A_BUF_EVENT_BIT0|BUF_OP_CTRL_COMPARE_A_BUF_EVENT_BIT1))
#define BUF_OP_CTRL_COMPARE_A_BUF_OP                ((uint32_t)0x04000000)
#define BUF_OP_CTRL_COMPARE_B_BUF_EVENT_BIT0        ((uint32_t)0x10000000)
#define BUF_OP_CTRL_COMPARE_B_BUF_EVENT_BIT1        ((uint32_t)0x20000000)
#define BUF_OP_CTRL_COMPARE_B_BUF_EVENT             ((uint32_t)(BUF_OP_CTRL_COMPARE_B_BUF_EVENT_BIT0|BUF_OP_CTRL_COMPARE_B_BUF_EVENT_BIT1))
#define BUF_OP_CTRL_COMPARE_B_BUF_OP                ((uint32_t)0x40000000)

/*****************  Bit definition for TIMER_RegInfo:chan_status register  ******************/
#define CHAN_STATUS_CAPTURE_A                       ((uint32_t)0x00000001)
#define CHAN_STATUS_CAPTURE_B                       ((uint32_t)0x00000002)
#define CHAN_STATUS_UNDER_FLOW                      ((uint32_t)0x00000040)
#define CHAN_STATUS_OVER_FLOW                       ((uint32_t)0x00000080)
#define CHAN_STATUS_CNTR_RUNNING                    ((uint32_t)0x00004000)
#define CHAN_STATUS_CNTR_DIR                        ((uint32_t)0x00008000)
#define CHAN_STATUS_COMPARE_A_UP                    ((uint32_t)0x00010000)
#define CHAN_STATUS_COMPARE_A_DOWN                  ((uint32_t)0x00020000)
#define CHAN_STATUS_COMPARE_B_UP                    ((uint32_t)0x00040000)
#define CHAN_STATUS_COMPARE_B_DOWN                  ((uint32_t)0x00080000)
#define CHAN_STATUS_DRV_A                           ((uint32_t)0x08000000)
#define CHAN_STATUS_DRV_B                           ((uint32_t)0x10000000)
#define CHAN_STATUS_DRV_A_B_1                       ((uint32_t)0x20000040)
#define CHAN_STATUS_DRV_A_B_0                       ((uint32_t)0x40000080)

/*****************  Bit definition for TIMER_RegInfo:chan_interrupt register  ******************/
#define CHAN_INTERRUPT_CAPTURE_A_MASK               ((uint32_t)0x00000001)
#define CHAN_INTERRUPT_CAPTURE_B_MASK               ((uint32_t)0x00000002)
#define CHAN_INTERRUPT_COMPARE_A_BUF1_MASK          ((uint32_t)0x00000004)
#define CHAN_INTERRUPT_COMPARE_A_BUF2_MASK          ((uint32_t)0x00000008)
#define CHAN_INTERRUPT_COMPARE_B_BUF1_MASK          ((uint32_t)0x00000010)
#define CHAN_INTERRUPT_COMPARE_B_BUF2_MASK          ((uint32_t)0x00000020)
#define CHAN_INTERRUPT_UNDER_FLOW_MASK              ((uint32_t)0x00000040)
#define CHAN_INTERRUPT_OVER_FLOW_MASK               ((uint32_t)0x00000080)


/*****************  Bit definition for TIMER_RegInfo:duty_cycle_ctrl register  ******************/
#define DUTY_CYCLE_CTRL_DC_ENABLE_A                 ((uint32_t)0x00000001)
#define DUTY_CYCLE_CTRL_DC_FORCE_A                  ((uint32_t)0x00000002)
#define DUTY_CYCLE_CTRL_DC_SETTING_A                ((uint32_t)0x0000000A)                         //D
#define DUTY_CYCLE_CTRL_DC_OVERFLOW_A               ((uint32_t)0x00000010)
#define DUTY_CYCLE_CTRL_DC_UNDERFLOW_A              ((uint32_t)0x00000020)
#define DUTY_CYCLE_CTRL_DC_ENABLE_B                 ((uint32_t)0x00000100)
#define DUTY_CYCLE_CTRL_DC_FORCE_B                  ((uint32_t)0x00000200)
#define DUTY_CYCLE_CTRL_DC_SETTING_B                ((uint32_t)0x00000A00)                         //D
#define DUTY_CYCLE_CTRL_DC_OVERFLOW_B               ((uint32_t)0x00001000)
#define DUTY_CYCLE_CTRL_DC_UNDERFLOW_B              ((uint32_t)0x00002000)

/*****************  Bit definition for TIMER_RegInfo:dead_time_ctrl register  ******************/
#define DEAD_TIME_CTRL_DT_EN                        ((uint32_t)0x00000001)
#define DEAD_TIME_CTRL_DT_BUF_EN                    ((uint32_t)0x00000002)

/*****************  Bit definition for TIMER_RegInfo:int_cntr_ctrl register  ******************/
#define INT_CNTR_CTRL_INT_CNTR_EN                   ((uint32_t)0x00010000)

/*****************  Bit definition for TIMER_RegInfo:fault_ctrl register  ******************/
#define FAULT_CTRL_FUALT_EN_TRIG0                   ((uint32_t)0x00000001)
#define FAULT_CTRL_FUALT_EN_TRIG1                   ((uint32_t)0x00000002)
#define FAULT_CTRL_FUALT_EN_TRIG2                   ((uint32_t)0x00000004)
#define FAULT_CTRL_FUALT_EN_TRIG3                   ((uint32_t)0x00000008)
#define FAULT_CTRL_FUALT_EN                         ((uint32_t)(FAULT_CTRL_FUALT_EN_A_TRIG0|FAULT_CTRL_FUALT_EN_A_TRIG1| \
                                                                FAULT_CTRL_FUALT_EN_A_TRIG2|FAULT_CTRL_FUALT_EN_A_TRIG3))
#define FAULT_CTRL_FUALT_POL_LOW_TRIG0              ((uint32_t)0x00000010)
#define FAULT_CTRL_FUALT_POL_LOW_TRIG1              ((uint32_t)0x00000020)
#define FAULT_CTRL_FUALT_POL_LOW_TRIG2              ((uint32_t)0x00000040)
#define FAULT_CTRL_FUALT_POL_LOW_TRIG3              ((uint32_t)0x00000080)
#define FAULT_CTRL_FUALT_TYPE                       ((uint32_t)0x00000100)

/*****************  Bit definition for TIMER_RegInfo:glb_cntr_start register  ******************/
#define GLB_CNTR_START                              ((uint32_t)0x0000FFFF)

/*****************  Bit definition for TIMER_RegInfo:glb_cntr_stop register  ******************/
#define GLB_CNTR_STOP                               ((uint32_t)0x0000FFFF)

/*****************  Bit definition for TIMER_RegInfo:glb_cntr_clear register  ******************/
#define GLB_CNTR_CLEAR                              ((uint32_t)0x000007FF)

/*****************  Bit definition for TIMER_RegInfo:glb_cntr_running register  ******************/
#define GLB_CNTR_RUNNING                            ((uint32_t)0x0000FFFF)

/*****************  Bit definition for TIMER_RegInfo:glb_driver_oen register  ******************/
#define GLB_DRIVER_CHAN_A_OEN                       (1U)
#define GLB_DRIVER_CHAN_B_OEN                       (2U)
#define GLB_DRIVER_CHAN_OEN                         (3U)

/*****************  Bit definition for TIMER_RegInfo:glb_clk_en register  ******************/
#define GLB_CLK_EN                                  ((uint32_t)0x0000FFFF)


/**
 * enum UTIMER_TRIGGER_FOR.
 * UTIMER trigger target.
 */
typedef enum _UTIMER_TRIGGER_TARGET {
    UTIMER_TRIGGER_START,                       /**< Trigger target UTIMER Channel counter start >*/
    UTIMER_TRIGGER_STOP,                        /**< Trigger target UTIMER Channel counter stop >*/
    UTIMER_TRIGGER_CLEAR,                       /**< Trigger target UTIMER Channel counter clear >*/
    UTIMER_TRIGGER_UPCOUNT,                     /**< Trigger target UTIMER Channel Up count >*/
    UTIMER_TRIGGER_DOWNCOUNT,                   /**< Trigger target UTIMER Channel Down count >*/
    UTIMER_TRIGGER_CAPTURE_A,                   /**< Trigger target UTIMER Channel to capture counter value in Drive A >*/
    UTIMER_TRIGGER_CAPTURE_B,                   /**< Trigger target UTIMER Channel to capture counter value in Drive B >*/
    UTIMER_TRIGGER_DMA_CLEAR_A,                 /**< Trigger target UTIMER Channel to clear Drive A DMA action >*/
    UTIMER_TRIGGER_DMA_CLEAR_B                  /**< Trigger target UTIMER Channel to clear Drive B DMA action >*/
} UTIMER_TRIGGER_TARGET;

/**
 * enum UTIMER_TRIGGER_SRC.
 * UTIMER external event source type.
 */
typedef enum _UTIMER_TRIGGER_SRC {
    UTIMER_SRC_0,                              /**< global triggers >*/
    UTIMER_SRC_1,                              /**< channel events >*/
    UTIMER_FAULT_TRIGGER,                      /**< fault triggers >*/
    UTIMER_CNTR_PAUSE_TRIGGER                 /**< counter pause triggers >*/
} UTIMER_TRIGGER_SRC;

/**
 * enum UTIMER_COUNTER.
 * UTIMER counters.
 */
typedef enum _UTIMER_COUNTER {
    UTIMER_CNTR,                                /**< UTIMER counter >*/
    UTIMER_CNTR_PTR,                            /**< UTIMER pointer counter >*/
    UTIMER_CNTR_PTR_BUF1,                       /**< UTIMER pointer buffer 1 counter >*/
    UTIMER_CNTR_PTR_BUF2,                       /**< UTIMER pointer buffer 2 counter >*/
    UTIMER_DT_UP,                               /**< UTIMER Dead Time UP >*/
    UTIMER_DT_UP_BUF1,                          /**< UTIMER Dead Time UP buffer 1 >*/
    UTIMER_DT_DOWN,                             /**< UTIMER Dead Time DOWN >*/
    UTIMER_DT_DOWN_BUF1,                        /**< UTIMER Dead Time DOWN buffer 1 >*/
    UTIMER_COMPARE_A,                           /**< UTIMER compare A >*/
    UTIMER_COMPARE_B,                           /**< UTIMER compare B >*/
    UTIMER_COMPARE_A_BUF1,                      /**< UTIMER compare A buffer 1 >*/
    UTIMER_COMPARE_B_BUF1,                      /**< UTIMER compare B buffer 1 >*/
    UTIMER_COMPARE_A_BUF2,                      /**< UTIMER compare A buffer 2 >*/
    UTIMER_COMPARE_B_BUF2,                      /**< UTIMER compare B buffer 1 >*/
    UTIMER_CAPTURE_A,                           /**< UTIMER capture A  >*/
    UTIMER_CAPTURE_B,                           /**< UTIMER capture B >*/
    UTIMER_CAPTURE_A_BUF1,                      /**< UTIMER capture A buffer 1 >*/
    UTIMER_CAPTURE_B_BUF1,                      /**< UTIMER capture B buffer 1 >*/
    UTIMER_CAPTURE_A_BUF2,                      /**< UTIMER capture A buffer 2 >*/
    UTIMER_CAPTURE_B_BUF2,                      /**< UTIMER capture B buffer 2 >*/
} UTIMER_COUNTER;

/**
 * enum UTIMER_MODE.
 * UTIMER modes.
 */
typedef enum _UTIMER_MODE {
    UTIMER_MODE_BASIC,                                  /**< UTIMER Channel mode configure to Basic mode >*/
    UTIMER_MODE_BUFFERING,                              /**< UTIMER Channel mode configure to Buffering mode >*/
    UTIMER_MODE_TRIGGERING,                             /**< UTIMER Channel mode configure to Triggering mode >*/
    UTIMER_MODE_CAPTURING,                              /**< UTIMER Channel mode configure to Capturing mode >*/
    UTIMER_MODE_COMPARING,                              /**< UTIMER Channel mode configure to Comparing mode >*/
    UTIMER_MODE_DEAD_TIME,                              /**< UTIMER Channel mode configure to Dead Timer mode >*/
} UTIMER_MODE;

/**
 * enum UTIMER_COUNTER_TYPE.
 * UTIMER counter type.
 */
typedef enum _UTIMER_COUNTER_DIR {
    UTIMER_COUNTER_UP,                                  /**< UTIMER Channel counter direction up >*/
    UTIMER_COUNTER_DOWN,                                /**< UTIMER Channel counter direction down >*/
    UTIMER_COUNTER_TRIANGLE                             /**< UTIMER Channel counter direction triangle >*/
} UTIMER_COUNTER_DIR;


/** \brief UTIMER trigger configuration. */
typedef struct _UTIMER_TRIGGER_CONFIG {
    UTIMER_TRIGGER_TARGET      trigger_target;            /**< UTIMER trigger target >*/
    UTIMER_TRIGGER_SRC         src_type;                  /**< UTIMER trigger source type >*/
    uint32_t                   trigger_type;              /**< UTIMER trigger type >*/
} UTIMER_TRIGGER_CONFIG ;

/**
  * @brief UTIMER (UTIMER)
  */
typedef struct {                                /*!< (@ 0x48000000) UTIMER Structure                                           */
    volatile uint32_t  UTIMER_GLB_CNTR_START;        /*!< (@ 0x00000000) Channels Global Counter Start Register                     */
    volatile uint32_t  UTIMER_GLB_CNTR_STOP;         /*!< (@ 0x00000004) Channels Global Counter Stop Register                      */
    volatile uint32_t  UTIMER_GLB_CNTR_CLEAR;        /*!< (@ 0x00000008) Channels Global Counter Clear Register                     */
    volatile uint32_t  UTIMER_GLB_CNTR_RUNNING;      /*!< (@ 0x0000000C) Channels Global Counter Running Status Register            */
    volatile uint32_t  UTIMER_GLB_DRIVER_OEN;        /*!< (@ 0x00000010) Channels Driver Output Enable Register                     */
    volatile uint32_t  RESERVED[3];
    volatile uint32_t  UTIMER_GLB_CLOCK_ENABLE;      /*!< (@ 0x00000020) Channels Clock Enable Register                             */
    volatile uint32_t  RESERVED1[1015];
    volatile uint32_t  UTIMER0_START_0_SRC;          /*!< (@ 0x00001000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER0_START_1_SRC;          /*!< (@ 0x00001004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER0_STOP_0_SRC;           /*!< (@ 0x00001008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER0_STOP_1_SRC;           /*!< (@ 0x0000100C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER0_CLEAR_0_SRC;          /*!< (@ 0x00001010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER0_CLEAR_1_SRC;          /*!< (@ 0x00001014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER0_UP_0_SRC;             /*!< (@ 0x00001018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER0_UP_1_SRC;             /*!< (@ 0x0000101C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER0_DOWN_0_SRC;           /*!< (@ 0x00001020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER0_DOWN_1_SRC;           /*!< (@ 0x00001024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER0_TRIG_CAPTURE_SRC_A_0; /*!< (@ 0x00001028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER0_TRIG_CAPTURE_SRC_A_1; /*!< (@ 0x0000102C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER0_TRIG_CAPTURE_SRC_B_0; /*!< (@ 0x00001030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER0_TRIG_CAPTURE_SRC_B_1; /*!< (@ 0x00001034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER0_DMA_CLEAR_SRC_A_0;    /*!< (@ 0x00001038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER0_DMA_CLEAR_SRC_A_1;    /*!< (@ 0x0000103C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER0_DMA_CLEAR_SRC_B_0;    /*!< (@ 0x00001040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER0_DMA_CLEAR_SRC_B_1;    /*!< (@ 0x00001044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER0_CNTR_PAUSE_SRC;       /*!< (@ 0x00001048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED2[13];
    volatile uint32_t  UTIMER0_CNTR_CTRL;            /*!< (@ 0x00001080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER0_FILTER_CTRL_A;        /*!< (@ 0x00001084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER0_FILTER_CTRL_B;        /*!< (@ 0x00001088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER0_COMPARE_CTRL_A;       /*!< (@ 0x0000108C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER0_COMPARE_CTRL_B;       /*!< (@ 0x00001090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER0_BUF_OP_CTRL;          /*!< (@ 0x00001094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED3[2];
    volatile uint32_t  UTIMER0_CNTR;                 /*!< (@ 0x000010A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER0_CNTR_PTR;             /*!< (@ 0x000010A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER0_CNTR_PTR_BUF1;        /*!< (@ 0x000010A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER0_CNTR_PTR_BUF2;        /*!< (@ 0x000010AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER0_CAPTURE_A;            /*!< (@ 0x000010B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER0_CAPTURE_A_BUF1;       /*!< (@ 0x000010B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER0_CAPTURE_A_BUF2;       /*!< (@ 0x000010B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED4;
    volatile uint32_t  UTIMER0_CAPTURE_B;            /*!< (@ 0x000010C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER0_CAPTURE_B_BUF1;       /*!< (@ 0x000010C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER0_CAPTURE_B_BUF2;       /*!< (@ 0x000010C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED5;
    volatile uint32_t  UTIMER0_COMPARE_A;            /*!< (@ 0x000010D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER0_COMPARE_A_BUF1;       /*!< (@ 0x000010D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER0_COMPARE_A_BUF2;       /*!< (@ 0x000010D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED6;
    volatile uint32_t  UTIMER0_COMPARE_B;            /*!< (@ 0x000010E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER0_COMPARE_B_BUF1;       /*!< (@ 0x000010E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER0_COMPARE_B_BUF2;       /*!< (@ 0x000010E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED7;
    volatile uint32_t  UTIMER0_DT_UP;                /*!< (@ 0x000010F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER0_DT_UP_BUF1;           /*!< (@ 0x000010F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER0_DT_DOWN;              /*!< (@ 0x000010F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER0_DT_DOWN_BUF1;         /*!< (@ 0x000010FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED8[5];
    volatile uint32_t  UTIMER0_CHAN_STATUS;          /*!< (@ 0x00001114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER0_CHAN_INTERRUPT;       /*!< (@ 0x00001118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER0_CHAN_INTERRUPT_MASK;  /*!< (@ 0x0000111C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER0_DUTY_CYCLE_CTRL;      /*!< (@ 0x00001120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER0_DEAD_TIME_CTRL;       /*!< (@ 0x00001124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED9[2];
    volatile uint32_t  UTIMER0_INT_CNTR_CTRL;        /*!< (@ 0x00001130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER0_FAULT_CTRL;           /*!< (@ 0x00001134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED10[946];
    volatile uint32_t  UTIMER1_START_0_SRC;          /*!< (@ 0x00002000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER1_START_1_SRC;          /*!< (@ 0x00002004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER1_STOP_0_SRC;           /*!< (@ 0x00002008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER1_STOP_1_SRC;           /*!< (@ 0x0000200C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER1_CLEAR_0_SRC;          /*!< (@ 0x00002010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER1_CLEAR_1_SRC;          /*!< (@ 0x00002014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER1_UP_0_SRC;             /*!< (@ 0x00002018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER1_UP_1_SRC;             /*!< (@ 0x0000201C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER1_DOWN_0_SRC;           /*!< (@ 0x00002020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER1_DOWN_1_SRC;           /*!< (@ 0x00002024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER1_TRIG_CAPTURE_SRC_A_0; /*!< (@ 0x00002028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER1_TRIG_CAPTURE_SRC_A_1; /*!< (@ 0x0000202C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER1_TRIG_CAPTURE_SRC_B_0; /*!< (@ 0x00002030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER1_TRIG_CAPTURE_SRC_B_1; /*!< (@ 0x00002034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER1_DMA_CLEAR_SRC_A_0;    /*!< (@ 0x00002038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER1_DMA_CLEAR_SRC_A_1;    /*!< (@ 0x0000203C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER1_DMA_CLEAR_SRC_B_0;    /*!< (@ 0x00002040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER1_DMA_CLEAR_SRC_B_1;    /*!< (@ 0x00002044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER1_CNTR_PAUSE_SRC;       /*!< (@ 0x00002048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED11[13];
    volatile uint32_t  UTIMER1_CNTR_CTRL;            /*!< (@ 0x00002080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER1_FILTER_CTRL_A;        /*!< (@ 0x00002084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER1_FILTER_CTRL_B;        /*!< (@ 0x00002088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER1_COMPARE_CTRL_A;       /*!< (@ 0x0000208C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER1_COMPARE_CTRL_B;       /*!< (@ 0x00002090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER1_BUF_OP_CTRL;          /*!< (@ 0x00002094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED12[2];
    volatile uint32_t  UTIMER1_CNTR;                 /*!< (@ 0x000020A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER1_CNTR_PTR;             /*!< (@ 0x000020A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER1_CNTR_PTR_BUF1;        /*!< (@ 0x000020A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER1_CNTR_PTR_BUF2;        /*!< (@ 0x000020AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER1_CAPTURE_A;            /*!< (@ 0x000020B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER1_CAPTURE_A_BUF1;       /*!< (@ 0x000020B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER1_CAPTURE_A_BUF2;       /*!< (@ 0x000020B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED13;
    volatile uint32_t  UTIMER1_CAPTURE_B;            /*!< (@ 0x000020C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER1_CAPTURE_B_BUF1;       /*!< (@ 0x000020C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER1_CAPTURE_B_BUF2;       /*!< (@ 0x000020C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED14;
    volatile uint32_t  UTIMER1_COMPARE_A;            /*!< (@ 0x000020D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER1_COMPARE_A_BUF1;       /*!< (@ 0x000020D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER1_COMPARE_A_BUF2;       /*!< (@ 0x000020D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED15;
    volatile uint32_t  UTIMER1_COMPARE_B;            /*!< (@ 0x000020E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER1_COMPARE_B_BUF1;       /*!< (@ 0x000020E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER1_COMPARE_B_BUF2;       /*!< (@ 0x000020E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED16;
    volatile uint32_t  UTIMER1_DT_UP;                /*!< (@ 0x000020F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER1_DT_UP_BUF1;           /*!< (@ 0x000020F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER1_DT_DOWN;              /*!< (@ 0x000020F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER1_DT_DOWN_BUF1;         /*!< (@ 0x000020FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED17[5];
    volatile uint32_t  UTIMER1_CHAN_STATUS;          /*!< (@ 0x00002114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER1_CHAN_INTERRUPT;       /*!< (@ 0x00002118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER1_CHAN_INTERRUPT_MASK;  /*!< (@ 0x0000211C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER1_DUTY_CYCLE_CTRL;      /*!< (@ 0x00002120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER1_DEAD_TIME_CTRL;       /*!< (@ 0x00002124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED18[2];
    volatile uint32_t  UTIMER1_INT_CNTR_CTRL;        /*!< (@ 0x00002130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER1_FAULT_CTRL;           /*!< (@ 0x00002134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED19[946];
    volatile uint32_t  UTIMER2_START_0_SRC;          /*!< (@ 0x00003000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER2_START_1_SRC;          /*!< (@ 0x00003004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER2_STOP_0_SRC;           /*!< (@ 0x00003008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER2_STOP_1_SRC;           /*!< (@ 0x0000300C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER2_CLEAR_0_SRC;          /*!< (@ 0x00003010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER2_CLEAR_1_SRC;          /*!< (@ 0x00003014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER2_UP_0_SRC;             /*!< (@ 0x00003018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER2_UP_1_SRC;             /*!< (@ 0x0000301C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER2_DOWN_0_SRC;           /*!< (@ 0x00003020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER2_DOWN_1_SRC;           /*!< (@ 0x00003024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER2_TRIG_CAPTURE_SRC_A_0; /*!< (@ 0x00003028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER2_TRIG_CAPTURE_SRC_A_1; /*!< (@ 0x0000302C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER2_TRIG_CAPTURE_SRC_B_0; /*!< (@ 0x00003030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER2_TRIG_CAPTURE_SRC_B_1; /*!< (@ 0x00003034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER2_DMA_CLEAR_SRC_A_0;    /*!< (@ 0x00003038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER2_DMA_CLEAR_SRC_A_1;    /*!< (@ 0x0000303C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER2_DMA_CLEAR_SRC_B_0;    /*!< (@ 0x00003040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER2_DMA_CLEAR_SRC_B_1;    /*!< (@ 0x00003044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER2_CNTR_PAUSE_SRC;       /*!< (@ 0x00003048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED20[13];
    volatile uint32_t  UTIMER2_CNTR_CTRL;            /*!< (@ 0x00003080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER2_FILTER_CTRL_A;        /*!< (@ 0x00003084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER2_FILTER_CTRL_B;        /*!< (@ 0x00003088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER2_COMPARE_CTRL_A;       /*!< (@ 0x0000308C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER2_COMPARE_CTRL_B;       /*!< (@ 0x00003090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER2_BUF_OP_CTRL;          /*!< (@ 0x00003094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED21[2];
    volatile uint32_t  UTIMER2_CNTR;                 /*!< (@ 0x000030A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER2_CNTR_PTR;             /*!< (@ 0x000030A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER2_CNTR_PTR_BUF1;        /*!< (@ 0x000030A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER2_CNTR_PTR_BUF2;        /*!< (@ 0x000030AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER2_CAPTURE_A;            /*!< (@ 0x000030B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER2_CAPTURE_A_BUF1;       /*!< (@ 0x000030B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER2_CAPTURE_A_BUF2;       /*!< (@ 0x000030B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED22;
    volatile uint32_t  UTIMER2_CAPTURE_B;            /*!< (@ 0x000030C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER2_CAPTURE_B_BUF1;       /*!< (@ 0x000030C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER2_CAPTURE_B_BUF2;       /*!< (@ 0x000030C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED23;
    volatile uint32_t  UTIMER2_COMPARE_A;            /*!< (@ 0x000030D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER2_COMPARE_A_BUF1;       /*!< (@ 0x000030D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER2_COMPARE_A_BUF2;       /*!< (@ 0x000030D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED24;
    volatile uint32_t  UTIMER2_COMPARE_B;            /*!< (@ 0x000030E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER2_COMPARE_B_BUF1;       /*!< (@ 0x000030E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER2_COMPARE_B_BUF2;       /*!< (@ 0x000030E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED25;
    volatile uint32_t  UTIMER2_DT_UP;                /*!< (@ 0x000030F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER2_DT_UP_BUF1;           /*!< (@ 0x000030F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER2_DT_DOWN;              /*!< (@ 0x000030F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER2_DT_DOWN_BUF1;         /*!< (@ 0x000030FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED26[5];
    volatile uint32_t  UTIMER2_CHAN_STATUS;          /*!< (@ 0x00003114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER2_CHAN_INTERRUPT;       /*!< (@ 0x00003118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER2_CHAN_INTERRUPT_MASK;  /*!< (@ 0x0000311C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER2_DUTY_CYCLE_CTRL;      /*!< (@ 0x00003120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER2_DEAD_TIME_CTRL;       /*!< (@ 0x00003124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED27[2];
    volatile uint32_t  UTIMER2_INT_CNTR_CTRL;        /*!< (@ 0x00003130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER2_FAULT_CTRL;           /*!< (@ 0x00003134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED28[946];
    volatile uint32_t  UTIMER3_START_0_SRC;          /*!< (@ 0x00004000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER3_START_1_SRC;          /*!< (@ 0x00004004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER3_STOP_0_SRC;           /*!< (@ 0x00004008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER3_STOP_1_SRC;           /*!< (@ 0x0000400C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER3_CLEAR_0_SRC;          /*!< (@ 0x00004010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER3_CLEAR_1_SRC;          /*!< (@ 0x00004014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER3_UP_0_SRC;             /*!< (@ 0x00004018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER3_UP_1_SRC;             /*!< (@ 0x0000401C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER3_DOWN_0_SRC;           /*!< (@ 0x00004020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER3_DOWN_1_SRC;           /*!< (@ 0x00004024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER3_TRIG_CAPTURE_SRC_A_0; /*!< (@ 0x00004028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER3_TRIG_CAPTURE_SRC_A_1; /*!< (@ 0x0000402C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER3_TRIG_CAPTURE_SRC_B_0; /*!< (@ 0x00004030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER3_TRIG_CAPTURE_SRC_B_1; /*!< (@ 0x00004034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER3_DMA_CLEAR_SRC_A_0;    /*!< (@ 0x00004038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER3_DMA_CLEAR_SRC_A_1;    /*!< (@ 0x0000403C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER3_DMA_CLEAR_SRC_B_0;    /*!< (@ 0x00004040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER3_DMA_CLEAR_SRC_B_1;    /*!< (@ 0x00004044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER3_CNTR_PAUSE_SRC;       /*!< (@ 0x00004048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED29[13];
    volatile uint32_t  UTIMER3_CNTR_CTRL;            /*!< (@ 0x00004080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER3_FILTER_CTRL_A;        /*!< (@ 0x00004084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER3_FILTER_CTRL_B;        /*!< (@ 0x00004088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER3_COMPARE_CTRL_A;       /*!< (@ 0x0000408C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER3_COMPARE_CTRL_B;       /*!< (@ 0x00004090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER3_BUF_OP_CTRL;          /*!< (@ 0x00004094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED30[2];
    volatile uint32_t  UTIMER3_CNTR;                 /*!< (@ 0x000040A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER3_CNTR_PTR;             /*!< (@ 0x000040A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER3_CNTR_PTR_BUF1;        /*!< (@ 0x000040A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER3_CNTR_PTR_BUF2;        /*!< (@ 0x000040AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER3_CAPTURE_A;            /*!< (@ 0x000040B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER3_CAPTURE_A_BUF1;       /*!< (@ 0x000040B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER3_CAPTURE_A_BUF2;       /*!< (@ 0x000040B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED31;
    volatile uint32_t  UTIMER3_CAPTURE_B;            /*!< (@ 0x000040C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER3_CAPTURE_B_BUF1;       /*!< (@ 0x000040C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER3_CAPTURE_B_BUF2;       /*!< (@ 0x000040C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED32;
    volatile uint32_t  UTIMER3_COMPARE_A;            /*!< (@ 0x000040D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER3_COMPARE_A_BUF1;       /*!< (@ 0x000040D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER3_COMPARE_A_BUF2;       /*!< (@ 0x000040D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED33;
    volatile uint32_t  UTIMER3_COMPARE_B;            /*!< (@ 0x000040E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER3_COMPARE_B_BUF1;       /*!< (@ 0x000040E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER3_COMPARE_B_BUF2;       /*!< (@ 0x000040E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED34;
    volatile uint32_t  UTIMER3_DT_UP;                /*!< (@ 0x000040F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER3_DT_UP_BUF1;           /*!< (@ 0x000040F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER3_DT_DOWN;              /*!< (@ 0x000040F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER3_DT_DOWN_BUF1;         /*!< (@ 0x000040FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED35[5];
    volatile uint32_t  UTIMER3_CHAN_STATUS;          /*!< (@ 0x00004114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER3_CHAN_INTERRUPT;       /*!< (@ 0x00004118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER3_CHAN_INTERRUPT_MASK;  /*!< (@ 0x0000411C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER3_DUTY_CYCLE_CTRL;      /*!< (@ 0x00004120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER3_DEAD_TIME_CTRL;       /*!< (@ 0x00004124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED36[2];
    volatile uint32_t  UTIMER3_INT_CNTR_CTRL;        /*!< (@ 0x00004130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER3_FAULT_CTRL;           /*!< (@ 0x00004134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED37[946];
    volatile uint32_t  UTIMER4_START_0_SRC;          /*!< (@ 0x00005000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER4_START_1_SRC;          /*!< (@ 0x00005004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER4_STOP_0_SRC;           /*!< (@ 0x00005008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER4_STOP_1_SRC;           /*!< (@ 0x0000500C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER4_CLEAR_0_SRC;          /*!< (@ 0x00005010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER4_CLEAR_1_SRC;          /*!< (@ 0x00005014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER4_UP_0_SRC;             /*!< (@ 0x00005018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER4_UP_1_SRC;             /*!< (@ 0x0000501C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER4_DOWN_0_SRC;           /*!< (@ 0x00005020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER4_DOWN_1_SRC;           /*!< (@ 0x00005024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER4_TRIG_CAPTURE_SRC_A_0; /*!< (@ 0x00005028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER4_TRIG_CAPTURE_SRC_A_1; /*!< (@ 0x0000502C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER4_TRIG_CAPTURE_SRC_B_0; /*!< (@ 0x00005030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER4_TRIG_CAPTURE_SRC_B_1; /*!< (@ 0x00005034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER4_DMA_CLEAR_SRC_A_0;    /*!< (@ 0x00005038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER4_DMA_CLEAR_SRC_A_1;    /*!< (@ 0x0000503C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER4_DMA_CLEAR_SRC_B_0;    /*!< (@ 0x00005040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER4_DMA_CLEAR_SRC_B_1;    /*!< (@ 0x00005044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER4_CNTR_PAUSE_SRC;       /*!< (@ 0x00005048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED38[13];
    volatile uint32_t  UTIMER4_CNTR_CTRL;            /*!< (@ 0x00005080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER4_FILTER_CTRL_A;        /*!< (@ 0x00005084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER4_FILTER_CTRL_B;        /*!< (@ 0x00005088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER4_COMPARE_CTRL_A;       /*!< (@ 0x0000508C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER4_COMPARE_CTRL_B;       /*!< (@ 0x00005090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER4_BUF_OP_CTRL;          /*!< (@ 0x00005094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED39[2];
    volatile uint32_t  UTIMER4_CNTR;                 /*!< (@ 0x000050A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER4_CNTR_PTR;             /*!< (@ 0x000050A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER4_CNTR_PTR_BUF1;        /*!< (@ 0x000050A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER4_CNTR_PTR_BUF2;        /*!< (@ 0x000050AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER4_CAPTURE_A;            /*!< (@ 0x000050B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER4_CAPTURE_A_BUF1;       /*!< (@ 0x000050B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER4_CAPTURE_A_BUF2;       /*!< (@ 0x000050B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED40;
    volatile uint32_t  UTIMER4_CAPTURE_B;            /*!< (@ 0x000050C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER4_CAPTURE_B_BUF1;       /*!< (@ 0x000050C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER4_CAPTURE_B_BUF2;       /*!< (@ 0x000050C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED41;
    volatile uint32_t  UTIMER4_COMPARE_A;            /*!< (@ 0x000050D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER4_COMPARE_A_BUF1;       /*!< (@ 0x000050D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER4_COMPARE_A_BUF2;       /*!< (@ 0x000050D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED42;
    volatile uint32_t  UTIMER4_COMPARE_B;            /*!< (@ 0x000050E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER4_COMPARE_B_BUF1;       /*!< (@ 0x000050E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER4_COMPARE_B_BUF2;       /*!< (@ 0x000050E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED43;
    volatile uint32_t  UTIMER4_DT_UP;                /*!< (@ 0x000050F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER4_DT_UP_BUF1;           /*!< (@ 0x000050F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER4_DT_DOWN;              /*!< (@ 0x000050F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER4_DT_DOWN_BUF1;         /*!< (@ 0x000050FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED44[5];
    volatile uint32_t  UTIMER4_CHAN_STATUS;          /*!< (@ 0x00005114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER4_CHAN_INTERRUPT;       /*!< (@ 0x00005118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER4_CHAN_INTERRUPT_MASK;  /*!< (@ 0x0000511C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER4_DUTY_CYCLE_CTRL;      /*!< (@ 0x00005120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER4_DEAD_TIME_CTRL;       /*!< (@ 0x00005124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED45[2];
    volatile uint32_t  UTIMER4_INT_CNTR_CTRL;        /*!< (@ 0x00005130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER4_FAULT_CTRL;           /*!< (@ 0x00005134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED46[946];
    volatile uint32_t  UTIMER5_START_0_SRC;          /*!< (@ 0x00006000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER5_START_1_SRC;          /*!< (@ 0x00006004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER5_STOP_0_SRC;           /*!< (@ 0x00006008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER5_STOP_1_SRC;           /*!< (@ 0x0000600C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER5_CLEAR_0_SRC;          /*!< (@ 0x00006010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER5_CLEAR_1_SRC;          /*!< (@ 0x00006014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER5_UP_0_SRC;             /*!< (@ 0x00006018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER5_UP_1_SRC;             /*!< (@ 0x0000601C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER5_DOWN_0_SRC;           /*!< (@ 0x00006020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER5_DOWN_1_SRC;           /*!< (@ 0x00006024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER5_TRIG_CAPTURE_SRC_A_0; /*!< (@ 0x00006028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER5_TRIG_CAPTURE_SRC_A_1; /*!< (@ 0x0000602C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER5_TRIG_CAPTURE_SRC_B_0; /*!< (@ 0x00006030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER5_TRIG_CAPTURE_SRC_B_1; /*!< (@ 0x00006034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER5_DMA_CLEAR_SRC_A_0;    /*!< (@ 0x00006038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER5_DMA_CLEAR_SRC_A_1;    /*!< (@ 0x0000603C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER5_DMA_CLEAR_SRC_B_0;    /*!< (@ 0x00006040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER5_DMA_CLEAR_SRC_B_1;    /*!< (@ 0x00006044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER5_CNTR_PAUSE_SRC;       /*!< (@ 0x00006048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED47[13];
    volatile uint32_t  UTIMER5_CNTR_CTRL;            /*!< (@ 0x00006080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER5_FILTER_CTRL_A;        /*!< (@ 0x00006084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER5_FILTER_CTRL_B;        /*!< (@ 0x00006088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER5_COMPARE_CTRL_A;       /*!< (@ 0x0000608C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER5_COMPARE_CTRL_B;       /*!< (@ 0x00006090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER5_BUF_OP_CTRL;          /*!< (@ 0x00006094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED48[2];
    volatile uint32_t  UTIMER5_CNTR;                 /*!< (@ 0x000060A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER5_CNTR_PTR;             /*!< (@ 0x000060A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER5_CNTR_PTR_BUF1;        /*!< (@ 0x000060A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER5_CNTR_PTR_BUF2;        /*!< (@ 0x000060AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER5_CAPTURE_A;            /*!< (@ 0x000060B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER5_CAPTURE_A_BUF1;       /*!< (@ 0x000060B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER5_CAPTURE_A_BUF2;       /*!< (@ 0x000060B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED49;
    volatile uint32_t  UTIMER5_CAPTURE_B;            /*!< (@ 0x000060C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER5_CAPTURE_B_BUF1;       /*!< (@ 0x000060C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER5_CAPTURE_B_BUF2;       /*!< (@ 0x000060C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED50;
    volatile uint32_t  UTIMER5_COMPARE_A;            /*!< (@ 0x000060D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER5_COMPARE_A_BUF1;       /*!< (@ 0x000060D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER5_COMPARE_A_BUF2;       /*!< (@ 0x000060D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED51;
    volatile uint32_t  UTIMER5_COMPARE_B;            /*!< (@ 0x000060E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER5_COMPARE_B_BUF1;       /*!< (@ 0x000060E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER5_COMPARE_B_BUF2;       /*!< (@ 0x000060E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED52;
    volatile uint32_t  UTIMER5_DT_UP;                /*!< (@ 0x000060F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER5_DT_UP_BUF1;           /*!< (@ 0x000060F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER5_DT_DOWN;              /*!< (@ 0x000060F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER5_DT_DOWN_BUF1;         /*!< (@ 0x000060FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED53[5];
    volatile uint32_t  UTIMER5_CHAN_STATUS;          /*!< (@ 0x00006114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER5_CHAN_INTERRUPT;       /*!< (@ 0x00006118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER5_CHAN_INTERRUPT_MASK;  /*!< (@ 0x0000611C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER5_DUTY_CYCLE_CTRL;      /*!< (@ 0x00006120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER5_DEAD_TIME_CTRL;       /*!< (@ 0x00006124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED54[2];
    volatile uint32_t  UTIMER5_INT_CNTR_CTRL;        /*!< (@ 0x00006130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER5_FAULT_CTRL;           /*!< (@ 0x00006134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED55[946];
    volatile uint32_t  UTIMER6_START_0_SRC;          /*!< (@ 0x00007000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER6_START_1_SRC;          /*!< (@ 0x00007004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER6_STOP_0_SRC;           /*!< (@ 0x00007008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER6_STOP_1_SRC;           /*!< (@ 0x0000700C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER6_CLEAR_0_SRC;          /*!< (@ 0x00007010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER6_CLEAR_1_SRC;          /*!< (@ 0x00007014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER6_UP_0_SRC;             /*!< (@ 0x00007018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER6_UP_1_SRC;             /*!< (@ 0x0000701C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER6_DOWN_0_SRC;           /*!< (@ 0x00007020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER6_DOWN_1_SRC;           /*!< (@ 0x00007024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER6_TRIG_CAPTURE_SRC_A_0; /*!< (@ 0x00007028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER6_TRIG_CAPTURE_SRC_A_1; /*!< (@ 0x0000702C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER6_TRIG_CAPTURE_SRC_B_0; /*!< (@ 0x00007030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER6_TRIG_CAPTURE_SRC_B_1; /*!< (@ 0x00007034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER6_DMA_CLEAR_SRC_A_0;    /*!< (@ 0x00007038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER6_DMA_CLEAR_SRC_A_1;    /*!< (@ 0x0000703C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER6_DMA_CLEAR_SRC_B_0;    /*!< (@ 0x00007040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER6_DMA_CLEAR_SRC_B_1;    /*!< (@ 0x00007044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER6_CNTR_PAUSE_SRC;       /*!< (@ 0x00007048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED56[13];
    volatile uint32_t  UTIMER6_CNTR_CTRL;            /*!< (@ 0x00007080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER6_FILTER_CTRL_A;        /*!< (@ 0x00007084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER6_FILTER_CTRL_B;        /*!< (@ 0x00007088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER6_COMPARE_CTRL_A;       /*!< (@ 0x0000708C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER6_COMPARE_CTRL_B;       /*!< (@ 0x00007090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER6_BUF_OP_CTRL;          /*!< (@ 0x00007094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED57[2];
    volatile uint32_t  UTIMER6_CNTR;                 /*!< (@ 0x000070A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER6_CNTR_PTR;             /*!< (@ 0x000070A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER6_CNTR_PTR_BUF1;        /*!< (@ 0x000070A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER6_CNTR_PTR_BUF2;        /*!< (@ 0x000070AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER6_CAPTURE_A;            /*!< (@ 0x000070B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER6_CAPTURE_A_BUF1;       /*!< (@ 0x000070B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER6_CAPTURE_A_BUF2;       /*!< (@ 0x000070B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED58;
    volatile uint32_t  UTIMER6_CAPTURE_B;            /*!< (@ 0x000070C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER6_CAPTURE_B_BUF1;       /*!< (@ 0x000070C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER6_CAPTURE_B_BUF2;       /*!< (@ 0x000070C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED59;
    volatile uint32_t  UTIMER6_COMPARE_A;            /*!< (@ 0x000070D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER6_COMPARE_A_BUF1;       /*!< (@ 0x000070D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER6_COMPARE_A_BUF2;       /*!< (@ 0x000070D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED60;
    volatile uint32_t  UTIMER6_COMPARE_B;            /*!< (@ 0x000070E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER6_COMPARE_B_BUF1;       /*!< (@ 0x000070E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER6_COMPARE_B_BUF2;       /*!< (@ 0x000070E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED61;
    volatile uint32_t  UTIMER6_DT_UP;                /*!< (@ 0x000070F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER6_DT_UP_BUF1;           /*!< (@ 0x000070F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER6_DT_DOWN;              /*!< (@ 0x000070F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER6_DT_DOWN_BUF1;         /*!< (@ 0x000070FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED62[5];
    volatile uint32_t  UTIMER6_CHAN_STATUS;          /*!< (@ 0x00007114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER6_CHAN_INTERRUPT;       /*!< (@ 0x00007118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER6_CHAN_INTERRUPT_MASK;  /*!< (@ 0x0000711C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER6_DUTY_CYCLE_CTRL;      /*!< (@ 0x00007120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER6_DEAD_TIME_CTRL;       /*!< (@ 0x00007124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED63[2];
    volatile uint32_t  UTIMER6_INT_CNTR_CTRL;        /*!< (@ 0x00007130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER6_FAULT_CTRL;           /*!< (@ 0x00007134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED64[946];
    volatile uint32_t  UTIMER7_START_0_SRC;          /*!< (@ 0x00008000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER7_START_1_SRC;          /*!< (@ 0x00008004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER7_STOP_0_SRC;           /*!< (@ 0x00008008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER7_STOP_1_SRC;           /*!< (@ 0x0000800C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER7_CLEAR_0_SRC;          /*!< (@ 0x00008010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER7_CLEAR_1_SRC;          /*!< (@ 0x00008014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER7_UP_0_SRC;             /*!< (@ 0x00008018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER7_UP_1_SRC;             /*!< (@ 0x0000801C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER7_DOWN_0_SRC;           /*!< (@ 0x00008020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER7_DOWN_1_SRC;           /*!< (@ 0x00008024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER7_TRIG_CAPTURE_SRC_A_0; /*!< (@ 0x00008028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER7_TRIG_CAPTURE_SRC_A_1; /*!< (@ 0x0000802C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER7_TRIG_CAPTURE_SRC_B_0; /*!< (@ 0x00008030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER7_TRIG_CAPTURE_SRC_B_1; /*!< (@ 0x00008034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER7_DMA_CLEAR_SRC_A_0;    /*!< (@ 0x00008038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER7_DMA_CLEAR_SRC_A_1;    /*!< (@ 0x0000803C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER7_DMA_CLEAR_SRC_B_0;    /*!< (@ 0x00008040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER7_DMA_CLEAR_SRC_B_1;    /*!< (@ 0x00008044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER7_CNTR_PAUSE_SRC;       /*!< (@ 0x00008048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED65[13];
    volatile uint32_t  UTIMER7_CNTR_CTRL;            /*!< (@ 0x00008080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER7_FILTER_CTRL_A;        /*!< (@ 0x00008084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER7_FILTER_CTRL_B;        /*!< (@ 0x00008088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER7_COMPARE_CTRL_A;       /*!< (@ 0x0000808C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER7_COMPARE_CTRL_B;       /*!< (@ 0x00008090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER7_BUF_OP_CTRL;          /*!< (@ 0x00008094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED66[2];
    volatile uint32_t  UTIMER7_CNTR;                 /*!< (@ 0x000080A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER7_CNTR_PTR;             /*!< (@ 0x000080A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER7_CNTR_PTR_BUF1;        /*!< (@ 0x000080A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER7_CNTR_PTR_BUF2;        /*!< (@ 0x000080AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER7_CAPTURE_A;            /*!< (@ 0x000080B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER7_CAPTURE_A_BUF1;       /*!< (@ 0x000080B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER7_CAPTURE_A_BUF2;       /*!< (@ 0x000080B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED67;
    volatile uint32_t  UTIMER7_CAPTURE_B;            /*!< (@ 0x000080C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER7_CAPTURE_B_BUF1;       /*!< (@ 0x000080C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER7_CAPTURE_B_BUF2;       /*!< (@ 0x000080C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED68;
    volatile uint32_t  UTIMER7_COMPARE_A;            /*!< (@ 0x000080D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER7_COMPARE_A_BUF1;       /*!< (@ 0x000080D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER7_COMPARE_A_BUF2;       /*!< (@ 0x000080D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED69;
    volatile uint32_t  UTIMER7_COMPARE_B;            /*!< (@ 0x000080E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER7_COMPARE_B_BUF1;       /*!< (@ 0x000080E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER7_COMPARE_B_BUF2;       /*!< (@ 0x000080E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED70;
    volatile uint32_t  UTIMER7_DT_UP;                /*!< (@ 0x000080F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER7_DT_UP_BUF1;           /*!< (@ 0x000080F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER7_DT_DOWN;              /*!< (@ 0x000080F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER7_DT_DOWN_BUF1;         /*!< (@ 0x000080FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED71[5];
    volatile uint32_t  UTIMER7_CHAN_STATUS;          /*!< (@ 0x00008114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER7_CHAN_INTERRUPT;       /*!< (@ 0x00008118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER7_CHAN_INTERRUPT_MASK;  /*!< (@ 0x0000811C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER7_DUTY_CYCLE_CTRL;      /*!< (@ 0x00008120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER7_DEAD_TIME_CTRL;       /*!< (@ 0x00008124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED72[2];
    volatile uint32_t  UTIMER7_INT_CNTR_CTRL;        /*!< (@ 0x00008130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER7_FAULT_CTRL;           /*!< (@ 0x00008134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED73[946];
    volatile uint32_t  UTIMER8_START_0_SRC;          /*!< (@ 0x00009000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER8_START_1_SRC;          /*!< (@ 0x00009004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER8_STOP_0_SRC;           /*!< (@ 0x00009008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER8_STOP_1_SRC;           /*!< (@ 0x0000900C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER8_CLEAR_0_SRC;          /*!< (@ 0x00009010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER8_CLEAR_1_SRC;          /*!< (@ 0x00009014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER8_UP_0_SRC;             /*!< (@ 0x00009018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER8_UP_1_SRC;             /*!< (@ 0x0000901C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER8_DOWN_0_SRC;           /*!< (@ 0x00009020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER8_DOWN_1_SRC;           /*!< (@ 0x00009024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER8_TRIG_CAPTURE_SRC_A_0; /*!< (@ 0x00009028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER8_TRIG_CAPTURE_SRC_A_1; /*!< (@ 0x0000902C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER8_TRIG_CAPTURE_SRC_B_0; /*!< (@ 0x00009030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER8_TRIG_CAPTURE_SRC_B_1; /*!< (@ 0x00009034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER8_DMA_CLEAR_SRC_A_0;    /*!< (@ 0x00009038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER8_DMA_CLEAR_SRC_A_1;    /*!< (@ 0x0000903C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER8_DMA_CLEAR_SRC_B_0;    /*!< (@ 0x00009040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER8_DMA_CLEAR_SRC_B_1;    /*!< (@ 0x00009044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER8_CNTR_PAUSE_SRC;       /*!< (@ 0x00009048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED74[13];
    volatile uint32_t  UTIMER8_CNTR_CTRL;            /*!< (@ 0x00009080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER8_FILTER_CTRL_A;        /*!< (@ 0x00009084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER8_FILTER_CTRL_B;        /*!< (@ 0x00009088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER8_COMPARE_CTRL_A;       /*!< (@ 0x0000908C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER8_COMPARE_CTRL_B;       /*!< (@ 0x00009090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER8_BUF_OP_CTRL;          /*!< (@ 0x00009094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED75[2];
    volatile uint32_t  UTIMER8_CNTR;                 /*!< (@ 0x000090A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER8_CNTR_PTR;             /*!< (@ 0x000090A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER8_CNTR_PTR_BUF1;        /*!< (@ 0x000090A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER8_CNTR_PTR_BUF2;        /*!< (@ 0x000090AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER8_CAPTURE_A;            /*!< (@ 0x000090B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER8_CAPTURE_A_BUF1;       /*!< (@ 0x000090B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER8_CAPTURE_A_BUF2;       /*!< (@ 0x000090B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED76;
    volatile uint32_t  UTIMER8_CAPTURE_B;            /*!< (@ 0x000090C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER8_CAPTURE_B_BUF1;       /*!< (@ 0x000090C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER8_CAPTURE_B_BUF2;       /*!< (@ 0x000090C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED77;
    volatile uint32_t  UTIMER8_COMPARE_A;            /*!< (@ 0x000090D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER8_COMPARE_A_BUF1;       /*!< (@ 0x000090D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER8_COMPARE_A_BUF2;       /*!< (@ 0x000090D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED78;
    volatile uint32_t  UTIMER8_COMPARE_B;            /*!< (@ 0x000090E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER8_COMPARE_B_BUF1;       /*!< (@ 0x000090E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER8_COMPARE_B_BUF2;       /*!< (@ 0x000090E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED79;
    volatile uint32_t  UTIMER8_DT_UP;                /*!< (@ 0x000090F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER8_DT_UP_BUF1;           /*!< (@ 0x000090F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER8_DT_DOWN;              /*!< (@ 0x000090F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER8_DT_DOWN_BUF1;         /*!< (@ 0x000090FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED80[5];
    volatile uint32_t  UTIMER8_CHAN_STATUS;          /*!< (@ 0x00009114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER8_CHAN_INTERRUPT;       /*!< (@ 0x00009118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER8_CHAN_INTERRUPT_MASK;  /*!< (@ 0x0000911C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER8_DUTY_CYCLE_CTRL;      /*!< (@ 0x00009120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER8_DEAD_TIME_CTRL;       /*!< (@ 0x00009124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED81[2];
    volatile uint32_t  UTIMER8_INT_CNTR_CTRL;        /*!< (@ 0x00009130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER8_FAULT_CTRL;           /*!< (@ 0x00009134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED82[946];
    volatile uint32_t  UTIMER9_START_0_SRC;          /*!< (@ 0x0000A000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER9_START_1_SRC;          /*!< (@ 0x0000A004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER9_STOP_0_SRC;           /*!< (@ 0x0000A008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER9_STOP_1_SRC;           /*!< (@ 0x0000A00C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER9_CLEAR_0_SRC;          /*!< (@ 0x0000A010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER9_CLEAR_1_SRC;          /*!< (@ 0x0000A014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER9_UP_0_SRC;             /*!< (@ 0x0000A018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER9_UP_1_SRC;             /*!< (@ 0x0000A01C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER9_DOWN_0_SRC;           /*!< (@ 0x0000A020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER9_DOWN_1_SRC;           /*!< (@ 0x0000A024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER9_TRIG_CAPTURE_SRC_A_0; /*!< (@ 0x0000A028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER9_TRIG_CAPTURE_SRC_A_1; /*!< (@ 0x0000A02C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER9_TRIG_CAPTURE_SRC_B_0; /*!< (@ 0x0000A030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER9_TRIG_CAPTURE_SRC_B_1; /*!< (@ 0x0000A034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER9_DMA_CLEAR_SRC_A_0;    /*!< (@ 0x0000A038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER9_DMA_CLEAR_SRC_A_1;    /*!< (@ 0x0000A03C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER9_DMA_CLEAR_SRC_B_0;    /*!< (@ 0x0000A040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER9_DMA_CLEAR_SRC_B_1;    /*!< (@ 0x0000A044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER9_CNTR_PAUSE_SRC;       /*!< (@ 0x0000A048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED83[13];
    volatile uint32_t  UTIMER9_CNTR_CTRL;            /*!< (@ 0x0000A080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER9_FILTER_CTRL_A;        /*!< (@ 0x0000A084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER9_FILTER_CTRL_B;        /*!< (@ 0x0000A088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER9_COMPARE_CTRL_A;       /*!< (@ 0x0000A08C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER9_COMPARE_CTRL_B;       /*!< (@ 0x0000A090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER9_BUF_OP_CTRL;          /*!< (@ 0x0000A094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED84[2];
    volatile uint32_t  UTIMER9_CNTR;                 /*!< (@ 0x0000A0A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER9_CNTR_PTR;             /*!< (@ 0x0000A0A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER9_CNTR_PTR_BUF1;        /*!< (@ 0x0000A0A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER9_CNTR_PTR_BUF2;        /*!< (@ 0x0000A0AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER9_CAPTURE_A;            /*!< (@ 0x0000A0B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER9_CAPTURE_A_BUF1;       /*!< (@ 0x0000A0B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER9_CAPTURE_A_BUF2;       /*!< (@ 0x0000A0B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED85;
    volatile uint32_t  UTIMER9_CAPTURE_B;            /*!< (@ 0x0000A0C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER9_CAPTURE_B_BUF1;       /*!< (@ 0x0000A0C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER9_CAPTURE_B_BUF2;       /*!< (@ 0x0000A0C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED86;
    volatile uint32_t  UTIMER9_COMPARE_A;            /*!< (@ 0x0000A0D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER9_COMPARE_A_BUF1;       /*!< (@ 0x0000A0D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER9_COMPARE_A_BUF2;       /*!< (@ 0x0000A0D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED87;
    volatile uint32_t  UTIMER9_COMPARE_B;            /*!< (@ 0x0000A0E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER9_COMPARE_B_BUF1;       /*!< (@ 0x0000A0E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER9_COMPARE_B_BUF2;       /*!< (@ 0x0000A0E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED88;
    volatile uint32_t  UTIMER9_DT_UP;                /*!< (@ 0x0000A0F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER9_DT_UP_BUF1;           /*!< (@ 0x0000A0F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER9_DT_DOWN;              /*!< (@ 0x0000A0F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER9_DT_DOWN_BUF1;         /*!< (@ 0x0000A0FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED89[5];
    volatile uint32_t  UTIMER9_CHAN_STATUS;          /*!< (@ 0x0000A114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER9_CHAN_INTERRUPT;       /*!< (@ 0x0000A118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER9_CHAN_INTERRUPT_MASK;  /*!< (@ 0x0000A11C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER9_DUTY_CYCLE_CTRL;      /*!< (@ 0x0000A120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER9_DEAD_TIME_CTRL;       /*!< (@ 0x0000A124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED90[2];
    volatile uint32_t  UTIMER9_INT_CNTR_CTRL;        /*!< (@ 0x0000A130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER9_FAULT_CTRL;           /*!< (@ 0x0000A134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED91[946];
    volatile uint32_t  UTIMER10_START_0_SRC;         /*!< (@ 0x0000B000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER10_START_1_SRC;         /*!< (@ 0x0000B004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER10_STOP_0_SRC;          /*!< (@ 0x0000B008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER10_STOP_1_SRC;          /*!< (@ 0x0000B00C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER10_CLEAR_0_SRC;         /*!< (@ 0x0000B010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER10_CLEAR_1_SRC;         /*!< (@ 0x0000B014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER10_UP_0_SRC;            /*!< (@ 0x0000B018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER10_UP_1_SRC;            /*!< (@ 0x0000B01C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER10_DOWN_0_SRC;          /*!< (@ 0x0000B020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER10_DOWN_1_SRC;          /*!< (@ 0x0000B024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER10_TRIG_CAPTURE_SRC_A_0;/*!< (@ 0x0000B028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER10_TRIG_CAPTURE_SRC_A_1;/*!< (@ 0x0000B02C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER10_TRIG_CAPTURE_SRC_B_0;/*!< (@ 0x0000B030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER10_TRIG_CAPTURE_SRC_B_1;/*!< (@ 0x0000B034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER10_DMA_CLEAR_SRC_A_0;   /*!< (@ 0x0000B038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER10_DMA_CLEAR_SRC_A_1;   /*!< (@ 0x0000B03C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER10_DMA_CLEAR_SRC_B_0;   /*!< (@ 0x0000B040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER10_DMA_CLEAR_SRC_B_1;   /*!< (@ 0x0000B044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER10_CNTR_PAUSE_SRC;      /*!< (@ 0x0000B048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED92[13];
    volatile uint32_t  UTIMER10_CNTR_CTRL;           /*!< (@ 0x0000B080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER10_FILTER_CTRL_A;       /*!< (@ 0x0000B084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER10_FILTER_CTRL_B;       /*!< (@ 0x0000B088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER10_COMPARE_CTRL_A;      /*!< (@ 0x0000B08C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER10_COMPARE_CTRL_B;      /*!< (@ 0x0000B090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER10_BUF_OP_CTRL;         /*!< (@ 0x0000B094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED93[2];
    volatile uint32_t  UTIMER10_CNTR;                /*!< (@ 0x0000B0A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER10_CNTR_PTR;            /*!< (@ 0x0000B0A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER10_CNTR_PTR_BUF1;       /*!< (@ 0x0000B0A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER10_CNTR_PTR_BUF2;       /*!< (@ 0x0000B0AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER10_CAPTURE_A;           /*!< (@ 0x0000B0B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER10_CAPTURE_A_BUF1;      /*!< (@ 0x0000B0B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER10_CAPTURE_A_BUF2;      /*!< (@ 0x0000B0B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED94;
    volatile uint32_t  UTIMER10_CAPTURE_B;           /*!< (@ 0x0000B0C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER10_CAPTURE_B_BUF1;      /*!< (@ 0x0000B0C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER10_CAPTURE_B_BUF2;      /*!< (@ 0x0000B0C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED95;
    volatile uint32_t  UTIMER10_COMPARE_A;           /*!< (@ 0x0000B0D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER10_COMPARE_A_BUF1;      /*!< (@ 0x0000B0D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER10_COMPARE_A_BUF2;      /*!< (@ 0x0000B0D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED96;
    volatile uint32_t  UTIMER10_COMPARE_B;           /*!< (@ 0x0000B0E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER10_COMPARE_B_BUF1;      /*!< (@ 0x0000B0E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER10_COMPARE_B_BUF2;      /*!< (@ 0x0000B0E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED97;
    volatile uint32_t  UTIMER10_DT_UP;               /*!< (@ 0x0000B0F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER10_DT_UP_BUF1;          /*!< (@ 0x0000B0F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER10_DT_DOWN;             /*!< (@ 0x0000B0F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER10_DT_DOWN_BUF1;        /*!< (@ 0x0000B0FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED98[5];
    volatile uint32_t  UTIMER10_CHAN_STATUS;         /*!< (@ 0x0000B114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER10_CHAN_INTERRUPT;      /*!< (@ 0x0000B118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER10_CHAN_INTERRUPT_MASK; /*!< (@ 0x0000B11C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER10_DUTY_CYCLE_CTRL;     /*!< (@ 0x0000B120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER10_DEAD_TIME_CTRL;      /*!< (@ 0x0000B124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED99[2];
    volatile uint32_t  UTIMER10_INT_CNTR_CTRL;       /*!< (@ 0x0000B130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER10_FAULT_CTRL;          /*!< (@ 0x0000B134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED100[946];
    volatile uint32_t  UTIMER11_START_0_SRC;         /*!< (@ 0x0000C000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER11_START_1_SRC;         /*!< (@ 0x0000C004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER11_STOP_0_SRC;          /*!< (@ 0x0000C008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER11_STOP_1_SRC;          /*!< (@ 0x0000C00C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER11_CLEAR_0_SRC;         /*!< (@ 0x0000C010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER11_CLEAR_1_SRC;         /*!< (@ 0x0000C014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER11_UP_0_SRC;            /*!< (@ 0x0000C018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER11_UP_1_SRC;            /*!< (@ 0x0000C01C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER11_DOWN_0_SRC;          /*!< (@ 0x0000C020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER11_DOWN_1_SRC;          /*!< (@ 0x0000C024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER11_TRIG_CAPTURE_SRC_A_0;/*!< (@ 0x0000C028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER11_TRIG_CAPTURE_SRC_A_1;/*!< (@ 0x0000C02C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER11_TRIG_CAPTURE_SRC_B_0;/*!< (@ 0x0000C030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER11_TRIG_CAPTURE_SRC_B_1;/*!< (@ 0x0000C034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER11_DMA_CLEAR_SRC_A_0;   /*!< (@ 0x0000C038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER11_DMA_CLEAR_SRC_A_1;   /*!< (@ 0x0000C03C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER11_DMA_CLEAR_SRC_B_0;   /*!< (@ 0x0000C040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER11_DMA_CLEAR_SRC_B_1;   /*!< (@ 0x0000C044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER11_CNTR_PAUSE_SRC;      /*!< (@ 0x0000C048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED101[13];
    volatile uint32_t  UTIMER11_CNTR_CTRL;           /*!< (@ 0x0000C080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER11_FILTER_CTRL_A;       /*!< (@ 0x0000C084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER11_FILTER_CTRL_B;       /*!< (@ 0x0000C088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER11_COMPARE_CTRL_A;      /*!< (@ 0x0000C08C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER11_COMPARE_CTRL_B;      /*!< (@ 0x0000C090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER11_BUF_OP_CTRL;         /*!< (@ 0x0000C094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED102[2];
    volatile uint32_t  UTIMER11_CNTR;                /*!< (@ 0x0000C0A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER11_CNTR_PTR;            /*!< (@ 0x0000C0A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER11_CNTR_PTR_BUF1;       /*!< (@ 0x0000C0A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER11_CNTR_PTR_BUF2;       /*!< (@ 0x0000C0AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER11_CAPTURE_A;           /*!< (@ 0x0000C0B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER11_CAPTURE_A_BUF1;      /*!< (@ 0x0000C0B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER11_CAPTURE_A_BUF2;      /*!< (@ 0x0000C0B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED103;
    volatile uint32_t  UTIMER11_CAPTURE_B;           /*!< (@ 0x0000C0C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER11_CAPTURE_B_BUF1;      /*!< (@ 0x0000C0C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER11_CAPTURE_B_BUF2;      /*!< (@ 0x0000C0C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED104;
    volatile uint32_t  UTIMER11_COMPARE_A;           /*!< (@ 0x0000C0D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER11_COMPARE_A_BUF1;      /*!< (@ 0x0000C0D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER11_COMPARE_A_BUF2;      /*!< (@ 0x0000C0D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED105;
    volatile uint32_t  UTIMER11_COMPARE_B;           /*!< (@ 0x0000C0E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER11_COMPARE_B_BUF1;      /*!< (@ 0x0000C0E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER11_COMPARE_B_BUF2;      /*!< (@ 0x0000C0E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED106;
    volatile uint32_t  UTIMER11_DT_UP;               /*!< (@ 0x0000C0F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER11_DT_UP_BUF1;          /*!< (@ 0x0000C0F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER11_DT_DOWN;             /*!< (@ 0x0000C0F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER11_DT_DOWN_BUF1;        /*!< (@ 0x0000C0FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED107[5];
    volatile uint32_t  UTIMER11_CHAN_STATUS;         /*!< (@ 0x0000C114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER11_CHAN_INTERRUPT;      /*!< (@ 0x0000C118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER11_CHAN_INTERRUPT_MASK; /*!< (@ 0x0000C11C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER11_DUTY_CYCLE_CTRL;     /*!< (@ 0x0000C120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER11_DEAD_TIME_CTRL;      /*!< (@ 0x0000C124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED108[2];
    volatile uint32_t  UTIMER11_INT_CNTR_CTRL;       /*!< (@ 0x0000C130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER11_FAULT_CTRL;          /*!< (@ 0x0000C134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED109[946];
    volatile uint32_t  UTIMER12_START_0_SRC;         /*!< (@ 0x0000D000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER12_START_1_SRC;         /*!< (@ 0x0000D004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER12_STOP_0_SRC;          /*!< (@ 0x0000D008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER12_STOP_1_SRC;          /*!< (@ 0x0000D00C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER12_CLEAR_0_SRC;         /*!< (@ 0x0000D010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER12_CLEAR_1_SRC;         /*!< (@ 0x0000D014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER12_UP_0_SRC;            /*!< (@ 0x0000D018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER12_UP_1_SRC;            /*!< (@ 0x0000D01C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER12_DOWN_0_SRC;          /*!< (@ 0x0000D020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER12_DOWN_1_SRC;          /*!< (@ 0x0000D024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER12_TRIG_CAPTURE_SRC_A_0;/*!< (@ 0x0000D028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER12_TRIG_CAPTURE_SRC_A_1;/*!< (@ 0x0000D02C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER12_TRIG_CAPTURE_SRC_B_0;/*!< (@ 0x0000D030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER12_TRIG_CAPTURE_SRC_B_1;/*!< (@ 0x0000D034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER12_DMA_CLEAR_SRC_A_0;   /*!< (@ 0x0000D038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER12_DMA_CLEAR_SRC_A_1;   /*!< (@ 0x0000D03C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER12_DMA_CLEAR_SRC_B_0;   /*!< (@ 0x0000D040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER12_DMA_CLEAR_SRC_B_1;   /*!< (@ 0x0000D044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER12_CNTR_PAUSE_SRC;      /*!< (@ 0x0000D048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED110[13];
    volatile uint32_t  UTIMER12_CNTR_CTRL;           /*!< (@ 0x0000D080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER12_FILTER_CTRL_A;       /*!< (@ 0x0000D084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER12_FILTER_CTRL_B;       /*!< (@ 0x0000D088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER12_COMPARE_CTRL_A;      /*!< (@ 0x0000D08C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER12_COMPARE_CTRL_B;      /*!< (@ 0x0000D090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER12_BUF_OP_CTRL;         /*!< (@ 0x0000D094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED111[2];
    volatile uint32_t  UTIMER12_CNTR;                /*!< (@ 0x0000D0A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER12_CNTR_PTR;            /*!< (@ 0x0000D0A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER12_CNTR_PTR_BUF1;       /*!< (@ 0x0000D0A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER12_CNTR_PTR_BUF2;       /*!< (@ 0x0000D0AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER12_CAPTURE_A;           /*!< (@ 0x0000D0B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER12_CAPTURE_A_BUF1;      /*!< (@ 0x0000D0B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER12_CAPTURE_A_BUF2;      /*!< (@ 0x0000D0B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED112;
    volatile uint32_t  UTIMER12_CAPTURE_B;           /*!< (@ 0x0000D0C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER12_CAPTURE_B_BUF1;      /*!< (@ 0x0000D0C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER12_CAPTURE_B_BUF2;      /*!< (@ 0x0000D0C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED113;
    volatile uint32_t  UTIMER12_COMPARE_A;           /*!< (@ 0x0000D0D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER12_COMPARE_A_BUF1;      /*!< (@ 0x0000D0D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER12_COMPARE_A_BUF2;      /*!< (@ 0x0000D0D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED114;
    volatile uint32_t  UTIMER12_COMPARE_B;           /*!< (@ 0x0000D0E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER12_COMPARE_B_BUF1;      /*!< (@ 0x0000D0E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER12_COMPARE_B_BUF2;      /*!< (@ 0x0000D0E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED115;
    volatile uint32_t  UTIMER12_DT_UP;               /*!< (@ 0x0000D0F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER12_DT_UP_BUF1;          /*!< (@ 0x0000D0F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER12_DT_DOWN;             /*!< (@ 0x0000D0F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER12_DT_DOWN_BUF1;        /*!< (@ 0x0000D0FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED116[5];
    volatile uint32_t  UTIMER12_CHAN_STATUS;         /*!< (@ 0x0000D114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER12_CHAN_INTERRUPT;      /*!< (@ 0x0000D118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER12_CHAN_INTERRUPT_MASK; /*!< (@ 0x0000D11C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER12_DUTY_CYCLE_CTRL;     /*!< (@ 0x0000D120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER12_DEAD_TIME_CTRL;      /*!< (@ 0x0000D124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED117[2];
    volatile uint32_t  UTIMER12_INT_CNTR_CTRL;       /*!< (@ 0x0000D130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER12_FAULT_CTRL;          /*!< (@ 0x0000D134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED118[946];
    volatile uint32_t  UTIMER13_START_0_SRC;         /*!< (@ 0x0000E000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER13_START_1_SRC;         /*!< (@ 0x0000E004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER13_STOP_0_SRC;          /*!< (@ 0x0000E008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER13_STOP_1_SRC;          /*!< (@ 0x0000E00C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER13_CLEAR_0_SRC;         /*!< (@ 0x0000E010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER13_CLEAR_1_SRC;         /*!< (@ 0x0000E014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER13_UP_0_SRC;            /*!< (@ 0x0000E018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER13_UP_1_SRC;            /*!< (@ 0x0000E01C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER13_DOWN_0_SRC;          /*!< (@ 0x0000E020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER13_DOWN_1_SRC;          /*!< (@ 0x0000E024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER13_TRIG_CAPTURE_SRC_A_0;/*!< (@ 0x0000E028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER13_TRIG_CAPTURE_SRC_A_1;/*!< (@ 0x0000E02C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER13_TRIG_CAPTURE_SRC_B_0;/*!< (@ 0x0000E030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER13_TRIG_CAPTURE_SRC_B_1;/*!< (@ 0x0000E034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER13_DMA_CLEAR_SRC_A_0;   /*!< (@ 0x0000E038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER13_DMA_CLEAR_SRC_A_1;   /*!< (@ 0x0000E03C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER13_DMA_CLEAR_SRC_B_0;   /*!< (@ 0x0000E040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER13_DMA_CLEAR_SRC_B_1;   /*!< (@ 0x0000E044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER13_CNTR_PAUSE_SRC;      /*!< (@ 0x0000E048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED119[13];
    volatile uint32_t  UTIMER13_CNTR_CTRL;           /*!< (@ 0x0000E080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER13_FILTER_CTRL_A;       /*!< (@ 0x0000E084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER13_FILTER_CTRL_B;       /*!< (@ 0x0000E088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER13_COMPARE_CTRL_A;      /*!< (@ 0x0000E08C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER13_COMPARE_CTRL_B;      /*!< (@ 0x0000E090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER13_BUF_OP_CTRL;         /*!< (@ 0x0000E094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED120[2];
    volatile uint32_t  UTIMER13_CNTR;                /*!< (@ 0x0000E0A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER13_CNTR_PTR;            /*!< (@ 0x0000E0A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER13_CNTR_PTR_BUF1;       /*!< (@ 0x0000E0A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER13_CNTR_PTR_BUF2;       /*!< (@ 0x0000E0AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER13_CAPTURE_A;           /*!< (@ 0x0000E0B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER13_CAPTURE_A_BUF1;      /*!< (@ 0x0000E0B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER13_CAPTURE_A_BUF2;      /*!< (@ 0x0000E0B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED121;
    volatile uint32_t  UTIMER13_CAPTURE_B;           /*!< (@ 0x0000E0C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER13_CAPTURE_B_BUF1;      /*!< (@ 0x0000E0C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER13_CAPTURE_B_BUF2;      /*!< (@ 0x0000E0C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED122;
    volatile uint32_t  UTIMER13_COMPARE_A;           /*!< (@ 0x0000E0D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER13_COMPARE_A_BUF1;      /*!< (@ 0x0000E0D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER13_COMPARE_A_BUF2;      /*!< (@ 0x0000E0D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED123;
    volatile uint32_t  UTIMER13_COMPARE_B;           /*!< (@ 0x0000E0E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER13_COMPARE_B_BUF1;      /*!< (@ 0x0000E0E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER13_COMPARE_B_BUF2;      /*!< (@ 0x0000E0E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED124;
    volatile uint32_t  UTIMER13_DT_UP;               /*!< (@ 0x0000E0F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER13_DT_UP_BUF1;          /*!< (@ 0x0000E0F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER13_DT_DOWN;             /*!< (@ 0x0000E0F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER13_DT_DOWN_BUF1;        /*!< (@ 0x0000E0FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED125[5];
    volatile uint32_t  UTIMER13_CHAN_STATUS;         /*!< (@ 0x0000E114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER13_CHAN_INTERRUPT;      /*!< (@ 0x0000E118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER13_CHAN_INTERRUPT_MASK; /*!< (@ 0x0000E11C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER13_DUTY_CYCLE_CTRL;     /*!< (@ 0x0000E120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER13_DEAD_TIME_CTRL;      /*!< (@ 0x0000E124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED126[2];
    volatile uint32_t  UTIMER13_INT_CNTR_CTRL;       /*!< (@ 0x0000E130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER13_FAULT_CTRL;          /*!< (@ 0x0000E134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED127[946];
    volatile uint32_t  UTIMER14_START_0_SRC;         /*!< (@ 0x0000F000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER14_START_1_SRC;         /*!< (@ 0x0000F004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER14_STOP_0_SRC;          /*!< (@ 0x0000F008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER14_STOP_1_SRC;          /*!< (@ 0x0000F00C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER14_CLEAR_0_SRC;         /*!< (@ 0x0000F010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER14_CLEAR_1_SRC;         /*!< (@ 0x0000F014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER14_UP_0_SRC;            /*!< (@ 0x0000F018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER14_UP_1_SRC;            /*!< (@ 0x0000F01C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER14_DOWN_0_SRC;          /*!< (@ 0x0000F020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER14_DOWN_1_SRC;          /*!< (@ 0x0000F024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER14_TRIG_CAPTURE_SRC_A_0;/*!< (@ 0x0000F028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER14_TRIG_CAPTURE_SRC_A_1;/*!< (@ 0x0000F02C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER14_TRIG_CAPTURE_SRC_B_0;/*!< (@ 0x0000F030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER14_TRIG_CAPTURE_SRC_B_1;/*!< (@ 0x0000F034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER14_DMA_CLEAR_SRC_A_0;   /*!< (@ 0x0000F038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER14_DMA_CLEAR_SRC_A_1;   /*!< (@ 0x0000F03C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER14_DMA_CLEAR_SRC_B_0;   /*!< (@ 0x0000F040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER14_DMA_CLEAR_SRC_B_1;   /*!< (@ 0x0000F044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER14_CNTR_PAUSE_SRC;      /*!< (@ 0x0000F048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED128[13];
    volatile uint32_t  UTIMER14_CNTR_CTRL;           /*!< (@ 0x0000F080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER14_FILTER_CTRL_A;       /*!< (@ 0x0000F084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER14_FILTER_CTRL_B;       /*!< (@ 0x0000F088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER14_COMPARE_CTRL_A;      /*!< (@ 0x0000F08C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER14_COMPARE_CTRL_B;      /*!< (@ 0x0000F090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER14_BUF_OP_CTRL;         /*!< (@ 0x0000F094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED129[2];
    volatile uint32_t  UTIMER14_CNTR;                /*!< (@ 0x0000F0A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER14_CNTR_PTR;            /*!< (@ 0x0000F0A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER14_CNTR_PTR_BUF1;       /*!< (@ 0x0000F0A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER14_CNTR_PTR_BUF2;       /*!< (@ 0x0000F0AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER14_CAPTURE_A;           /*!< (@ 0x0000F0B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER14_CAPTURE_A_BUF1;      /*!< (@ 0x0000F0B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER14_CAPTURE_A_BUF2;      /*!< (@ 0x0000F0B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED130;
    volatile uint32_t  UTIMER14_CAPTURE_B;           /*!< (@ 0x0000F0C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER14_CAPTURE_B_BUF1;      /*!< (@ 0x0000F0C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER14_CAPTURE_B_BUF2;      /*!< (@ 0x0000F0C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED131;
    volatile uint32_t  UTIMER14_COMPARE_A;           /*!< (@ 0x0000F0D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER14_COMPARE_A_BUF1;      /*!< (@ 0x0000F0D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER14_COMPARE_A_BUF2;      /*!< (@ 0x0000F0D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED132;
    volatile uint32_t  UTIMER14_COMPARE_B;           /*!< (@ 0x0000F0E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER14_COMPARE_B_BUF1;      /*!< (@ 0x0000F0E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER14_COMPARE_B_BUF2;      /*!< (@ 0x0000F0E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED133;
    volatile uint32_t  UTIMER14_DT_UP;               /*!< (@ 0x0000F0F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER14_DT_UP_BUF1;          /*!< (@ 0x0000F0F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER14_DT_DOWN;             /*!< (@ 0x0000F0F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER14_DT_DOWN_BUF1;        /*!< (@ 0x0000F0FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED134[5];
    volatile uint32_t  UTIMER14_CHAN_STATUS;         /*!< (@ 0x0000F114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER14_CHAN_INTERRUPT;      /*!< (@ 0x0000F118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER14_CHAN_INTERRUPT_MASK; /*!< (@ 0x0000F11C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER14_DUTY_CYCLE_CTRL;     /*!< (@ 0x0000F120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER14_DEAD_TIME_CTRL;      /*!< (@ 0x0000F124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED135[2];
    volatile uint32_t  UTIMER14_INT_CNTR_CTRL;       /*!< (@ 0x0000F130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER14_FAULT_CTRL;          /*!< (@ 0x0000F134) Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED136[946];
    volatile uint32_t  UTIMER15_START_0_SRC;         /*!< (@ 0x00010000) Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER15_START_1_SRC;         /*!< (@ 0x00010004) Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER15_STOP_0_SRC;          /*!< (@ 0x00010008) Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER15_STOP_1_SRC;          /*!< (@ 0x0001000C) Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER15_CLEAR_0_SRC;         /*!< (@ 0x00010010) Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER15_CLEAR_1_SRC;         /*!< (@ 0x00010014) Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER15_UP_0_SRC;            /*!< (@ 0x00010018) Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER15_UP_1_SRC;            /*!< (@ 0x0001001C) Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER15_DOWN_0_SRC;          /*!< (@ 0x00010020) Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER15_DOWN_1_SRC;          /*!< (@ 0x00010024) Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER15_TRIG_CAPTURE_SRC_A_0;/*!< (@ 0x00010028) Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER15_TRIG_CAPTURE_SRC_A_1;/*!< (@ 0x0001002C) Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER15_TRIG_CAPTURE_SRC_B_0;/*!< (@ 0x00010030) Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER15_TRIG_CAPTURE_SRC_B_1;/*!< (@ 0x00010034) Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER15_DMA_CLEAR_SRC_A_0;   /*!< (@ 0x00010038) Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER15_DMA_CLEAR_SRC_A_1;   /*!< (@ 0x0001003C) Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER15_DMA_CLEAR_SRC_B_0;   /*!< (@ 0x00010040) Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER15_DMA_CLEAR_SRC_B_1;   /*!< (@ 0x00010044) Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER15_CNTR_PAUSE_SRC;      /*!< (@ 0x00010048) Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED137[13];
    volatile uint32_t  UTIMER15_CNTR_CTRL;           /*!< (@ 0x00010080) Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER15_FILTER_CTRL_A;       /*!< (@ 0x00010084) Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER15_FILTER_CTRL_B;       /*!< (@ 0x00010088) Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER15_COMPARE_CTRL_A;      /*!< (@ 0x0001008C) Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER15_COMPARE_CTRL_B;      /*!< (@ 0x00010090) Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER15_BUF_OP_CTRL;         /*!< (@ 0x00010094) Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED138[2];
    volatile uint32_t  UTIMER15_CNTR;                /*!< (@ 0x000100A0) Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER15_CNTR_PTR;            /*!< (@ 0x000100A4) Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER15_CNTR_PTR_BUF1;       /*!< (@ 0x000100A8) Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER15_CNTR_PTR_BUF2;       /*!< (@ 0x000100AC) Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER15_CAPTURE_A;           /*!< (@ 0x000100B0) Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER15_CAPTURE_A_BUF1;      /*!< (@ 0x000100B4) Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER15_CAPTURE_A_BUF2;      /*!< (@ 0x000100B8) Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED139;
    volatile uint32_t  UTIMER15_CAPTURE_B;           /*!< (@ 0x000100C0) Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER15_CAPTURE_B_BUF1;      /*!< (@ 0x000100C4) Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER15_CAPTURE_B_BUF2;      /*!< (@ 0x000100C8) Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED140;
    volatile uint32_t  UTIMER15_COMPARE_A;           /*!< (@ 0x000100D0) Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER15_COMPARE_A_BUF1;      /*!< (@ 0x000100D4) Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER15_COMPARE_A_BUF2;      /*!< (@ 0x000100D8) Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED141;
    volatile uint32_t  UTIMER15_COMPARE_B;           /*!< (@ 0x000100E0) Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER15_COMPARE_B_BUF1;      /*!< (@ 0x000100E4) Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER15_COMPARE_B_BUF2;      /*!< (@ 0x000100E8) Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED142;
    volatile uint32_t  UTIMER15_DT_UP;               /*!< (@ 0x000100F0) Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER15_DT_UP_BUF1;          /*!< (@ 0x000100F4) Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER15_DT_DOWN;             /*!< (@ 0x000100F8) Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER15_DT_DOWN_BUF1;        /*!< (@ 0x000100FC) Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED143[5];
    volatile uint32_t  UTIMER15_CHAN_STATUS;         /*!< (@ 0x00010114) Channel (n) Status Register                                */
    volatile uint32_t  UTIMER15_CHAN_INTERRUPT;      /*!< (@ 0x00010118) Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER15_CHAN_INTERRUPT_MASK; /*!< (@ 0x0001011C) Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER15_DUTY_CYCLE_CTRL;     /*!< (@ 0x00010120) Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER15_DEAD_TIME_CTRL;      /*!< (@ 0x00010124) Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED144[2];
    volatile uint32_t  UTIMER15_INT_CNTR_CTRL;       /*!< (@ 0x00010130) Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER15_FAULT_CTRL;          /*!< (@ 0x00010134) Channel (n) Fault Control Register                         */
} UTIMER_Type;                                  /*!< Size = 65848 (0x10138)                                                    */

/** \brief UTIMER Channel specific registers */
typedef struct _UTIMER_CHANNEL_Type {
    volatile uint32_t  UTIMER_START_0_SRC;          /*!< Channel (n) Counter Start Source 0 Register                */
    volatile uint32_t  UTIMER_START_1_SRC;          /*!< Channel (n) Counter Start Source 1 Register                */
    volatile uint32_t  UTIMER_STOP_0_SRC;           /*!< Channel (n) Counter Stop Source 0 Register                 */
    volatile uint32_t  UTIMER_STOP_1_SRC;           /*!< Channel (n) Counter Stop Source 1 Register                 */
    volatile uint32_t  UTIMER_CLEAR_0_SRC;          /*!< Channel (n) Counter Clear Source 0 Register                */
    volatile uint32_t  UTIMER_CLEAR_1_SRC;          /*!< Channel (n) Counter Clear Source 1 Register                */
    volatile uint32_t  UTIMER_UP_0_SRC;             /*!< Channel (n) Counter Up Count Source 0 Register             */
    volatile uint32_t  UTIMER_UP_1_SRC;             /*!< Channel (n) Counter Up Count Source 1 Register             */
    volatile uint32_t  UTIMER_DOWN_0_SRC;           /*!< Channel (n) Counter Down Count Source 0 Register           */
    volatile uint32_t  UTIMER_DOWN_1_SRC;           /*!< Channel (n) Counter Down Count Source 1 Register           */
    volatile uint32_t  UTIMER_TRIG_CAPTURE_SRC_A_0; /*!< Channel (n) Trigger Capture Source A 0 Register            */
    volatile uint32_t  UTIMER_TRIG_CAPTURE_SRC_A_1; /*!< Channel (n) Trigger Capture Source A 1 Register            */
    volatile uint32_t  UTIMER_TRIG_CAPTURE_SRC_B_0; /*!< Channel (n) Trigger Capture Source B 0 Register            */
    volatile uint32_t  UTIMER_TRIG_CAPTURE_SRC_B_1; /*!< Channel (n) Trigger Capture Source B 1 Register            */
    volatile uint32_t  UTIMER_DMA_CLEAR_SRC_A_0;    /*!< Channel (n) DMA Clear Source A 0 Register                  */
    volatile uint32_t  UTIMER_DMA_CLEAR_SRC_A_1;    /*!< Channel (n) DMA Clear Source A 1 Register                  */
    volatile uint32_t  UTIMER_DMA_CLEAR_SRC_B_0;    /*!< Channel (n) DMA Clear Source B 0 Register                  */
    volatile uint32_t  UTIMER_DMA_CLEAR_SRC_B_1;    /*!< Channel (n) DMA Clear Source B 1 Register                  */
    volatile uint32_t  UTIMER_CNTR_PAUSE_SRC;       /*!< Channel (n) Counter Pause Source Register                  */
    volatile uint32_t  RESERVED2[13];
    volatile uint32_t  UTIMER_CNTR_CTRL;            /*!< Channel (n) Counter Control Register                       */
    volatile uint32_t  UTIMER_FILTER_CTRL_A;        /*!< Channel (n) Filter Control A Register                      */
    volatile uint32_t  UTIMER_FILTER_CTRL_B;        /*!< Channel (n) Filter Control B Register                      */
    volatile uint32_t  UTIMER_COMPARE_CTRL_A;       /*!< Channel (n) Compare Control A Register                     */
    volatile uint32_t  UTIMER_COMPARE_CTRL_B;       /*!< Channel (n) Compare Control B Register                     */
    volatile uint32_t  UTIMER_BUF_OP_CTRL;          /*!< Channel (n) Buffer Operation Control Register              */
    volatile uint32_t  RESERVED3[2];
    volatile uint32_t  UTIMER_CNTR;                 /*!< Channel (n) Counter Register                               */
    volatile uint32_t  UTIMER_CNTR_PTR;             /*!< Channel (n) Counter Pointer Register                       */
    volatile uint32_t  UTIMER_CNTR_PTR_BUF1;        /*!< Channel (n) Counter Pointer Buffer 1 Register              */
    volatile uint32_t  UTIMER_CNTR_PTR_BUF2;        /*!< Channel (n) Counter Pointer Buffer 2 Register              */
    volatile uint32_t  UTIMER_CAPTURE_A;            /*!< Channel (n) Capture A Register                             */
    volatile uint32_t  UTIMER_CAPTURE_A_BUF1;       /*!< Channel (n) Capture A Buffer 1 Register                    */
    volatile uint32_t  UTIMER_CAPTURE_A_BUF2;       /*!< Channel (n) Capture A Buffer 2 Register                    */
    volatile uint32_t  RESERVED4;
    volatile uint32_t  UTIMER_CAPTURE_B;            /*!< Channel (n) Capture B Register                             */
    volatile uint32_t  UTIMER_CAPTURE_B_BUF1;       /*!< Channel (n) Capture B Buffer 1 Register                    */
    volatile uint32_t  UTIMER_CAPTURE_B_BUF2;       /*!< Channel (n) Capture B Buffer 2 Register                    */
    volatile uint32_t  RESERVED5;
    volatile uint32_t  UTIMER_COMPARE_A;            /*!< Channel (n) Compare A Register                             */
    volatile uint32_t  UTIMER_COMPARE_A_BUF1;       /*!< Channel (n) Compare A Buffer 1 Register                    */
    volatile uint32_t  UTIMER_COMPARE_A_BUF2;       /*!< Channel (n) Compare A Buffer 2 Register                    */
    volatile uint32_t  RESERVED6;
    volatile uint32_t  UTIMER_COMPARE_B;            /*!< Channel (n) Compare B Register                             */
    volatile uint32_t  UTIMER_COMPARE_B_BUF1;       /*!< Channel (n) Compare B Buffer 1 Register                    */
    volatile uint32_t  UTIMER_COMPARE_B_BUF2;       /*!< Channel (n) Compare B Buffer 2 Register                    */
    volatile uint32_t  RESERVED7;
    volatile uint32_t  UTIMER_DT_UP;                /*!< Channel (n) Dead-time Up Register                          */
    volatile uint32_t  UTIMER_DT_UP_BUF1;           /*!< Channel (n) Dead-time Up Buffer 1 Register                 */
    volatile uint32_t  UTIMER_DT_DOWN;              /*!< Channel (n) Dead-time Down Register                        */
    volatile uint32_t  UTIMER_DT_DOWN_BUF1;         /*!< Channel (n) Dead-time Down Buffer 1 Register               */
    volatile uint32_t  RESERVED8[5];
    volatile uint32_t  UTIMER_CHAN_STATUS;          /*!< Channel (n) Status Register                                */
    volatile uint32_t  UTIMER_CHAN_INTERRUPT;       /*!< Channel (n) Interrupt Control Register                     */
    volatile uint32_t  UTIMER_CHAN_INTERRUPT_MASK;  /*!< Channel (n) Interrupt Mask Register                        */
    volatile uint32_t  UTIMER_DUTY_CYCLE_CTRL;      /*!< Channel (n) Duty Cycle Control Register                    */
    volatile uint32_t  UTIMER_DEAD_TIME_CTRL;       /*!< Channel (n) Dead-time Control Register                     */
    volatile uint32_t  RESERVED9[2];
    volatile uint32_t  UTIMER_INT_CNTR_CTRL;        /*!< Channel (n) Interrupt Counter Control Register             */
    volatile uint32_t  UTIMER_FAULT_CTRL;           /*!< Channel (n) Fault Control Register                         */
    volatile uint32_t  RESERVED10[946];
} UTIMER_CHANNEL_Type;

typedef struct _utimer_channel_config
{
    bool     utimer_mode;                            /**< SET: UTIMER(channel 0-11), CLEAR: QEC(channel 12-15) >*/
    bool     driver_A;                               /**< output drive type A, SET: enabled, CLEAR: disabled >*/
    bool     driver_B;                               /**< output drive type B, SET: enabled, CLEAR: disabled >*/
    bool     dma_ctrl;                               /**< SET: Enable DMA control, CLEAR: Disable DMA control >*/
    bool     fault_type;                             /**< For fault triggers: SET: low until counter stop, CLEAR: until overflow/underflow event >*/
    bool     fixed_buffer;                           /**< SET: Enable Fixed buffer feature, CLEAR: Disable Fixed buffer feature in compare mode >*/
    bool     driver_a_start_state;                   /**< initial state of output driver A, SET: Driver state is HIGH, CLEAR: Driver state is LOW >*/
    bool     driver_a_stop_state;                    /**< end state of output driver A, SET: Driver state is HIGH, CLEAR: Driver state is LOW >*/
    bool     buffering_type;                         /**< Buffering type, SET: Double, CLEAR: Single >*/
    bool     driver_b_start_state;                   /**< initial state of output driver B, SET: Driver state is HIGH, CLEAR: Driver state is LOW >*/
    bool     driver_b_stop_state;                    /**< end state of output driver B, SET: Driver state is HIGH, CLEAR: Driver state is LOW >*/
    bool     comp_buffer_at_crest;                   /**< buffering at crest transfer >*/
    bool     comp_buffer_at_trough;                  /**< buffering at trough transfer >*/
    bool     buffer_operation;                       /**< buffer operation enable for capture/compare/dt modes >*/
    bool     buf_trough_n_crest;                     /**< buffer config for triangle timer- SET: buffer at crest&trough, CLEAR: buffer at trough >*/
    uint8_t  capt_buffer_type_A;                     /**< Only for Capture mode: Buffering type for Drive A Single or Double >*/
    uint8_t  capt_buffer_type_B;                     /**< Only for Capture mode: Buffering type for Drive B Single or Double >*/
    uint8_t  driver_a_at_comp_match;                /**< COMPARE: state of driver A output on count matches with compare buffer values >*/
    uint8_t  driver_a_at_cycle_end;                  /**< state of driver A output on end of cycle, counter_overflow/counter_underflow >*/
    uint8_t  driver_b_at_comp_match;                /**< COMPARE: state of driver B output on count matches with compare buffer values >*/
    uint8_t  driver_b_at_cycle_end;                  /**< state of driver B output on end of cycle, counter_overflow/counter_underflow >*/
    uint8_t  dc_value;                               /**< UTIMER duty cycle value: 0/1: compare match or 2: 0% or 3: 100% >*/
} utimer_channel_config;

/**
  \fn           static inline void utimer_clock_enable (UTIMER_Type *utimer, uint8_t channel)
  \brief        Enable utimer channel clock
  \param[in]    utimer   : Pointer to utimer register block
  \param[in]    channel  : utimer channel number
  \return       none
*/
static inline void utimer_clock_enable (UTIMER_Type *utimer, uint8_t channel)
{
    utimer->UTIMER_GLB_CLOCK_ENABLE |= (1 << channel);
}

/**
  \fn           static inline void utimer_clock_disable (UTIMER_Type *utimer, uint8_t channel)
  \brief        Disable utimer channel clock
  \param[in]    utimer   : Pointer to utimer register block
  \param[in]    channel  : utimer channel number
  \return       none
*/
static inline void utimer_clock_disable (UTIMER_Type *utimer, uint8_t channel)
{
    utimer->UTIMER_GLB_CLOCK_ENABLE &= ~(1 << channel);
}

/**
  \fn           static inline void utimer_control_enable (UTIMER_Type *utimer, uint8_t channel)
  \brief        Enable utimer control
  \param[in]    utimer   : Pointer to utimer register block
  \param[in]    channel  : utimer channel number
  \return       none
*/
static inline void utimer_control_enable (UTIMER_Type *utimer, uint8_t channel)
{
    UTIMER_CHANNEL_Type *utimer_ch = (UTIMER_CHANNEL_Type*) (&utimer->UTIMER0_START_0_SRC) + channel;

    utimer_ch->UTIMER_START_1_SRC = CNTR_SRC1_PGM_EN;
    utimer_ch->UTIMER_STOP_1_SRC = CNTR_SRC1_PGM_EN;
    utimer_ch->UTIMER_CLEAR_1_SRC = CNTR_SRC1_PGM_EN;
}

/**
  \fn           static inline void utimer_control_disable (UTIMER_Type *utimer, uint8_t channel)
  \brief        Disable utimer control
  \param[in]    utimer   : Pointer to utimer register block
  \param[in]    channel  : utimer channel number
  \return       none
*/
static inline void utimer_control_disable (UTIMER_Type *utimer, uint8_t channel)
{
    UTIMER_CHANNEL_Type *utimer_ch = (UTIMER_CHANNEL_Type*) (&utimer->UTIMER0_START_0_SRC) + channel;

    utimer_ch->UTIMER_START_1_SRC = ~CNTR_SRC1_PGM_EN;
    utimer_ch->UTIMER_STOP_1_SRC = ~CNTR_SRC1_PGM_EN;
    utimer_ch->UTIMER_CLEAR_1_SRC = ~CNTR_SRC1_PGM_EN;
}

/**
  \fn           static inline void utimer_driver_output_enable (UTIMER_Type *utimer, uint8_t channel, utimer_channel_config *ch_config)
  \brief        Enable utimer channel driver output
  \param[in]    utimer    : Pointer to utimer register block
  \param[in]    channel   : utimer channel number
  \param[in]    ch_config : pointer for utimer channel configuration structure
  \return       none
*/
static inline void utimer_driver_output_enable (UTIMER_Type *utimer, uint8_t channel, utimer_channel_config *ch_config)
{
    if (ch_config->driver_A)
    {
        utimer->UTIMER_GLB_DRIVER_OEN &= ~(GLB_DRIVER_CHAN_A_OEN << (channel << 1));
    }
    if (ch_config->driver_B)
    {
        utimer->UTIMER_GLB_DRIVER_OEN &= ~(GLB_DRIVER_CHAN_A_OEN << (channel << 1));
    }
}

/**
  \fn           static inline void utimer_driver_output_disable (UTIMER_Type *utimer, uint8_t channel)
  \brief        Disable utimer channel driver output
  \param[in]    utimer    : Pointer to utimer register block
  \param[in]    channel   : utimer channel number
  \return       none
*/
static inline void utimer_driver_output_disable (UTIMER_Type *utimer, uint8_t channel)
{
        utimer->UTIMER_GLB_DRIVER_OEN |= (GLB_DRIVER_CHAN_A_OEN << (channel << 1));
}

/**
  \fn           static inline void utimer_counter_start (UTIMER_Type *utimer, uint8_t channel)
  \brief        Start utimer channel counter
  \param[in]    utimer    : Pointer to utimer register block
  \param[in]    channel   : utimer channel number
  \return       none
*/
static inline void utimer_counter_start (UTIMER_Type *utimer, uint8_t channel)
{
    utimer->UTIMER_GLB_CNTR_START |= (1 << channel);
}

/**
  \fn           static inline bool utimer_counter_running (UTIMER_Type *utimer, uint8_t channel)
  \brief        Read state of utimer channel counter
  \param[in]    utimer    : Pointer to utimer register block
  \param[in]    channel   : utimer channel number
  \return       counter state
*/
static inline bool utimer_counter_running (UTIMER_Type *utimer, uint8_t channel)
{
    return (utimer->UTIMER_GLB_CNTR_RUNNING & (1 << channel)) ? 1 : 0;
}

/**
  \fn           static inline void utimer_counter_stop (UTIMER_Type *utimer, uint8_t channel, bool clear_count)
  \brief        Stop utimer channel counter with counter clear option
  \param[in]    utimer      : Pointer to utimer register block
  \param[in]    channel     : utimer channel number
  \param[in]    clear_count : counter clear option
  \return       none
*/
static inline void utimer_counter_stop (UTIMER_Type *utimer, uint8_t channel, bool clear_count)
{
    utimer->UTIMER_GLB_CNTR_STOP |= (1 << channel);
    if (clear_count)
    {
        utimer->UTIMER_GLB_CNTR_CLEAR |= (1 << channel);
    }
}

/**
  \fn           static inline void utimer_reset (UTIMER_Type *utimer, uint8_t channel)
  \brief        reset utimer channel configurations
  \param[in]    utimer      : Pointer to utimer register block
  \param[in]    channel     : utimer channel number
  \return       none
*/
static inline void utimer_reset (UTIMER_Type *utimer, uint8_t channel)
{
    utimer->UTIMER_GLB_CNTR_START &= ~(1U << channel);
    utimer->UTIMER_GLB_CNTR_STOP  &= ~(1U << channel);
    utimer->UTIMER_GLB_CNTR_CLEAR &= ~(1U << channel);
    utimer->UTIMER_GLB_CLOCK_ENABLE &= ~(1U << channel);
}

/**
  \fn           static inline void utimer_clear_interrupt (UTIMER_Type *utimer, uint8_t channel, uint8_t interrupt)
  \brief        clear utimer channel interrupt
  \param[in]    utimer      : Pointer to utimer register block
  \param[in]    channel     : utimer channel number
  \param[in]    interrupt   : interrupt needs to be cleared
  \return       none
*/
static inline void utimer_clear_interrupt (UTIMER_Type *utimer, uint8_t channel, uint8_t interrupt)
{
    UTIMER_CHANNEL_Type *utimer_ch = (UTIMER_CHANNEL_Type*) (&utimer->UTIMER0_START_0_SRC) + channel;

    utimer_ch->UTIMER_CHAN_INTERRUPT |= interrupt;
}

/**
  \fn           static inline void utimer_unmask_interrupt (UTIMER_Type *utimer, uint8_t channel, uint8_t interrupt)
  \brief        unmask utimer channel interrupt
  \param[in]    utimer      : Pointer to utimer register block
  \param[in]    channel     : utimer channel number
  \param[in]    interrupt   : interrupt needs to be unmasked
  \return       none
*/
static inline void utimer_unmask_interrupt (UTIMER_Type *utimer, uint8_t channel, uint8_t interrupt)
{
    UTIMER_CHANNEL_Type *utimer_ch = (UTIMER_CHANNEL_Type*) (&utimer->UTIMER0_START_0_SRC) + channel;

    utimer_ch->UTIMER_CHAN_INTERRUPT_MASK &= ~interrupt;
}

/**
  \fn           static inline void utimer_mask_interrupt (UTIMER_Type *utimer, uint8_t channel, uint8_t interrupt)
  \brief        mask utimer channel interrupt
  \param[in]    utimer      : Pointer to utimer register block
  \param[in]    channel     : utimer channel number
  \param[in]    interrupt   : interrupt needs to be masked
  \return       none
*/
static inline void utimer_mask_interrupt (UTIMER_Type *utimer, uint8_t channel, uint8_t interrupt)
{
    UTIMER_CHANNEL_Type *utimer_ch = (UTIMER_CHANNEL_Type*) (&utimer->UTIMER0_START_0_SRC) + channel;

    utimer_ch->UTIMER_CHAN_INTERRUPT_MASK |= interrupt;
}

/**
  \fn           static inline void utimer_enable_duty_cycle (UTIMER_Type *utimer, uint8_t channel, utimer_channel_config *ch_config)
  \brief        enable utimer channel duty cycle
  \param[in]    utimer      : Pointer to utimer register block
  \param[in]    channel     : utimer channel number
  \param[in]    ch_config   : pointer for utimer channel configuration structure
  \return       none
*/
static inline void utimer_enable_duty_cycle (UTIMER_Type *utimer, uint8_t channel, utimer_channel_config *ch_config)
{
    UTIMER_CHANNEL_Type *utimer_ch = (UTIMER_CHANNEL_Type*) (&utimer->UTIMER0_START_0_SRC) + channel;

    if (ch_config->driver_A)
    {
        utimer_ch->UTIMER_DUTY_CYCLE_CTRL |= (DUTY_CYCLE_CTRL_DC_ENABLE_A |
                                              DUTY_CYCLE_CTRL_DC_FORCE_A |
                                              DUTY_CYCLE_CTRL_DC_UNDERFLOW_A |
                                              (ch_config->dc_value) << 2);
    }
    if (ch_config->driver_B)
    {
        utimer_ch->UTIMER_DUTY_CYCLE_CTRL |= (DUTY_CYCLE_CTRL_DC_ENABLE_B |
                                              DUTY_CYCLE_CTRL_DC_FORCE_B |
                                              DUTY_CYCLE_CTRL_DC_UNDERFLOW_B |
                                              (ch_config->dc_value) << 10);
    }
}

/**
  \fn          void utimer_config_direction (UTIMER_Type *utimer, uint8_t channel, UTIMER_TYPE type, utimer_channel_config *ch_config)
  \brief       configure counter type for the UTIMER instance.
  \param[in]   utimer      Pointer to the UTIMER register map
  \param[in]   channel     channel number
  \param[in]   dir         counter direction
  \param[in]   ch_config   Pointer to the UTIMER channel specific config structure
  \return      none
*/
void utimer_config_direction (UTIMER_Type *utimer, uint8_t channel, UTIMER_COUNTER_DIR dir, utimer_channel_config *ch_config);

/**
  \fn          void utimer_config_mode (UTIMER_Type *utimer, uint8_t channel, UTIMER_MODE mode, utimer_channel_config *ch_config)
  \brief       configure counter mode for the UTIMER instance.
  \param[in]   utimer      Pointer to the UTIMER register map
  \param[in]   channel     channel number
  \param[in]   mode        counter mode
  \param[in]   ch_config   Pointer to the UTIMER channel specific config structure
  \return      none
*/
void utimer_config_mode (UTIMER_Type *utimer, uint8_t channel, UTIMER_MODE mode, utimer_channel_config *ch_config);

/**
  \fn          void utimer_set_count (UTIMER_Type *utimer, uint8_t channel, UTIMER_SET_COUNTER_TYPE counter_type, utimer_channel_config *ch_config)
  \brief       set counter value for the UTIMER instance.
  \param[in]   utimer          Pointer to the UTIMER register map
  \param[in]   channel         channel number
  \param[in]   counter_type    counter type
  \param[in]   value           counter value
  \return      none
*/
void utimer_set_count (UTIMER_Type *utimer, uint8_t channel, UTIMER_COUNTER counter_type, uint32_t value);

/**
  \fn          uint32_t utimer_get_count (UTIMER_Type *utimer, uint8_t channel, UTIMER_GET_COUNTER_TYPE counter, utimer_channel_config *ch_config)
  \brief       get counter direction for the UTIMER instance.
  \param[in]   utimer         Pointer to the UTIMER register map
  \param[in]   channel        channel number
  \param[in]   counter_type   counter type
  \return      current counter value
*/
uint32_t utimer_get_count (UTIMER_Type *utimer, uint8_t channel, UTIMER_COUNTER counter_type);

/**
  \fn          void utimer_config_trigger (UTIMER_Type *utimer, uint8_t channel, UT_TRIGGER_CONFIG *config_control, utimer_channel_config *ch_config)
  \brief       configure trigger for the UTIMER instance.
  \param[in]   utimer          Pointer to the UTIMER register map
  \param[in]   channel         channel number
  \param[in]   config_control  Pointer to a trigger configure type argument
  \param[in]   ch_config       Pointer to the UTIMER channel specific config structure
  \return      none
*/
void utimer_config_trigger (UTIMER_Type *utimer, uint8_t channel, UTIMER_TRIGGER_CONFIG *config_control, utimer_channel_config *ch_config);

#ifdef __cplusplus
}
#endif

#endif /* UTIMER_H_ */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
