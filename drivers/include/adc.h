/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <stdbool.h>

/* Structure ADC_Type : Register map for ADC */
typedef struct {
  volatile uint32_t  ADC_START_SRC;                /*!< (@ 0x00000000) ADC Start-of-Conversion Source Register                    */
  volatile uint32_t  ADC_COMP_THRESH_A;            /*!< (@ 0x00000004) ADC Comparator Threshold A Register                        */
  volatile uint32_t  ADC_COMP_THRESH_B;            /*!< (@ 0x00000008) ADC Comparator Threshold B Register                        */
  volatile uint32_t  ADC_CLK_DIVISOR;              /*!< (@ 0x0000000C) ADC Clock Divider Value Register                           */
  volatile uint32_t  ADC_INTERRUPT;                /*!< (@ 0x00000010) ADC Interrupt Status and Clear Register                    */
  volatile uint32_t  ADC_INTERRUPT_MASK;           /*!< (@ 0x00000014) ADC Interrupt Mask Register                                */
  volatile uint32_t  ADC_SAMPLE_WIDTH;             /*!< (@ 0x00000018) ADC Sampling Signal Duration Register                      */
  volatile const uint32_t RESERVED;
  volatile uint32_t  ADC_AVG_NUM;                  /*!< (@ 0x00000020) ADC Number of Samples for Averaging Register               */
  volatile uint32_t  ADC_SHIFT_CONTROL;            /*!< (@ 0x00000024) ADC Data Shift Select Register                             */
  volatile const uint32_t RESERVED1[2];
  volatile uint32_t  ADC_CONTROL;                  /*!< (@ 0x00000030) ADC Single-shot Conversion Start and Comparator
                                                         Threshold Mode Register                                    */
  volatile uint32_t  ADC_SEQUENCER_CTRL;           /*!< (@ 0x00000034) ADC Sequencer Control Register                             */
  volatile uint32_t  ADC_REG1;                     /*!< (@ 0x00000038) ADC Analog Control Register for ADC12 Modules              */
  volatile uint32_t  ADC_SEL;                      /*!< (@ 0x0000003C) ADC Sample Register Selected (read-only value of n) */
  volatile const uint32_t RESERVED2[4];
  volatile uint32_t  ADC_SAMPLE_REG_0;             /*!< (@ 0x00000050) ADC Sampled Value From Input n Register                    */
  volatile uint32_t  ADC_SAMPLE_REG_1;             /*!< (@ 0x00000054) ADC Sampled Value From Input n Register                    */
  volatile uint32_t  ADC_SAMPLE_REG_2;             /*!< (@ 0x00000058) ADC Sampled Value From Input n Register                    */
  volatile uint32_t  ADC_SAMPLE_REG_3;             /*!< (@ 0x0000005C) ADC Sampled Value From Input n Register                    */
  volatile uint32_t  ADC_SAMPLE_REG_4;             /*!< (@ 0x00000060) ADC Sampled Value From Input n Register                    */
  volatile uint32_t  ADC_SAMPLE_REG_5;             /*!< (@ 0x00000064) ADC Sampled Value From Input n Register                    */
  volatile uint32_t  ADC_SAMPLE_REG_6;             /*!< (@ 0x00000068) ADC Sampled Value From Input n Register                    */
  volatile uint32_t  ADC_SAMPLE_REG_7;             /*!< (@ 0x0000006C) ADC Sampled Value From Input n Register                    */
  volatile uint32_t  ADC_SAMPLE_REG_8;             /*!< (@ 0x00000070) ADC Sampled Value From Input n Register                    */
} ADC120_Type;                                  /*!< Size = 116 (0x74)                                                         */

/****ADC Register macros****/
#define ADC_START_CONTINUOUS_CONV                (1U << 6)    /* start continuous conversion           */
#define ADC_START_ENABLE                         (1U << 7)    /* Enable ENA for single shot conversion */
#define ADC_START_SINGLE_SHOT_CONV               (1U << 0)    /* start single shot conversion          */
#define ADC_SHIFT_CTRL_RIGHT_OR_LEFT             (1U << 16)   /* Enable the right or left shift        */
#define ADC_SEQUENCER_CTRL_FIXED_OR_ROTATE       (1U << 16)   /* Enable to select particular scan mode */

/****ADC Differential macros****/
#define ADC_DIFFERENTIAL_ENABLE                  (0X01)        /* Enable differential mode              */
#define ADC_MAX_DIFFERENTIAL_CHANNEL             (0X03)        /* Max differential input channel        */

/****ADC24 Differential macros****/
#define ADC24_MAX_DIFFERENTIAL_CHANNEL           (0x04)        /* ADC24 Max differential input channel  */

/****ADC24 sample width macros****/
#define ADC24_SAMPLE_HOLD_ENABLE_Pos             (16)          /* ADC24 enable sample hold bit          */

/****ADC Differential mask macros****/
#define ADC120_DIFFERENTIAL_ENABLE               (1UL << 1)    /* ADC instance 0 differential enable position */
#define ADC121_DIFFERENTIAL_ENABLE               (1UL << 7)    /* ADC instance 1 differential enable position */
#define ADC122_DIFFERENTIAL_ENABLE               (1UL << 13)   /* ADC instance 2 differential enable position */
#define ADC_DIFFERENTIAL_ENABLE_MSK              (ADC120_DIFFERENTIAL_ENABLE |\
                                                  ADC121_DIFFERENTIAL_ENABLE |\
                                                  ADC122_DIFFERENTIAL_ENABLE)

/* ADC external trigger mask macro */
#define ADC_EXTERNAL_TRIGGER_MAX_VAL             (0x3F)        /* ADC External trigger max value */

/* ADC reg1 position macro */
#define ADC120_DIFFERENTIAL_EN_Pos               (1)
#define ADC120_COMPARATOR_EN_Pos                 (2)
#define ADC120_COMPARATOR_BIAS_Pos               (3)
#define ADC120_VCM_DIV_Pos                       (5)

#define ADC121_DIFFERENTIAL_EN_Pos               (7)
#define ADC121_COMPARATOR_EN_Pos                 (8)
#define ADC121_COMPARATOR_BIAS_Pos               (9)
#define ADC121_VCM_DIV_Pos                       (11)

#define ADC122_DIFFERENTIAL_EN_Pos               (13)
#define ADC122_COMPARATOR_EN_Pos                 (14)
#define ADC122_COMPARATOR_BIAS_Pos               (15)
#define ADC122_VCM_DIV_Pos                       (17)

/********Interrupt mask macro*******/
#define ADC_INTR_CMPA_POS                        (2)                                    /* Interrupt comparator A mask position   */
#define ADC_INTR_CMPA_MSK                        (1 << ADC_INTR_CMPA_POS)               /* Interrupt comparator A mask            */
#define ADC_INTR_CMPB_POS                        (3)                                    /* Interrupt comparator B mask position   */
#define ADC_INTR_CMPB_MSK                        (1 << ADC_INTR_CMPB_POS)               /* Interrupt comparator B mask            */
#define ADC_THRSHLD_CMP_MASK_BIT_POS             (16)                                   /* Comparator threshold mask bit position */
#define ADC_THRSHLD_CMP_MASK_BIT                 (0x03 << ADC_THRSHLD_CMP_MASK_BIT_POS) /* Comparator threshold mask bit          */

/****Interrupt clear macros****/
#define ADC_INTR_DONE0_CLEAR                     (0x01)       /* Interrupt done0 clear bit               */
#define ADC_INTR_DONE1_CLEAR                     (0x02)       /* Interrupt done1 clear bit              */
#define ADC_INTR_COMPA_CLEAR                     (0x04)       /* Interrupt compA clear bit              */
#define ADC_INTR_COMPB_CLEAR                     (0x08)       /* Interrupt compB clear bit              */

/****Comparator Macros****/
#define ADC_CMP_THRHLD_ABOVE_A                    0           /* ADC comparator threshold above A       */
#define ADC_CMP_THRHLD_BELOW_A                    1           /* ADC comparator threshold below A       */
#define ADC_CMP_THRHLD_BETWEEN_A_B                2           /* ADC comparator threshold between a_b   */

#define ADC_CMP_THRHLD_ABOVE_B                    0           /* ADC comparator threshold above B       */
#define ADC_CMP_THRHLD_BELOW_B                    1           /* ADC comparator threshold above B       */
#define ADC_CMP_THRHLD_OUTSIDE_A_B                2           /* ADC comparator threshold outside A_B   */

/****channels Macros****/
#define ADC_LAST_AVAILABLE_CHANNEL                8           /* ADC last available channels */

/****limit Macros****/
#define ADC_CLOCK_DIV_MIN_VALUE                   2           /* ADC Clock divisor minimum value        */
#define ADC_CLOCK_DIV_MAX_VALUE                   64          /* ADC Clock divisor maximum value        */
#define ADC_AVG_SAMPLES_FOR_AVG_MIN               2           /* ADC Average sample for Avergae minimum */
#define ADC_AVG_SAMPLES_FOR_AVG_MAX               256         /* ADC Average sample for Avergae maximum */
#define ADC_SAMPLE_WIDTH_MIN_VALUE                2           /* ADC sample width minimum value         */
#define ADC_SAMPLE_WIDTH_MAX_VALUE                32          /* ADC sample width maximum value         */

/****Shift bit macro****/
#define ADC_SHIFT_BIT                             16          /* Shift bit                  */
#define ADC_SEQUENCER_INIT_Pos                    12          /* Initial shift bit position */

/****Sequencer Macros****/
#define ADC_SEQUENCER_MSK_BIT                    (0x01)       /* Sequencer mask bit   */
#define ADC_MAX_INIT_CHANNEL                     (0X100)      /* Initial max channel  */
#define ADC_MSK_INIT_CHANNEL                     (0X0F)       /* Initial mask channel */
#define ADC_MSK_ALL_CHANNELS                     (0X1FF)      /* Masking all channel  */

/****Interrupt****/
#define ADC_INT_AVG_SAMPLE_RDY                    (1U)        /* Interrupt for average sample ready */
#define ADC_INT_AVG_SAMPLE_TAKEN                  (2U)        /* Interrupt for all sample taken     */

/**
 * enum _ADC_SCAN_MODE.
 * Set the scan mode for ADC conversion.
 */
typedef enum _ADC_SCAN_MODE{
    ADC_SCAN_MODE_MULTI_CH,              /* ADC SCAN MODE MULTIPLE CHANNEL */
    ADC_SCAN_MODE_SINGLE_CH              /* ADC SCAN MODE SINGLE CHANNEL   */
} ADC_SCAN_MODE;

/**
 * enum ADC_CONV_MODE.
 * Set the conversion mode for ADC.
 */
typedef enum _ADC_CONV_MODE{
    ADC_CONV_MODE_CONTINUOUS,            /* ADC CONVERSRION MODE CONTINUOUS  */
    ADC_CONV_MODE_SINGLE_SHOT            /* ADC CONVERSRION MODE SINGLE SHOT */
}ADC_CONV_MODE;

/**
 * enum ADC_CONV_STAT.
 * Status of an ongoing ADC conversion.
 */
typedef enum _ADC_CONV_STAT{
  ADC_CONV_STAT_NONE,                              /* ADC Conversion status none                               */
  ADC_CONV_STAT_COMPLETE             = (1U << 0),  /* ADC Conversion status complete                           */
  ADC_CONV_STAT_CMP_THLD_ABOVE_A     = (1U << 1),  /* ADC Conversion status comparator threshold above A       */
  ADC_CONV_STAT_CMP_THLD_ABOVE_B     = (1U << 2),  /* ADC Conversion status comparator threshold above B       */
  ADC_CONV_STAT_CMP_THLD_BELOW_A     = (1U << 3),  /* ADC Conversion status comparator threshold below A       */
  ADC_CONV_STAT_CMP_THLD_BELOW_B     = (1U << 4),  /* ADC Conversion status comparator threshold below B       */
  ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B = (1U << 5),  /* ADC Conversion status comparator threshold between A & B */
  ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B = (1U << 6),  /* ADC Conversion status comparator threshold outside A & B */
} ADC_CONV_STAT;

/* Structure to store the conversion info */
typedef struct conv_info{
  const uint32_t                  user_input;             /* user channel input             */
  volatile uint32_t               sampled_value;          /* Conversion value stored        */
  volatile uint32_t               curr_channel;           /* conversion current channel     */
  ADC_SCAN_MODE                   sequencer_ctrl_status;  /* sequencer control status       */
  volatile ADC_CONV_STAT          status;                 /* Conversion status              */
  volatile ADC_CONV_MODE          mode;                   /* Conversion status control      */
  bool                            differential_status;    /* ADC Differential enable status */
  volatile uint8_t                read_channel;           /* Store channel                  */
}conv_info_t;

/**
 * @fn        : void adc_enable_single_shot_conv(ADC120_Type *adc)
 * @brief     : Enable the single shot conversion
 * @param[in] : adc : Pointer to the ADC register map
 * @return    : none
*/
static inline void adc_enable_single_shot_conv(ADC120_Type *adc)
{
    adc->ADC_START_SRC |= ADC_START_ENABLE;

    adc->ADC_CONTROL |= ADC_START_SINGLE_SHOT_CONV;
}

/**
 * @fn        : void adc_disable_single_shot_conv(ADC120_Type *adc)
 * @brief     : disable the single shot conversion
 * @param[in] : adc : Pointer to the ADC register map
 * @return    : none
*/
static inline void adc_disable_single_shot_conv(ADC120_Type *adc)
{
    adc->ADC_START_SRC |= ~(ADC_START_ENABLE);

    adc->ADC_CONTROL &= ~ADC_START_SINGLE_SHOT_CONV;
}

/**
 * @fn        : void adc_enable_continuous_conv(ADC120_Type *adc)
 * @brief     : Enable the continuous conversion
 * @param[in] : adc : Pointer to the ADC register map
 * @return    : none
*/
static inline void adc_enable_continuous_conv(ADC120_Type *adc)
{
    adc->ADC_START_SRC |= ADC_START_CONTINUOUS_CONV;
}

/**
 * @fn        : void adc_disable_continuous_conv(ADC120_Type *adc)
 * @brief     : Disable the continuous conversion
 * @param[in] : adc : Pointer to the ADC register map
 * @return    : none
*/
static inline void adc_disable_continuous_conv(ADC120_Type *adc)
{
    adc->ADC_START_SRC &= ~ADC_START_CONTINUOUS_CONV;
}

/**
 * @fn        : void adc_enable_external_trigger(ADC120_Type *adc, uint8_t arg)
 * @brief     : Enable external trigger from UTIMER and QEC
 * @param[in] : adc : Pointer to the ADC register map
 * @param[in] : arg : Enable trigger bit fields
 * @return    : none
*/
static inline void adc_enable_external_trigger(ADC120_Type *adc, uint8_t arg)
{
    adc->ADC_START_SRC |= arg;
}

/**
 * @fn        : void adc_disable_external_trigger(ADC120_Type *adc)
 * @brief     : Disable conversion from external trigger from UTIMER and QEC
 * @param[in] : adc : Pointer to the ADC register map
 * @param[in] : arg : Disable trigger bit fields
 * @return    : none
*/
static inline void adc_disable_external_trigger(ADC120_Type *adc, uint8_t arg)
{
    adc->ADC_START_SRC &= ~arg;

}

/*
 * @func         : void adc_init_channel_select(ADC120_Type *adc, uint32_t arg)
 * @brief        : control to select initial channel for storing sample value
 * @parameter[1] : adc  : Pointer to the ADC register map
 * @parameter[2] : arg  : selecting initial channel
 * @return       : ARM_DRIVER_OK              : if driver initialized successfully
 *               : ARM_DRIVER_ERROR_PARAMETER : if parameter is invalid or not
 */
static inline void adc_init_channel_select(ADC120_Type *adc, uint32_t channel)
{
    /* clearing the channels */
    adc->ADC_SEQUENCER_CTRL &= ~(ADC_MSK_INIT_CHANNEL << ADC_SEQUENCER_INIT_Pos);

    /* masking the channels */
    adc->ADC_SEQUENCER_CTRL |= (channel << ADC_SEQUENCER_INIT_Pos);
}

/*
 * @func         : void adc_set_clock_divisor(ADC120_Type *adc, uint32_t divisor)
 * @brief        : Setting the value for the clock divisor and
 *                  clock value should be between 2 to 64
 * @parameter[1] : adc  : Pointer to the ADC register map
 * @parameter[2] : clock_value : value to set clock divisor
 * @return       : NONE
 */
static inline void adc_set_clk_div(ADC120_Type *adc, uint32_t divisor)
{
    adc->ADC_CLK_DIVISOR = divisor;
}

/*
 * @func         : void adc_set_avg_sample(ADC120_Type *adc, uint32_t average)
 * @brief        : Setting the average value, the value must be
 *                 up to 256 and value should be power of 2
 * @parameter[1] : adc  : Pointer to the ADC register map
 * @parameter[2] : average : value to set average number
 * @return       : NONE
 */
static inline void adc_set_avg_sample(ADC120_Type *adc, uint32_t average)
{
    adc->ADC_AVG_NUM = average;
}

/*
 * @func         : void adc_set_sample_width(ADC120_Type *adc, uint32_t width)
 * @brief        : Setting the sample width and value must be between
 *                 2 to 32
 * @parameter[1] : adc   : Pointer to the ADC register map
 * @parameter[2] : width : value to set sample width
 * @return       : NONE
 */
static inline void adc_set_sample_width(ADC120_Type *adc, uint32_t width)
{
    adc->ADC_SAMPLE_WIDTH = width;
}

/*
 * @func         : void set_adc24_sample_width(ADC120_Type *adc)
 * @brief        : Set sample hold bit for adc24
 * @parameter[1] : adc   : Pointer to the ADC register map
 * @return       : NONE
 */
static inline void set_adc24_sample_width(ADC120_Type *adc)
{
    adc->ADC_SAMPLE_WIDTH = (1 << ADC24_SAMPLE_HOLD_ENABLE_Pos);
}

/*
 * @func         : void adc_set_n_shift_bit(ADC120_Type *adc,
                                            uint32_t shift_number,
                                            uint32_t shift_left_right_control)
 * @brief        : Setting the number of shift to bit (as per user input)
 * @parameter[1] : adc                      : Pointer to the ADC register map
 * @parameter[2] : shift number             : number of bytes to shift
 * @parameter[3] : shift_left_right_control : enable shift control 0 for left
 *                                            1 for right
 *    @return       : NONE
*/
static inline void adc_set_n_shift_bit(ADC120_Type *adc,
                                       uint32_t shift_number,
                                       uint32_t shift_left_right_control)
{
    adc->ADC_SHIFT_CONTROL = (shift_number | shift_left_right_control << 16);
}

/*
 * @func         : void adc_unmask_interrupt(ADC120_Type *adc)
 * @brief        : Enable the interrupts
 * @parameter[1] : adc  : Pointer to the ADC register map
 * @return       : NONE
*/
static inline void adc_unmask_interrupt(ADC120_Type *adc)
{
    adc->ADC_INTERRUPT_MASK = 0x0;
}

/*
 * @func         : void adc_mask_interrupt(ADC120_Type *adc)
 * @brief        : Disable the interrupts
 * @parameter[1] : adc  : Pointer to the ADC register map
 * @return       : NONE
*/
static inline void adc_mask_interrupt(ADC120_Type *adc)
{
    adc->ADC_INTERRUPT_MASK = 0xF;
}

/*
 * @func         : void adc_sequencer_msk_ch_control(ADC120_Type *adc, uint32_t mask_channel)
 * @brief        : Masking the channel which are not required
 * @parameter[1] : adc  : Pointer to the ADC register map
 * @parameter[2] : arg  : value for masking the un-required channel
 * @return       : NONE
 */
static inline void adc_sequencer_msk_ch_control(ADC120_Type *adc, uint32_t mask_channel)
{
    /* clearing the previous mask bits */
    adc->ADC_SEQUENCER_CTRL &= ~(ADC_MSK_ALL_CHANNELS);

    /* masking the channels */
    adc->ADC_SEQUENCER_CTRL |= mask_channel;
}

/*
 * @func         : void adc_set_comparator_A(ADC120_Type *adc, uint32_t threshold)
 * @brief        : setting comparator A threshold value
 * @parameter[1] : adc  : Pointer to the ADC register map
 * @parameter[2] : arg  : value for threshold
 * @return       : NONE
 */
static inline void adc_set_comparator_A(ADC120_Type *adc, uint32_t threshold)
{
    adc->ADC_COMP_THRESH_A = threshold;
}

/*
 * @func         : void adc_set_comparator_B(ADC120_Type *adc, uint32_t threshold)
 * @brief        : setting comparator A threshold value
 * @parameter[1] : adc  : Pointer to the ADC register map
 * @parameter[2] : arg  : value for threshold
 * @return       : NONE
 */
static inline void adc_set_comparator_B(ADC120_Type *adc, uint32_t threshold)
{
    adc->ADC_COMP_THRESH_B = threshold;
}

/*
 * @func         : void adc_set_comparator_ctrl_bit(ADC120_Type *adc, uint32_t arg)
 * @brief        : setting comparator control bit
 * @parameter[1] : adc  : Pointer to the ADC register map
 * @parameter[2] : arg  : value for threshold
 * @return       : NONE
 */
static inline void adc_set_comparator_ctrl_bit(ADC120_Type *adc, uint32_t arg)
{
    adc->ADC_CONTROL = (arg << 16);
}

/*
 * @func         : void adc_output_right_shift(ADC120_Type *adc)
 * @brief        : control for right shift of bit
 * @parameter[1] : adc  : Pointer to the ADC register map
 * @return       : NONE
 */
static inline void adc_output_right_shift(ADC120_Type *adc)
{
    adc->ADC_SHIFT_CONTROL |= (ADC_SHIFT_CTRL_RIGHT_OR_LEFT);
}

/*
 * @func         : void adc_output_left_shift(ADC120_Type *adc)
 * @brief        : control for left shift of bit
 * @parameter    : adc  : Pointer to the ADC register map
 * @return       : NONE
 */
static inline void adc_output_left_shift(ADC120_Type *adc)
{
    adc->ADC_SHIFT_CONTROL &= ~(ADC_SHIFT_CTRL_RIGHT_OR_LEFT);
}

/*
 * @func         : void adc_set_single_ch_scan_mode(ADC120_Type *adc, conv_info_t *conv_info)
 * @brief        : control to single channel scan
 * @parameter[1] : adc       : Pointer to the ADC register map
 * @parameter[2] : conv_info : Pointer to the conv_info_t structure
 * @return       : NONE
 */
static inline void adc_set_single_ch_scan_mode(ADC120_Type *adc, conv_info_t *conv_info)
{
    /* set to single input scan */
    adc->ADC_SEQUENCER_CTRL |= (ADC_SEQUENCER_CTRL_FIXED_OR_ROTATE);
    conv_info->sequencer_ctrl_status = ADC_SCAN_MODE_SINGLE_CH;
}

/*
 * @func         : void adc_set_multi_ch_scan_mode(ADC120_Type *adc, conv_info_t *conv_info)
 * @brief        : control to rotate through all channels
 * @parameter[1] : adc       : Pointer to the ADC register map
 * @parameter[2] : conv_info : Pointer to the conv_info_t structure
 * @return       : NONE
 */
static inline void adc_set_multi_ch_scan_mode(ADC120_Type *adc, conv_info_t *conv_info)
{
    /* Set to continuous input scan */
    adc->ADC_SEQUENCER_CTRL &= ~(ADC_SEQUENCER_CTRL_FIXED_OR_ROTATE);
    conv_info->sequencer_ctrl_status = ADC_SCAN_MODE_MULTI_CH;
}

/**
 * @fn       : void adc_irq_handler(ADC120_Type *adc, conv_info_t *conversion)
 * @brief    : Handle DONE0 (avg sample ready)interrupts for the ADC instance.
 * @param[1] : adc        : Pointer to the ADC register map
 * @param[2] : conversion : The conversion structure for the ADC instance
 * @return   : none
*/
void adc_done0_irq_handler(ADC120_Type *adc, conv_info_t *conversion);

/**
 * @fn       : void adc_done1_irq_handler(ADC120_Type *adc, conv_info_t *conversion)
 * @brief    : Handle DONE1 (all sample taken)interrupts for the ADC instance.
 * @param[1] : adc        : Pointer to the ADC register map
 * @param[2] : conversion : The conversion structure for the ADC instance
 * @return   : none
*/
void adc_done1_irq_handler(ADC120_Type *adc, conv_info_t *conversion);

/**
 * @fn       : void adc_cmpa_irq_handler(ADC120_Type *adc, conv_info_t *conversion)
 * @brief    : Handle CMPA interrupts for the ADC instance.
 * @param[1] : adc        : Pointer to the ADC register map
 * @param[2] : conversion : The conversion structure for the ADC instance
 * @return   : none
*/
void adc_cmpa_irq_handler(ADC120_Type *adc, conv_info_t *conversion);

/**
 * @fn       : void adc_cmpb_irq_handler(ADC120_Type *adc, conv_info_t *conversion)
 * @brief    : Handle CMPB interrupts for the ADC instance.
 * @param[1] : adc        : Pointer to the ADC register map
 * @param[2] : conversion : The conversion structure for the ADC instance
 * @return   : none
*/
void adc_cmpb_irq_handler(ADC120_Type *adc, conv_info_t *conversion);

#endif /* ADC_H_ */
