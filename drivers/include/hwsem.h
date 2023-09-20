/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef HWSEM_H_
#define HWSEM_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef struct {
    volatile uint32_t       HWSEM0_REQ_REG;               /*!< (@ 0x00000000) Request register                                           */
    volatile uint32_t       HWSEM0_REL_REG;               /*!< (@ 0x00000004) Release register                                           */
    volatile  uint32_t      HWSEM0_RST_REG;               /*!< (@ 0x00000008) Reset register                                             */
    const volatile uint32_t RESERVED;
    volatile uint32_t       HWSEM1_REQ_REG;               /*!< (@ 0x00000010) Request register                                           */
    volatile uint32_t       HWSEM1_REL_REG;               /*!< (@ 0x00000014) Release register                                           */
    volatile  uint32_t      HWSEM1_RST_REG;               /*!< (@ 0x00000018) Reset register                                             */
    const volatile uint32_t RESERVED1;
    volatile uint32_t       HWSEM2_REQ_REG;               /*!< (@ 0x00000020) Request register                                           */
    volatile uint32_t       HWSEM2_REL_REG;               /*!< (@ 0x00000024) Release register                                           */
    volatile  uint32_t      HWSEM2_RST_REG;               /*!< (@ 0x00000028) Reset register                                             */
    const volatile uint32_t RESERVED2;
    volatile uint32_t       HWSEM3_REQ_REG;               /*!< (@ 0x00000030) Request register                                           */
    volatile uint32_t       HWSEM3_REL_REG;               /*!< (@ 0x00000034) Release register                                           */
    volatile  uint32_t      HWSEM3_RST_REG;               /*!< (@ 0x00000038) Reset register                                             */
    const volatile uint32_t RESERVED3;
    volatile uint32_t       HWSEM4_REQ_REG;               /*!< (@ 0x00000040) Request register                                           */
    volatile uint32_t       HWSEM4_REL_REG;               /*!< (@ 0x00000044) Release register                                           */
    volatile  uint32_t      HWSEM4_RST_REG;               /*!< (@ 0x00000048) Reset register                                             */
    const volatile uint32_t RESERVED4;
    volatile uint32_t       HWSEM5_REQ_REG;               /*!< (@ 0x00000050) Request register                                           */
    volatile uint32_t       HWSEM5_REL_REG;               /*!< (@ 0x00000054) Release register                                           */
    volatile  uint32_t      HWSEM5_RST_REG;               /*!< (@ 0x00000058) Reset register                                             */
    const volatile uint32_t RESERVED5;
    volatile uint32_t       HWSEM6_REQ_REG;               /*!< (@ 0x00000060) Request register                                           */
    volatile uint32_t       HWSEM6_REL_REG;               /*!< (@ 0x00000064) Release register                                           */
    volatile  uint32_t      HWSEM6_RST_REG;               /*!< (@ 0x00000068) Reset register                                             */
    const volatile uint32_t RESERVED6;
    volatile uint32_t       HWSEM7_REQ_REG;               /*!< (@ 0x00000070) Request register                                           */
    volatile uint32_t       HWSEM7_REL_REG;               /*!< (@ 0x00000074) Release register                                           */
    volatile  uint32_t      HWSEM7_RST_REG;               /*!< (@ 0x00000078) Reset register                                             */
    const volatile uint32_t RESERVED7;
    volatile uint32_t       HWSEM8_REQ_REG;               /*!< (@ 0x00000080) Request register                                           */
    volatile uint32_t       HWSEM8_REL_REG;               /*!< (@ 0x00000084) Release register                                           */
    volatile  uint32_t      HWSEM8_RST_REG;               /*!< (@ 0x00000088) Reset register                                             */
    const volatile uint32_t RESERVED8;
    volatile uint32_t       HWSEM9_REQ_REG;               /*!< (@ 0x00000090) Request register                                           */
    volatile uint32_t       HWSEM9_REL_REG;               /*!< (@ 0x00000094) Release register                                           */
    volatile  uint32_t      HWSEM9_RST_REG;               /*!< (@ 0x00000098) Reset register                                             */
    const volatile uint32_t RESERVED9;
    volatile uint32_t       HWSEM10_REQ_REG;              /*!< (@ 0x000000A0) Request register                                           */
    volatile uint32_t       HWSEM10_REL_REG;              /*!< (@ 0x000000A4) Release register                                           */
    volatile  uint32_t      HWSEM10_RST_REG;              /*!< (@ 0x000000A8) Reset register                                             */
    const volatile uint32_t RESERVED10;
    volatile uint32_t       HWSEM11_REQ_REG;              /*!< (@ 0x000000B0) Request register                                           */
    volatile uint32_t       HWSEM11_REL_REG;              /*!< (@ 0x000000B4) Release register                                           */
    volatile  uint32_t      HWSEM11_RST_REG;              /*!< (@ 0x000000B8) Reset register                                             */
    const volatile uint32_t RESERVED11;
    volatile uint32_t       HWSEM12_REQ_REG;              /*!< (@ 0x000000C0) Request register                                           */
    volatile uint32_t       HWSEM12_REL_REG;              /*!< (@ 0x000000C4) Release register                                           */
    volatile  uint32_t      HWSEM12_RST_REG;              /*!< (@ 0x000000C8) Reset register                                             */
    const volatile uint32_t RESERVED12;
    volatile uint32_t       HWSEM13_REQ_REG;              /*!< (@ 0x000000D0) Request register                                           */
    volatile uint32_t       HWSEM13_REL_REG;              /*!< (@ 0x000000D4) Release register                                           */
    volatile  uint32_t      HWSEM13_RST_REG;              /*!< (@ 0x000000D8) Reset register                                             */
    const volatile uint32_t RESERVED13;
    volatile uint32_t       HWSEM14_REQ_REG;              /*!< (@ 0x000000E0) Request register                                           */
    volatile uint32_t       HWSEM14_REL_REG;              /*!< (@ 0x000000E4) Release register                                           */
    volatile  uint32_t      HWSEM14_RST_REG;              /*!< (@ 0x000000E8) Reset register                                             */
    const volatile uint32_t RESERVED14;
    volatile uint32_t       HWSEM15_REQ_REG;              /*!< (@ 0x000000F0) Request register                                           */
    volatile uint32_t       HWSEM15_REL_REG;              /*!< (@ 0x000000F4) Release register                                           */
    volatile  uint32_t      HWSEM15_RST_REG;              /*!< (@ 0x000000F8) Reset register                                             */
} HWSEM_Type;

typedef struct _HWSEM_INSTANCE_Type {
    volatile uint32_t       HWSEM_REQ_REG;              /*!< (@ 0x00000000) Request register                                             */
    volatile uint32_t       HWSEM_REL_REG;              /*!< (@ 0x00000004) Release register                                             */
    volatile uint32_t       HWSEM_RST_REG;              /*!< (@ 0x00000008) Reset register                                               */
    const volatile uint32_t RESERVED;
} HWSEM_INSTANCE_Type;

/**
  \fn          static inline uint32_t hwsem_request(HWSEM_Type *hwsem, uint8_t instance, uint32_t master_id)
  \brief       Request ownership of a hwsem instance.
  \param[in]   hwsem     Pointer to the HWSEM register map
  \param[in]   instance  The hwsem instance
  \param[in]   master_id The master id to be used to request the ownership
  \return      Current master id. Will be same as the parameter master_id if the request was successful.
*/
static inline uint32_t hwsem_request(HWSEM_Type *hwsem, uint8_t instance, uint32_t master_id)
{
    HWSEM_INSTANCE_Type *hwsem_instance = (HWSEM_INSTANCE_Type *) hwsem + instance;

    /* Write the master_id to the request register */
    hwsem_instance->HWSEM_REQ_REG = master_id;

    /* Read back and return the value */
    return hwsem_instance->HWSEM_REQ_REG;
}

/**
  \fn          static inline uint32_t hwsem_getcount(HWSEM_Type *hwsem, uint8_t instance)
  \brief       Get the count for a hwsem instance.
  \param[in]   hwsem     Pointer to the HWSEM register map
  \param[in]   instance  The hwsem instance
  \return      HWSEM count
*/
static inline uint32_t hwsem_getcount(HWSEM_Type *hwsem, uint8_t instance)
{
    HWSEM_INSTANCE_Type *hwsem_instance = (HWSEM_INSTANCE_Type *) hwsem + instance;
    return hwsem_instance->HWSEM_REL_REG;
}

/**
  \fn          static inline void hwsem_release(HWSEM_Type *hwsem, uint8_t instance, uint32_t master_id)
  \brief       Release ownership of a hwsem instance.
  \param[in]   hwsem     Pointer to the HWSEM register map
  \param[in]   instance  The hwsem instance
  \param[in]   master_id The master id to be used to release ownership
  \return      none
*/
static inline void hwsem_release(HWSEM_Type *hwsem, uint8_t instance, uint32_t master_id)
{
    HWSEM_INSTANCE_Type *hwsem_instance = (HWSEM_INSTANCE_Type *) hwsem + instance;
    hwsem_instance->HWSEM_REL_REG = master_id;
}

/**
  \fn          static void hwsem_reset(HWSEM_Type *hwsem, uint8_t instance)
  \brief       Reset a hwsem instance.
  \param[in]   hwsem     Pointer to the HWSEM register map
  \param[in]   instance  The hwsem instance
  \return      none
*/
static void hwsem_reset(HWSEM_Type *hwsem, uint8_t instance)
{
    HWSEM_INSTANCE_Type *hwsem_instance = (HWSEM_INSTANCE_Type *) hwsem + instance;
    hwsem_instance->HWSEM_RST_REG = 0x1U;
}

#ifdef __cplusplus
}
#endif
#endif /* HWSEM_H_ */
