/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * @Use, distribution and modification of this code is permitted under the
 * @terms stated in the Alif Semiconductor Software License Agreement
 *
 * @You should have received a copy of the Alif Semiconductor Software
 * @License Agreement with this file. If not, please write to:
 * @contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef OSPI_DELAY_H_
#define OSPI_DELAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define OSPI_DELAY_INVALID_IDX (0xFFFFFFFFU)
/* OSPI delay configuration: 16 individual per-line delay taps for each of the
 * TXD, RXD and SSI output-enable (OE_N) paths, plus the two RXDS strobe delays
 * and the two TXD data-mask (DM) data delays. */

// Pack the structure so that it can be serialized to a binary format for storage
// Structure is named with ospi prefix due to contextual reasons.
// The related registers are in the AES address space.
#pragma pack(push, 1)
typedef struct ospi_delay_cfg {
    uint32_t idx;       /* OSPI controller the config belongs to (0 or 1) */
    uint32_t sclk_freq; /* OSPI controller SCLK frequency (calibration was done with this frequency) */
    uint8_t txd[16];    /* per-line TXD delay taps (index 0..15)     */
    uint8_t rxd[16];    /* per-line RXD delay taps (index 0..15)     */
    uint8_t ssioen[16]; /* per-line OE_N delay taps (index 0..15)    */
    uint8_t rxds[2];    /* RXDS strobe delays (index 0..1)           */
    uint8_t txddm[2];   /* TXD DM data delays (index 0..1)           */
    uint8_t dmoen[2];   /* DM OE_N delays (index 0..1) */
    uint8_t sclk;       /* SCLK delay */
    uint8_t sclkn;      /* SCLK_N delay */
    uint8_t ssn[2];     /* SS_N delay (index 0..1) */
} ospi_delay_cfg_t;
#pragma pack(pop)

#ifdef __cplusplus
}
#endif
#endif /* OSPI_DELAY_H_ */