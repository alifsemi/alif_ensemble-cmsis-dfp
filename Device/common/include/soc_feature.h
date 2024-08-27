/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SOC_FEATURE_H
#define SOC_FEATURE_H

#ifdef AE722F80F55D5XX
#include "AE722F80F55D5XX.h"
#elif defined  AE512F80F55D5XX
#include "AE512F80F55D5XX.h"
#elif defined AE512F80F5582XX
#include "AE512F80F5582XX.h"
#elif defined AE302F80F55D5XX
#include "AE302F80F55D5XX.h"
#elif defined AE302F80F5582XX
#include "AE302F80F5582XX.h"
#elif defined AE302F80C1557LE
#include "AE302F80C1557LE.h"
#elif defined AE302F40C1537LE
#include "AE302F40C1537LE.h"
#elif defined AE101F4071542LH
#include "AE101F4071542LH.h"
#elif defined AE1C1F4051920PH
#include "AE1C1F4051920PH.h"
#elif defined AE1C1F4051920HH
#include "AE1C1F4051920HH.h"
#elif defined AE1C1F40319205H
#include "AE1C1F40319205H.h"
#elif defined AE1C1F1041010XX
#include "AE1C1F1041010XX.h"
#elif defined AE1C1F1040505XX
#include "AE1C1F1040505XX.h"
#else
#error "Error: SOC Features not defined. Please check!"
#endif

#endif  /* SOC_FEATURE_H */
