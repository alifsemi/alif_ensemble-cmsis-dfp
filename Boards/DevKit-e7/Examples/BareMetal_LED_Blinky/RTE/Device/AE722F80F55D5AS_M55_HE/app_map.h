/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef APP_MAP_H
#define APP_MAP_H

/* Max size of applications for each core;
 * User shall adjust this based on app need.
 */
#define _APP_MAX_SIZE_HE                 0x100000
#define _APP_MAX_SIZE_HP                 0x100000

/* XIP address for cores;
 * By default M55_HE would boot from 0x8000_0000
 * however, User can choose any other address. 
 */

#define _APP_ADDRESS_HE                  (MRAM_BASE)
#define _APP_ADDRESS_HP                  (MRAM_BASE + _APP_MAX_SIZE_HE)


#endif
