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
 * @file     WiFi_we310f5.h
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    Wi-Fi Specific defines and data types
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#ifndef WIFI_WE310F5_H
#define WIFI_WE310F5_H

#ifdef  __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "Driver_Common.h"
#include "Driver_WiFi.h"
#include "Driver_SPI.h"
#include "Driver_GPIO.h"

/****** Additional WiFi SetOption/GetOption Function Option Codes *****/
#define PARAM_CNT                           (22U)

#define ARM_WIFI_CONNECTION_TYPE            (23U)       ///< Station/AP Set/Get STA/AP mode;        data = &interface,         len =  1, uint8_t[1]
#define ARM_WIFI_SCAN_TIME                  (24U)       ///< Station/AP Scan Time Configuration;    data = &scanTime,          len =  2, uint8_t[2]
#define ARM_WIFI_TX_RATE                    (25U)       ///< Station/AP Set/Get Transmission Rate;  data = &txRate,            len =  1, uint8_t[1]
#define ARM_WIFI_COUNTRY_CODE               (26U)       ///< Station/AP Set/Get Country Code;       data = &countryCode,       len =  2, uint8_t[2]
#define ARM_WIFI_REGULATORY_DOMAIN          (27U)       ///< Station/AP Set/Get regulatoryDomain;   data = &regulatoryDomain,  len =  5, uint8_t[5]
#define ARM_WIFI_PHYSICAL_MODE              (28U)       ///< Station/AP Set/Get physicalMode;       data = &physicalMode,      len =  3, uint8_t[3]
#define ARM_WIFI_POWER_SAVE                 (29U)       ///< Station/AP Set/Get powerSave;          data = &countryCode,       len =  1, uint8_t[1]
#define ARM_WIFI_IDLE_TIME                  (30U)       ///< Not Supported
#define ARM_WIFI_PS_POLL_POLICY             (31U)       ///< Station/AP Set/Get psPollpolicy;       data = &countryCode,       len =  1, uint8_t[1]
#define ARM_WIFI_DTIM_POLICY                (32U)       ///< Station/AP Set/Get dtimPolicy;         data = &countryCode,       len =  1, uint8_t[1]
#define ARM_WIFI_TX_WAKEUP                  (33U)       ///< Not Supported
#define ARM_WIFI_SSID                       (34U)       ///< Station/AP Set/Get SSID;               data = &countryCode,       len = 64, uint8_t[64]
#define ARM_WIFI_BANDWIDTH                  (35U)       ///< Station/AP Set/Get Bandwidth;          data = &bandwidth,         len =  1, uint8_t[1]
#define ARM_WIFI_CONCURRENT_MODE            (36U)       ///< Station/AP Set/Get Concurrent;         data = &wifiMode,          len =  1, uint8_t[1]

/**
\brief IP Info Information
*/
typedef struct _IP_CONFIG_INFO_s {
  uint8_t   ip[4]     ;             ///< IP Address (IPV4)
  uint8_t   subNetmask[4];          ///< netmask
  uint8_t   gateway[4];             ///< gateway
  uint8_t   ipV6[6]   ;             ///< IP address (IPV6)
  uint32_t  ipFlag;                 ///< 1 - Static IPV4 mode; 2 - DHCP IPV4 mode; 5 - Both Static IPV4 and Auto IPV6
  uint8_t   hostName[16+1];         ///
} IP_CONFIG_INFO_t;

/**
 \brief WIFI driver options
 */
typedef struct {
    IP_CONFIG_INFO_t    ipCfg;                  /// IP Configuration
    uint8_t             bssid[6];               ///< BSSID (AP MAC address)
    uint8_t             mac[6];                 ///< MAC address.
    uint16_t            scanTime;               ///< 5-1500 msec
    uint8_t             txPower;                ///< Tx Power; 0=>100%, 1=>75%, 2=>50%, 3=>25%
                                                ///<           4=>12.5%
    uint8_t             txRate;                 ///< Transmission Rate;                10 => 36 Mbps (Not Supported)
                                                ///< 1 => 1 Mbps                       11 => 48 Mbps (Not Supported)
                                                ///< 2 => 2 Mbps   (Not Supported)     12 => 54 Mbps
                                                ///< 3 => 5.5 Mbps (Not Supported)     13 => MCS0     (Not Supported)
                                                ///< 4 => 6 Mbps   (Not Supported)     14 => MCS1     (Not Supported)
                                                ///< 5 => 9 Mbps   (Not Supported)     15 => MCS2     (Not Supported)
                                                ///< 6 => 11 Mbps                      16 => MCS3     (Not Supported)
                                                ///< 7 => 12 Mbps                      17 => MCS4     (Not Supported)
                                                ///< 8 => 18 Mbps (Not Supported)      18 => MCS5     (Not Supported)
                                                ///< 9 => 24 Mbps (Not Supported)      19 => MCS6     (Not Supported)
                                                ///<                                   20 => MCS7
    uint8_t             countryCode[2];         ///< "IN","US","JP" etc Refer AT Document (Appendix C - List of Country Code)
    uint8_t             regulatoryDomain[5];    ///< "ETSI", "FCC"and "TELEC"
    uint8_t             physicalMode[3];        ///< "B", "G", "N" , "BG","BGN"
    bool                powerSave;              ///< 1: Enable, 0: Disable
    uint32_t            idleTime;               ///< time (in ms) taken by the WLAN driver to enter power save mode
                                                ///< after device connection - Not supported
    bool                psPollpolicy;           ///< 0 - Disable, 1 - Enable
    bool                dtimPolicy;             ///< 0 - Disable, 1 - Enable
    uint32_t            txNwakeup;              ///< number of contiguous packet transmissions after which the
                                                ///< device will come out of power save mode - not supported
    uint8_t             wps_method    ;         ///< WiFi Protected Setup (WPS) method (ARM_WIFI_WPS_METHOD_xxx)
    const int8_t        *wps_pin      ;         ///< Pointer to WiFi Protected Setup (WPS) PIN null-terminated string
    uint32_t            wHandle       ;         ///< Interface  0: AP mode, 1: STA mode
    ARM_WIFI_NET_INFO_t interfaceInfo ;         ///< Interface Info
    uint8_t             bandwidth     ;         ///< 2: 20 MHz, 3: 40 MHz (allowed only 2 values)
    uint8_t             wifiMode      ;         ///< 0: Concurrent(default), 1:concurrent
    uint8_t             ssid[32+1];
} WIFI_OPTIONS;


/* Function Declarations */
/**
  \fn            int32_t we310f5_device_status()
  \brief         Get device status for read/write operation
  \return        Read or Write status
*/
int32_t we310f5_device_status(void);

/**
  \fn            int32_t we310f5_hw_init()
  \brief         It will send AT command to Telit chip and wait for response.
                        process the received response.
  \return        execution status
                   - \ref WIFI_SUCCESS                : Operation successful
                   - \ref WIFI_FAILED                 : Operation failed
*/
int32_t we310f5_hw_init();

#ifdef  __cplusplus
}
#endif

#endif /* WIFI_WE310F5_H */
