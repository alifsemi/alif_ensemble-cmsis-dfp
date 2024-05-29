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
 * @file     WiFi_we310f5.c
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     18-Dec-2023
 * @brief    CMSIS-Driver of Wi-Fi.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#include <Driver_WiFi.h>
#include "we310f5.h"
#include "WiFi_utils.h"
#include "WiFi_we310f5.h"
#include "WiFi_we310f5_HW_Config.h"

// Driver version
#define ARM_WIFI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)
#define AT_PREFIX       (const char*) "AT"
#define CRLF            (const char*) "\r\n"

// Driver Version
static const ARM_DRIVER_VERSION driver_version = {
    ARM_WIFI_API_VERSION,
    ARM_WIFI_DRV_VERSION
};

// Wi-Fi Resource
static WIFI_RESOURCES   WIFI                                = {0};
static uint8_t          scratchBuffTx[AT_TX_BUFF_SIZE]      = {0};
static uint8_t          scratchBuffRx[AT_RX_BUFF_SIZE]      = {0};

// Driver Capabilities
static const ARM_WIFI_CAPABILITIES driver_capabilities = {
    1U,     // Station supported
    1U,     // Access Point supported
    1U,     // Concurrent Station and Access Point supported
    1U,     // WiFi Protected Setup (WPS) for Station supported
    1U,     // WiFi Protected Setup (WPS) for Access Point not supported
    1U,     // Access Point: event generated on Station connect
    0U,     // Access Point: event not generated on Station disconnect
    0U,     // Event not generated on Ethernet frame reception in bypass mode
    0U,     // Bypass or pass-through mode (Ethernet interface) not supported
    1U,     // IP (UDP/TCP) (Socket interface) supported
    0U,     // IPv6 (Socket interface) not supported
    1U,     // Ping (ICMP) supported
    0U      // Reserved (must be zero)
};

/**
  \fn            ARM_DRIVER_VERSION ARM_WIFI_GetVersion (void)
  \brief         Get driver version.
  \return        \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION ARM_WiFi_GetVersion (void) {
    return driver_version;
}

/**
  \fn            ARM_WIFI_CAPABILITIES ARM_WIFI_GetCapabilities (void)
  \brief         Get driver capabilities.
  \return        \ref ARM_WIFI_CAPABILITIES
*/
static ARM_WIFI_CAPABILITIES ARM_WiFi_GetCapabilities (void) {
    return driver_capabilities;
}

/**
  \fn            int32_t ARM_WIFI_Initialize (ARM_WIFI_SignalEvent_t cb_event,
                                ARM_WIFI_HW_CTRL_t *wifiCtrl)
  \brief         Initialize WiFi Module.
  \param[in]     cb_event Pointer to \ref ARM_WIFI_SignalEvent_t
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
  \note  User has to set proper values in ARM_WIFI_HW_CTRL_t variable
*/

static int32_t ARM_WiFi_Initialize (ARM_WIFI_SignalEvent_t cb_event)
{
    int32_t ret;

    if(!cb_event)
        return ARM_DRIVER_ERROR_PARAMETER;

    WIFI.callBack               = cb_event;

    return ARM_DRIVER_OK;
}

/**
  \fn            int32_t ARM_WIFI_Uninitialize (void)
  \brief         De-initialize WiFi Module.
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t ARM_WiFi_Uninitialize (void) {
    int32_t ret = ARM_DRIVER_ERROR;

    WIFI.atCmd.msgLen    = SPRINTF(WIFI.atCmd.msg, "%s+YSR=%d%s", AT_PREFIX, \
            WIFI.interface, CRLF);
    WIFI.atCmd.atRespType   = AT_RESP_OK;
    WIFI.atCmd.msgRespLen   = 64;
    WIFI.atCmd.msgResp      = &scratchBuffRx[0];

    ret = we310f5_cmd_transaction(&WIFI);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    WIFI.state.initialized = 0;

    return ARM_DRIVER_OK;
}

/**
  \fn            int32_t ARM_WIFI_PowerControl (ARM_POWER_STATE state)
  \brief         Control WiFi Module Power.
  \param[in]     state     Power state
                   - \ref ARM_POWER_OFF                : Power off: no operation possible
                   - \ref ARM_POWER_LOW                : Low-power mode: sleep or deep-sleep depending on \ref ARM_WIFI_LP_TIMER option set
                   - \ref ARM_POWER_FULL               : Power on: full operation at maximum performance
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
                   - \ref ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - \ref ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid state)
*/
static int32_t ARM_WiFi_PowerControl (ARM_POWER_STATE state) {
    int32_t ret = ARM_DRIVER_ERROR;

    switch (state)
    {
        case ARM_POWER_OFF:
        {
            if (WIFI.state.powered == 0)
            {
                return ARM_DRIVER_OK;
            }

            WIFI.state.powered = 0;
            break;
        }
        case ARM_POWER_FULL:
        {
            WIFI.delayBetweenOpr        = DELAY_BETWEEN_READ_RESP;
            WIFI.atCmd.msgRespLen       = 64;
            WIFI.atCmd.msgResp          = &scratchBuffRx[0];
            WIFI.atCmd.msg              = &scratchBuffTx[0];

            /* Do Hardware setup */
            ret = we310f5_hw_init();
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }
            WIFI.state.initialized = 1;

            /* Look for Device Banner which confirms device is ready to use */
            ret = we310f5_start(&WIFI);
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }

            if (WIFI.state.initialized == 0)
            {
                return ARM_DRIVER_ERROR;
            }

            WIFI.state.powered = 1;
            break;
        }
        case ARM_POWER_LOW:
        default:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }
    return ARM_DRIVER_OK;
}

/**
  \fn            int32_t ARM_WIFI_GetModuleInfo (char *module_info,
                                                 uint32_t max_len)
  \brief         Get Module information.
  \param[out]    module_info    Pointer to character buffer were info string
                                will be returned
  \param[in]     max_len        Maximum length of string to return (including
                                null terminator)
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
                   - \ref ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL
                               module_info pointer or max_len equals to 0)
*/
static int32_t ARM_WiFi_GetModuleInfo (char *module_info, uint32_t max_len) {
    int32_t ret    = ARM_DRIVER_ERROR;

    if( !(module_info) || (max_len == 0) )
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    WIFI.atCmd.msgLen       = SPRINTF(WIFI.atCmd.msg, "%s+YVEREXT%s", \
            AT_PREFIX, CRLF);
    WIFI.atCmd.atRespType   = AT_RESP_OK;
    WIFI.atCmd.msgRespLen   = 64;
    WIFI.atCmd.msgResp      = &scratchBuffRx[0];

    ret = we310f5_cmd_transaction(&WIFI);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    STRNCPY(module_info, WIFI.atCmd.msgResp, max_len);

    return ret;
}

/**
  \fn            int32_t ARM_WIFI_SetOption (uint32_t interface,
                                uint32_t option, const void *data, uint32_t len)
  \brief         Set WiFi Module Options.
  \param[in]     interface Interface (0 = Station, 1 = Access Point)
  \param[in]     option    Option to set
  \param[in]     data      Pointer to data relevant to selected option
  \param[in]     len       Length of data (in bytes)
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
                   - \ref ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - \ref ARM_DRIVER_ERROR_PARAMETER   : Parameter error
                                (invalid interface, NULL data pointer or
                                 len less than option specifies)
*/
static int32_t ARM_WiFi_SetOption (uint32_t interface, uint32_t option,  \
        const void *data, uint32_t len)
{
    int32_t ret = ARM_DRIVER_ERROR;
    WIFI_OPTIONS *wifiOpt;

    if( (interface > 2))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if(!data)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    memset(scratchBuffTx, 0, 32);
    WIFI.atCmd.atRespType   = AT_RESP_OK;
    WIFI.atCmd.msgRespLen   = 128;
    WIFI.atCmd.msgResp      = &scratchBuffRx[0];
    wifiOpt                 = &WIFI.wifiOptions[!interface];

    switch(option)
    {
        case ARM_WIFI_BSSID                 :
            if(len < sizeof(wifiOpt->bssid))
                STRNCPY(wifiOpt->bssid, data, len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_TX_POWER              :
            wifiOpt->txPower = ATOI(data);
            if(wifiOpt->txPower > 5)
            {
                wifiOpt->txPower = 0;
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            WIFI.atCmd.msgLen = SPRINTF(WIFI.atCmd.msg, "%s+WTXRATES=%d,%d%s", \
                    AT_PREFIX, interface, wifiOpt->txPower, CRLF);
            break;

        case ARM_WIFI_LP_TIMER              :
            return ARM_DRIVER_ERROR_PARAMETER;

        case ARM_WIFI_DTIM                  :
            if(interface == 0)
            {
                WIFI.atCmd.msgLen = SPRINTF(WIFI.atCmd.msg, \
                    "%s+WPOWERSAVE=%d,%d%s", AT_PREFIX, interface, \
                    wifiOpt->txPower, CRLF);
            } else if (interface == 1) {

            }
            break;

        case ARM_WIFI_BEACON                :
            break;

        case ARM_WIFI_MAC                    :
            if(len < sizeof(wifiOpt->mac))
            {
                STRNCPY(wifiOpt->mac, data, len);
                WIFI.atCmd.msgLen = SPRINTF((int8_t*) WIFI.atCmd.msg,   \
                    "%s+WTXRATES=%d,%d:%d:%d:%d:%d:%d,0%s",AT_PREFIX, interface,\
                    wifiOpt->mac[0], wifiOpt->mac[1], wifiOpt->mac[2],          \
                    wifiOpt->mac[3], wifiOpt->mac[4], wifiOpt->mac[5], CRLF);
            }
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_IP                     :
            if(len < sizeof(wifiOpt->ipCfg.ip))
            {
                STRNCPY(wifiOpt->ipCfg.ip, data, len);
                WIFI.atCmd.msgLen = SPRINTF((int8_t*) WIFI.atCmd.msg,   \
                    "%s+WTXRATES=%d,%d:%d:%d:%d:%d:%d,0%s",AT_PREFIX, interface,\
                    wifiOpt->mac[0], wifiOpt->mac[1], wifiOpt->mac[2],          \
                    wifiOpt->mac[3], wifiOpt->mac[4], wifiOpt->mac[5], CRLF);
            }
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_IP_SUBNET_MASK         :
            if(len < sizeof(wifiOpt->ipCfg.subNetmask))
                STRNCPY(wifiOpt->ipCfg.subNetmask, data, len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_IP_GATEWAY             :
            if(len < sizeof(wifiOpt->ipCfg.gateway))
                STRNCPY(wifiOpt->ipCfg.gateway, data, len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_CONNECTION_TYPE      :
            if(len <= sizeof(WIFI.interface))
            {
                WIFI.interface = interface;

                if( wifiOpt->bandwidth != 3 )
                    wifiOpt->bandwidth = 2;

                if( wifiOpt->wifiMode != 1 )
                    wifiOpt->wifiMode = 0;

                WIFI.atCmd.msgLen = SPRINTF(WIFI.atCmd.msg, "%s+WNI=%d,%d,%d%s",\
                    AT_PREFIX, interface, wifiOpt->bandwidth, wifiOpt->wifiMode,\
                    CRLF);
            }
            else
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            break;

        case ARM_WIFI_SCAN_TIME              :
            wifiOpt->scanTime = atoi(data);
            if( (wifiOpt->scanTime < 5 ) || (wifiOpt->scanTime < 1500 ) )
            {
                wifiOpt->scanTime = 5;
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            break;

        case ARM_WIFI_TX_RATE                :
            wifiOpt->txRate = atoi(data);
            if( !( (wifiOpt->txRate ==  1 ) || (wifiOpt->txRate ==  6 ) || \
                   (wifiOpt->txRate ==  7 ) || (wifiOpt->txRate == 12 ) || \
                   (wifiOpt->txRate == 20 ) ))
            {
                wifiOpt->txRate = 12;
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            break;

        case ARM_WIFI_COUNTRY_CODE           :
            if(len < sizeof(wifiOpt->countryCode))
                STRNCPY(wifiOpt->countryCode, data, len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_REGULATORY_DOMAIN      :
            if(len < sizeof(wifiOpt->regulatoryDomain))
                STRNCPY(wifiOpt->regulatoryDomain, data, len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_PHYSICAL_MODE        :
            if(len < sizeof(wifiOpt->physicalMode))
                STRNCPY(wifiOpt->physicalMode, data, len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_POWER_SAVE             :
            wifiOpt->powerSave = (bool) atoi(data);
            break;

        case ARM_WIFI_IDLE_TIME              :
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_WIFI_PS_POLL_POLICY         :
            wifiOpt->psPollpolicy = (bool) atoi(data);
            break;

        case ARM_WIFI_DTIM_POLICY            :
            wifiOpt->dtimPolicy  = (bool) atoi(data);
            break;

        case ARM_WIFI_TX_WAKEUP              :
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_WIFI_SSID                  :
            if(len < sizeof(wifiOpt->ssid))
                STRNCPY(wifiOpt->ssid, data, len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        default: ret = ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    ret = we310f5_cmd_transaction(&WIFI);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn            int32_t ARM_WIFI_GetOption (uint32_t interface, uint32_t option, void *data, uint32_t *len)
  \brief         Get WiFi Module Options.
  \param[in]     interface Interface (0 = Station, 1 = Access Point)
  \param[in]     option    Option to get
  \param[out]    data      Pointer to memory where data for selected option will be returned
  \param[in,out] len       Pointer to length of data (input/output)
                   - input: maximum length of data that can be returned (in bytes)
                   - output: length of returned data (in bytes)
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
                   - \ref ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - \ref ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface, NULL data or len pointer, or *len less than option specifies)
*/
static int32_t ARM_WiFi_GetOption (uint32_t interface, uint32_t option, \
        void *data, uint32_t *len)
{
    int32_t ret = ARM_DRIVER_ERROR;
    WIFI_OPTIONS *wifiOpt;

    if( (interface > 2))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if(!data)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    memset(scratchBuffTx, 0, 16);
    WIFI.atCmd.msgLen = SPRINTF((int8_t*) WIFI.atCmd.msg, \
            "%s+WNIFCFG%s", AT_PREFIX, CRLF);
    WIFI.atCmd.atRespType  = AT_RESP_OK;
    WIFI.atCmd.msgRespLen  = 500;
    WIFI.atCmd.msgResp     = &scratchBuffRx[0];

    ret = we310f5_get_all_config(&WIFI);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }
    wifiOpt     = &WIFI.wifiOptions[interface];
    switch(option)
    {
        case ARM_WIFI_BSSID                  :
            if(*len < sizeof(wifiOpt->bssid))
                strncpy(data, (const char*)wifiOpt->bssid, *len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_TX_POWER               :
            if(*len > 1)
            {
                wifiOpt->txPower = 0;
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            else
            {
                sprintf(data, "%d",  wifiOpt->txPower);
            }
            break;

        case ARM_WIFI_MAC                    :
            if(*len < sizeof(wifiOpt->mac))
                strncpy(data, (const char*) wifiOpt->mac, *len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_IP                     :
            if(*len < sizeof(wifiOpt->ipCfg.ip))
                strncpy(data, (const char*) wifiOpt->ipCfg.ip, *len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_IP_SUBNET_MASK         :
            if(*len < sizeof(wifiOpt->ipCfg.subNetmask))
                strncpy(data, (const char*)wifiOpt->ipCfg.subNetmask, *len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_IP_GATEWAY             :
            if(*len < sizeof(wifiOpt->ipCfg.gateway))
                strncpy(data, (const char*)wifiOpt->ipCfg.gateway, *len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_SCAN_TIME              :
            if( *len > 4 )
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            else
            {
                sprintf(data, "%d",  wifiOpt->scanTime);
            }
            break;

        case ARM_WIFI_TX_RATE                :
            if( *len > 3 )
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            else
            {
                sprintf(data, "%d",  wifiOpt->txRate);
            }
            break;

        case ARM_WIFI_COUNTRY_CODE           :
            if(*len < sizeof(wifiOpt->countryCode))
                strncpy(data, (const char*)wifiOpt->countryCode, *len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_REGULATORY_DOMAIN      :
            if(*len < sizeof(wifiOpt->regulatoryDomain))
                strncpy(data, \
                    (const char*) wifiOpt->regulatoryDomain, *len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_PHYSICAL_MODE        :
            if(*len < sizeof(wifiOpt->physicalMode))
                strncpy(data, (const char*) wifiOpt->physicalMode, *len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        case ARM_WIFI_POWER_SAVE             :
            if( *len > 1 )
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            else
            {
                sprintf(data, "%d",  wifiOpt->powerSave);
            }
            break;

        case ARM_WIFI_IDLE_TIME              :
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_WIFI_PS_POLL_POLICY         :
            if( *len > 1 )
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            else
            {
                sprintf(data, "%d",  wifiOpt->psPollpolicy);
            }
            break;

        case ARM_WIFI_DTIM_POLICY            :
            if( *len > 1 )
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            else
            {
                sprintf(data, "%d",  wifiOpt->dtimPolicy);
            }
            break;

        case ARM_WIFI_TX_WAKEUP              :
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_WIFI_SSID                  :
            if(*len < sizeof(wifiOpt->ssid))
                strncpy(data, (const char*)wifiOpt->ssid, *len);
            else
                return ARM_DRIVER_ERROR_PARAMETER;
            break;

        default: ret = ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

static int32_t ARM_WiFi_Scan (ARM_WIFI_SCAN_INFO_t scan_info[], \
        uint32_t max_num)
{
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_Activate (uint32_t interface, \
        const ARM_WIFI_CONFIG_t *config)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_Deactivate (uint32_t interface)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t ARM_WiFi_IsConnected (void)
{
    return 0U;
}

static int32_t ARM_WiFi_GetNetInfo (ARM_WIFI_NET_INFO_t *net_info) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_BypassControl(uint32_t interface, uint32_t mode) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_EthSendFrame(uint32_t interface, const uint8_t *frame,
        uint32_t len) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_EthReadFrame(uint32_t interface, uint8_t *frame,
        uint32_t len) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t ARM_WiFi_EthGetRxFrameSize(uint32_t interface) {
    return 0U;
}

static int32_t ARM_WiFi_SocketCreate(int32_t af, int32_t type, \
        int32_t protocol)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketBind(int32_t socket, const uint8_t *ip,
        uint32_t ip_len, uint16_t port) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketListen(int32_t socket, int32_t backlog) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketAccept(int32_t socket, uint8_t *ip,
        uint32_t *ip_len, uint16_t *port) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketConnect(int32_t socket, const uint8_t *ip,
        uint32_t ip_len, uint16_t port) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketRecv(int32_t socket, void *buf, uint32_t len) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketRecvFrom(int32_t socket, void *buf, uint32_t len,
        uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketSend(int32_t socket, const void *buf,
        uint32_t len) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketSendTo(int32_t socket, const void *buf,
        uint32_t len, const uint8_t *ip, uint32_t ip_len, uint16_t port) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketGetSockName(int32_t socket, uint8_t *ip,
        uint32_t *ip_len, uint16_t *port) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketGetPeerName(int32_t socket, uint8_t *ip,
        uint32_t *ip_len, uint16_t *port) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketGetOpt(int32_t socket, int32_t opt_id,
        void *opt_val, uint32_t *opt_len) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketSetOpt(int32_t socket, int32_t opt_id,
        const void *opt_val, uint32_t opt_len) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketClose(int32_t socket) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_SocketGetHostByName(const char *name, int32_t af,
        uint8_t *ip, uint32_t *ip_len) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_WiFi_Ping(const uint8_t *ip, uint32_t ip_len) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/* WiFi Driver Control Block */
extern ARM_DRIVER_WIFI Driver_WiFi0;
ARM_DRIVER_WIFI Driver_WiFi0 = {
    ARM_WiFi_GetVersion,
    ARM_WiFi_GetCapabilities,
    ARM_WiFi_Initialize,
    ARM_WiFi_Uninitialize,
    ARM_WiFi_PowerControl,
    ARM_WiFi_GetModuleInfo,
    ARM_WiFi_SetOption,
    ARM_WiFi_GetOption,
    ARM_WiFi_Scan,
    ARM_WiFi_Activate,
    ARM_WiFi_Deactivate,
    ARM_WiFi_IsConnected,
    ARM_WiFi_GetNetInfo,
    ARM_WiFi_BypassControl,
    ARM_WiFi_EthSendFrame,
    ARM_WiFi_EthReadFrame,
    ARM_WiFi_EthGetRxFrameSize,
    ARM_WiFi_SocketCreate,
    ARM_WiFi_SocketBind,
    ARM_WiFi_SocketListen,
    ARM_WiFi_SocketAccept,
    ARM_WiFi_SocketConnect,
    ARM_WiFi_SocketRecv,
    ARM_WiFi_SocketRecvFrom,
    ARM_WiFi_SocketSend,
    ARM_WiFi_SocketSendTo,
    ARM_WiFi_SocketGetSockName,
    ARM_WiFi_SocketGetPeerName,
    ARM_WiFi_SocketGetOpt,
    ARM_WiFi_SocketSetOpt,
    ARM_WiFi_SocketClose,
    ARM_WiFi_SocketGetHostByName,
    ARM_WiFi_Ping
};
