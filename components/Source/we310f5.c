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
 * @file     we310f5.c
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    Board Specific Wi-Fi Functions
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include <stdint.h>
#include "we310f5.h"
#include "we310f5_SPI.h"
#include "WiFi_utils.h"
#include "WiFi_we310f5.h"
#include "WiFi_we310f5_HW_Config.h"
#include "system_utils.h"

/**
  \fn            void we310f5_reset_spi_packet (uint8_t *packet, uint16_t size)
  \brief         Reset (i.e. zeros) input buffer for given size
  \param[in]     packet Address of input buffer
  \param[in]     size   Count number of bytes to be reset
  \return        none
*/
void we310f5_reset_spi_packet(void *packet, uint16_t size)
{
    MEMSET(packet, 0x0, size);
    return ;
}

/**
  \fn            int32_t we310f5_get_all_config(WIFI_RESOURCES *wifi)
  \brief         It will retrieve all Wi-Fi setting
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_get_all_config(WIFI_RESOURCES *wifi)
{
    int32_t ret = ARM_DRIVER_ERROR;
    uint8_t *param[21], *prm[8];   /* Ensure all param should be char type */
    uint8_t         *noOfInterface[2], *tmp;
    uint8_t         loop, cnt;
    WIFI_OPTIONS    *wifiOpt;
    int32_t         interface;

    ret = we310f5_cmd_transaction(wifi);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    cnt = sizeof(noOfInterface) / sizeof(uint32_t);
    tmp = wifi->atCmd.msgResp;

    for(loop = 0; loop < cnt; loop++)
    {
        noOfInterface[loop]     = (uint8_t*) STRSTR(tmp, "+");
        tmp                     = noOfInterface[loop] + strlen("+");
        tmp                     = (uint8_t*) STRSTR(tmp, "\r\n");
        *tmp++ = 0;
    }

    for(loop = 0; loop < cnt; loop++)
    {

        ret = get_nth_pos_delimiterStr(noOfInterface[loop], param, ",", NULL, 21);
        if (ret > 0)
        {
            wifiOpt = &wifi->wifiOptions[loop];
            switch(ret)
            {
                case 20:  /* <IPv6 DNS2>                    */
                case 19:  /* <IPv6 DNS1>                    */
                case 18:  /* <IPv6 global address>          */
                case 17:  /* <IPv6 global address>          */
                case 16:  /* <IPv6 link-local address>      */
                case 15:  /* <DNS3>                         */
                case 14:  /* <DNS2>                         */
                case 13:  /* <DNS1>                         */
                case 12:
                    /* <Gateway address> */
                    get_nth_pos_delimiterStr(param[12], prm, ".", NULL, 4);
                    wifiOpt->ipCfg.gateway[0] = ATOI(prm[0]);
                    wifiOpt->ipCfg.gateway[1] = ATOI(prm[1]);
                    wifiOpt->ipCfg.gateway[2] = ATOI(prm[2]);
                    wifiOpt->ipCfg.gateway[3] = ATOI(prm[3]);
                case 11:
                    /* <Subnet address> */
                    get_nth_pos_delimiterStr(param[11], prm, ".", NULL, 4);
                    wifiOpt->ipCfg.subNetmask[0] = ATOI(prm[0]);
                    wifiOpt->ipCfg.subNetmask[1] = ATOI(prm[1]);
                    wifiOpt->ipCfg.subNetmask[2] = ATOI(prm[2]);
                    wifiOpt->ipCfg.subNetmask[3] = ATOI(prm[3]);
                case 10:
                    /* <IP address> */
                    get_nth_pos_delimiterStr(param[10], prm, ".", NULL, 4);
                    wifiOpt->ipCfg.ip[0] = ATOI(prm[0]);
                    wifiOpt->ipCfg.ip[1] = ATOI(prm[1]);
                    wifiOpt->ipCfg.ip[2] = ATOI(prm[2]);
                    wifiOpt->ipCfg.ip[3] = ATOI(prm[3]);
                case  9:
                    /* <RSSI> */
                    //strcpy(wifi->, param[9]);
                case  8:
                    /* <Security> */
                    //strcpy(wifi->, param[8]);
                case  7:
                    /* <Channel> */
                    //strcpy(wifi->, param[7]);
                case  6:
                    /* <SSID> */
                    STRCPY(wifiOpt->interfaceInfo.ssid, param[6]);
                case  5:
                    /* <BSSID> */
                    get_nth_pos_delimiterStr(param[5], prm,":", NULL, 6);
                    wifiOpt->bssid[0] = ATOI(prm[0]);
                    wifiOpt->bssid[1] = ATOI(prm[1]);
                    wifiOpt->bssid[2] = ATOI(prm[2]);
                    wifiOpt->bssid[3] = ATOI(prm[3]);
                    wifiOpt->bssid[4] = ATOI(prm[4]);
                    wifiOpt->bssid[5] = ATOI(prm[5]);
                case  4:
                    /* <Mode> */
                    //strcpy(wifi->, param[4]);
                case  3:
                    /* <WLAN State> */
                    //strcpy(wifi->, param[3]);
                case  2:
                    /* <State> */
                    //strcpy(wifi->, param[2]);
                case  1:
                    /* <MAC address> */
                    get_nth_pos_delimiterStr(param[1], prm,":", NULL, 5);
                    wifiOpt->mac[0] = ATOI(prm[0]);
                    wifiOpt->mac[1] = ATOI(prm[1]);
                    wifiOpt->mac[2] = ATOI(prm[2]);
                    wifiOpt->mac[3] = ATOI(prm[3]);
                    wifiOpt->mac[4] = ATOI(prm[4]);
                    wifiOpt->mac[5] = ATOI(prm[5]);
                case 0:
                    get_nth_pos_delimiterStr(param[0], prm, ":", NULL, 1);
                    wifiOpt->wHandle = ATOI(prm[1]);
                default:
                    break;
            }
        }
        else
        {
            continue;
        }
    }

    return 0;
}

/**
  \fn        int32_t we310f5_spi_packet_sanity(SPI_Packet_Frame_t *packet)
  \brief     Check given SPI packet is valid as per Telit SPI structure .
  \param[in] packet     Pointer to \ref SPI_Packet_Frame_t SPI packet
  \return    execution status
               - 0 : Invalid SPI packet
               - 1 : Valid   SPI packet
*/
int32_t we310f5_spi_packet_sanity(SPI_Packet_Frame_t *packet)
{
    int32_t ret = 0;
    int32_t cmd = packet->CMD;

    if(packet->SOF != WE310F5_SPI_SOF)
    {
        return 0;
    }

    if(packet->PayloadLength > 1536)
    {
        return 0;
    }

    switch(cmd)
    {
        case CMD_WRITE_REQUEST       :
        case CMD_READ_REQUEST        :
        case CMD_WRITE_DATA          :
        case CMD_WRITE_RESPONSE_OK   :
        case CMD_READ_RESPONSE_OK    :
        case CMD_WRITE_RESPONSE_NOK  :
        case CMD_READ_RESPONSE_NOK   :
        case CMD_READ_DATA           :
            ret = 1;
            break;
        default:
            ret = 0;
            break;
    }

    return ret;
}

/**
  \fn            int32_t we310f5_wait_cond(WIFI_RESOURCES *wifi, uint32_t waitCond)
  \brief         Wait for read/write state of device based on 'waitCond' value
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \param[in]     waitCond   Value (Device Read/Write/Invalid state) on which
                            wait should over
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
  \note:  'timeoutMSretryCnt' value should be updated in \ref WIFI_RESOURCES
           wi-fi resource as per need, if 0, it waits forever.
*/
int32_t we310f5_wait_cond(WIFI_RESOURCES *wifi, uint32_t waitCond)
{
    int32_t ret = ARM_DRIVER_ERROR;
    uint32_t tMSretryCnt = wifi->timeoutMSretryCnt;

    if(tMSretryCnt)
    {
        do
        {
            delay_in_ms(1);

            if(we310f5_device_status() == waitCond)
            {
                ret = ARM_DRIVER_OK;
                break;
            }
        }while(tMSretryCnt--);
    }
    else
    {
        while(we310f5_device_status() != waitCond);
        ret = ARM_DRIVER_OK;
    }

    return ret;
}

/**
  \fn            int32_t we310f5_read_data(WIFI_RESOURCES *wifi, uint8_t *data,
                               int16_t len)
  \brief         Telit read data operation.
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \param[out]    data       Address of output buffer
  \param[in]     len        Number of bytes which needs to read
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_read_data(WIFI_RESOURCES *wifi, uint8_t *data, int16_t len)
{
    int32_t ret = ARM_DRIVER_ERROR;
    int16_t packet_len, remainData;
    void *sendPtr, *recvPtr;

    remainData = len;

    we310f5_reset_spi_packet((uint8_t*)&wifi->sendPacket, WE310F4_SPI_PKT_SIZE);

    ret = we310f5_spi_transfer(wifi, (uint8_t *)&wifi->sendPacket,  \
            (uint8_t *)&wifi->recvPacket, WE310F4_SPI_PKT_SIZE,     \
            WE310F5_TXNS_READ);
    if( ret < 0)
        return ret;

    if(we310f5_spi_packet_sanity(&wifi->recvPacket))
    {
        if(wifi->recvPacket.CMD == CMD_READ_DATA)
        {
            if( len > WE310F4_SPI_PAYLOAD_SIZE)
            {
                MEMCPY(data, wifi->recvPacket.Payload, WE310F4_SPI_PAYLOAD_SIZE);
                data        += WE310F4_SPI_PAYLOAD_SIZE;
                remainData  = len - WE310F4_SPI_PAYLOAD_SIZE;
                while(remainData != 0)
                {
                    packet_len = (remainData > WE310F4_SPI_PKT_SIZE) ? \
                            WE310F4_SPI_PKT_SIZE : remainData;

                    ret = we310f5_spi_transfer(wifi,        \
                            (uint8_t *)&wifi->sendPacket,   \
                            data, packet_len, INVALID_CMD);
                    if( ret < 0)
                        return ret;

                    remainData  -= packet_len;
                    data        += packet_len;
                }
            }
            else
            {
                MEMCPY(data, wifi->recvPacket.Payload, len);
            }
        }
     }

    return ARM_DRIVER_OK;
}

/**
  \fn            int32_t we310f5_read_resp(WIFI_RESOURCES *wifi)
  \brief         Telit read response operation.
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_read_resp(WIFI_RESOURCES *wifi)
{
    int32_t ret = ARM_DRIVER_ERROR, retryCnt = 0, retryCnt1 = 0;
    uint8_t *tmp = wifi->atCmd.msgResp;

    if(!wifi->atCmd.msgResp)
        return ARM_DRIVER_ERROR_PARAMETER;

    if(wifi->delayBetweenOpr)
        delay_in_ms(wifi->delayBetweenOpr);

    we310f5_reset_spi_packet((uint8_t*)&wifi->sendPacket, WE310F4_SPI_PKT_SIZE);
    wifi->atCmd.msgRespLen = 0;

    do
    {
        ret = we310f5_spi_transfer(wifi, (uint8_t *)&wifi->sendPacket,  \
                (uint8_t *)&wifi->recvPacket, WE310F4_SPI_PKT_SIZE,     \
                WE310F5_TXNS_READ);
        if( ret < 0)
            return ret;

        if(we310f5_spi_packet_sanity(&wifi->recvPacket))
        {
            if(wifi->recvPacket.CMD == CMD_READ_RESPONSE_OK)
            {
                wifi->atCmd.msgRespLen = wifi->recvPacket.PayloadLength;
                we310f5_reset_spi_packet(wifi->atCmd.msgResp, \
                        wifi->atCmd.msgRespLen + 1);

                if(wifi->recvPacket.PayloadLength <= WE310F4_SPI_PAYLOAD_SIZE)
                {
                    MEMCPY(wifi->atCmd.msgResp, wifi->recvPacket.Payload, \
                            wifi->recvPacket.PayloadLength);
                }
                else
                {
                    MEMCPY(wifi->atCmd.msgResp, wifi->recvPacket.Payload, \
                            WE310F4_SPI_PAYLOAD_SIZE);
                }
            }
            ret = 0;
            break;
        }
        else
        {
            sys_busy_loop_us(100);
            ret = -1;
        }
    } while(retryCnt1++ < 2);

    return ret;
}

/**
  \fn            int32_t we310f5_read_req(WIFI_RESOURCES *wifi)
  \brief         Telit read request operation.
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_read_req(WIFI_RESOURCES *wifi)
{
    int32_t ret =  ARM_DRIVER_ERROR;
    uint32_t retryCnt = 0;

    if(wifi->delayBetweenOpr)
        delay_in_ms(wifi->delayBetweenOpr);

    we310f5_reset_spi_packet((uint8_t*)&wifi->sendPacket, WE310F4_SPI_PKT_SIZE);
    wifi->sendPacket.SOF             = WE310F5_SPI_SOF;
    wifi->sendPacket.CMD             = CMD_READ_REQUEST;
    wifi->sendPacket.PayloadLength   = 0;

    if(!we310f5_spi_packet_sanity(&wifi->sendPacket))
    {
        return -1;
    }

    ret = we310f5_spi_transfer(wifi, (uint8_t *)&wifi->sendPacket, \
                (uint8_t *)&wifi->recvPacket, WE310F4_SPI_PKT_SIZE,\
                WE310F5_TXNS_READ);
    if( ret < 0)
        return ret;


    return ARM_DRIVER_OK;
}

/**
  \fn            int32_t we310f5_write_data(WIFI_RESOURCES *wifi, uint8_t *data,
                               int16_t len)
  \brief         Telit write data operation.
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \param[out]    data       Address of input buffer
  \param[in]     len        Number of bytes which needs to send
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_write_data(WIFI_RESOURCES *wifi, uint8_t *data, int16_t len)
{
    int32_t ret = ARM_DRIVER_ERROR;
    int16_t packet_len, remainData;
    void *sendPtr, *recvPtr;

    we310f5_reset_spi_packet((uint8_t*)&wifi->sendPacket, WE310F4_SPI_PKT_SIZE);

    wifi->sendPacket.SOF             = WE310F5_SPI_SOF;
    wifi->sendPacket.CMD             = CMD_WRITE_DATA;

    // 52 is because already sent in WRITE_REQ
    wifi->sendPacket.PayloadLength   = len + WE310F4_SPI_PAYLOAD_SIZE;

    if(len <= WE310F4_SPI_PAYLOAD_SIZE)
    {
        remainData = len;
        memcpy(wifi->sendPacket.Payload, data, remainData);
    }
    else
    {
        memcpy(wifi->sendPacket.Payload, data, WE310F4_SPI_PAYLOAD_SIZE);
        data        += WE310F4_SPI_PAYLOAD_SIZE;
        remainData   = len - WE310F4_SPI_PAYLOAD_SIZE;
    }

    ret = we310f5_spi_transfer(wifi, (uint8_t *)&wifi->sendPacket, \
                (uint8_t *)&wifi->recvPacket, WE310F4_SPI_PKT_SIZE,\
                WE310F5_TXNS_READ);
    if( ret < 0)
        return ret;

    we310f5_reset_spi_packet((uint8_t*)&wifi->sendPacket, WE310F4_SPI_PKT_SIZE);
    sendPtr = data;

    while(remainData != 0)
    {
        packet_len = (remainData > WE310F4_SPI_PKT_SIZE) ? \
                WE310F4_SPI_PKT_SIZE : remainData;

        if(packet_len < WE310F4_SPI_PKT_SIZE)
        {
            memcpy((uint8_t*)&wifi->sendPacket, sendPtr, packet_len);
            sendPtr = &wifi->sendPacket;
        }

        ret = we310f5_spi_transfer(wifi, sendPtr, (uint8_t *)&wifi->recvPacket,\
                WE310F4_SPI_PKT_SIZE, INVALID_CMD);
        if( ret < 0)
            return ret;

        remainData -= packet_len;
        sendPtr +=packet_len;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn            int32_t we310f5_write_resp(WIFI_RESOURCES *wifi)
  \brief         Telit write response operation.
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_write_resp(WIFI_RESOURCES *wifi)
{
    int32_t ret = ARM_DRIVER_ERROR, retryCnt = 0;
    uint16_t lenTobeSent =  wifi->atCmd.msgRespLen;

    we310f5_reset_spi_packet((uint8_t*)&wifi->sendPacket, WE310F4_SPI_PKT_SIZE);

    do
    {
        ret = we310f5_spi_transfer(wifi, (uint8_t *)&wifi->sendPacket,  \
                    (uint8_t *)&wifi->recvPacket, WE310F4_SPI_PKT_SIZE, \
                    WE310F5_TXNS_READ);
        if( ret < 0)
            return ret;

        if(we310f5_spi_packet_sanity(&wifi->recvPacket))
        {
            if(wifi->recvPacket.CMD == CMD_WRITE_RESPONSE_OK)
            {
                wifi->atCmd.msgRespAcceptedLen = wifi->recvPacket.PayloadLength;
                if(wifi->recvPacket.PayloadLength != lenTobeSent)
                {
                    ret = -1;
                }

                if(wifi->recvPacket.AIH)
                {
                    if(wifi->recvPacket.DASLength <= WE310F4_SPI_PAYLOAD_SIZE )
                    {
                        memcpy(wifi->aihPayload, wifi->recvPacket.Payload, \
                                wifi->recvPacket.DASLength);
                        wifi->aihPayloadLen             = 0;
                        wifi->atCmd.msgRespAcceptedLen  = 0;
                    }
                    else
                    {
                        wifi->aihPayloadLen = wifi->recvPacket.DASLength;
                    }
                }
            }
           ret = 0;
           break;
        }
        else
        {
            sys_busy_loop_us(1000);
            ret = -1;
        }
    } while(retryCnt++ < 4);

    sys_busy_loop_us(100); // 100 us

    if(wifi->aihPayloadLen)
    {
        ret = we310f5_read_data(wifi, wifi->atCmd.msgResp, wifi->aihPayloadLen);
        wifi->aihPayloadLen = 0;
    }

    return ret;
}

/**
  \fn            int32_t we310f5_write_req(WIFI_RESOURCES *wifi)
  \brief         Telit write request operation.
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_write_req(WIFI_RESOURCES *wifi)
{
    int32_t ret         = ARM_DRIVER_ERROR;
    uint16_t len        = wifi->atCmd.msgLen;
    uint8_t *dataToSend = wifi->atCmd.msg;

    if(!dataToSend)
        return ARM_DRIVER_ERROR_PARAMETER;

    we310f5_reset_spi_packet((uint8_t*)&wifi->sendPacket, WE310F4_SPI_PKT_SIZE);

    wifi->sendPacket.SOF             = WE310F5_SPI_SOF;
    wifi->sendPacket.CMD             = CMD_WRITE_REQUEST;
    wifi->sendPacket.PayloadLength   = len;

    if(len > WE310F4_SPI_PAYLOAD_SIZE)
    {
        len = WE310F4_SPI_PAYLOAD_SIZE;
    }

    memcpy(wifi->sendPacket.Payload, dataToSend, len);
    ret = we310f5_spi_transfer(wifi, (uint8_t *)&wifi->sendPacket,  \
                (uint8_t *)&wifi->recvPacket, WE310F4_SPI_PKT_SIZE, \
                WE310F5_TXNS_WRITE);
    if( ret < 0)
        return ret;

    return ARM_DRIVER_OK;
}

/**
  \fn            int32_t we310f5_process_cmd_response(uint8_t *response)
  \brief         Check given buffer has valid response or not as per Telit
  \param[in]     response   Pointer of response buffer
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_process_cmd_response(uint8_t *response)
{
    int32_t ret = ARM_DRIVER_ERROR;

    if(!strcmp((const char*)response, "\r\nOK\r\n"))
    {
        ret = AT_RESP_OK;
    }

    if(!strcmp((const char*)response, "\r\nERROR\r\n"))
    {
        ret = AT_RESP_ERROR;
    }

    if(!strcmp((const char*)response, "\r\nINVALID PARAM\r\n"))
    {
        ret = AT_RESP_INVALID_PARAM;
    }
    if(!strcmp((const char*)response, "\r\nINVALID COMMAND\r\n"))
    {
        ret = AT_RESP_INVALID_COMMAND;
    }

    if(!strcmp((const char*)response, "\r\nPARAMETER OUT OF RANGE\r\n"))
    {
        ret = AT_RESP_PARAMETER_OUT_OF_RANGE;
    }

    if(!strcmp((const char*)response, "\r\nNO MEMORY\r\n"))
    {
        ret = AT_RESP_NO_MEMORY;
    }

    if(!strcmp((const char*)response, "\r\nEXCESS DATA RECEIVE\r\n"))
    {
        ret = AT_RESP_EXCESS_DATA_RECEIVE;
    }

    if(response[2] == '+')
    {
        ret = AT_RESP_OF_AT_CMD;
    }

    return ret;
}

/**
  \fn            int32_t we310f5_cmd_transaction(WIFI_RESOURCES *wifi)
  \brief         It will send AT command to Telit chip and wait for response.
                        process the received response.
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
  \note         Critical : User must fill/configure \ref atCmdObj_t wifi->atCmd,
                in ref WIFI_RESOURCES wifi. \ref atCmdObj_t wifi->atCmd should
                have following values properly
                msg         [IN] : AT message/data
                msgLen      [IN] : AT message/data length
                msgResp     [OUT]: Address of allocated AT message response buffer
                msgRespLen  [IN] : AT message response length
                atRespType  [IN] : Type of AT response ( helps to terminate loop)

                otherwise it will fail/crash.
*/
int32_t we310f5_cmd_transaction(WIFI_RESOURCES *wifi)
{
    uint32_t    retryCnt = 0;
    int32_t     ret = ARM_DRIVER_ERROR;
    uint8_t     *tmp, *tmp1;

    we310f5_write_req(wifi);

    we310f5_write_resp(wifi);

    tmp     = wifi->atCmd.msgResp;
    tmp1    = tmp;
    wifi->atCmd.respCnt = 0;

    do
    {
        we310f5_read_req(wifi);

        ret = we310f5_read_resp(wifi);

        if(ret == 0)
        {
            if(wifi->atCmd.msgRespLen > WE310F4_SPI_PAYLOAD_SIZE)
            {
                tmp += WE310F4_SPI_PAYLOAD_SIZE;
                we310f5_read_data(wifi, tmp, \
                        (wifi->atCmd.msgRespLen - WE310F4_SPI_PAYLOAD_SIZE));
                tmp += (wifi->atCmd.msgRespLen - WE310F4_SPI_PAYLOAD_SIZE);
            }
            else
            {
                tmp += wifi->atCmd.msgRespLen;
            }

            wifi->atCmd.respCnt++;

            ret = we310f5_process_cmd_response(wifi->atCmd.msgResp);

            if(ret == wifi->atCmd.atCommandType)
            {
                retryCnt = 0;
                break;
            }
            else if( (ret > AT_RESP_ERROR) && (ret < AT_RESP_OF_AT_CMD) )
            {
                break;
            }
        }

        wifi->atCmd.msgResp = tmp;

    } while(retryCnt++ < 16);

    wifi->atCmd.msgResp = tmp1;

    return ret;
}

/**
  \fn            int32_t we310f5_start(WIFI_RESOURCES *wifi)
  \brief         It will start telit communication and wait for ready to use.
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_start(WIFI_RESOURCES *wifi)
{
    uint32_t retryCnt = 0;
    uint16_t tmp;
    int32_t ret = -1;

    ret = we310f5_read_req(wifi);
    if(ret != 0)
        return ret;

    delay_in_ms(wifi->delayBetweenOpr);

    ret = we310f5_read_resp(wifi);
    if(ret != 0)
        return ret;
    delay_in_ms(wifi->delayBetweenOpr);

    str_wait_cond(wifi->atCmd.msgResp, WE310F5_BANNER, wifi->timeoutMSretryCnt);

    wifi->state.initialized = 2;

    return 0;
}
