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
 * @file     we310f5.h
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    Board Specific Wi-Fi defines and data types
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#ifndef _WE310F5_H
#define _WE310F5_H

#include <Driver_WiFi.h>
#include "WiFi_we310f5.h"
#include "pinconf.h"


#define WE310F5_BANNER                          (uint8_t*)"\r\nSerial2Wireless APP\r\n"
#define WE310F5_SPI_SOF                         0xA5A5A5A5
#define WE310F4_SPI_PKT_SIZE                    64
#define WE310F4_SPI_HDR_SIZE                    8
#define WE310F4_SPI_PAYLOAD_SIZE                52
#define WE310F4_SPI_MAX_PAYLOAD_SIZE            1536
#define WE310F4_SPI_MAX_PKT_SIZE                \
    (WE310F4_SPI_MAX_PAYLOAD_SIZE - WE310F4_SPI_PAYLOAD_SIZE)

#define INVALID_CMD                             (-1)

/**<
 _______________________________________________________________________________________________
 | SOF         |CMD         |Additional Info |Length      |Checksum    |Reserved |Extra         |
 | (4 Bytes)   |(1 Byte)    |(1 Byte)        |(2 Bytes)   |(1 Byte)    |(1 Byte) |Len (2 Bytes) |
 |______________________________________________________________________________________________|
 ------------------------------------------------------------------------------------------------
 */

typedef struct __attribute__((packed, aligned(4))) spiFrame {
    uint32_t    SOF;
    uint8_t     CMD;
    uint8_t     AIH;
    uint16_t    PayloadLength;
    uint8_t     CRC;
    uint8_t     RFU;
    uint16_t    DASLength;
    uint8_t     Payload[WE310F4_SPI_PAYLOAD_SIZE];
} SPI_Packet_Frame_t;

#define CMD_WRITE_REQUEST           0x01    /*!< When the Master wants to send data to the Slave > */
#define CMD_READ_REQUEST            0x02    /*!< When the Master wants to read data from the Slave > */
#define CMD_WRITE_DATA              0x03    /*!< When Master has more data to send which cannot fit WRITE_REQUEST > */
#define CMD_WRITE_RESPONSE_OK       0x0B    /*!< Response to WRITE_REQUEST from Slave when ready to receive data > */
#define CMD_READ_RESPONSE_OK        0x0C    /*!< Response to READ_REQUEST from Slave when it is ready to send data, This is indicated to host by making interrupt line high > */
#define CMD_WRITE_RESPONSE_NOK      0x0D    /*!< Response to WRITE_REQUEST from Slave when not ready to receive data > */
#define CMD_READ_RESPONSE_NOK       0x0E    /*!< Response to READ_REQUEST from Slave when not  ready to send data > */
#define CMD_READ_DATA               0x0F    /*!< When Master reads data, which cannot fit in READ_RESPONSE > */

#define WE310F5_TXNS_WRITE          0
#define WE310F5_TXNS_READ           1

#define WIFI_INTERFACE_STATION_MODE 0
#define WIFI_INTERFACE_AP_MODE      1

#define WIFI_2POINT5_GHZ_20MHZ      2
#define WIFI_5_GHZ_40MHZ            3

typedef enum _AT_RESP
{
    AT_RESP_OK                         ,   /*!< Parsing and processing of commands done                >*/
    AT_RESP_ERROR                      ,   /*!< Successfully. Parsing was done but processing failed.  >*/
    AT_RESP_INVALID_PARAM              ,   /*!< Parsing of parameter failed, because one of the        >*/
                                           /*!< parameters entered is wrong-it may be because of a data>*/
                                           /*!< type mismatch.                                         >*/
    AT_RESP_INVALID_COMMAND            ,   /*!< Parsing of command failed as the command entered is    >*/
                                           /*!< not a valid command.                                   >*/
    AT_RESP_PARAMETER_OUT_OF_RANGE     ,   /*!< Parsing of parameter failed as the parameter value     >*/
                                           /*!< entered is not within the range.                       >*/
    AT_RESP_NO_MEMORY                  ,   /*!< Parsing or processing of command failed since the      >*/
                                           /*!< memory allocation failed >*/
    AT_RESP_EXCESS_DATA_RECEIVE        ,   /*!< Parsing of the parameter of type data failed because   >*/
                                           /*!< the data entered is excess compared to the length      >*/
                                           /*!< mentioned in the command                               >*/
    AT_RESP_OF_AT_CMD,
}AT_RESP;

typedef enum _AT_CMD_RESP {
    AT_CMD_RESP_EXPECT_ONLY_OK,
    AT_CMD_RESP_EXPECT_RESP_WITH_OK,
}AT_CMD_RESP;

typedef struct _WE310F5_Ctrl_Cmd {
    uint8_t         *msg;
    uint8_t         *msgResp;
    uint32_t        msgRespLen;
    uint32_t        msgRespAcceptedLen;  /* Device confirmed length to send as response*/
    AT_CMD_RESP     atCommandType;
    AT_RESP         atRespType;
    uint32_t        msgLen;
    uint32_t        terminateCond;
    uint32_t        respCnt;
}atCmdObj_t;

typedef struct _WIFI_DRV_STATE {
    uint32_t initialized : 2;   /*  Driver Initialized  */
    uint32_t powered     : 1;   /*  Driver powered      */
    uint32_t reserved    : 29;  /*  Reserved            */
} WIFI_DRV_STATE;

typedef struct _WIFI_RESOURCES {
    ARM_WIFI_SignalEvent_t          callBack;
    uint32_t                        timeoutMSretryCnt;
    uint32_t                        delayBetweenOpr;
    atCmdObj_t                      atCmd;
    uint8_t                         devInitDone;
    uint8_t                         interface;      // 0: STA, 1:AP mode
    uint32_t                        wifiDevReady;
    SPI_Packet_Frame_t              sendPacket;
    SPI_Packet_Frame_t              recvPacket;
    uint8_t                         aihPayload[WE310F4_SPI_PAYLOAD_SIZE];
    uint32_t                        aihPayloadLen;
    WIFI_DRV_STATE                  state;
    WIFI_OPTIONS                    wifiOptions[2];
}WIFI_RESOURCES;

/******************************************************************************/

/* Function Declarations */
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
int32_t we310f5_wait_cond(WIFI_RESOURCES *wifi, uint32_t waitCond);

/**
  \fn            int32_t we310f5_get_all_config(WIFI_RESOURCES *wifi)
  \brief         It will retrieve all Wi-Fi setting
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_get_all_config(WIFI_RESOURCES *wifi);

/**
  \fn            int32_t we310f5_cmd_transaction(WIFI_RESOURCES *wifi)
  \brief         It will send AT command to Telit chip and wait for response.
                        process the received response.
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
  \note         Critical : user must allocate and give response buff, otherwise
                it will fail.
*/
int32_t we310f5_cmd_transaction(WIFI_RESOURCES *wifi);

/**
  \fn            int32_t we310f5_start(WIFI_RESOURCES *wifi)
  \brief         It will start telit communication and wait for ready to use.
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/

int32_t we310f5_start(WIFI_RESOURCES *wifi);

#endif  /*   _WE310F5_H   */
