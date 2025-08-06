/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::USB:Device:CDC
 * Copyright (c) 2004-2020 Arm Limited (or its affiliates). All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    USBD_User_CDC_ACM_%Instance%.c
 * Purpose: USB Device Communication Device Class (CDC)
 *          Abstract Control Model (ACM) User module
 * Rev.:    V6.4.1
 *----------------------------------------------------------------------------
 */
/**
 * \addtogroup usbd_cdcFunctions
 *
 * USBD_User_CDC_ACM_%Instance%.c implements the application specific functionality
 * of the CDC ACM class and is used to receive and send data to the USB Host.
 *
 * The implementation must match the configuration file USBD_Config_CDC_%Instance%.h.
 * The following values in USBD_Config_CDC_%Instance%.h affect the user code:
 *
 *  - 'Maximum Communication Device Send Buffer Size' specifies the maximum
 *    value for \em len in \ref USBD_CDC_ACM_WriteData
 *
 *  - 'Maximum Communication Device Receive Buffer Size' specifies the maximum
 *    value for \em len in \ref USBD_CDC_ACM_ReadData
 *
 */

//! [code_USBD_User_CDC_ACM]

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "rl_usb.h"

#define UART_BUFFER_SIZE          (512)
/* USB read event */
#define USB_DATA_READ_EVENT_FLAG  1

static uint8_t  uart_tx_buf[UART_BUFFER_SIZE];
static uint32_t recv_len;
static void     *cdc_acm_thread_id;

/* Declare a variable to hold the created event group. */

osEventFlagsAttr_t usb_event_flag;
static void       *event_flag_id;

#ifdef RTE_CMSIS_RTOS2_RTX5
static osRtxThread_t cdc0_acm_uart_to_usb_thread_cb_mem
    __attribute__((section(".bss.os.thread.cb")));
static uint64_t cdc0_acm_uart_to_usb_thread_stack_mem[512U / 8U]
    __attribute__((section(".bss.os.thread.stack")));
#endif

static const osThreadAttr_t cdc0_acm_uart_to_usb_thread_attr = {
    "CDC0_ACM_UART_to_USB_Thread",
    0U,
#ifdef RTE_CMSIS_RTOS2_RTX5
    &cdc0_acm_uart_to_usb_thread_cb_mem,
    sizeof(osRtxThread_t),
    &cdc0_acm_uart_to_usb_thread_stack_mem[0],
#else
    NULL,
    0U,
    NULL,
#endif
    512U,
    osPriorityNormal,
    0U,
    0U};
// Local Variables
static CDC_LINE_CODING cdc_acm_line_coding = {9600U, 0U, 0U, 8U};

// Thread: Sends data received on UART to USB
// \param[in]     arg           not used.
__NO_RETURN static void CDC0_ACM_UART_to_USB_Thread(void *arg)
{
    int32_t  cnt;
    uint32_t ret;

    (void) (arg);

    for (;;) {
        /* waiting for USB read event */
        ret = osEventFlagsWait(event_flag_id, USB_DATA_READ_EVENT_FLAG, osFlagsWaitAny, osWaitForever);
        if (!ret) {
            printf("failed to get event\n");
        }
        cnt = USBD_CDC_ACM_WriteData(0U, uart_tx_buf, recv_len);
        if (cnt < 0) {
            printf("failed to write data to the host\n");
        }
    }
}

// Called during USBD_Initialize to initialize the USB CDC class instance (ACM).
void USBD_CDC0_ACM_Initialize(void)
{

    cdc_acm_thread_id =
        osThreadNew(CDC0_ACM_UART_to_USB_Thread, NULL, &cdc0_acm_uart_to_usb_thread_attr);
    if (cdc_acm_thread_id == NULL) {
        printf("Creation of Thread failed\n");
        return;
    }
    /* Create and Initialize an Event Flag */
    event_flag_id = osEventFlagsNew(&usb_event_flag);
    if (event_flag_id == NULL) {
        printf("Creation of event flag failed\n");
        return;
    }
}

// Called during USBD_Uninitialize to de-initialize the USB CDC class instance (ACM).
void USBD_CDC0_ACM_Uninitialize(void)
{
    // Add code for de-initialization
}

// Called upon USB Bus Reset Event.
void USBD_CDC0_ACM_Reset(void)
{
    // Add code for reset
}

// Callback function called upon reception of request send encapsulated command sent by the USB
// Host. \param[in]   buf           buffer that contains send encapsulated command request.
// \param[in]   len           length of send encapsulated command request.
// \return      true          send encapsulated command request processed.
// \return      false         send encapsulated command request not supported or not processed.
bool USBD_CDC0_ACM_SendEncapsulatedCommand(const uint8_t *buf, uint16_t len)
{
    (void) buf;
    (void) len;

    return true;
}

// Callback function called upon reception of request to get encapsulated response sent by the USB
// Host.
// \param[in]   max_len       maximum number of data bytes that USB Host expects to receive
// \param[out]  buf           pointer to buffer containing get encapsulated response to be returned
//                            to USB Host.
// \param[out]  len           pointer to number of data bytes to be returned to USB Host.
// \return      true          get encapsulated response request processed.
// \return      false         get encapsulated response request not supported or not processed.
bool USBD_CDC0_ACM_GetEncapsulatedResponse(uint16_t max_len, uint8_t **buf, uint16_t *len)
{
    (void) max_len;
    (void) buf;
    (void) len;

    return true;
}

// Called upon USB Host request to change communication settings.
// \param[in]   line_coding   pointer to CDC_LINE_CODING structure.
// \return      true          set line coding request processed.
// \return      false         set line coding request not supported or not processed.
bool USBD_CDC0_ACM_SetLineCoding(const CDC_LINE_CODING *line_coding)
{

    // Store requested settings to local variable
    cdc_acm_line_coding = *line_coding;

    return true;
}

// Called upon USB Host request to retrieve communication settings.
// \param[out]  line_coding   pointer to CDC_LINE_CODING structure.
// \return      true          get line coding request processed.
// \return      false         get line coding request not supported or not processed.
bool USBD_CDC0_ACM_GetLineCoding(CDC_LINE_CODING *line_coding)
{

    *line_coding = cdc_acm_line_coding;

    return true;
}

// Called upon USB Host request to set control line states.
// \param [in]  state         control line settings bitmap.
//                - bit 0: DTR state
//                - bit 1: RTS state
// \return      true          set control line state request processed.
// \return      false         set control line state request not supported or not processed.
bool USBD_CDC0_ACM_SetControlLineState(uint16_t state)
{
    // Add code for set control line state

    (void) (state);

    return true;
}

// Called when new data was received.
// \param [in]  len           number of bytes available for reading.
void USBD_CDC0_ACM_DataReceived(uint32_t len)
{
    int32_t  cnt;
    uint32_t ret;
    // Add code for handling new data reception
    (void) len;
    cnt = USBD_CDC_ACM_ReadData(0U, uart_tx_buf, UART_BUFFER_SIZE);
    if (cnt > 0) {
        recv_len = cnt;
        /* Set the USB read event flag */
        ret      = osEventFlagsSet(event_flag_id, USB_DATA_READ_EVENT_FLAG);
        if (ret != 0) {
            printf("failed to set the event\n");
        }
    }
}

// Called when all data was sent.
void USBD_CDC0_ACM_DataSent(void)
{
    // Add code for handling new data send
}

//! [code_USBD_User_CDC_ACM]
