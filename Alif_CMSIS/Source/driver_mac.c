/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     driver_mac.c
 * @author   Silesh C V
 * @email    silesh@alifsemi.com
 * @version  V1.1.0
 * @date     04-Feb-2026
 * @brief    CMSIS driver for ETH MAC.
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#include "RTE_Components.h"

#if defined(RTE_Drivers_ETH_MAC)

#include <string.h>
#include "driver_mac.h"
#include "sys_ctrl_eth.h"

/* Receive/transmit checksum offload enabled by default */
#ifndef EMAC_CHECKSUM_OFFLOAD
  #define EMAC_CHECKSUM_OFFLOAD 1
#endif

static MAC_DEV MAC0 = {
    .regs         = (volatile MAC_REGS *) ETH_BASE,
    .irq          = ETH_SBD_IRQ_IRQn,
    .irq_priority = RTE_ETH_MAC_IRQ_PRIORITY,
    .flags        = 0,
};

/* area for descriptors */
static DMA_DESC dma_descs[RX_DESC_COUNT + TX_DESC_COUNT] __attribute__((section("eth_buf")))
__attribute__((aligned(16)));
static uint32_t rx_buffers[RX_DESC_COUNT][ETH_BUF_SIZE >> 2] __attribute__((section("eth_buf")));
static uint32_t tx_buffers[TX_DESC_COUNT][ETH_BUF_SIZE >> 2] __attribute__((section("eth_buf")));

#define ARM_ETH_MAC_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 1) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {ARM_ETH_MAC_API_VERSION, ARM_ETH_MAC_DRV_VERSION};

/* Driver Capabilities */
static const ARM_ETH_MAC_CAPABILITIES DriverCapabilities = {
    (EMAC_CHECKSUM_OFFLOAD) ? 1U : 0U,  /* IPv4 header checksum verified on receive */
    (EMAC_CHECKSUM_OFFLOAD) ? 1U : 0U,  /* IPv6 checksum verification supported on receive */
    (EMAC_CHECKSUM_OFFLOAD) ? 1U : 0U,  /* UDP payload checksum verified on receive */
    (EMAC_CHECKSUM_OFFLOAD) ? 1U : 0U,  /* TCP payload checksum verified on receive */
    (EMAC_CHECKSUM_OFFLOAD) ? 1U : 0U,  /* ICMP payload checksum verified on receive */
    (EMAC_CHECKSUM_OFFLOAD) ? 1U : 0U,  /* IPv4 header checksum generated on transmit */
    (EMAC_CHECKSUM_OFFLOAD) ? 1U : 0U,  /* IPv6 checksum generation supported on transmit */
    (EMAC_CHECKSUM_OFFLOAD) ? 1U : 0U,  /* UDP payload checksum generated on transmit */
    (EMAC_CHECKSUM_OFFLOAD) ? 1U : 0U,  /* TCP payload checksum generated on transmit */
    (EMAC_CHECKSUM_OFFLOAD) ? 1U : 0U,  /* ICMP payload checksum generated on transmit */
    ARM_ETH_INTERFACE_RMII,             /* Ethernet Media Interface type */
    0,                                  /* driver provides initial valid MAC address */
    1,                                  /* callback event \ref ARM_ETH_MAC_EVENT_RX_FRAME generated */
    1,                                  /* callback event \ref ARM_ETH_MAC_EVENT_TX_FRAME generated */
    0,                                  /* wakeup event \ref ARM_ETH_MAC_EVENT_WAKEUP generated */
    0,                                  /* Precision Timer supported */
    0                                   /* Reserved (must be zero) */
};

/**
  \fn          void setup_rxdesc(MAC_DEV *dev, uint32_t desc_id)
  \brief       Setup a single Rx DMA descriptor.
  \param[in]   dev        Pointer to the MAC device instance
  \param[in]   desc_id    The descriptor id to setup
  \return      none.
*/
static void setup_rxdesc(MAC_DEV *dev, uint32_t desc_id)
{
    DMA_DESC *desc = &dev->rx_descs[desc_id];

    SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));

    desc->des0 = LocalToGlobal(&rx_buffers[desc_id]);
    desc->des3 = RDES3_OWN | RDES3_INT_ON_COMPLETION_EN | RDES3_BUFFER1_VALID_ADDR;

    SCB_CleanDCache_by_Addr(desc, sizeof(DMA_DESC));
}

/**
  \fn          void init_rx_descs(MAC_DEV *dev)
  \brief       Initialize Rx DMA descriptors.
  \param[in]   dev        Pointer to the MAC device instance
  \return      none.
*/
static void init_rx_descs(MAC_DEV *dev)
{
    uint32_t i;

    for (i = 0; i < RX_DESC_COUNT; i++) {
        setup_rxdesc(dev, i);
    }

    dev->regs->DMA_CH0_RX_BASE_ADDR = LocalToGlobal(dev->rx_descs);
    dev->regs->DMA_CH0_RX_RING_LEN  = RX_DESC_COUNT - 1;
    dev->regs->DMA_CH0_RX_END_ADDR  = LocalToGlobal(&dev->rx_descs[RX_DESC_COUNT - 1]);
}

/**
  \fn          void init_tx_desc (MAC_DEV *dev)
  \brief       Initialize Tx DMA descriptors.
  \param[in]   dev        Pointer to the MAC device instance
  \return      none.
*/
static void init_tx_descs(MAC_DEV *dev)
{
    uint32_t i;

    for (i = 0; i < TX_DESC_COUNT; i++) {
        dev->tx_descs[i] = (DMA_DESC){0, 0, 0, 0};
    }

    dev->regs->DMA_CH0_TX_BASE_ADDR = LocalToGlobal(dev->tx_descs);
    dev->regs->DMA_CHO_TX_RING_LEN  = TX_DESC_COUNT - 1;

    SCB_CleanDCache_by_Addr(dev->tx_descs, TX_DESC_COUNT * sizeof(DMA_DESC));
}

/**
  \fn          void init_descriptors(MAC_DEV *dev)
  \brief       Initialize DMA descriptors.
  \param[in]   dev        Pointer to the MAC device instance
  \return      none.
*/
static void init_descriptors(MAC_DEV *dev)
{
    dev->descs    = dma_descs;
    dev->tx_descs = dev->descs;
    dev->rx_descs = dev->tx_descs + TX_DESC_COUNT;

    init_rx_descs(dev);
    init_tx_descs(dev);
}

/**
  \fn          static int32_t mac_hw_init(MAC_DEV *dev)
  \brief       Initialize the MAC hardware.
  \param[in]   dev        Pointer to the MAC device instance
  \return      \ref execution_status.
*/
static int32_t mac_hw_init(MAC_DEV *dev)
{
    uint32_t val;
    uint32_t timeout = 1000;

    /* Soft reset the logic */
    dev->regs->DMA_BUS_MODE |= DMA_BUS_MODE_SFT_RESET;

    do {
        sys_busy_loop_us(100);
        timeout--;
    } while ((dev->regs->DMA_BUS_MODE & DMA_BUS_MODE_SFT_RESET) && timeout);

    if (!timeout) {
        return ARM_DRIVER_ERROR;
    }

    /* Configure MTL Tx Q0 Operating mode */
#if (EMAC_CHECKSUM_OFFLOAD)
    dev->regs->MTL_TXQ0_OP_MODE |= MTL_OP_MODE_TSF;
#endif

    /* Configure MTL RX Q0 operating mode */
    dev->regs->MTL_RXQ0_OP_MODE = (MTL_OP_MODE_FEP | MTL_OP_MODE_FUP);
#if (EMAC_CHECKSUM_OFFLOAD)
    dev->regs->MTL_RXQ0_OP_MODE |= MTL_OP_MODE_RSF;
#endif

    /* Configure tx and rx flow control */
    dev->regs->MAC_Q0_TX_FLOW_CTRL |=
        0xffff << MAC_Q0_TX_FLOW_CTRL_PT_SHIFT | MAC_Q0_TX_FLOW_CTRL_TFE;
    dev->regs->MAC_RX_FLOW_CTRL  |= MAC_RX_FLOW_CTRL_RFE;

    dev->regs->MAC_CONFIG         = (MAC_CONFIG_CST | MAC_CONFIG_ACS);

    /* Configure the DMA block */
    dev->regs->DMA_CH0_TX_CTRL   |= (16 << DMA_CH0_TX_CONTROL_TXPBL_SHIFT);

    dev->regs->DMA_CH0_RX_CTRL   |=
        ((16 << DMA_CH0_RX_CONTROL_RXPBL_SHIFT) | (2048 << DMA_CH0_RX_CONTROL_RBSZ_SHIFT));

    val  = dev->regs->DMA_SYS_BUS_MODE;
    val |= DMA_SYSBUS_MODE_BLEN4 | DMA_SYSBUS_MODE_BLEN8 | DMA_SYSBUS_MODE_BLEN16;
    val |= 3 << DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT;
    val |= 1 << DMA_SYSBUS_MODE_WR_OSR_LMT_SHIFT;
    val |= DMA_SYSBUS_MODE_ONEKBBE;
    dev->regs->DMA_SYS_BUS_MODE = val;

    init_descriptors(dev);

    /* Enable DMA Channel 0 interrupts, rx and tx */
    dev->regs->DMA_CH0_INT_ENABLE |=
        (DMA_CHAN_INTR_ENA_RIE | DMA_CHAN_INTR_ENA_NIE | DMA_CHAN_INTR_ENA_TIE);

    dev->regs->DMA_CH0_TX_CTRL |= DMA_CH0_TX_CONTROL_ST;
    dev->regs->DMA_CH0_RX_CTRL |= DMA_CH0_RX_CONTROL_SR;

    dev->regs->MAC_CONFIG      |= (MAC_CONFIG_RE | MAC_CONFIG_TE);

    return ARM_DRIVER_OK;
}

/* Ethernet Driver functions */

/**
  \fn          ARM_DRIVER_VERSION GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION ETH_MAC_GetVersion(void)
{
    return DriverVersion;
}

/**
  \fn          ARM_ETH_MAC_CAPABILITIES GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_ETH_MAC_CAPABILITIES
*/
static ARM_ETH_MAC_CAPABILITIES ETH_MAC_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
  \fn          int32_t Initialize (ARM_ETH_MAC_SignalEvent_t cb_event, MAC_DEV *dev)
  \brief       Initialize Ethernet MAC Device.
  \param[in]   cb_event  Pointer to \ref ARM_ETH_MAC_SignalEvent
  \param[in]   dev       Pointer to the MAC device instance
  \return      \ref execution_status
*/
static int32_t Initialize(ARM_ETH_MAC_SignalEvent_t cb_event, MAC_DEV *dev)
{
    dev->cb_event   = cb_event;
    dev->rx_desc_id = 0U;
    dev->tx_desc_id = 0U;
    dev->tx_len     = 0U;

    dev->flags      = ETH_INIT;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Uninitialize (MAC_DEV *dev)
  \brief       De-initialize Ethernet MAC Device.
  \param[in]   dev       Pointer to the MAC device instance
  \return      \ref execution_status
*/
static int32_t Uninitialize(MAC_DEV *dev)
{
    dev->flags = 0U;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PowerControl (ARM_POWER_STATE state, MAC_DEV *dev)
  \brief       Control Ethernet MAC Device Power.
  \param[in]   state  Power state
  \param[in]   dev       Pointer to the MAC device instance
  \return      \ref execution_status
*/
static int32_t PowerControl(ARM_POWER_STATE state, MAC_DEV *dev)
{
    int32_t ret, val;

    switch (state) {
    case ARM_POWER_OFF:
        if (!(dev->flags & ETH_POWER)) {
            break;
        }

        /* Disable Ethernet peripheral clock */
        disable_eth_periph_clk();

        NVIC_DisableIRQ(dev->irq);

        dev->flags &= ~ETH_POWER;
        break;

    case ARM_POWER_LOW:
        return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
        if (!(dev->flags & ETH_INIT)) {
            return ARM_DRIVER_ERROR;
        }

        if (dev->flags & ETH_POWER) {
            break;
        }

        /* Enable Ethernet peripheral clock */
        enable_eth_periph_clk();

        ret = mac_hw_init(dev);

        if (ret < 0) {
            return ARM_DRIVER_ERROR;
        }

        NVIC_ClearPendingIRQ(dev->irq);
        NVIC_SetPriority(dev->irq, dev->irq_priority);
        NVIC_EnableIRQ(dev->irq);

        dev->flags |= ETH_POWER;
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr, MAC_DEV *dev)
  \brief       Get Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \param[in]   dev       Pointer to the MAC device instance
  \return      \ref execution_status
*/
static int32_t GetMacAddress(ARM_ETH_MAC_ADDR *ptr_addr, MAC_DEV *dev)
{
    uint32_t hi, lo;

    if (!ptr_addr) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (!(dev->flags & ETH_POWER)) {
        return ARM_DRIVER_ERROR;
    }

    lo             = dev->regs->MAC_ADDR_LOW_0;
    hi             = dev->regs->MAC_ADDR_HIGH_0 & 0xFFFF;

    ptr_addr->b[0] = (lo >> 0) & 0xff;
    ptr_addr->b[1] = (lo >> 8) & 0xff;
    ptr_addr->b[2] = (lo >> 16) & 0xff;
    ptr_addr->b[3] = (lo >> 24) & 0xff;
    ptr_addr->b[4] = (hi >> 0) & 0xff;
    ptr_addr->b[5] = (hi >> 8) & 0xff;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr, MAC_DEV *dev)
  \brief       Set Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \param[in]   dev       Pointer to MAC device instance
  \return      \ref execution_status
*/
static int32_t SetMacAddress(const ARM_ETH_MAC_ADDR *ptr_addr, MAC_DEV *dev)
{
    uint32_t hi, lo;

    if (!ptr_addr) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (!(dev->flags & ETH_POWER)) {
        return ARM_DRIVER_ERROR;
    }

    hi = ((ptr_addr->b[5] << 8) | ptr_addr->b[4]) | MAC_ADDR_HIGH_AE;

    lo = (ptr_addr->b[3] << 24) | (ptr_addr->b[2] << 16) | (ptr_addr->b[1] << 8) | ptr_addr->b[0];

    dev->regs->MAC_ADDR_LOW_0  = lo;
    dev->regs->MAC_ADDR_HIGH_0 = hi;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr,
                                               uint32_t          num_addr)
  \brief       Configure Address Filter.
  \param[in]   ptr_addr  Pointer to addresses
  \param[in]   num_addr  Number of addresses to configure
  \param[in]   dev       Pointer to MAC device instance
  \return      \ref execution_status
*/
static int32_t SetAddressFilter(const ARM_ETH_MAC_ADDR *ptr_addr, uint32_t num_addr, MAC_DEV *dev)
{
    (void)ptr_addr;
    (void)num_addr;

    /* Not supported by ETH module */
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags,
                                    MAC_DEV *dev)
  \brief       Send Ethernet frame.
  \param[in]   frame  Pointer to frame buffer with data to send
  \param[in]   len    Frame buffer length in bytes
  \param[in]   flags  Frame transmit flags (see ARM_ETH_MAC_TX_FRAME_...)
  \param[in]   dev    Pointer to the MAC device instance
  \return      \ref execution_status
*/
static int32_t SendFrame(const uint8_t *frame, uint32_t len, uint32_t flags, MAC_DEV *dev)
{
    uint32_t  cur_idx;
    DMA_DESC *desc;

    if (!frame || !len) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (!(dev->flags & ETH_POWER)) {
        return ARM_DRIVER_ERROR;
    }

    cur_idx = dev->tx_desc_id;
    desc    = &dev->tx_descs[cur_idx];

    SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));

    if (dev->tx_len == 0U) {
        /* new frame */
        if (desc->des3 & TDES3_OWN) {
            return ARM_DRIVER_ERROR_BUSY;
        }
    }

    /* Copy the frame to the buffer */
    memcpy ((void *)&tx_buffers[cur_idx] + dev->tx_len, frame, len);
    dev->tx_len += len;

    if (flags & ARM_ETH_MAC_TX_FRAME_FRAGMENT) {
        /* More data to come */
        return ARM_DRIVER_OK;
    }

    SCB_CleanDCache_by_Addr(tx_buffers[cur_idx], dev->tx_len);

    dev->tx_desc_id++;
    dev->tx_desc_id %= TX_DESC_COUNT;
    desc->des0       = LocalToGlobal(tx_buffers[cur_idx]);
    desc->des2       = TDES2_INTERRUPT_ON_COMPLETION | dev->tx_len;
    desc->des3       = TDES3_OWN | TDES3_LAST_DESCRIPTOR | TDES3_FIRST_DESCRIPTOR | dev->tx_len;
#if (EMAC_CHECKSUM_OFFLOAD)
    desc->des3      |= 0x3 << TDES3_CHECKSUM_INSERTION_SHIFT;
#endif
    SCB_CleanDCache_by_Addr(desc, sizeof(DMA_DESC));

    dev->regs->DMA_CH0_TX_END_ADDR = LocalToGlobal((void *)&dev->tx_descs[dev->tx_desc_id]);

    dev->tx_len      = 0U;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ReadFrame (uint8_t *frame, uint32_t len, MAC_DEV *dev)
  \brief       Read data of received Ethernet frame.
  \param[in]   frame  Pointer to frame buffer for data to read into
  \param[in]   len    Frame buffer length in bytes
  \param[in]   dev    Pointer to the MAC device instance
  \return      number of data bytes read or execution status
                 - value >= 0: number of data bytes read
                 - value < 0: error occurred, value is execution status as defined with \ref
  execution_status
*/
static int32_t ReadFrame(uint8_t *frame, uint32_t len, MAC_DEV *dev)
{
    uint32_t cur_idx;

    if (!frame && len) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (!(dev->flags & ETH_POWER)) {
        return ARM_DRIVER_ERROR;
    }

    cur_idx = dev->rx_desc_id;

    SCB_InvalidateDCache_by_Addr(rx_buffers[cur_idx], len);

    /* copy data to the buffer */
    memcpy (frame, rx_buffers[cur_idx], len);

    /* refresh the descriptor */
    setup_rxdesc(dev, cur_idx);

    dev->regs->DMA_CH0_RX_END_ADDR = LocalToGlobal(&dev->rx_descs[cur_idx]);

    dev->rx_desc_id++;
    dev->rx_desc_id %= RX_DESC_COUNT;

    return (int32_t)len;
}

/**
  \fn          uint32_t GetRxFrameSize (MAC_DEV *dev)
  \param[in]   dev    Pointer to the MAC device instance
  \brief       Get size of received Ethernet frame.
  \return      number of bytes in received frame
*/
static uint32_t GetRxFrameSize(MAC_DEV *dev)
{
    DMA_DESC *desc;

    if (!(dev->flags & ETH_POWER)) {
        return ARM_DRIVER_ERROR;
    }

    desc = &dev->rx_descs[dev->rx_desc_id];

    SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));

    if (desc->des3 & RDES3_OWN) {
        return 0;
    }

    if (desc->des3 & RDES3_ERROR_SUMMARY) {
        return ARM_DRIVER_ERROR;
    }
    if (!(desc->des3 & RDES3_FIRST_DESCRIPTOR) || !(desc->des3 & RDES3_LAST_DESCRIPTOR)) {
        return ARM_DRIVER_ERROR;
    }

    if ((dev->flags & ETH_VLAN) && (desc->des3 & RDES3_PACKET_LEN_TYPE_MASK) != RDES3_PACKET_LEN_TYPE_VLAN) {
        /* type packet with no VLAN tag */
        return ARM_DRIVER_ERROR;
    }
    return (desc->des3 & RDES3_PACKET_SIZE_MASK);
}

/**
  \fn          int32_t GetRxFrameTime (ARM_ETH_MAC_TIME *time, MAC_DEV *dev)
  \brief       Get time of received Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \param[in]   dev    Pointer to the MAC device instance
  \return      \ref execution_status
*/
static int32_t GetRxFrameTime(ARM_ETH_MAC_TIME *time, MAC_DEV *dev)
{
    DMA_DESC *desc, *next_desc;
    uint32_t  cur_idx = dev->rx_desc_id, next_idx;
    uint32_t  own, ctxt;

    if (!time) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (!(dev->flags & ETH_POWER)) {
        return ARM_DRIVER_ERROR;
    }

    desc      = &dev->rx_descs[cur_idx];
    next_idx  = (cur_idx + 1) % RX_DESC_COUNT;
    next_desc = &dev->rx_descs[next_idx];

    SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));

    if (desc->des3 & RDES3_OWN) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* get preliminary status from current normal w/b descriptor */
    if (!(desc->des3 & RDES3_RDES1_VALID)) {
        return ARM_DRIVER_ERROR;
    }

    if (!(desc->des1 & RDES1_TIMESTAMP_AVAILABLE)) {
        return ARM_DRIVER_ERROR;
    }

    /* OK, move on to the context descriptor */
    own  = next_desc->des3 & RDES3_OWN;
    ctxt = ((next_desc->des3 & RDES3_CONTEXT_DESCRIPTOR) >> RDES3_CONTEXT_DESCRIPTOR_SHIFT);

    if (!own && ctxt) {
        if ((next_desc->des0 == 0xffffffff) && (next_desc->des1 == 0xffffffff)) {
            /* Corrupted timestamp */
            return ARM_DRIVER_ERROR;
        }

        time->ns  = next_desc->des0;
        time->sec = next_desc->des1;
        return ARM_DRIVER_OK;
    }

    return ARM_DRIVER_ERROR;
}

/**
  \fn          int32_t GetTxFrameTime (ARM_ETH_MAC_TIME *time, MAC_DEV *dev)
  \brief       Get time of transmitted Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \param[in]   dev   Pointer to the MAC device instance
  \return      \ref execution_status
*/
static int32_t GetTxFrameTime(ARM_ETH_MAC_TIME *time, MAC_DEV *dev)
{
    DMA_DESC *desc;

    if (!time) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (!(dev->flags & ETH_POWER)) {
        return ARM_DRIVER_ERROR;
    }

    desc = &dev->tx_descs[dev->tx_desc_id];

    SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));

    if (desc->des3 & TDES3_OWN) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (desc->des3 & TDES3_CONTEXT_TYPE) {
        return ARM_DRIVER_ERROR;
    }

    if (desc->des3 & TDES3_TIMESTAMP_STATUS) {
        time->ns  = desc->des0;
        time->sec = desc->des1;
        return ARM_DRIVER_OK;
    }

    return ARM_DRIVER_ERROR;
}

/**
  \fn          int32_t Control (uint32_t control, uint32_t arg, MAC_DEV *dev)
  \brief       Control Ethernet Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \param[in]   dev      Pointer to the MAC device instance
  \return      \ref execution_status
*/
static int32_t Control(uint32_t control, uint32_t arg, MAC_DEV *dev)
{
    uint32_t val, reg;

    if (!(dev->flags & ETH_POWER)) {
        return ARM_DRIVER_ERROR;
    }

    switch (control) {
    case ARM_ETH_MAC_CONFIGURE:
        val = dev->regs->MAC_CONFIG & ~(MAC_CONFIG_FES | MAC_CONFIG_LM | MAC_CONFIG_DM);

        switch (arg & ARM_ETH_MAC_SPEED_Msk) {
        case ARM_ETH_MAC_SPEED_10M:
            /* Nothing to do here as FES is already cleared */
            break;
        case ARM_ETH_SPEED_100M:
            val |= MAC_CONFIG_FES;
            break;
        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        switch (arg & ARM_ETH_MAC_DUPLEX_Msk) {
        case ARM_ETH_MAC_DUPLEX_FULL:
            val |= MAC_CONFIG_DM;
            break;
        }

        if (arg & ARM_ETH_MAC_LOOPBACK) {
            val |= MAC_CONFIG_LM;
        }

#if (EMAC_CHECKSUM_OFFLOAD)
        /* rx checksum verification */
        if (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_RX) {
            dev->regs->MTL_RXQ0_OP_MODE |=  MTL_OP_MODE_RSF;
            dev->regs->MAC_CONFIG       |=  MAC_CONFIG_IPC;
        } else {
            dev->regs->MTL_RXQ0_OP_MODE &= ~MTL_OP_MODE_RSF;
            dev->regs->MAC_CONFIG       &= ~MAC_CONFIG_IPC;
        }

        /* tx checksum generation */
        if (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_TX) {
            dev->regs->MTL_TXQ0_OP_MODE |=  MTL_OP_MODE_TSF;
        } else {
            dev->regs->MTL_TXQ0_OP_MODE &= ~MTL_OP_MODE_TSF;
        }
#else
        if ((arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_RX) || (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_TX)) {
            /* Checksum offload is disabled in the driver */
            return ARM_DRIVER_ERROR;
        }
#endif

        dev->regs->MAC_CONFIG = val;

        val                   = (dev->regs->MAC_PACKET_FILTER) &
              ~(MAC_PACKET_FILTER_PR | MAC_PACKET_FILTER_PM | MAC_PACKET_FILTER_DBF);

        /* Disable broadcast frame reception */
        if (!(arg & ARM_ETH_MAC_ADDRESS_BROADCAST)) {
            val |= MAC_PACKET_FILTER_DBF;
        }

        /* Enable multicast frame reception */
        if (arg & ARM_ETH_MAC_ADDRESS_MULTICAST) {
            val |= MAC_PACKET_FILTER_PM;
        }

        /* Promiscuous mode */
        if (arg & ARM_ETH_MAC_ADDRESS_ALL) {
            val |= MAC_PACKET_FILTER_PR;
        }

        dev->regs->MAC_PACKET_FILTER = val;
        break;

    case ARM_ETH_MAC_CONTROL_TX:
        val = dev->regs->MAC_CONFIG;
        reg = dev->regs->DMA_CH0_TX_CTRL;
        if (arg != 0) {
            val                        |= MAC_CONFIG_TE;
            reg                        |= DMA_CONTROL_ST;
            dev->regs->MAC_CONFIG       = val;
            dev->regs->DMA_CH0_TX_CTRL  = reg;
        } else {
            reg                        &= ~DMA_CONTROL_ST;
            val                        &= ~MAC_CONFIG_TE;
            dev->regs->DMA_CH0_TX_CTRL  = reg;
            dev->regs->MAC_CONFIG       = val;
        }
        break;

    case ARM_ETH_MAC_CONTROL_RX:
        val = dev->regs->MAC_CONFIG;
        reg = dev->regs->DMA_CH0_RX_CTRL;
        if (arg != 0) {
            val                        |= MAC_CONFIG_RE;
            reg                        |= DMA_CONTROL_SR;
            dev->regs->MAC_CONFIG       = val;
            dev->regs->DMA_CH0_RX_CTRL  = reg;
        } else {
            reg                        &= ~DMA_CONTROL_SR;
            val                        &= ~MAC_CONFIG_RE;
            dev->regs->DMA_CH0_RX_CTRL  = reg;
            dev->regs->MAC_CONFIG       = val;
        }
        break;

    case ARM_ETH_MAC_FLUSH:
        if (arg & ARM_ETH_MAC_FLUSH_RX) {
            val                         = dev->regs->DMA_CH0_RX_CTRL;
            reg                         = val;
            val                        &= ~DMA_CONTROL_SR;
            dev->regs->DMA_CH0_RX_CTRL  = val;
            init_rx_descs(dev);
            dev->regs->DMA_CH0_RX_CTRL = reg;
        }
        if (arg & ARM_ETH_MAC_FLUSH_TX) {
            val                         = dev->regs->DMA_CH0_TX_CTRL;
            reg                         = val;
            val                        &= ~DMA_CONTROL_ST;
            dev->regs->DMA_CH0_TX_CTRL  = val;
            init_tx_descs(dev);
            dev->regs->DMA_CH0_TX_CTRL = reg;
        }
        break;

    case ARM_ETH_MAC_SLEEP:
        if (arg != 0) {
            dev->regs->DMA_CH0_TX_CTRL  &= ~DMA_CONTROL_ST;
            dev->regs->MAC_CONFIG       &= ~(MAC_CONFIG_TE | MAC_CONFIG_RE);
            dev->regs->MAC_INT_ENABLE   |= MAC_INT_EN_PMTIE;
            dev->regs->MAC_CONFIG       |= MAC_CONFIG_RE;
            dev->regs->MAC_PMT_CTRL_STS |= (MAC_PMT_CTRL_STS_MGKPKTEN | MAC_PMT_CTRL_STS_PWRDWN);
        } else {
            dev->regs->MAC_INT_ENABLE   &= ~MAC_INT_EN_PMTIE;
            dev->regs->MAC_PMT_CTRL_STS  = 0x0;
        }
        break;

    case ARM_ETH_MAC_VLAN_FILTER:
        if (arg != 0U) {
            val = MAC_VLAN_TAG_DOVLTC | MAC_VLAN_TAG_EVLRXS;
            if (arg & ARM_ETH_MAC_VLAN_FILTER_ID_ONLY) {
                val |= MAC_VLAN_TAG_ETV;
            }
            dev->regs->MAC_VLAN_TAG       = val | (arg & MAC_VLAN_TAG_VL);
            dev->regs->MAC_PACKET_FILTER |= MAC_PACKET_FILTER_VTFE;
            dev->flags                   |= ETH_VLAN;
        } else {
            dev->regs->MAC_VLAN_TAG       =  0U;
            dev->regs->MAC_PACKET_FILTER &= ~MAC_PACKET_FILTER_VTFE;
            dev->flags                   &= ~ETH_VLAN;
        }
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time, MAC_DEV *dev)
  \brief       Control Precision Timer.
  \param[in]   control  Operation
  \param[in]   time     Pointer to time structure
  \param[in]   dev      Pointer to the MAC device instance
  \return      \ref ARM_DRIVER_ERROR_UNSUPPORTED
*/
static int32_t ControlTimer(uint32_t control, ARM_ETH_MAC_TIME *time, MAC_DEV *dev)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data, MAC_DEV *dev)
  \brief       Read Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[out]  data      Pointer where the result is written to
  \param[in]   dev       Pointer to the MAC device instance
  \return      \ref execution_status
*/
static int32_t PHY_Read(uint8_t phy_addr, uint8_t reg_addr, uint16_t *data, MAC_DEV *dev)
{
    uint32_t val, timeout = 100;

    if (!data) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (!(dev->flags & ETH_POWER)) {
        return ARM_DRIVER_ERROR;
    }

    val = (phy_addr << MAC_MDIO_ADDR_PA_SHIFT) | (reg_addr << MAC_MDIO_ADDR_RDA_SHIFT) |
          (MAC_MDIO_ADDR_GOC_READ << MAC_MDIO_ADDR_GOC_SHIFT) |
          (MAC_MDIO_ADDR_CR_150_250 << MAC_MDIO_ADDR_CR_SHIFT) | MAC_MDIO_ADDR_GB;

    dev->regs->MAC_MDIO_ADDR = val;

    do {
        if (!(dev->regs->MAC_MDIO_ADDR & MAC_MDIO_ADDR_GB)) {
            *data = dev->regs->MAC_MDIO_DATA;
            break;
        }
        sys_busy_loop_us(25);
        timeout--;
    } while (timeout);

    if (!timeout) {
        return ARM_DRIVER_ERROR_TIMEOUT;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data,
                                                    MAC_DEV *dev)
  \brief       Write Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[in]   data      16-bit data to write
  \param[in]   dev       Pointer to the MAC device instance
  \return      \ref execution_status
*/
static int32_t PHY_Write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data, MAC_DEV *dev)
{
    uint32_t val, timeout = 100;

    if (!(dev->flags & ETH_POWER)) {
        return ARM_DRIVER_ERROR;
    }

    val = (phy_addr << MAC_MDIO_ADDR_PA_SHIFT) | (reg_addr << MAC_MDIO_ADDR_RDA_SHIFT) |
          (MAC_MDIO_ADDR_GOC_WRITE << MAC_MDIO_ADDR_GOC_SHIFT) |
          (MAC_MDIO_ADDR_CR_150_250 << MAC_MDIO_ADDR_CR_SHIFT) | MAC_MDIO_ADDR_GB;

    dev->regs->MAC_MDIO_DATA = data;
    dev->regs->MAC_MDIO_ADDR = val;

    do {
        if (!(dev->regs->MAC_MDIO_ADDR & MAC_MDIO_ADDR_GB)) {
            break;
        }
        sys_busy_loop_us(25);
        timeout--;
    } while (timeout);

    if (!timeout) {
        return ARM_DRIVER_ERROR_TIMEOUT;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          void ETH_IRQHandler (MAC_DEV *dev)
  \param[in]   dev       Pointer to the MAC device instance
  \brief       MAC instance specific part of Ethernet Interrupt handler.
*/
static void ETH_IRQHandler(MAC_DEV *dev)
{
    uint32_t ch0_stat, event = 0;

    if (dev->regs->DMA_STATUS & DMA_STATUS_CHAN0) {
        ch0_stat = dev->regs->DMA_CH0_STATUS;

        dev->regs->DMA_CH0_STATUS =
            ch0_stat & (DMA_CHAN_STATUS_NIS | DMA_CHAN_STATUS_RI | DMA_CHAN_STATUS_TI);

        if (ch0_stat & DMA_CHAN_STATUS_RI) {
            event |= ARM_ETH_MAC_EVENT_RX_FRAME;
        }

        if (ch0_stat & DMA_CHAN_STATUS_TI) {
            event |= ARM_ETH_MAC_EVENT_TX_FRAME;
        }

        if (event && dev->cb_event) {
            dev->cb_event(event);
        }
    }
}

/**
  \fn          void ETH_SBD_IRQHandler (void)
  \brief       Ethernet Interrupt handler.
*/
void ETH_SBD_IRQHandler(void)
{
    ETH_IRQHandler(&MAC0);
}

/**
  \fn          int32_t ETH_MAC0_Initialize (ARM_ETH_MAC_SignalEvent_t cb_event)
  \brief       Initialize Ethernet MAC Device.
  \param[in]   cb_event  Pointer to \ref ARM_ETH_MAC_SignalEvent
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_Initialize(ARM_ETH_MAC_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &MAC0);
}

/**
  \fn          int32_t ETH_MAC0_Uninitialize (void)
  \brief       De-initialize Ethernet MAC Device.
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_Uninitialize(void)
{
    return Uninitialize(&MAC0);
}

/**
  \fn          int32_t ETH_MAC0_PowerControl (ARM_POWER_STATE state)
  \brief       Control Ethernet MAC Device Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_PowerControl(ARM_POWER_STATE state)
{
    return PowerControl(state, &MAC0);
}

/**
  \fn          int32_t ETH_MAC0_GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Get Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_GetMacAddress(ARM_ETH_MAC_ADDR *ptr_addr)
{
    return GetMacAddress(ptr_addr, &MAC0);
}

/**
  \fn          int32_t ETH_MAC0_SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Set Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_SetMacAddress(const ARM_ETH_MAC_ADDR *ptr_addr)
{
    return SetMacAddress(ptr_addr, &MAC0);
}

/**
  \fn          int32_t ETH_MAC0_SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr,
                                               uint32_t          num_addr)
  \brief       Configure Address Filter.
  \param[in]   ptr_addr  Pointer to addresses
  \param[in]   num_addr  Number of addresses to configure
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_SetAddressFilter(const ARM_ETH_MAC_ADDR *ptr_addr, uint32_t num_addr)
{
    return SetAddressFilter(ptr_addr, num_addr, &MAC0);
}

/**
  \fn          int32_t ETH_MAC0_SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags)
  \brief       Send Ethernet frame.
  \param[in]   frame  Pointer to frame buffer with data to send
  \param[in]   len    Frame buffer length in bytes
  \param[in]   flags  Frame transmit flags (see ARM_ETH_MAC_TX_FRAME_...)
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_SendFrame(const uint8_t *frame, uint32_t len, uint32_t flags)
{
    return SendFrame(frame, len, flags, &MAC0);
}

/**
  \fn          int32_t ETH_MAC0_ReadFrame (uint8_t *frame, uint32_t len)
  \brief       Read data of received Ethernet frame.
  \param[in]   frame  Pointer to frame buffer for data to read into
  \param[in]   len    Frame buffer length in bytes
  \return      number of data bytes read or execution status
                 - value >= 0: number of data bytes read
                 - value < 0: error occurred, value is execution status as defined with \ref
  execution_status
*/
static int32_t ETH_MAC0_ReadFrame(uint8_t *frame, uint32_t len)
{
    return ReadFrame(frame, len, &MAC0);
}

/**
  \fn          uint32_t ETH_MAC0_GetRxFrameSize (void)
  \brief       Get size of received Ethernet frame.
  \return      number of bytes in received frame
*/
static uint32_t ETH_MAC0_GetRxFrameSize(void)
{
    return GetRxFrameSize(&MAC0);
}

/**
  \fn          int32_t ETH_MAC0_GetRxFrameTime (ARM_ETH_MAC_TIME *time)
  \brief       Get time of received Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_GetRxFrameTime(ARM_ETH_MAC_TIME *time)
{
    return GetRxFrameTime(time, &MAC0);
}

/**
  \fn          int32_t ETH_MAC0_Control (uint32_t control, uint32_t arg)
  \brief       Control Ethernet Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_GetTxFrameTime(ARM_ETH_MAC_TIME *time)
{
    return GetTxFrameTime(time, &MAC0);
}

/**
  \fn          int32_t ETH_MAC0_Control (uint32_t control, uint32_t arg)
  \brief       Control Ethernet Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_Control(uint32_t control, uint32_t arg)
{
    return Control(control, arg, &MAC0);
}

/**
  \fn          int32_t ETH_MAC0_ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time)
  \brief       Control Precision Timer.
  \param[in]   control  Operation
  \param[in]   time     Pointer to time structure
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_ControlTimer(uint32_t control, ARM_ETH_MAC_TIME *time)
{
    return ControlTimer(control, time, &MAC0);
}

/**
  \fn          int32_t ETH_MAC0_PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data)
  \brief       Read Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[out]  data      Pointer where the result is written to
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_PHY_Read(uint8_t phy_addr, uint8_t reg_addr, uint16_t *data)
{
    return PHY_Read(phy_addr, reg_addr, data, &MAC0);
}

/**
  \fn          int32_t ETH_MAC0_PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
  \brief       Write Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[in]   data      16-bit data to write
  \return      \ref execution_status
*/
static int32_t ETH_MAC0_PHY_Write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
{
    return PHY_Write(phy_addr, reg_addr, data, &MAC0);
}

ARM_DRIVER_ETH_MAC Driver_ETH_MAC0 = {
    ETH_MAC_GetVersion,
    ETH_MAC_GetCapabilities,
    ETH_MAC0_Initialize,
    ETH_MAC0_Uninitialize,
    ETH_MAC0_PowerControl,
    ETH_MAC0_GetMacAddress,
    ETH_MAC0_SetMacAddress,
    ETH_MAC0_SetAddressFilter,
    ETH_MAC0_SendFrame,
    ETH_MAC0_ReadFrame,
    ETH_MAC0_GetRxFrameSize,
    ETH_MAC0_GetRxFrameTime,
    ETH_MAC0_GetTxFrameTime,
    ETH_MAC0_ControlTimer,
    ETH_MAC0_Control,
    ETH_MAC0_PHY_Read,
    ETH_MAC0_PHY_Write
};
#endif /* RTE_Drivers_ETH_MAC */
