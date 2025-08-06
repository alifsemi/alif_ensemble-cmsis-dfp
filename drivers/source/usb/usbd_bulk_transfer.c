/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     usbd_bulk_transfer.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     13-March-2024
 * @brief    This file will transfers the BULK IN and BULK OUT data
 * @bug      None
 * @Note     None
 ******************************************************************************/

/* Include necessary system files.  */

#include "usbd.h"

/**
 *\fn           usbd_bulk_send
 *\brief        This function will prepare the TRB for sending bulk data to the host.
 *\param[in]    pointer to the controller context structure
 *\param[in]    endpoint number
 *\param[in]    endpoint direction
 *\param[in]    buffer pointer
 *\param[in]    buffer length
 *\return       On success 0, failure case returns error.
 */
int32_t usbd_bulk_send(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t *bufferptr,
                       uint32_t buf_len)
{
    USBD_EP       *ept;
    USBD_TRB      *trb_ptr;
    USBD_EP_PARAMS params = {0};
    uint8_t        phy_ep;
    int32_t        ret;
    uint32_t       cmd;

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
    ept    = &drv->eps[phy_ep];
    if (ept->ep_dir != USB_DIR_IN) {
#ifdef DEBUG
        printf("Direction is wrong returning\n");
#endif
        return USB_EP_DIRECTION_WRONG;
    }
    ept->bytes_txed         = 0U;
    ept->ep_requested_bytes = buf_len;

    trb_ptr                 = &ept->ep_trb[ept->trb_enqueue];
    ept->trb_enqueue++;
    if (ept->trb_enqueue == NO_OF_TRB_PER_EP) {
        ept->trb_enqueue = 0U;
    }
    RTSS_CleanDCache_by_Addr(bufferptr, buf_len);

    trb_ptr->buf_ptr_low  = LOWER_32_BITS(LocalToGlobal((uint32_t *) bufferptr));
    trb_ptr->buf_ptr_high = 0;
    trb_ptr->size         = USB_TRB_SIZE_LENGTH(buf_len);
    if (buf_len == 0) {
        /* Normal ZLP(BULK IN) - set to 9 for BULK IN TRB for zero
         * length packet termination
         */
        trb_ptr->ctrl = USB_TRBCTL_NORMAL_ZLP;
    } else {
        /* For Bulk TRB control(TRBCTL) as Normal  */
        trb_ptr->ctrl = USB_TRBCTL_NORMAL;
    }
    SET_BIT(trb_ptr->ctrl, USB_TRB_CTRL_HWO | USB_TRB_CTRL_IOC);

    RTSS_CleanDCache_by_Addr(trb_ptr, sizeof(*trb_ptr));
    params.param1 = (uint32_t) trb_ptr;
    if ((ept->ep_status & USB_EP_BUSY) != 0U) {
        cmd  = USB_DEPCMD_UPDATETRANSFER;
        cmd |= USB_DEPCMD_PARAM(ept->ep_resource_index);
    } else {
        cmd = USB_DEPCMD_STARTTRANSFER;
    }
    ret = usbd_send_ep_cmd(drv, phy_ep, cmd, params);
    if (ret < USB_SUCCESS) {
#ifdef DEBUG
        printf("failed to send the command\n");
#endif
        return ret;
    }
    if ((ept->ep_status & USB_EP_BUSY) == 0U) {
        ept->ep_resource_index =
            usbd_get_ep_transfer_resource_index(drv, ept->ep_index, ept->ep_dir);

        SET_BIT(ept->ep_status, USB_EP_BUSY);
    }

    return ret;
}
/**
 *\fn           usbd_bulk_recv
 *\brief        This function will prepare the TRB for receiving bulk data from host.
 *\param[in]    pointer to the controller context structure
 *\param[in]    endpoint number
 *\param[in]    endpoint direction
 *\param[in]    buffer pointer
 *\param[in]    buffer length
 *\return       On success 0, failure case returns error.
 */

int32_t usbd_bulk_recv(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t *bufferptr,
                       uint32_t buf_len)
{
    USBD_EP       *ept;
    USBD_TRB      *trb_ptr;
    USBD_EP_PARAMS params = {0};
    uint8_t        phy_ep;
    uint32_t       size;
    uint32_t       cmd;
    int32_t        ret;

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);

    ept    = &drv->eps[phy_ep];
    if (ept->ep_dir != dir) {
#ifdef DEBUG
        printf("Wrong BULK endpoint direction\n");
#endif
        return USB_EP_DIRECTION_WRONG;
    }
    if (buf_len < ept->ep_maxpacket) {
        size                    = ept->ep_maxpacket;
        ept->ep_requested_bytes = ept->ep_maxpacket;
    } else {
        size                    = buf_len;
        ept->ep_requested_bytes = buf_len;
    }
    ept->bytes_txed = 0U;
    /*
     * An OUT transfer size (Total TRB buffer allocation)
     * must be a multiple of MaxPacketSize even if software is expecting a
     * fixed non-multiple of MaxPacketSize transfer from the Host.
     */
    if (IS_ALIGNED(buf_len, ept->ep_maxpacket)) {
        size                = ROUND_UP(buf_len, ept->ep_maxpacket);
        ept->unaligned_txed = 1U;
    }
    trb_ptr = &ept->ep_trb[ept->trb_enqueue];

    ept->trb_enqueue++;
    if (ept->trb_enqueue == NO_OF_TRB_PER_EP) {
        ept->trb_enqueue = 0U;
    }

    trb_ptr->buf_ptr_low  = LOWER_32_BITS(LocalToGlobal((uint32_t *) bufferptr));
    trb_ptr->buf_ptr_high = 0;

    trb_ptr->size         = USB_TRB_SIZE_LENGTH(size);
    trb_ptr->ctrl         = USB_TRBCTL_NORMAL;
    SET_BIT(trb_ptr->ctrl,
            USB_TRB_CTRL_CSP | USB_TRB_CTRL_IOC | USB_TRB_CTRL_ISP_IMI | USB_TRB_CTRL_HWO);

    RTSS_CleanDCache_by_Addr(trb_ptr, sizeof(*trb_ptr));

    params.param1 = (uint32_t) trb_ptr;
    if ((ept->ep_status & USB_EP_BUSY) != 0U) {
        cmd  = USB_DEPCMD_UPDATETRANSFER;
        cmd |= USB_DEPCMD_PARAM(ept->ep_resource_index);
    } else {
        cmd = USB_DEPCMD_STARTTRANSFER;
    }
    ret = usbd_send_ep_cmd(drv, phy_ep, cmd, params);
    if (ret < USB_SUCCESS) {
#ifdef DEBUG
        printf("SendEpCmd failed\n");
#endif
        return ret;
    }

    if ((ept->ep_status & USB_EP_BUSY) == 0U) {
        ept->ep_resource_index =
            usbd_get_ep_transfer_resource_index(drv, ept->ep_index, ept->ep_dir);

        SET_BIT(ept->ep_status, USB_EP_BUSY);
    }

    return ret;
}
