/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     usbd_endpoint_create.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function will create and configure the all the endpoints.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/



/* Include necessary system files.  */

#include "usbd.h"
#include "system_utils.h"

/**
  \fn           usbd_endpoint_create
  \brief        This function will creates the all types of endpoints.
  \param[in]    pointer to the controller context structure
  \param[in]    endpoint type
  \param[in]    endpoint num
  \param[in]    endpoint direction
  \param[in]    max endpoint pkt size
  \return       On success 0, failure case returns error.
 **/

int32_t  usbd_endpoint_create(USB_DRIVER *drv, uint8_t ep_type, uint8_t ep_num, uint8_t dir,
        uint16_t ep_max_packet_size, uint8_t ep_interval)
{
    int32_t status;

    switch (ep_type) {

    case USB_CONTROL_EP:
        /* Enable the control endpoint  */
        status = usbd_ep_enable(drv, ep_num, dir, ep_type,
                ep_max_packet_size, ep_interval);
        if (status) {
#ifdef DEBUG
            printf("Failed to enable control ep num : %d direction: %d\n", ep_num, dir);
#endif
            return status;
        }
        break;
    case USB_BULK_EP:
        status = usbd_ep_enable(drv, ep_num, dir, ep_type,
                ep_max_packet_size, ep_interval);
        if (status) {
#ifdef DEBUG
            printf("Failed to enable bulk ep num %d: direction: %d\n", ep_num, dir);
#endif
            return status;
        }
        break;
    case USB_ISOCRONOUS_EP:

        /* Not yet implemented   */
        break;
    case USB_INTERRUPT_EP:
        /* Enable the Interrupt endpoint  */
        status = usbd_ep_enable(drv, ep_num, dir, ep_type,
                ep_max_packet_size, ep_interval);
        if (status) {
#ifdef DEBUG
            printf("failed to enable interrupt ep num %d: direction: %d\n", ep_num, dir);
#endif
            return status;
        }
        break;
    default:
#ifdef DEBUG
        printf("Invalid Endpoint\n");
#endif
        return USB_EP_INVALID;
    }
    /* Return successful completion.  */
    return status;
}

uint32_t usbd_ep_enable(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t ep_type,
        uint16_t ep_max_packet_size, uint8_t ep_interval)
{
    USBD_EP *ept;
    uint32_t reg;
    uint8_t phy_ep;
    uint32_t ret;

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
    ept = &drv->eps[phy_ep];
    ept->ep_index = ep_num;
    ept->ep_dir = dir;
    ept->ep_maxpacket = ep_max_packet_size;
    ept->phy_ep   = phy_ep;
    if (!(ept->ep_status & USB_EP_ENABLED)) {
        ret =  usbd_start_ep_config(drv, ep_num, dir);
        if (ret) {
#ifdef DEBUG
            printf("Start EPConfig failed\n");
#endif
            return ret;
        }
    }
    ret = usbd_set_ep_config(drv, ep_num, dir, ep_type, ep_max_packet_size, ep_interval);
    if (ret) {
#ifdef DEBUG
        printf("SetEP failed\n");
#endif
        return ret;
    }

    if (!(ept->ep_status & USB_EP_ENABLED)) {
        ret = usbd_set_xferResource(drv, phy_ep);
        if (ret) {
#ifdef DEBUG
            printf("Set XferResource failed\n");
#endif
            return ret;
        }
    }
    if (!(ept->ep_status & USB_EP_ENABLED)) {
        SET_BIT(ept->ep_status, USB_EP_ENABLED);

        reg = drv->regs->DALEPENA;
        reg |=  USB_DALEPENA_EP(ept->phy_ep);
        drv->regs->DALEPENA = reg;
        if (phy_ep > 1) {
            USBD_TRB *trb_ptr, *trb_link;
            /* Initialize TRB ring   */
            ept->trb_enqueue = 0;
            ept->trb_dequeue = 0;
            trb_ptr = &ept->ep_trb[0U];
            /* Link TRB. The HWO bit is never reset */
            trb_link = &ept->ep_trb[NO_OF_TRB_PER_EP];
            memset(trb_link, 0x0, sizeof(USBD_TRB));

            trb_link->buf_ptr_low = LOWER_32_BITS(LocalToGlobal(trb_ptr));
            trb_link->buf_ptr_high = 0;
            trb_link->ctrl |= USB_TRBCTL_LINK_TRB;
            SET_BIT(trb_link->ctrl, USB_TRB_CTRL_HWO);

            return USB_SUCCESS;
        }

        return USB_SUCCESS;
    }

    return USB_EP_ENABLE_ERROR;

}
uint32_t usbd_set_ep_config(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t ep_type,
        uint16_t ep_max_packet_size, uint8_t ep_interval)
{
    USBD_EP_PARAMS params = {0};
    uint8_t phy_ep;

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);

    params.param0 = USB_DEPCFG_EP_TYPE(ep_type)
            | USB_DEPCFG_MAX_PACKET_SIZE(ep_max_packet_size);

    params.param0 |= USB_DEPCFG_ACTION_INIT;
    SET_BIT(params.param1, USB_DEPCFG_XFER_COMPLETE_EN | USB_DEPCFG_XFER_NOT_READY_EN);

    /*
     * We are doing 1:1 mapping for endpoints, meaning
     * Physical Endpoints 2 maps to Logical Endpoint 2 and
     * so on. We consider the direction bit as part of the physical
     * endpoint number. So USB endpoint 0x81 is 0x03.
     */
    params.param1 |= USB_DEPCFG_EP_NUMBER(phy_ep);

    if (dir != USB_DIR_OUT) {
        params.param0 |= USB_DEPCFG_FIFO_NUMBER(phy_ep >> 1U);
    }
    if (ep_type != 0)
        SET_BIT(params.param1, USB_DEPCFG_XFER_IN_PROGRESS_EN);

    if (ep_interval) {
        params.param1 |= USB_DEPCFG_BINTERVAL_M1(ep_interval - 1);
    }

    return usbd_SendEpCmd(drv, phy_ep, USB_DEPCMD_SETEPCONFIG, params);
}


uint32_t usbd_start_ep_config(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir)
{
    USBD_EP_PARAMS params = {0};
    uint32_t     cmd;
    uint8_t phy_ep;
    uint32_t ret;

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);

    if (phy_ep != 1U) {
        cmd = USB_DEPCMD_DEPSTARTCFG;
        /* XferRscIdx == 0 for EP0 and 2 for the remaining */
        if (phy_ep > 1U) {
            if (drv->IsConfigDone != 0U) {
                return USB_SUCCESS;
            }
            drv->IsConfigDone = 1U;
            cmd |= USB_DEPCMD_PARAM(2U);
        }
        ret  = usbd_SendEpCmd(drv, 0, cmd, params);
        if (ret) {
#ifdef DEBUG
            printf("Ep cmd failed\n");
#endif
            return ret;
        } else {
            return ret;
        }
    }

    return USB_SUCCESS;
}
uint32_t usbd_set_xferResource(USB_DRIVER *drv, uint8_t phy_ep)
{
    USBD_EP_PARAMS params = {0};
    /* Set Endpoint Transfer Resource configuration parameter
     * This field must be set to 1
     */
    params.param0 = USB_DEPXFERCFG_NUM_XFER_RES(1U);
    return usbd_SendEpCmd(drv, phy_ep, USB_DEPCMD_SETTRANSFRESOURCE, params);
}

int32_t usbd_ep_disable(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir)
{
    USBD_EP *ept;
    uint32_t reg;
    uint8_t phy_ep;

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
    ept = &drv->eps[phy_ep];
    reg = drv->regs->DALEPENA;
    reg &= ~USB_DALEPENA_EP(phy_ep);
    drv->regs->DALEPENA = reg;
    CLEAR_BIT(ept->ep_status, USB_EP_ENABLED);

    return USB_SUCCESS;
}


int32_t usbd_ep_stall(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, bool stall)
{
    uint8_t  phy_ep;
    USBD_EP *ept = NULL;
    USBD_EP_PARAMS params = {0};
    int32_t ret;

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
    /* if stall is true send the SETSTALL command else send the CLEARSTALL command  */
    if(stall)
    {
         ept = &drv->eps[0];
         ret = usbd_SendEpCmd(drv, 0, USB_DEPCMD_SETSTALL, params);
         if(ret < 0)
         {
#ifdef DEBUG
              printf("failed to send STALL command\n");
#endif
              return ret;
          }
          ept->ep_status |= USB_EP_STALL;
    }
    else
    {
        ept = &drv->eps[phy_ep];
        ret = usbd_SendEpCmd(drv, phy_ep, USB_DEPCMD_CLEARSTALL, params);
        if(ret < 0)
        {
#ifdef DEBUG
             printf("failed to send CLEARSTALL command\n");
#endif
             return ret;
        }
        ept->ep_status  &= ~(USB_EP_STALL | USB_EP_WEDGE);
    }
    return ret;
}

int32_t usbd_ep_transfer_abort(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir)
{
    uint8_t force_rm = 1;
    int32_t ret;
    /* As per the data sheet ENDTRANSFER COMMAND,
     * Software issues this command requesting DMA to stop for the endpoint/stream specifying
     * the transfer resource index of the TRB and the ForceRM parameter to be set to 1 in
     *  the DEPCMD register. */

    /* When issuing an End Transfer command, software must set the CmdIOC bit (field 8)
     * so that an Endpoint Command Complete event is generated after the transfer ends.
     */
    ret = usbd_StopTransfer(drv, ep_num, dir ,force_rm);
    return ret;
}
