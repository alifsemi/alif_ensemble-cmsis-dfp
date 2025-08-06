/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     usbd_initialize.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This file  initializes the USB controller in device/host mode.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* Include necessary system files.  */

#include "usbd.h"
#include "sys_ctrl_usb.h"
#include "sys_clocks.h"

static USBD_EVENT_BUFFER    evt_buf;
static uint8_t              event_buff[USB_EVENT_BUFFER_SIZE]       ATTR_SECTION("usb_dma_buf");
static uint8_t              scratch_pad[USB_SCRATCHPAD_BUF_SIZE]    ATTR_SECTION("usb_dma_buf");

/**
 *\fn         USB_IRQHandler
 *\brief      Process the USB device interrupts
 */

#ifndef USB_HOST
/* In case of USB host mode xhci driver has USB_IRQHandler  */
void USB_IRQHandler(void)
{
    usbd_interrupt_handler();
}
#endif

/**
 *\fn           usbd_initialize_eps
 *\brief        This function initialize the physical endpoints.
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/

static void usbd_initialize_eps(USB_DRIVER *drv)
{
    uint8_t i;
    uint8_t ep_num;

    for (i = 0U; i < drv->out_eps; i++) {
        ep_num                             = (i << 1U) | USB_DIR_OUT;
        drv->eps[ep_num].ep_status         = 0U;
        drv->eps[ep_num].phy_ep            = ep_num;
        drv->eps[ep_num].ep_dir            = USB_DIR_OUT;
        drv->eps[ep_num].ep_resource_index = 0U;
    }
    for (i = 0U; i < drv->in_eps; i++) {
        ep_num                             = (i << 1U) | USB_DIR_IN;
        drv->eps[ep_num].ep_status         = 0U;
        drv->eps[ep_num].phy_ep            = ep_num;
        drv->eps[ep_num].ep_dir            = USB_DIR_IN;
        drv->eps[ep_num].ep_resource_index = 0U;
    }
    for (i = 0; i < (drv->out_eps + drv->in_eps); i++) {
        memset(&drv->eps[i].ep_trb[0], 0x00, USB_TRBS_PER_EP * USB_TRB_STRUCTURE_SIZE);
    }
}

/**
 *\fn           usbd_set_speed
 *\brief        This function sets speed of USB controller.
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/

static void usbd_set_speed(USB_DRIVER *drv)
{
    uint32_t reg;

    reg              = drv->regs->DCFG;
    reg             &= ~(USB_DCFG_SPEED_MASK);
    reg             |= USB_DCFG_HIGHSPEED;
    drv->regs->DCFG  = reg;
}
/**
 *\fn           usbd_send_generic_cmd
 *\brief        This function send the cmd to the controller for scratchpad buffer.
 *\param[in]    pointer to the controller context structure
 *\param[in]    command to be send
 *\param[in]    params which contains the scratchpad buffer address
 *\return       On success 0, else error
 **/
static uint32_t usbd_send_generic_cmd(USB_DRIVER *drv, uint32_t cmd, uint32_t param)
{
    uint32_t reg, retry = USB_GENERIC_CMD_TIMEOUT;
    uint32_t status     = 0;
    uint32_t ret        = USB_SUCCESS;

    drv->regs->DGCMDPAR = param;
    drv->regs->DGCMD    = cmd | USB_DGCMD_CMDACT;
    do {
        reg = drv->regs->DGCMD;
        if (!(reg & USB_DGCMD_CMDACT)) {
            status = USB_DGCMD_STATUS(reg);
            if (status) {
                ret = USB_DGCMD_CMPLT_ERROR;
            }
            break;
        }
    } while (--retry);
    if (!retry) {
        ret = USB_DGCMD_TIMEOUT_ERROR;
    }

    return ret;
}

/**
 *\fn           usbd_setup_scratchpad
 *\brief        This function setup the scratchpad buffer.
 *\param[in]    pointer to the controller context structure
 *\return       On success 0, else error
 **/
static uint32_t usbd_setup_scratchpad(USB_DRIVER *drv)
{
    uint32_t  ret;
    uint32_t *scratchbuff_ptr;
    uint32_t  param;

    scratchbuff_ptr = (uint32_t *) scratch_pad;
    param           = LOWER_32_BITS(scratchbuff_ptr);
    ret             = usbd_send_generic_cmd(drv, USB_DGCMD_SET_SCRATCHPAD_ADDR_LO, param);
    if (ret != USB_SUCCESS) {
#ifdef DEBUG
        printf("Failed to set scratchpad low addr:%" PRIu32 "\n", ret);
#endif
        return ret;
    }
    param = UPPER_32_BITS(scratchbuff_ptr);
    ret   = usbd_send_generic_cmd(drv, USB_DGCMD_SET_SCRATCHPAD_ADDR_HI, param);
    if (ret != USB_SUCCESS) {
#ifdef DEBUG
        printf("Failed to set scratchpad high addr:%" PRIu32 "\n", ret);
#endif
        return ret;
    }

    return ret;
}

/**
 *\fn           usbd_core_is_valid
 *\brief        This function reads the controller core id and check if it's valid core or not.
 *\param[in]    pointer to the controller context structure
 *\return       On success returns true, else false
 **/

static bool usb_core_is_valid(USB_DRIVER *drv)
{
    uint32_t reg;

    reg = drv->regs->GSNPSID;
    /* Read the USB core ID and followed by revision number */
    if ((reg & USB_GSNPSID_MASK) == 0x55330000) {
        /* Detected DWC_usb3 IP */
        drv->revision = reg;
    } else {
#ifdef DEBUG
        printf("Detected wrong IP\n");
#endif
        return false;
    }

    return true;
}

/**
 *\fn           usbd_event_buf_allocation
 *\brief        This function allocates the events buffers with required size.
 *\param[in]    pointer to the controller context structure
 *\return       On success return event buf pointer
 **/

static USBD_EVENT_BUFFER *usbd_event_buf_allocation(USB_DRIVER *drv)
{
    USBD_EVENT_BUFFER *event_buf;

    event_buf         = &evt_buf;
    event_buf->length = USB_EVENT_BUFFER_SIZE;
    event_buf->buf    = event_buff;

    memset(event_buf->buf, 0, USB_EVENT_BUFFER_SIZE);
    return event_buf;
}

/**
 *\fn           usbd_setup_event_buffer
 *\brief        This function setup the allocated events buffers.
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/

static void usbd_setup_event_buffer(USB_DRIVER *drv)
{
    USBD_EVENT_BUFFER *event_buf;

    event_buf              = drv->event_buf;
    event_buf->lpos        = 0;
    drv->regs->GEVNTADRLO0 = LocalToGlobal((void *) LOWER_32_BITS(event_buf->buf));
    drv->regs->GEVNTADRHI0 = 0;
    drv->regs->GEVNTSIZ0   = event_buf->length;
    drv->regs->GEVNTCOUNT0 = 0;
}

/**
 *\fn           usbd_core_soft_reset
 *\brief        This function reset the controller core
 *\param[in]    pointer to the controller context structure
 *\return       On success 0, else error
 **/
static int32_t usbd_core_soft_reset(USB_DRIVER *drv)
{
    uint32_t reg;
    int32_t  timeout;

    /*
     * We're resetting only the device side because, if we're in host mode,
     * XHCI driver will reset the host block. If controller was configured for
     * host-only mode, then we can return early.
     */
    if (drv->current_dr_role == USB_GCTL_PRTCAP_HOST) {
        return USB_SUCCESS;
    }
    reg = drv->regs->DCTL;
    SET_BIT(reg, USB_DCTL_CSFTRST);
    drv->regs->DCTL = reg;
    timeout         = USB_DCTL_CSFTRST_TIMEOUT;
    /* Wait for Soft Reset to be completed.  */
    do {
        reg = drv->regs->DCTL;
        if (!(reg & USB_DCTL_CSFTRST)) {
            return USB_SUCCESS;
        }
    } while (--timeout);

    if (timeout == 0) {
        return USB_CORE_SFTRST_TIMEOUT_ERROR;
    }

    return USB_SUCCESS;
}

/**
 *\fn           usbd_get_properties
 *\brief        This function gets the USB properties
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/

static void usbd_get_properties(USB_DRIVER *drv)
{
#ifdef USB_HOST
    drv->dr_mode         = USB_DR_MODE_HOST;
    drv->current_dr_role = USB_GCTL_PRTCAP_HOST;
#else
    drv->dr_mode = USB_DR_MODE_PERIPHERAL;
#endif
    /* default setting the phy mode */
    drv->hsphy_mode = USB_PHY_INTERFACE_MODE_UTMIW;
    /* default value for frame length  */
    drv->fladj      = USB_GFLADJ_DEFAULT_VALUE;
}
/**
 *\fn           usbd_hwparams
 *\brief        Read the Global Hardware Parameters Registers
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/

static void usbd_hwparams(USB_DRIVER *drv)
{
    USB_HWPARAMS *parms = &drv->hwparams;

    parms->hwparams0    = drv->regs->GHWPARAMS0;
    parms->hwparams1    = drv->regs->GHWPARAMS1;
    parms->hwparams2    = drv->regs->GHWPARAMS2;
    parms->hwparams3    = drv->regs->GHWPARAMS3;
    parms->hwparams4    = drv->regs->GHWPARAMS4;
    parms->hwparams5    = drv->regs->GHWPARAMS5;
    parms->hwparams6    = drv->regs->GHWPARAMS6;
    parms->hwparams7    = drv->regs->GHWPARAMS7;
    parms->hwparams8    = drv->regs->GHWPARAMS8;
}

/**
 *\fn           usbd_get_mode
 *\brief        This function gets the controller mode .
 *\param[in]    pointer to the controller context structure
 *\return       On success 0, else error
 **/
static int32_t usbd_get_mode(USB_DRIVER *drv)
{
    USB_DR_MODE mode;
    uint32_t    hw_mode;

    mode    = drv->dr_mode;
    hw_mode = USB_GHWPARAMS0_MODE(drv->hwparams.hwparams0);
    switch (hw_mode) {

    case USB_GHWPARAMS0_MODE_DEVICE:
        mode = USB_DR_MODE_PERIPHERAL;
        break;
    case USB_GHWPARAMS0_MODE_HOST:
        mode = USB_DR_MODE_HOST;
        break;
    default:
#ifdef USB_HOST
        mode = USB_DR_MODE_HOST;
#else
        mode = USB_DR_MODE_PERIPHERAL;
#endif
    }
    if (mode != drv->dr_mode) {
#ifdef DEBUG
        printf("mode mismatched\n");
#endif
        drv->dr_mode = mode;
        return -1;
    }

    return 0;
}

/**
 *\fn           usbd_phy_setup
 *\brief        This function setup controller phy.
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/
static void usbd_phy_setup(USB_DRIVER *drv)
{
    uint32_t reg;

    /*Configure Global USB2 PHY Configuration Register */
    reg = drv->regs->GUSB2PHYCFG0;

    /* The PHY must not be enabled for auto-resume in device mode.
     * Therefore, the field GUSB2PHYCFG[15] (ULPIAutoRes) must be written with '0'
     *  during the power-on initialization in case the reset value is '1'.
     */
    if (reg & USB_GUSB2PHYCFG_ULPIAUTORES) {
        reg &= ~USB_GUSB2PHYCFG_ULPIAUTORES;
    }

    /* Select the HS PHY interface */
    switch (USB_GHWPARAMS3_HSPHY_IFC(drv->hwparams.hwparams3)) {
    case USB_GHWPARAMS3_HSPHY_IFC_UTMI_ULPI:
        reg &= ~USB_GUSB2PHYCFG_ULPI_UTMI;
        break;
    case USB_GHWPARAMS3_HSPHY_IFC_ULPI:
        /* FALLTHROUGH */
    default:
        break;
    }
    /* Enable PHYIF  */
    switch (drv->hsphy_mode) {
    case USB_PHY_INTERFACE_MODE_UTMI:
        reg &= ~(USB_GUSB2PHYCFG_PHYIF_MASK | USB_GUSB2PHYCFG_USBTRDTIM_MASK);
        reg |= USB_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_8_BIT) |
               USB_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_8_BIT);
        break;
    case USB_PHY_INTERFACE_MODE_UTMIW:
        reg &= ~(USB_GUSB2PHYCFG_PHYIF_MASK | USB_GUSB2PHYCFG_USBTRDTIM_MASK |
                 USB_GUSB2PHYCFG_ULPI_UTMI);
        reg |= USB_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_16_BIT) |
               USB_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_16_BIT) | USB_GUSB2PHYCFG_SUSPHY;
        break;
    default:
        break;
    }
    drv->regs->GUSB2PHYCFG0 = reg;
}
/**
 *\fn           usbd_setup_global_control
 *\brief        This function setup the global control register.
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/
static void usbd_setup_global_control(USB_DRIVER *drv)
{
    uint32_t reg;

    reg = drv->regs->GCTL;
    SET_BIT(reg, USB_GCTL_DSBLCLKGTNG);
    drv->regs->GCTL = reg;

    if (drv->dr_mode == USB_DR_MODE_HOST) {
        reg = drv->regs->GUCTL;
        SET_BIT(reg, USB_GUCTL_HSTINAUTORETRY);
        drv->regs->GUCTL = reg;
    }
}
/**
 *\fn           usbd_set_incr_burst_type
 *\brief        This function configure the controller global burst type.
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/
static void usbd_set_incr_burst_type(USB_DRIVER *drv)
{
    uint32_t reg;

    /* Define incr in global soc bus configuration.  */
    reg = drv->regs->GSBUSCFG0;
    SET_BIT(reg, USB_GSBUSCFG0_INCRBRSTENA | USB_GSBUSCFG0_INCR16BRSTENA);
    drv->regs->GSBUSCFG0 = reg;
}
/**
 *\fn           usbd_frame_length_adjustment
 *\brief        This function sets the controller frame length.
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/
static void usbd_frame_length_adjustment(USB_DRIVER *drv)
{
    uint32_t reg;
    uint32_t frame_legth;

    reg         = drv->regs->GFLADJ;
    frame_legth = reg & USB_GFLADJ_30MHZ_MASK;
    if (frame_legth != drv->fladj) {
        reg               &= ~USB_GFLADJ_30MHZ_MASK;
        reg               |= USB_GFLADJ_30MHZ_SDBND_SEL | drv->fladj;
        drv->regs->GFLADJ  = reg;
    }
}
/**
 *\fn           usbd_set_prtcap
 *\brief        This function sets the controller mode.
 *\param[in]    pointer to the controller context structure
 *\param[in]    mode(device/host)
 *\return       none
 **/
static void usbd_set_prtcap(USB_DRIVER *drv, uint32_t mode)
{
    uint32_t reg;

    reg                   = drv->regs->GCTL;
    reg                  &= ~(USB_GCTL_PRTCAPDIR(USB_GCTL_PRTCAP_OTG));
    /* Set the mode   */
    reg                  |= USB_GCTL_PRTCAPDIR(mode);
    drv->regs->GCTL       = reg;
    drv->current_dr_role  = mode;
}

/**
 *\fn           usbd_device_init
 *\brief        This function initialize the USB controller in device mode.
 *\param[in]    pointer to controller context structure
 *\return       On success returns 0, else  error
 **/

static uint32_t usbd_device_init(USB_DRIVER *drv)
{
    USBD_EVENT_BUFFER *event_buf;
    uint32_t           reg;
    uint32_t           ret = USB_SUCCESS;

    /* Allocate memory for event buffer.  */
    event_buf              = usbd_event_buf_allocation(drv);
    if (event_buf == NULL) {
#ifdef DEBUG
        printf("Event buff allocation failed\n");
#endif
        return USB_EVNT_BUFF_ALLOC_ERROR;
    }
    drv->event_buf = event_buf;

    /* Initialize and setup USB device event buffer.  */
    usbd_setup_event_buffer(drv);

    reg          = drv->regs->GHWPARAMS3;
    drv->in_eps  = USB_IN_EPS(reg);
    drv->out_eps = (USB_NUM_EPS(reg) - drv->in_eps);

    usbd_initialize_eps(drv);
    /* allocate and setup scratchpad buffers  */
    ret = usbd_setup_scratchpad(drv);
    if (ret != 0) {
        drv->event_buf = NULL;
        return ret;
    }
    /* Set speed on Controller */
    usbd_set_speed(drv);

    /* Check controller USB2 phy config before issuing DEPCMD always */
    usbd_usb2phy_config_check(drv);

    /* Issuing DEPSTARTCFG Command to ep_resource[0] (for EP 0/CONTROL/) */
    drv->regs->USB_ENDPNT_CMD[0].DEPCMD = USB_DEPCMD_DEPSTARTCFG | USB_DEPCMD_CMDACT;
    do {
        reg = drv->regs->USB_ENDPNT_CMD[0].DEPCMD;
    } while ((reg & USB_DEPCMD_CMDACT) != 0);
    usbd_usb2phy_config_reset(drv);

    /* Check usb2phy config before issuing DEPCMD for EP0 OUT */
    usbd_usb2phy_config_check(drv);

    /* Issuing DEPCFG Command to ep_resource[0] (for EP 0 OUT/CONTROL/)
     *bit[8] = XferComplete, bit[10] = XferNotReady *
     *bit[25]: Endpoint direction:
     *  0: OUT
     *  1: IN
     * bit[29:26] = Endpoint number
     * Physical endpoint 0 (EP0) must be allocated for control endpoint 0 OUT.
     * Physical endpoint 1 (EP1) must be allocated for control endpoint 0 IN.
     */
    drv->regs->USB_ENDPNT_CMD[0].DEPCMDPAR1 = USB_DEPCMD_EP0_OUT_PAR1;
    /* bit[25:22] = Burst size and Burst size =0 for Control endpoint
     * bit[21:17] = FIFONum, and FIFONum = 0;
     * bit[13:3] =  Maximum Packet Size (MPS)
     * bit[2:1] = Endpoint Type (EPType)
     *  00: Control
     *  01: Isochronous
     *  10: Bulk
     *  11: Interrupt
     */
    drv->regs->USB_ENDPNT_CMD[0].DEPCMDPAR0 = USB_DEPCMD_EP0_OUT_PAR0;
    drv->regs->USB_ENDPNT_CMD[0].DEPCMD     = USB_DEPCMD_SETEPCONFIG | USB_DEPCMD_CMDACT;
    do {
        reg = drv->regs->USB_ENDPNT_CMD[0].DEPCMD;
    } while ((reg & USB_DEPCMD_CMDACT) != 0);
    usbd_usb2phy_config_reset(drv);

    /* Check usb2phy config before issuing DEPCMD for EP1 IN */
    usbd_usb2phy_config_check(drv);
    /* Issuing DEPCFG Command to ep_resource[1] (for EP 1 IN/CONTROL/) */
    drv->regs->USB_ENDPNT_CMD[1].DEPCMDPAR1 = USB_DEPCMD_EP1_IN_PAR1;
    drv->regs->USB_ENDPNT_CMD[1].DEPCMDPAR0 = USB_DEPCMD_EP1_IN_PAR0;
    drv->regs->USB_ENDPNT_CMD[1].DEPCMD     = USB_DEPCMD_SETEPCONFIG | USB_DEPCMD_CMDACT;
    do {
        /* Read the device endpoint Command register.  */
        reg = drv->regs->USB_ENDPNT_CMD[1].DEPCMD;
    } while ((reg & USB_DEPCMD_CMDACT) != 0);
    /* Restore the USB2 phy state  */
    usbd_usb2phy_config_reset(drv);

    /* Issue a DEPXFERCFG(Endpoint Transfer Resource Configuration)
     *  command for physical endpoints 0 & 1 with DEPCMDPAR0_0/1 set to 1,
     *  and poll CmdAct for completions
     */

    /* Check usb2phy config before issuing DEPCMD for EP0 OUT */
    usbd_usb2phy_config_check(drv);
    /* Issuing Transfer Resource command for each initialized endpoint */
    drv->regs->USB_ENDPNT_CMD[0].DEPCMDPAR0 = USB_DEPCMD_EP_XFERCFG_PAR0;
    drv->regs->USB_ENDPNT_CMD[0].DEPCMD     = USB_DEPCMD_SETTRANSFRESOURCE | USB_DEPCMD_CMDACT;
    do {
        /* Read the device endpoint Command register.  */
        reg = drv->regs->USB_ENDPNT_CMD[0].DEPCMD;
    } while ((reg & USB_DEPCMD_CMDACT) != 0);
    /* Restore the USB2 phy state  */
    usbd_usb2phy_config_reset(drv);

    /* Check usb2phy config before issuing DEPCMD for EP1 IN */
    usbd_usb2phy_config_check(drv);
    drv->regs->USB_ENDPNT_CMD[1].DEPCMDPAR0 = USB_DEPCMD_EP_XFERCFG_PAR0;
    drv->regs->USB_ENDPNT_CMD[1].DEPCMD     = USB_DEPCMD_SETTRANSFRESOURCE | USB_DEPCMD_CMDACT;
    do {
        /* Read the device endpoint Command register.  */
        reg = drv->regs->USB_ENDPNT_CMD[1].DEPCMD;
    } while ((reg & USB_DEPCMD_CMDACT) != 0);
    /* Restore the USB2 phy state  */
    usbd_usb2phy_config_reset(drv);

    /* enable USB Reset, Connection Done, and USB/Link State Change events */
    reg = drv->regs->DEVTEN;
    SET_BIT(reg, USB_DEV_DISSCONNEVTEN);
    SET_BIT(reg, USB_DEV_USBRSTEVTEN);
    SET_BIT(reg, USB_DEV_CONNECTDONEEVTEN);
    drv->regs->DEVTEN = reg;

    return ret;
}
/**
 *\fn           usbd_set_mode
 *\brief        This function sets the controller mode(device/host).
 *\param[in]    pointer to the controller context structure
 *\return       On success returns 0, else  error
 */
static int32_t usbd_set_mode(USB_DRIVER *drv)
{
    int32_t ret = USB_INIT_ERROR;

    switch (drv->dr_mode) {
    case USB_DR_MODE_HOST:
        usbd_set_prtcap(drv, USB_GCTL_PRTCAP_HOST);
        break;
    case USB_DR_MODE_PERIPHERAL:
        usbd_set_prtcap(drv, USB_GCTL_PRTCAP_DEVICE);
        ret = usbd_device_init(drv);
        if (ret) {
#ifdef DEBUG
            printf("USB device init failed\n");
#endif
        }
        break;
    case USB_DR_MODE_OTG:
        break;
    default:
        return ret;
    }

    return ret;
}

/**
 *\fn           usbd_cleanup_event_buffer
 *\brief        This function clear the event buffer register.
 *\param[in]    pointer to the controller context structure
 *\return       On success returns 0, else  error
 */
static void usbd_cleanup_event_buffer(USB_DRIVER *drv)
{
    USBD_EVENT_BUFFER *event_buf;

    event_buf              = drv->event_buf;
    event_buf->lpos        = 0;
    drv->regs->GEVNTADRLO0 = 0;
    drv->regs->GEVNTADRHI0 = 0;
    drv->regs->GEVNTSIZ0   = USB_GEVNTSIZ_INTMASK | USB_GEVNTSIZ_SIZE(0);
    drv->regs->GEVNTCOUNT0 = 0;
}

/**
 *\fn           usbd_phy_reset
 *\brief        This function resets the phy.
 *\param[in]    pointer to the controller context structure
 *\return       On success returns 0, else  error
 */
void usbd_phy_reset(USB_DRIVER *drv)
{
    uint32_t reg;

    /* Before Resetting PHY, put Core in Reset */
    reg = drv->regs->GCTL;
    SET_BIT(reg, USB_GCTL_CORESOFTRESET);
    drv->regs->GCTL = reg;

    /* Assert USB2 PHY reset */
    reg             = drv->regs->GUSB2PHYCFG0;
    SET_BIT(reg, USB_GUSB2PHYCFG_PHYSOFTRST);
    drv->regs->GUSB2PHYCFG0 = reg;
    sys_busy_loop_us(50000);

    /* Clear USB2 PHY reset */
    reg = drv->regs->GUSB2PHYCFG0;
    CLEAR_BIT(reg, USB_GUSB2PHYCFG_PHYSOFTRST);
    drv->regs->GUSB2PHYCFG0 = reg;
    sys_busy_loop_us(50000);

    /* Take Core out of reset state after PHYS are stable*/
    reg = drv->regs->GCTL;
    CLEAR_BIT(reg, USB_GCTL_CORESOFTRESET);
    drv->regs->GCTL = reg;
}

/**
 *\fn           usbd_disconnect
 *\brief        This function stop the usb controller and disables the interrupts.
 *\param[in]    pointer to the controller context structure
 *\return       On success returns 0, else  error
 */
void usbd_disconnect(USB_DRIVER *drv)
{
    uint32_t reg;

    reg = drv->regs->DCTL;
    CLEAR_BIT(reg, USB_DCTL_START);
    drv->regs->DCTL = reg;
    NVIC_ClearPendingIRQ(USB_IRQ_IRQn);
    NVIC_DisableIRQ(USB_IRQ_IRQn);
}

/**
 *\fn           usbd_connect
 *\brief        This function start the usb controller and enables the interrupts.
 *\param[in]    pointer to the controller context structure
 *\return       On success returns 0, else  error
 */
int32_t usbd_connect(USB_DRIVER *drv)
{
    uint32_t reg;
    int32_t  retries;

    /* Starting controller . */
    reg = drv->regs->DCTL;
    SET_BIT(reg, USB_DCTL_START);
    drv->regs->DCTL = reg;
    retries         = USB_DCTL_START_TIMEOUT;
    do {
        reg = drv->regs->DSTS;
        if (!(reg & USB_DSTS_DEVCTRLHLT)) {
            goto next;
        }
    } while (--retries);
    if (retries == 0) {
        drv->event_buf = NULL;
        return USB_CONTROLLER_INIT_FAILED;
    }

next:
    NVIC_ClearPendingIRQ(USB_IRQ_IRQn);
    NVIC_EnableIRQ(USB_IRQ_IRQn);
#ifdef DEBUG
    printf("started drv controller\n");
#endif
    return 0;
}

/**
 *\fn           usbd_initialize
 *\brief        This function initialize the USB controller.
 *\param[in]    pointer to the controller context structure
 *\return       On success returns 0, else  error
 **/

int32_t usbd_initialize(USB_DRIVER *drv)
{
    int32_t ret = USB_SUCCESS;
    /* Enable peripheral clk for USB  */
    enable_usb_periph_clk();
    /* USB phy reset */
    usb_ctrl2_phy_power_on_reset_clear();

    drv->regs = (USB_Type *) USB_BASE;

    /* Validate the controller core */
    if (!usb_core_is_valid(drv)) {
#ifdef DEBUG
        printf("this is not controller Core\n");
#endif
        ret = USB_CORE_INVALID;
        return ret;
    }
    usbd_get_properties(drv);

    usbd_hwparams(drv);

    /* get the usb mode */
    ret = usbd_get_mode(drv);
    if (ret) {
        return ret;
    }

    /* Perform the core soft reset.  */
    ret = usbd_core_soft_reset(drv);
    if (ret != 0) {
#ifdef DEBUG
        printf("core soft reset was failed\n");
#endif
        return ret;
    }
    /* Spec says wait for few cycles.  */
    sys_busy_loop_us(5000);

    usbd_phy_reset(drv);

    usbd_phy_setup(drv);
    /* Setup Global control register  */
    usbd_setup_global_control(drv);

    /* Adjust Frame Length */
    usbd_frame_length_adjustment(drv);

    usbd_set_incr_burst_type(drv);

    /* Set the usb mode */
    ret = usbd_set_mode(drv);
    if (ret != USB_SUCCESS) {
        usbd_cleanup_event_buffer(drv);
#ifdef DEBUG
        printf("failed to set the usb mode\n");
#endif
        return ret;
    }
    /* Return successful completion.  */

    return ret;
}
