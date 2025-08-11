/*
 * LWIP http webserver demo application
 *
 * Derived from CMSIS LWIP example projects.
 *
 * Author   : Silesh C V <silesh@alifsemi.com>
 *
 * Copyright (C) 2022 ALIF SEMICONDUCTOR
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ALIF SEMICONDUCTOR nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>

#include "ethernetif.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/apps/httpd.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"

#include "board_config.h"

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

static void net_init(void);
static void net_periodic(uint32_t tick);
static void net_timer(uint32_t *tick);

static struct netif netif;

#define HTTP_DEMO_DEBUG

/* Initialize lwIP */
static void net_init(void)
{
    ip4_addr_t ipaddr;
    ip4_addr_t netmask;
    ip4_addr_t gw;

    lwip_init();

#if LWIP_DHCP
    ipaddr.addr  = IPADDR_ANY;
    netmask.addr = IPADDR_ANY;
    gw.addr      = IPADDR_ANY;
#else
    IP4_ADDR(&ipaddr, 192, 168, 1, 11);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 192, 168, 1, 1);
#endif

    /* Add the network interface to the netif_list. */
    netif_add(&netif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

    /* Register the default network interface. */
    netif_set_default(&netif);
    netif_set_up(&netif);

#if LWIP_DHCP
    dhcp_start(&netif);
#endif
}

/* Link check */
static void net_periodic(uint32_t tick)
{
    static uint32_t   old_tick;
    static ip4_addr_t ip = {0};

    if (tick == old_tick) {
        return;
    }
    old_tick = tick;

    ethernetif_check_link(&netif);

#ifdef HTTP_DEMO_DEBUG
    if (netif_is_link_up(&netif)) {
        /* Print IP address if debug is enabled */
        if (ip.addr != netif.ip_addr.addr) {
            ip.addr = netif.ip_addr.addr;
            printf("Link up. IP: %s.\n", ipaddr_ntoa(&ip));
        }
    }
#endif
}

/* Tick timer callback */
static void net_timer(uint32_t *tick)
{
    ++*tick;
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main(void *argument)
{
    static uint32_t tick;
    int32_t         ret;
    osTimerId_t     id;

#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    ret = stdout_init();
    if (ret != 0) {
        while (1) {
        }
    }
#endif

    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
#ifdef HTTP_DEMO_DEBUG
        printf("ERROR: Pin configuration failed: %" PRId32 "\n", ret);
#endif
        goto error;
    }

    /* Create tick timer, tick interval = 500ms */
    id = osTimerNew((osTimerFunc_t) &net_timer, osTimerPeriodic, &tick, NULL);
    osTimerStart(id, 500);

    net_init();

    httpd_init();

    while (1) {
        /* check for packet reception */
        ethernetif_poll(&netif);
        /* handle system timers for lwIP */
        sys_check_timeouts();
        net_periodic(tick);
    }
error:
    WAIT_FOREVER_LOOP
}

int main(void)
{
    // System Initialization
    SystemCoreClockUpdate();

    osKernelInitialize();               // Initialize CMSIS-RTOS
    osThreadNew(app_main, NULL, NULL);  // Create application main thread
    osKernelStart();                    // Start thread execution
    for (;;) {
    }
}
