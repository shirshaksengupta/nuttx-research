/****************************************************************************
 * drivers/net/vnet.c
 *
 *   Copyright (C) 2011 Yu Qiang. All rights reserved.
 *   Author: Yu Qiang <yuq825@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2011, 2014 Gregory Nutt. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_VNET)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include <rgmp/vnet.h>
#include <rgmp/stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_VNET_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_VNET_NINTERFACES
#  define CONFIG_VNET_NINTERFACES 1
#endif

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define VNET_WDDELAY   (1*CLK_TCK)

/* TX timeout = 1 minute */

#define VNET_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct eth_hdr_s *)vnet->sk_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The vnet_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct vnet_driver_s
{
  bool sk_bifup;               /* true:ifup false:ifdown */
  WDOG_ID sk_txpoll;           /* TX poll timer */
  struct rgmp_vnet *vnet;

  /* This holds the information visible to the NuttX */

  struct net_driver_s sk_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct vnet_driver_s g_vnet[CONFIG_VNET_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  vnet_transmit(FAR struct vnet_driver_s *vnet);
static int  vnet_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void vnet_txdone(FAR struct vnet_driver_s *vnet);

/* Watchdog timer expirations */

static void vnet_polltimer(int argc, uint32_t arg, ...);
static void vnet_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int vnet_ifup(struct net_driver_s *dev);
static int vnet_ifdown(struct net_driver_s *dev);
static int vnet_txavail(struct net_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int vnet_addmac(struct net_driver_s *dev, FAR const uint8_t *mac);
static int vnet_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: vnet_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   vnet  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int vnet_transmit(FAR struct vnet_driver_s *vnet)
{
  int errcode;

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is not transmission in progress.
   */

  /* Send the packet: address=vnet->sk_dev.d_buf, length=vnet->sk_dev.d_len */

  errcode = vnet_xmit(vnet->vnet, (char *)vnet->sk_dev.d_buf, vnet->sk_dev.d_len);
  if (errcode)
    {
      /* When vnet_xmit fail, it means TX buffer is full. Watchdog
       * is of no use here because no TX done INT will happen. So
       * we reset the TX buffer directly.
       */

#ifdef CONFIG_DEBUG_FEATURES
      cprintf("VNET: TX buffer is full\n");
#endif
      return ERROR;
    }
  else
    {
      /* This step may be unnecessary here */

      vnet_txdone(vnet);
    }

  return OK;
}

/****************************************************************************
 * Function: vnet_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll(). 
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int vnet_txpoll(struct net_driver_s *dev)
{
  FAR struct vnet_driver_s *vnet = (FAR struct vnet_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (vnet->sk_dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(vnet->sk_dev.d_flags))
#endif
        {
          arp_out(&vnet->sk_dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&vnet->sk_dev);
        }
#endif /* CONFIG_NET_IPv6 */

      /* Send the packet */

      vnet_transmit(vnet);

      /* Check if there is room in the device to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */

      if (vnet_is_txbuff_full(vnet->vnet))
        {
          return 1;
        }
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: rtos_vnet_recv
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   vnet  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

void rtos_vnet_recv(struct rgmp_vnet *rgmp_vnet, char *data, int len)
{
  struct vnet_driver_s *vnet = rgmp_vnet->priv;

  do
    {
      /* Check if the packet is a valid size for the network buffer
       * configuration.
       */

      if (len > CONFIG_NET_ETH_MTU || len < 14)
        {
#ifdef CONFIG_DEBUG_FEATURES
          cprintf("VNET: receive invalid packet of size %d\n", len);
#endif
          return;
        }

      /* Copy the data data from the hardware to vnet->sk_dev.d_buf.  Set
       * amount of data in vnet->sk_dev.d_len
       */

      memcpy(vnet->sk_dev.d_buf, data, len);
      vnet->sk_dev.d_len = len;

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the packet tap */

       pkt_input(&vnet->sk_dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          nllinfo("IPv4 frame\n");

          /* Handle ARP on input then give the IPv4 packet to the network
           * layer
           */

          arp_ipin(&vnet->sk_dev);
          ipv4_input(&vnet->sk_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (vnet->sk_dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
              if (IFF_IS_IPv4(vnet->sk_dev.d_flags))
#endif
                {
                  arp_out(&vnet->sk_dev);
                }
#ifdef CONFIG_NET_IPv6
              else
                {
                  neighbor_out(&vnet->sk_dev);
                }
#endif

              /* And send the packet */

              vnet_transmit(vnet);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          nllinfo("Iv6 frame\n");

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&vnet->sk_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (vnet->sk_dev.d_len > 0)
           {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
              if (IFF_IS_IPv4(vnet->sk_dev.d_flags))
                {
                  arp_out(&vnet->sk_dev);
                }
              else
#endif
#ifdef CONFIG_NET_IPv6
                {
                  neighbor_out(&vnet->sk_dev);
                }
#endif

              /* And send the packet */

              vnet_transmit(vnet);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == htons(ETHTYPE_ARP))
        {
          arp_arpin(&vnet->sk_dev);

          /* If the above function invocation resulted in data that should
           * be sent out on the network, the field  d_len will set to a
           * value > 0.
           */

          if (vnet->sk_dev.d_len > 0)
            {
              vnet_transmit(vnet);
            }
        }
#endif
    }
  while (0); /* While there are more packets to be processed */
}

/****************************************************************************
 * Function: vnet_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   vnet  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void vnet_txdone(FAR struct vnet_driver_s *vnet)
{
  /* Poll the network for new XMIT data */

  (void)devif_poll(&vnet->sk_dev, vnet_txpoll);
}

/****************************************************************************
 * Function: vnet_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void vnet_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct vnet_driver_s *vnet = (FAR struct vnet_driver_s *)arg;

  /* Poll the network for new XMIT data */

  (void)devif_poll(&vnet->sk_dev, vnet_txpoll);
}

/****************************************************************************
 * Function: vnet_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void vnet_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct vnet_driver_s *vnet = (FAR struct vnet_driver_s *)arg;

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  if (vnet_is_txbuff_full(vnet->vnet))
    {
#ifdef CONFIG_DEBUG_FEATURES
      cprintf("VNET: TX buffer is full\n");
#endif
      return;
    }

  /* If so, update TCP timing states and poll the network for new XMIT data.
   * Hmmm..  might be bug here.  Does this mean if there is a transmit in
   * progress, we will missing TCP time state updates?
   */

  (void)devif_timer(&vnet->sk_dev, vnet_txpoll);

  /* Setup the watchdog poll timer again */

  (void)wd_start(vnet->sk_txpoll, VNET_WDDELAY, vnet_polltimer, 1,
                 (wdparm_t)arg);
}

/****************************************************************************
 * Function: vnet_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int vnet_ifup(struct net_driver_s *dev)
{
  FAR struct vnet_driver_s *vnet = (FAR struct vnet_driver_s *)dev->d_private;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Initialize PHYs, the Ethernet interface, and setup up Ethernet interrupts */

  /* Set and activate a timer process */

  (void)wd_start(vnet->sk_txpoll, VNET_WDDELAY, vnet_polltimer, 1,
                 (wdparm_t)vnet);

  vnet->sk_bifup = true;
  return OK;
}

/****************************************************************************
 * Function: vnet_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int vnet_ifdown(struct net_driver_s *dev)
{
  FAR struct vnet_driver_s *vnet = (FAR struct vnet_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(vnet->sk_txpoll);

  /* Put the EMAC is its reset, non-operational state.  This should be
   * a known configuration that will guarantee the vnet_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  vnet->sk_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: vnet_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int vnet_txavail(struct net_driver_s *dev)
{
  FAR struct vnet_driver_s *vnet = (FAR struct vnet_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable interrupts because this function may be called from interrupt
   * level processing.
   */

  flags = enter_critical_section();

  /* Ignore the notification if the interface is not yet up */

  if (vnet->sk_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      if (vnet_is_txbuff_full(vnet->vnet))
        {
#ifdef CONFIG_DEBUG_FEATURES
          cprintf("VNET: TX buffer is full\n");
#endif
          goto out;
        }

      /* If so, then poll the network for new XMIT data */

      (void)devif_poll(&vnet->sk_dev, vnet_txpoll);
    }

out:
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: vnet_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int vnet_addmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct vnet_driver_s *vnet = (FAR struct vnet_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: vnet_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int vnet_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct vnet_driver_s *vnet = (FAR struct vnet_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: vnet_initialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int vnet_init(struct rgmp_vnet *vnet)
{
  struct vnet_driver_s *priv;
  static int i = 0;

  if (i >= CONFIG_VNET_NINTERFACES)
    {
      return -1;
    }

  priv = &g_vnet[i++];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct vnet_driver_s));
  priv->sk_dev.d_ifup    = vnet_ifup;     /* I/F down callback */
  priv->sk_dev.d_ifdown  = vnet_ifdown;   /* I/F up (new IP address) callback */
  priv->sk_dev.d_txavail = vnet_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->sk_dev.d_addmac  = vnet_addmac;   /* Add multicast MAC address */
  priv->sk_dev.d_rmmac   = vnet_rmmac;    /* Remove multicast MAC address */
#endif
  priv->sk_dev.d_private = (FAR void *)priv; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->sk_txpoll        = wd_create();   /* Create periodic poll timer */

  priv->vnet             = vnet;
  vnet->priv             = priv;

  /* Register the device with the OS */

  (void)netdev_register(&priv->sk_dev), NET_LL_ETHERNET;

  return 0;
}

#endif /* CONFIG_NET && CONFIG_NET_VNET */
