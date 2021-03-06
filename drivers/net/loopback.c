/****************************************************************************
 * drivers/net/loopback.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#if defined(CONFIG_NET) && defined(CONFIG_NETDEV_LOOPBACK)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>
#include <net/if.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/loopback.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_NOINTS
#  error CONFIG_NET_NOINTS must be selected
#endif

#ifndef CONFIG_SCHED_HPWORK
#  error High priority work queue support is required (CONFIG_SCHED_HPWORK)
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define LO_WDDELAY   (1*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define IPv4BUF ((FAR struct ipv4_hdr_s *)priv->lo_dev.d_buf)
#define IPv6BUF ((FAR struct ipv6_hdr_s *)priv->lo_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lo_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct lo_driver_s
{
  bool lo_bifup;               /* true:ifup false:ifdown */
  bool lo_txdone;              /* One RX packet was looped back */
  WDOG_ID lo_polldog;          /* TX poll timer */
  struct work_s lo_work;       /* For deferring work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s lo_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lo_driver_s g_loopback;

#ifdef CONFIG_NET_MULTIBUFFER
static uint8_t g_iobuffer[MAX_NET_DEV_MTU + CONFIG_NET_GUARDSIZE];
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Polling logic */

static int  lo_txpoll(FAR struct net_driver_s *dev);
static void lo_poll_work(FAR void *arg);
static void lo_poll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int lo_ifup(FAR struct net_driver_s *dev);
static int lo_ifdown(FAR struct net_driver_s *dev);
static void lo_txavail_work(FAR void *arg);
static int lo_txavail(FAR struct net_driver_s *dev);
#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int lo_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#ifdef CONFIG_NET_IGMP
static int lo_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: lo_txpoll
 *
 * Description:
 *   Check if the network has any outgoing packets ready to send.  This is
 *   a callback from devif_poll() or devif_timer().  devif_poll() will be
 *   called only during normal TX polling.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   the network is locked.
 *
 ****************************************************************************/

static int lo_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  /* Loop while there is data "sent", i.e., while d_len > 0.  That should be
   * the case upon entry here and while the processing of the IPv4/6 packet
   * generates a new packet to be sent.  Sending, of course, just means
   * relaying back through the network for this driver.
   */

  while (priv->lo_dev.d_len > 0)
    {
       NETDEV_TXPACKETS(&priv->lo_dev);
       NETDEV_RXPACKETS(&priv->lo_dev);

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the packet tap */

       pkt_input(&priv->lo_dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if ((IPv4BUF->vhl & IP_VERSION_MASK) == IPv4_VERSION)
        {
          nllinfo("IPv4 frame\n");
          NETDEV_RXIPV4(&priv->lo_dev);
          ipv4_input(&priv->lo_dev);
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if ((IPv6BUF->vtc & IP_VERSION_MASK) == IPv6_VERSION)
        {
          nllinfo("Iv6 frame\n");
          NETDEV_RXIPV6(&priv->lo_dev);
          ipv6_input(&priv->lo_dev);
        }
      else
#endif
        {
          nwarn("WARNING: Unrecognized packet type dropped: %02x\n", IPv4BUF->vhl);
          NETDEV_RXDROPPED(&priv->lo_dev);
          priv->lo_dev.d_len = 0;
        }

      priv->lo_txdone = true;
      NETDEV_TXDONE(&priv->lo_dev);
    }

  return 0;
}

/****************************************************************************
 * Function: lo_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static void lo_poll_work(FAR void *arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;
  net_lock_t state;

  /* Perform the poll */

  state = net_lock();
  priv->lo_txdone = false;
  (void)devif_timer(&priv->lo_dev, lo_txpoll);

  /* Was something received and looped back? */

  while (priv->lo_txdone)
    {
      /* Yes, poll again for more TX data */

      priv->lo_txdone = false;
      (void)devif_poll(&priv->lo_dev, lo_txpoll);
    }

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->lo_polldog, LO_WDDELAY, lo_poll_expiry, 1, priv);
  net_unlock(state);
}

/****************************************************************************
 * Function: lo_poll_expiry
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
 *   The network is locked.
 *
 ****************************************************************************/

static void lo_poll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions.
   */

  if (work_available(&priv->lo_work))
    {
      /* Schedule to perform the interrupt processing on the worker thread. */

      work_queue(HPWORK, &priv->lo_work, lo_poll_work, priv, 0);
    }
  else
    {
      /* No.. Just re-start the watchdog poll timer, missing one polling
       * cycle.
       */

      (void)wd_start(priv->lo_polldog, LO_WDDELAY, lo_poll_expiry, 1, arg);
    }
}

/****************************************************************************
 * Function: lo_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lo_ifup(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Set and activate a timer process */

  (void)wd_start(priv->lo_polldog, LO_WDDELAY, lo_poll_expiry, 1, (wdparm_t)priv);

  priv->lo_bifup = true;
  return OK;
}

/****************************************************************************
 * Function: lo_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lo_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->lo_polldog);

  /* Mark the device "down" */

  priv->lo_bifup = false;
  return OK;
}

/****************************************************************************
 * Function: lo_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void lo_txavail_work(FAR void *arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;
  net_lock_t state;

  /* Ignore the notification if the interface is not yet up */

  state = net_lock();
  if (priv->lo_bifup)
    {
      do
        {
          /* If so, then poll the network for new XMIT data */

          priv->lo_txdone = false;
          (void)devif_poll(&priv->lo_dev, lo_txpoll);
        }
      while (priv->lo_txdone);
    }

  net_unlock(state);
}

/****************************************************************************
 * Function: lo_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int lo_txavail(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->lo_work))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(HPWORK, &priv->lo_work, lo_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: lo_addmac
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

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int lo_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  /* There is no multicast support in the loopback driver */

  return OK;
}
#endif

/****************************************************************************
 * Function: lo_rmmac
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
static int lo_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  /* There is no multicast support in the loopback driver */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: localhost_initialize
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

int localhost_initialize(void)
{
  FAR struct lo_driver_s *priv;

  /* Get the interface structure associated with this interface number. */

  priv = &g_loopback;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct lo_driver_s));
  priv->lo_dev.d_ifup    = lo_ifup;      /* I/F up (new IP address) callback */
  priv->lo_dev.d_ifdown  = lo_ifdown;    /* I/F down callback */
  priv->lo_dev.d_txavail = lo_txavail;   /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->lo_dev.d_addmac  = lo_addmac;    /* Add multicast MAC address */
  priv->lo_dev.d_rmmac   = lo_rmmac;     /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NET_MULTIBUFFER
  priv->lo_dev.d_buf     = g_iobuffer;   /* Attach the IO buffer */
#endif
  priv->lo_dev.d_private = (FAR void *)priv; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmissions */

  priv->lo_polldog       = wd_create();  /* Create periodic poll timer */

  /* Register the loopabck device with the OS so that socket IOCTLs can b
   * performed.
   */

  (void)netdev_register(&priv->lo_dev, NET_LL_LOOPBACK);

  /* Set the local loopback IP address */

#ifdef CONFIG_NET_IPv4
  net_ipv4addr_copy(priv->lo_dev.d_ipaddr, g_lo_ipv4addr);
  net_ipv4addr_copy(priv->lo_dev.d_draddr, g_lo_ipv4addr);
  net_ipv4addr_copy(priv->lo_dev.d_netmask, g_lo_ipv4mask);
#endif

#ifdef CONFIG_NET_IPv6
  net_ipv6addr_copy(priv->lo_dev.d_ipv6addr, g_lo_ipv6addr);
  net_ipv6addr_copy(priv->lo_dev.d_ipv6draddr, g_lo_ipv6addr);
  net_ipv6addr_copy(priv->lo_dev.d_ipv6netmask, g_ipv6_alloneaddr);
#endif

  /* Put the network in the UP state */

  priv->lo_dev.d_flags = IFF_UP;
  return lo_ifup(&priv->lo_dev);
}

#endif /* CONFIG_NET && CONFIG_NETDEV_LOOPBACK */
