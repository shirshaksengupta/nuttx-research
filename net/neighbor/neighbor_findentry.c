/****************************************************************************
 * net/neighbor/neighbor_findentry.c
 *
 *   Copyright (C) 2007-2009, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * A leverage of logic from uIP which also has a BSD style license
 *
 *   Copyright (c) 2006, Swedish Institute of Computer Science.  All rights
 *     reserved.
 *   Author: Adam Dunkels <adam@sics.se>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <debug.h>

#include "neighbor/neighbor.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: neighbor_findentry
 *
 * Description:
 *   Find an entry in the Neighbor Table.  This interface is internal to
 *   the neighbor implementation; Consider using neighbor_lookup() instead;
 *
 * Input Parameters:
 *   ipaddr - The IPv6 address to use in the lookup;
 *
 * Returned Value:
 *   The Neighbor Table entry corresponding to the IPv6 address;  NULL is
 *   returned if there is no matching entry in the Neighbor Table.
 *
 ****************************************************************************/

FAR struct neighbor_entry *neighbor_findentry(const net_ipv6addr_t ipaddr)
{
  int i;

  nllinfo("Find neighbor: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
          ntohs(ipaddr[0]), ntohs(ipaddr[1]), ntohs(ipaddr[2]),
          ntohs(ipaddr[3]), ntohs(ipaddr[4]), ntohs(ipaddr[5]),
          ntohs(ipaddr[6]), ntohs(ipaddr[7]));

  for (i = 0; i < CONFIG_NET_IPv6_NCONF_ENTRIES; ++i)
    {
      FAR struct neighbor_entry *neighbor = &g_neighbors[i];

      if (net_ipv6addr_cmp(neighbor->ne_ipaddr, ipaddr))
        {
          nllinfo("  at: %02x:%02x:%02x:%02x:%02x:%02x\n",
                  neighbor->ne_addr.na_addr.ether_addr_octet[0],
                  neighbor->ne_addr.na_addr.ether_addr_octet[1],
                  neighbor->ne_addr.na_addr.ether_addr_octet[2],
                  neighbor->ne_addr.na_addr.ether_addr_octet[3],
                  neighbor->ne_addr.na_addr.ether_addr_octet[4],
                  neighbor->ne_addr.na_addr.ether_addr_octet[5]);

          return &g_neighbors[i];
        }
    }

  nllinfo("  Not found\n");
  return NULL;
}
