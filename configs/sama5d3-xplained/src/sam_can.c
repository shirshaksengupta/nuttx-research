/************************************************************************************
 * configs/sama5d3-xplained/src/sam_can.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/can.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "sam_can.h"
#include "sama5d3-xplained.h"

#if defined(CONFIG_CAN) && (defined(CONFIG_SAMA5_CAN0) || defined(CONFIG_SAMA5_CAN1))

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#if defined(CONFIG_SAMA5_CAN0) && defined(CONFIG_SAMA5_CAN1)
#  warning "Both CAN0 and CAN1 are enabled.  Assuming only CAN0."
#  undef CONFIG_SAMA5_CAN1
#endif

#ifdef CONFIG_SAMA5_CAN0
#  define CAN_PORT 0
#else
#  define CAN_PORT 1
#endif

/* Debug ***************************************************************************/
/* Non-standard debug that may be enabled just for testing CAN */

#ifdef CONFIG_DEBUG_CAN
#  define canerr    err
#  define caninfo   info
#  define canllerr  llerr
#  define canllinfo llinfo
#else
#  define canerr(x...)
#  define caninfo(x...)
#  define canllerr(x...)
#  define canllinfo(x...)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: board_can_initialize
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work with
 *   examples/can.
 *
 ************************************************************************************/

int board_can_initialize(void)
{
  static bool initialized = false;
  struct can_dev_s *can;
  int ret;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Call stm32_caninitialize() to get an instance of the CAN interface */

      can = sam_caninitialize(CAN_PORT);
      if (can == NULL)
        {
          canerr("ERROR:  Failed to get CAN interface\n");
          return -ENODEV;
        }

      /* Register the CAN driver at "/dev/can0" */

      ret = can_register("/dev/can0", can);
      if (ret < 0)
        {
          canerr("ERROR: can_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_CAN && (CONFIG_SAMA5_CAN0 || CONFIG_SAMA5_CAN1) */
