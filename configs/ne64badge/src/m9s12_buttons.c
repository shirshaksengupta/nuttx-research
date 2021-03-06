/****************************************************************************
 * configs/ne64badge/src/m9s12_buttons.c
 *
 *   Copyright (C) 2011, 2014-2015 Gregory Nutt. All rights reserved.
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

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "ne64badge.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG_FEATURES with
 * CONFIG_DEBUG_INFO too)
 */

#undef BUTTON_DEBUG   /* Define to enable debug */
#undef BUTTON_VERBOSE /* Define to enable verbose debug */

#ifdef BUTTON_DEBUG
#  define btnerr  llerr
#  ifdef BUTTON_VERBOSE
#    define btninfo llerr
#  else
#    define btninfo(x...)
#  endif
#else
#  undef BUTTON_VERBOSE
#  define btnerr(x...)
#  define btninfo(x...)
#endif

/* Dump GPIO registers */

#ifdef BUTTON_VERBOSE
#  define btn_dumpgpio(m) m9s12_dumpgpio(m)
#else
#  define btn_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 ****************************************************************************/

void board_button_initialize(void)
{
  /* Configure all button GPIO lines */

  btn_dumpgpio("board_button_initialize() Entry)");

  hcs12_configgpio(NE64BADGE_BUTTON1);
  hcs12_configgpio(NE64BADGE_BUTTON2);

  btn_dumpgpio("board_button_initialize() Exit");
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint8_t board_buttons(void)
{
  uint8_t ret    = 0;

  if (hcs12_gpioread(NE64BADGE_BUTTON1))
    {
      ret |= BUTTON1;
    }

  if (hcs12_gpioread(NE64BADGE_BUTTON2))
    {
      ret |= BUTTON2;
    }

  return ret;
}

#endif /* CONFIG_ARCH_BUTTONS */
