/****************************************************************************
 * arch/arm/src/efm32/efm32_timer.c
 *
 *   Copyright (C) 2014 Pierre-Noel Bouteville. All rights reserved.
 *   Author: Pierre-Noel Bouteville <pnb990@gmail.com>
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
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip/efm32_timer.h"
#include "efm32_config.h"
#include "efm32_gpio.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing TIMER */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_TIMER
#endif

#ifdef CONFIG_DEBUG_TIMER
#  define efm32_timererr              err
#  define efm32_timerllerr            llerr
#  ifdef CONFIG_DEBUG_INFO
#    define efm32_timerinfo           info
#    define efm32_timerllinfo         llinfo
#    define efm32_timer_dumpgpio(p,m) efm32_dumpgpio(p,m)
#  else
#    define efm32_timerllerr(x...)
#    define efm32_timerllinfo(x...)
#    define efm32_timer_dumpgpio(p,m)
#  endif
#else
#  define efm32_timererr(x...)
#  define efm32_timerllerr(x...)
#  define efm32_timerinfo(x...)
#  define efm32_timerllinfo(x...)
#  define efm32_timer_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_timer_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input parameters:
 *   base - A base address of timer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void efm32_timer_dumpregs(uintptr_t base, FAR const char *msg)
{
  int i;

  efm32_timerinfo("%s:\n", msg);
  efm32_timerinfo("  CTRL: %04x STATUS: %04x   IEN: %04x     IF: %04x\n",
                  getreg32(base + EFM32_TIMER_CTRL_OFFSET   ),
                  getreg32(base + EFM32_TIMER_STATUS_OFFSET ),
                  getreg32(base + EFM32_TIMER_IEN_OFFSET    ),
                  getreg32(base + EFM32_TIMER_IF_OFFSET     )
                 );
  efm32_timerinfo("   TOP: %04x   TOPB: %04x   CNT: %04x  ROUTE: %04x\n",
                  getreg32(base + EFM32_TIMER_TOP_OFFSET    ),
                  getreg32(base + EFM32_TIMER_TOPB_OFFSET   ),
                  getreg32(base + EFM32_TIMER_CNT_OFFSET    ),
                  getreg32(base + EFM32_TIMER_ROUTE_OFFSET  )
                 );

  for (i = 0; i < EFM32_TIMER_NCC; i++)
    {
#if defined(CONFIG_DEBUG_TIMER) && defined(CONFIG_DEBUG_INFO)
      uintptr_t base_cc = base + EFM32_TIMER_CC_OFFSET(i);
#endif
      efm32_timerinfo("CC%d => CTRL: %04x    CCV:  %04x  CCVP: %04x CCVB: %04x\n",
                      i
                      getreg32(base_cc + EFM32_TIMER_CC_CTRL_OFFSET ),
                      getreg32(base_cc + EFM32_TIMER_CC_CCV_OFFSET  ),
                      getreg32(base_cc + EFM32_TIMER_CC_CCVP_OFFSET ),
                      getreg32(base_cc + EFM32_TIMER_CC_CCVB_OFFSET )
                     );
    }

  efm32_timerinfo("DTCTRL: %04x DTTIME: %04x  DTFC: %04x DTOGEN:  %04x\n",
                  getreg32(base + EFM32_TIMER_CTRL_OFFSET   ),
                  getreg32(base + EFM32_TIMER_STATUS_OFFSET ),
                  getreg32(base + EFM32_TIMER_IEN_OFFSET    ),
                  getreg32(base + EFM32_TIMER_IF_OFFSET     )
                 );
  efm32_timerinfo("DTFAULT: %04x DTFAULTC: %04x  DTLOCK: %04x \n",
                  getreg32(base + EFM32_TIMER_CTRL_OFFSET   ),
                  getreg32(base + EFM32_TIMER_STATUS_OFFSET ),
                  getreg32(base + EFM32_TIMER_IEN_OFFSET    ),
                  getreg32(base + EFM32_TIMER_IF_OFFSET     )
                 );
}

/****************************************************************************
 * Name: efm32_timer_reset
 *
 * Description:
 *   reset timer into reset state
 *
 * Input parameters:
 *   base - A base address of timer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void efm32_timer_reset(uintptr_t base)
{
  int i;

  /* Make sure disabled first, before resetting other registers */

  putreg32(TIMER_CMD_STOP, base + EFM32_TIMER_CMD_OFFSET);

  /* Reset timer register */

  putreg32(_TIMER_CTRL_RESETVALUE,  base + EFM32_TIMER_CTRL_OFFSET     );
  putreg32(_TIMER_IEN_RESETVALUE,   base + EFM32_TIMER_STATUS_OFFSET   );
  putreg32(_TIMER_IFC_MASK,         base + EFM32_TIMER_IEN_OFFSET      );
  putreg32(_TIMER_TOP_RESETVALUE,   base + EFM32_TIMER_IF_OFFSET       );
  putreg32(_TIMER_TOPB_RESETVALUE,  base + EFM32_TIMER_CTRL_OFFSET     );
  putreg32(_TIMER_CNT_RESETVALUE,   base + EFM32_TIMER_CMD_OFFSET      );

  /* Do not reset route register, setting should be done independently
   * (Note: ROUTE register may be locked by DTLOCK register.)
   */

  //putreg32(_TIMER_ROUTE_RESETVALUE, base + EFM32_TIMER_ROUTE_OFFSET    );

  for (i = 0; i < EFM32_TIMER_NCC; i++)
    {
      uintptr_t base_cc = base + EFM32_TIMER_CC_OFFSET(i);
      putreg32(_TIMER_CC_CTRL_RESETVALUE, base_cc+EFM32_TIMER_CC_CTRL_OFFSET);
      putreg32(_TIMER_CC_CCV_RESETVALUE,  base_cc+EFM32_TIMER_CC_CCV_OFFSET );
      putreg32(_TIMER_CC_CCVB_RESETVALUE, base_cc+EFM32_TIMER_CC_CCVB_OFFSET);
    }

  /* Reset dead time insertion module, no effect on timers without DTI */

#ifdef TIMER_DTLOCK_LOCKKEY_UNLOCK
  /* Unlock DTI registers first in case locked */

  putreg32(TIMER_DTLOCK_LOCKKEY_UNLOCK, base + EFM32_TIMER_DTLOCK_OFFSET);

  putreg32(_TIMER_DTCTRL_RESETVALUE, base + EFM32_TIMER_DTCTRL_OFFSET);
  putreg32(_TIMER_DTTIME_RESETVALUE, base + EFM32_TIMER_DTTIME_OFFSET);
  putreg32(_TIMER_DTFC_RESETVALUE, base + EFM32_TIMER_DTFC_OFFSET);
  putreg32(_TIMER_DTOGEN_RESETVALUE,base + EFM32_TIMER_DTOGEN_OFFSET);
  putreg32(_TIMER_DTFAULTC_MASK, base + EFM32_TIMER_DTFAULTC_OFFSET);
#endif
}

/****************************************************************************
 * Name: efm32_timer_set_freq
 *
 * Description:
 *   set prescaler and top timer with best value to have "freq"
 *
 * Input parameters:
 *   base       - A base address of timer
 *   clk_freq   - Clock soure of timer.
 *   freq       - Wanted freqency.
 *
 * Returned Value:
 *   prescaler setted, -1 in case of error.
 *
 ****************************************************************************/
int efm32_timer_set_freq(uintptr_t base, uint32_t clk_freq, uint32_t freq)
{
  int prescaler = 0;
  int cnt_freq = clk_freq >> 16;
  int reload;

  while (cnt_freq > freq)
    {
      prescaler++;
      cnt_freq >>= 1;
      if (prescaler > (_TIMER_CTRL_PRESC_MASK >> _TIMER_CTRL_PRESC_SHIFT))
        {
          return -1;
        }
    }

  modifyreg32(base + EFM32_TIMER_CTRL_OFFSET,
              _TIMER_CTRL_PRESC_MASK,
              prescaler << _TIMER_CTRL_PRESC_SHIFT);

  prescaler = 1 << prescaler;

  reload = (clk_freq / prescaler / freq);

  efm32_timererr("Source: %4xHz Div: %4x Reload: %4x \n",
                 clk_freq, prescaler, reload);

  putreg32(reload, base + EFM32_TIMER_TOP_OFFSET);

  return prescaler;
}
