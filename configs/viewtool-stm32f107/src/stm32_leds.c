/****************************************************************************
 * configs/viewtool-stm32f107/src/stm32_leds.c
 *
 *   Copyright (C) 2013, 2015 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "viewtool_stm32f107.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_LEDS enables debug output from this file (needs CONFIG_DEBUG_FEATURES
 * with CONFIG_DEBUG_INFO too)
 */

#ifdef CONFIG_DEBUG_LEDS
#  define lederr  llerr
#  define ledinfo llinfo
#else
#  define lederr(x...)
#  define ledinfo(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_onbits
 *
 * Description:
 *   Clear all LEDs to the bit encoded state
 *
 ****************************************************************************/

static void led_onbits(unsigned int clrbits)
{
  if ((clrbits & BOARD_LED1_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED1, false);
    }

  if ((clrbits & BOARD_LED2_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED2, false);
    }

  if ((clrbits & BOARD_LED3_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED3, false);
    }

  if ((clrbits & BOARD_LED4_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED4, false);
    }
}

/****************************************************************************
 * Name: led_offbits
 *
 * Description:
 *   Clear all LEDs to the bit encoded state
 *
 ****************************************************************************/

static void led_offbits(unsigned int clrbits)
{
  if ((clrbits & BOARD_LED1_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED1, true);
    }

  if ((clrbits & BOARD_LED2_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED2, true);
    }

  if ((clrbits & BOARD_LED3_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED3, true);
    }

  if ((clrbits & BOARD_LED4_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED4, true);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_led_initialize
 *
 * Description:
 *   Configure LEDs.  LEDs are left in the OFF state.
 *
 ****************************************************************************/

void stm32_led_initialize(void)
{
  /* Configure LED1-4 GPIOs for output.  Initial state is OFF */

  stm32_configgpio(GPIO_LED1);
  stm32_configgpio(GPIO_LED2);
  stm32_configgpio(GPIO_LED3);
  stm32_configgpio(GPIO_LED4);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Select the "logical" ON state:
 *
 *   SYMBOL            Val    Meaning                     LED state
 *                                                       LED1 LED2 LED3 LED4
 *   ----------------- ---   -----------------------  ---- ---- ---- ----
 *   LED_STARTED        0   NuttX has been started     ON  OFF  OFF  OFF
 *   LED_HEAPALLOCATE   1   Heap has been allocated   OFF   ON  OFF  OFF
 *   LED_IRQSENABLED    2   Interrupts enabled         ON   ON  OFF  OFF
 *   LED_STACKCREATED   3   Idle stack created        OFF  OFF  ON   OFF
 *   LED_INIRQ          4   In an interrupt           N/C  N/C  N/C  GLOW
 *   LED_SIGNAL         4   In a signal handler       N/C  N/C  N/C  GLOW
 *   LED_ASSERTION      4   An assertion failed       N/C  N/C  N/C  GLOW
 *   LED_PANIC          4   The system has crashed    N/C  N/C  N/C  FLASH
 *   ED_IDLE                MCU is is sleep mode         Not used
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_on(int led)
{
  switch (led)
    {
    case 0:
      led_offbits(BOARD_LED2_BIT | BOARD_LED3_BIT |BOARD_LED4_BIT);
      led_onbits(BOARD_LED1_BIT);
      break;

    case 1:
      led_offbits(BOARD_LED1_BIT | BOARD_LED3_BIT |BOARD_LED4_BIT);
      led_onbits(BOARD_LED2_BIT);
      break;

    case 2:
      led_offbits(BOARD_LED3_BIT |BOARD_LED4_BIT);
      led_onbits(BOARD_LED1_BIT | BOARD_LED2_BIT);
      break;

    case 3:
      led_offbits(BOARD_LED1_BIT | BOARD_LED2_BIT |BOARD_LED4_BIT);
      led_onbits(BOARD_LED3_BIT);
      break;

    case 4:
      stm32_gpiowrite(GPIO_LED4, false);
      break;
    }
}
#endif

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Select the "logical" OFF state:
 *
 *   SYMBOL            Val    Meaning                     LED state
 *                                                       LED1 LED2 LED3 LED4
 *   ----------------- ---   -----------------------  ---- ---- ---- ----
 *   LED_STARTED        0   NuttX has been started     ON  OFF  OFF  OFF
 *   LED_HEAPALLOCATE   1   Heap has been allocated   OFF   ON  OFF  OFF
 *   LED_IRQSENABLED    2   Interrupts enabled         ON   ON  OFF  OFF
 *   LED_STACKCREATED   3   Idle stack created        OFF  OFF  ON   OFF
 *   LED_INIRQ          4   In an interrupt           N/C  N/C  N/C  GLOW
 *   LED_SIGNAL         4   In a signal handler       N/C  N/C  N/C  GLOW
 *   LED_ASSERTION      4   An assertion failed       N/C  N/C  N/C  GLOW
 *   LED_PANIC          4   The system has crashed    N/C  N/C  N/C  FLASH
 *   ED_IDLE                MCU is is sleep mode         Not used
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_off(int led)
{
  switch (led)
    {
    case 0:
    case 1:
    case 2:
    case 3:
      break;

    case 4:
      stm32_gpiowrite(GPIO_LED4, true);
      break;
    }
}
#endif

/************************************************************************************
 * Name:  board_userled_initialize, board_userled, and board_userled_all
 *
 * Description:
 *   These interfaces allow user control of the board LEDs.
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control both on-board LEDs up
 *   until the completion of boot.  The it will continue to control LED2; LED1 is
 *   avaiable for application use.
 *
 *   If CONFIG_ARCH_LEDS is not defined, then both LEDs are available for application
 *   use.
 *
 ************************************************************************************/

void board_userled_initialize(void)
{
  /* Already initialized by stm32_led_initialize */
}

void board_userled(int led, bool ledon)
{
  uint32_t pinset;

  switch (led)
    {
      case BOARD_LED1:
        pinset = GPIO_LED1;
        break;

      case BOARD_LED2:
        pinset = GPIO_LED2;
        break;

      case BOARD_LED3:
        pinset = GPIO_LED3;
        break;

      case BOARD_LED4:
#ifndef CONFIG_ARCH_LEDS
        pinset = GPIO_LED4;
        break;
#endif
      default:
        return;
    }

  stm32_gpiowrite(pinset, !ledon);
}

void board_userled_all(uint8_t ledset)
{
#ifdef CONFIG_ARCH_LEDS
  led_onbits(ledset & ~BOARD_LED4_BIT);
  led_offbits(~(ledset | BOARD_LED4_BIT));
#else
  led_onbits(ledset);
  led_offbits(~ledset);
#endif
}
