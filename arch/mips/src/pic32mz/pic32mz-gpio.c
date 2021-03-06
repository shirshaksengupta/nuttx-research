/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz-gpio.c
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "chip/pic32mz-ioport.h"
#include "pic32mz-gpio.h"

#if CHIP_NPORTS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* This table can be used to map a port number to a IOPORT base address.  For
 * example, an index of zero would correspond to IOPORTA, one with IOPORTB,
 * etc.
 */

const uintptr_t g_gpiobase[CHIP_NPORTS] =
{
  PIC32MZ_IOPORTA_K1BASE
#if CHIP_NPORTS > 1
  , PIC32MZ_IOPORTB_K1BASE
#endif
#if CHIP_NPORTS > 2
  , PIC32MZ_IOPORTC_K1BASE
#endif
#if CHIP_NPORTS > 3
  , PIC32MZ_IOPORTD_K1BASE
#endif
#if CHIP_NPORTS > 4
  , PIC32MZ_IOPORTE_K1BASE
#endif
#if CHIP_NPORTS > 5
  , PIC32MZ_IOPORTF_K1BASE
#endif
#if CHIP_NPORTS > 6
  , PIC32MZ_IOPORTG_K1BASE
#endif
#if CHIP_NPORTS > 7
  , PIC32MZ_IOPORTH_K1BASE
#endif
#if CHIP_NPORTS > 8
  , PIC32MZ_IOPORTJ_K1BASE
#endif
#if CHIP_NPORTS > 9
  , PIC32MZ_IOPORTK_K1BASE
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: Inline PIN set field extractors
 ****************************************************************************/

static inline bool pic32mz_output(pinset_t pinset)
{
  return ((pinset & GPIO_OUTPUT) != 0);
}

static inline bool pic32mz_opendrain(pinset_t pinset)
{
  return ((pinset & GPIO_MODE_MASK) == GPIO_OPENDRAN);
}

static inline bool pic32mz_outputhigh(pinset_t pinset)
{
  return ((pinset & GPIO_VALUE_MASK) != 0);
}

static inline bool pic32mz_value(pinset_t pinset)
{
  return ((pinset & GPIO_VALUE_MASK) != GPIO_VALUE_ZERO);
}

static inline unsigned int pic32mz_portno(pinset_t pinset)
{
  return ((pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
}

static inline unsigned int pic32mz_pinno(pinset_t pinset)
{
  return ((pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

static inline unsigned int pic32mz_analog(pinset_t pinset)
{
  return ((pinset & GPIO_ANALOG_MASK) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin (the
 *   interrupt will be configured when pic32mz_attach() is called.
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int pic32mz_configgpio(pinset_t cfgset)
{
  unsigned int port = pic32mz_portno(cfgset);
  unsigned int pin  = pic32mz_pinno(cfgset);
  uint32_t     mask = (1 << pin);
  uintptr_t    base;

  /* Verify that the port number is within range */

  if (port < CHIP_NPORTS)
    {
      /* Get the base address of the ports */

      base = g_gpiobase[port];

      /* Is this an input or an output? */

      sched_lock();
      if (pic32mz_output(cfgset))
        {
          /* Not analog */

          putreg32(mask, base + PIC32MZ_IOPORT_ANSELCLR_OFFSET);

          /* It is an output; clear the corresponding bit in the TRIS register */

          putreg32(mask, base + PIC32MZ_IOPORT_TRISCLR_OFFSET);

          /* Is it an open drain output? */

          if (pic32mz_opendrain(cfgset))
            {
              /* It is an open drain output.  Set the corresponding bit in
               * the ODC register.
               */

              putreg32(mask, base + PIC32MZ_IOPORT_ODCSET_OFFSET);
            }
          else
            {
              /* Is is a normal output.  Clear the corresponding bit in the
               * ODC register.
               */

              putreg32(mask, base + PIC32MZ_IOPORT_ODCCLR_OFFSET);
            }

          /* Set the initial output value */

          pic32mz_gpiowrite(cfgset, pic32mz_outputhigh(cfgset));
        }
      else
        {
          /* It is an input; set the corresponding bit in the TRIS register. */

          putreg32(mask, base + PIC32MZ_IOPORT_TRISSET_OFFSET);
          putreg32(mask, base + PIC32MZ_IOPORT_ODCCLR_OFFSET);

          /* Is it an analog input? */

          if (pic32mz_analog(cfgset))
            {
              putreg32(mask, base + PIC32MZ_IOPORT_ANSELSET_OFFSET);
            }
          else
            {
              putreg32(mask, base + PIC32MZ_IOPORT_ANSELCLR_OFFSET);
            }
        }

      sched_unlock();
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: pic32mz_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void pic32mz_gpiowrite(pinset_t pinset, bool value)
{
  unsigned int port = pic32mz_portno(pinset);
  unsigned int pin  = pic32mz_pinno(pinset);
  uintptr_t    base;

  /* Verify that the port number is within range */

  if (port < CHIP_NPORTS)
    {
      /* Get the base address of the ports */

      base = g_gpiobase[port];

      /* Set or clear the output */

      if (value)
        {
          putreg32(1 << pin, base + PIC32MZ_IOPORT_PORTSET_OFFSET);
        }
      else
        {
          putreg32(1 << pin, base + PIC32MZ_IOPORT_PORTCLR_OFFSET);
        }
    }
}

/****************************************************************************
 * Name: pic32mz_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool pic32mz_gpioread(pinset_t pinset)
{
  unsigned int port = pic32mz_portno(pinset);
  unsigned int pin  = pic32mz_pinno(pinset);
  uintptr_t    base;

  /* Verify that the port number is within range */

  if (port < CHIP_NPORTS)
    {
      /* Get the base address of the ports */

      base = g_gpiobase[port];

      /* Get ane return the input value */

      return (getreg32(base + PIC32MZ_IOPORT_PORT_OFFSET) & (1 << pin)) != 0;
    }

  return false;
}

/****************************************************************************
 * Function:  pic32mz_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_INFO) && defined(CONFIG_DEBUG_GPIO)
void pic32mz_dumpgpio(uint32_t pinset, const char *msg)
{
  unsigned int port = pic32mz_portno(pinset);
  irqstate_t   flags;
  uintptr_t    base;

  /* Verify that the port number is within range */

  if (port < CHIP_NPORTS)
    {
      /* Get the base address of the ports */

      base = g_gpiobase[port];

      /* The following requires exclusive access to the GPIO registers */

      sched_lock();
      llerr("IOPORT%c pinset: %04x base: %08x -- %s\n",
            'A'+port, pinset, base, msg);
      llerr("   TRIS: %08x   PORT: %08x    LAT: %08x    ODC: %08x\n",
            getreg32(base + PIC32MZ_IOPORT_TRIS_OFFSET),
            getreg32(base + PIC32MZ_IOPORT_PORT_OFFSET),
            getreg32(base + PIC32MZ_IOPORT_LAT_OFFSET),
            getreg32(base + PIC32MZ_IOPORT_ODC_OFFSET));
      llerr("  CNCON: %08x   CNEN: %08x  CNPUE: %08x\n",
            getreg32(PIC32MZ_IOPORT_CNCON),
            getreg32(PIC32MZ_IOPORT_CNEN),
            getreg32(PIC32MZ_IOPORT_CNPUE));
      sched_unlock();
    }
}
#endif

#endif /* CHIP_NPORTS > 0 */
