/****************************************************************************
 * configs/sama5d3-xplained/src/sam_ajoystick.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
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
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/input/ajoystick.h>

#include "sam_pio.h"
#include "sam_adc.h"
#include "chip/sam_adc.h"
#include "sama5d3-xplained.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Check for pre-requisites and pin conflicts */

#ifdef CONFIG_AJOYSTICK
#  if !defined(CONFIG_ADC)
#    error CONFIG_ADC is required for the Itead joystick
#    undef CONFIG_AJOYSTICK
#  elif !defined(CONFIG_SAMA5_ADC_CHAN0) || !defined(CONFIG_SAMA5_ADC_CHAN1)
#    error CONFIG_SAMA5_ADC_CHAN0 and 1 are required for Itead joystick
#  elif !defined(CONFIG_SAMA5_PIOC_IRQ)
#    error CONFIG_SAMA5_PIOC_IRQ is required for the Itead joystick
#    undef CONFIG_AJOYSTICK
#  elif defined(CONFIG_SAMA5_EMACA)
#    error EMAC conflicts with the Itead PIO usage
#    undef CONFIG_AJOYSTICK
#  elif defined(CONFIG_SAMA5_SSC0)
#    error SSC0 conflicts with the Itead PIO usage
#    undef CONFIG_AJOYSTICK
#  elif defined(CONFIG_SAMA5_SPI1)
#    warning SPI1 may conflict with the Itead PIO usage
#  elif defined(CONFIG_SAMA5_ISI)
#    warning ISI may conflict with the Itead PIO usage
#  endif
#endif /* CONFIG_AJOYSTICK */

#ifdef CONFIG_AJOYSTICK

/* Number of Joystick buttons */

#define AJOY_NGPIOS  7

/* Bitset of supported Joystick buttons */

#define AJOY_SUPPORTED (AJOY_BUTTON_1_BIT | AJOY_BUTTON_2_BIT | \
                        AJOY_BUTTON_3_BIT | AJOY_BUTTON_4_BIT | \
                        AJOY_BUTTON_5_BIT | AJOY_BUTTON_6_BIT | \
                        AJOY_BUTTON_7_BIT )

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ajoy_buttonset_t ajoy_supported(FAR const struct ajoy_lowerhalf_s *lower);
static int ajoy_sample(FAR const struct ajoy_lowerhalf_s *lower,
                       FAR struct ajoy_sample_s *sample);
static ajoy_buttonset_t ajoy_buttons(FAR const struct ajoy_lowerhalf_s *lower);
static void ajoy_enable(FAR const struct ajoy_lowerhalf_s *lower,
                         ajoy_buttonset_t press, ajoy_buttonset_t release,
                         ajoy_handler_t handler, FAR void *arg);

static void ajoy_disable(void);
static int ajoy_interrupt(int irq, FAR void *context);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Pin configuration for each Itead joystick button.  Index using AJOY_*
 * button definitions in include/nuttx/input/ajoystick.h.
 */

static const pio_pinset_t g_joypio[AJOY_NGPIOS] =
{
  PIO_BUTTON_1, PIO_BUTTON_2, PIO_BUTTON_3, PIO_BUTTON_4,
  PIO_BUTTON_5, PIO_BUTTON_6, PIO_BUTTON_6
};

static const uint8_t g_joyirq[AJOY_NGPIOS] =
{
  IRQ_BUTTON_1, IRQ_BUTTON_2, IRQ_BUTTON_3, IRQ_BUTTON_4,
  IRQ_BUTTON_5, IRQ_BUTTON_6, IRQ_BUTTON_6
};

/* This is the button joystick lower half driver interface */

static const struct ajoy_lowerhalf_s g_ajoylower =
{
  .al_supported  = ajoy_supported,
  .al_sample     = ajoy_sample,
  .al_buttons    = ajoy_buttons,
  .al_enable     = ajoy_enable,
};

/* Thread-independent file structure for the open ADC driver */

static struct file g_adcfile;

/* Current interrupt handler and argument */

static ajoy_handler_t g_ajoyhandler;
static FAR void *g_ajoyarg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ajoy_supported
 *
 * Description:
 *   Return the set of buttons supported on the button joystick device
 *
 ****************************************************************************/

static ajoy_buttonset_t ajoy_supported(FAR const struct ajoy_lowerhalf_s *lower)
{
  iinfo("Supported: %02x\n", AJOY_SUPPORTED);
  return (ajoy_buttonset_t)AJOY_SUPPORTED;
}

/****************************************************************************
 * Name: ajoy_sample
 *
 * Description:
 *   Return the current state of all button joystick buttons
 *
 ****************************************************************************/

static int ajoy_sample(FAR const struct ajoy_lowerhalf_s *lower,
                       FAR struct ajoy_sample_s *sample)
{
  struct adc_msg_s adcmsg[SAM_ADC_NCHANNELS];
  FAR struct adc_msg_s *ptr;
  ssize_t nread;
  ssize_t offset;
  int have;
  int i;

  /* Read all of the available samples (handling the case where additional
   * channels are enabled).
   */

  nread = file_read(&g_adcfile, adcmsg, MAX_ADC_CHANNELS * sizeof(struct adc_msg_s));
  if (nread < 0)
    {
      int errcode = get_errno();
      if (errcode != EINTR)
        {
          ierr("ERROR: read failed: %d\n", errcode);
        }

      return -errcode;
    }
  else if (nread < 2 * sizeof(struct adc_msg_s))
    {
      ierr("ERROR: read too small: %ld\n", (long)nread);
      return -EIO;
    }

  /* Sample and the raw analog inputs */

  for (i = 0, offset = 0, have = 0;
       i < SAM_ADC_NCHANNELS && offset < nread && have != 3;
       i++, offset += sizeof(struct adc_msg_s))
    {
      ptr = &adcmsg[i];

      /* Is this one of the channels that we need? */

      if ((have & 1) == 0 && ptr->am_channel == 0)
        {
          int32_t tmp = ptr->am_data;
          sample->as_x = (int16_t)tmp;
          have |= 1;

          iinfo("X sample: %ld -> %d\n", (long)tmp, (int)sample->as_x);
        }

      if ((have & 2) == 0 && ptr->am_channel == 1)
        {
          int32_t tmp = ptr->am_data;
          sample->as_y = (int16_t)tmp;
          have |= 2;

          iinfo("Y sample: %ld -> %d\n", (long)tmp, (int)sample->as_y);
        }
    }

  if (have != 3)
    {
      ierr("ERROR: Could not find joystack channels\n");
      return -EIO;
    }


  /* Sample the discrete button inputs */

  sample->as_buttons = ajoy_buttons(lower);
  iinfo("Returning: %02x\n", AJOY_SUPPORTED);
  return OK;
}

/****************************************************************************
 * Name: ajoy_buttons
 *
 * Description:
 *   Return the current state of button data (only)
 *
 ****************************************************************************/

static ajoy_buttonset_t ajoy_buttons(FAR const struct ajoy_lowerhalf_s *lower)
{
  ajoy_buttonset_t ret = 0;
  int i;

  /* Read each joystick GPIO value */

  for (i = 0; i < AJOY_NGPIOS; i++)
    {
      /* Button outputs are pulled high. So a sensed low level means that the
       * button is pressed.
       */

      if (!sam_pioread(g_joypio[i]))
        {
          ret |= (1 << i);
        }
    }

  iinfo("Returning: %02x\n", ret);
  return ret;
}

/****************************************************************************
 * Name: ajoy_enable
 *
 * Description:
 *   Enable interrupts on the selected set of joystick buttons.  And empty
 *   set will disable all interrupts.
 *
 ****************************************************************************/

static void ajoy_enable(FAR const struct ajoy_lowerhalf_s *lower,
                         ajoy_buttonset_t press, ajoy_buttonset_t release,
                         ajoy_handler_t handler, FAR void *arg)
{
  irqstate_t flags;
  ajoy_buttonset_t either = press | release;
  ajoy_buttonset_t bit;
  int i;

  /* Start with all interrupts disabled */

  flags = enter_critical_section();
  ajoy_disable();

  illinfo("press: %02x release: %02x handler: %p arg: %p\n",
          press, release, handler, arg);

  /* If no events are indicated or if no handler is provided, then this
   * must really be a request to disable interrupts.
   */

  if (either && handler)
    {
      /* Save the new the handler and argument */

      g_ajoyhandler = handler;
      g_ajoyarg     = arg;

      /* Check each GPIO. */

      for (i = 0; i < AJOY_NGPIOS; i++)
        {
           /* Enable interrupts on each pin that has either a press or
            * release event associated with it.
            */

           bit = (1 << i);
           if ((either & bit) != 0)
             {
               /* REVISIT:  It would be better if we reconfigured for
                * the edges of interest so that we do not get spurious
                * interrupts.
                */

               sam_pioirqenable(g_joyirq[i]);
             }
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ajoy_disable
 *
 * Description:
 *   Disable all joystick interrupts
 *
 ****************************************************************************/

static void ajoy_disable(void)
{
  irqstate_t flags;
  int i;

  /* Disable each joystick interrupt */

  flags = enter_critical_section();
  for (i = 0; i < AJOY_NGPIOS; i++)
    {
      sam_pioirqdisable(g_joyirq[i]);
    }

  leave_critical_section(flags);

  /* Nullify the handler and argument */

  g_ajoyhandler = NULL;
  g_ajoyarg     = NULL;
}

/****************************************************************************
 * Name: ajoy_interrupt
 *
 * Description:
 *   Discrete joystick interrupt handler
 *
 ****************************************************************************/

static int ajoy_interrupt(int irq, FAR void *context)
{
  DEBUGASSERT(g_ajoyhandler);
  if (g_ajoyhandler)
    {
      g_ajoyhandler(&g_ajoylower, g_ajoyarg);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_ajoy_initialization
 *
 * Description:
 *   Initialize and register the button joystick driver
 *
 ****************************************************************************/

int sam_ajoy_initialization(void)
{
  int ret;
  int fd;
  int i;

  /* Initialize ADC.  We will need this to read the ADC inputs */

  ret = board_adc_initialize();
  if (ret < 0)
    {
      ierr("ERROR: board_adc_initialize() failed: %d\n", ret);
      return ret;
    }

  /* Open the ADC driver for reading. */

  fd = open("/dev/adc0", O_RDONLY);
  if (fd < 0)
    {
      int errcode = get_errno();
      ierr("ERROR: Failed to open /dev/adc0: %d\n", errcode);
      return -errcode;
    }

  /* Detach the file structure from the file descriptor so that it can be
   * used on any thread.
   */

  ret = file_detach(fd, &g_adcfile);
  if (ret < 0)
    {
      ierr("ERROR: Failed to detach from file descriptor: %d\n", ret);
      (void)close(fd);
      return ret;
    }

  /* Configure the GPIO pins as interrupting inputs. */

  for (i = 0; i < AJOY_NGPIOS; i++)
    {
      /* Configure the PIO as an input */

      sam_configpio(g_joypio[i]);

      /* Configure PIO interrupts, attach the interrupt handler, but leave
       * the interrupt disabled.
       */

      sam_pioirq(g_joypio[i]);
      (void)irq_attach(g_joyirq[i], ajoy_interrupt);
      sam_pioirqdisable(g_joyirq[i]);
    }

  /* Register the joystick device as /dev/ajoy0 */

  ret = ajoy_register("/dev/ajoy0", &g_ajoylower);
  if (ret < 0)
    {
      ierr("ERROR: ajoy_register failed: %d\n", ret);
      file_close_detached(&g_adcfile);
    }

  return ret;
}

#endif /* CONFIG_AJOYSTICK */
