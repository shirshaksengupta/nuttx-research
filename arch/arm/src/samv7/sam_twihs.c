/****************************************************************************
 * arch/arm/src/samv7/sam_twihs.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This driver derives from the SAMA5Dx TWIHS driver.  References:
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *    Copyright (c) 2011, Atmel Corporation
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
 * 3. Neither the name NuttX, Atmel, nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "chip/sam_pmc.h"
#include "chip/sam_pinmap.h"

#include "sam_periphclks.h"
#include "sam_gpio.h"
#include "sam_twihs.h"

#if defined(CONFIG_SAMV7_TWIHS0) || defined(CONFIG_SAMV7_TWIHS1) || \
    defined(CONFIG_SAMV7_TWIHS2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ***************************************************************/

#ifndef CONFIG_SAMV7_TWIHS0_FREQUENCY
#  define CONFIG_SAMV7_TWIHS0_FREQUENCY 100000
#endif

#ifndef CONFIG_SAMV7_TWIHS1_FREQUENCY
#  define CONFIG_SAMV7_TWIHS1_FREQUENCY 100000
#endif

#ifndef CONFIG_SAMV7_TWIHS2_FREQUENCY
#  define CONFIG_SAMV7_TWIHS2_FREQUENCY 100000
#endif

/* Driver internal definitions *************************************************/
/* If verbose I2C debug output is enable, then allow more time before we declare
 * a timeout.  The debug output from twi_interrupt will really slow things down!
 *
 * With a very slow clock (say 100,000 Hz), less than 100 usec would be required
 * to transfer on byte.  So these define a "long" timeout.
 */

#if defined(CONFIG_DEBUG_I2C) && defined(CONFIG_DEBUG_INFO)
#  define TWIHS_TIMEOUT_MSPB (50)  /* 50 msec/byte */
#else
#  define TWIHS_TIMEOUT_MSPB (5)   /* 5 msec/byte */
#endif

/* Clocking to the TWIHS module(s) is provided by the main clock, divided down
 * as necessary.
 * REVISIT -- This number came from the SAMA5Dx driver.
 */

#define TWIHS_MAX_FREQUENCY 66000000   /* Maximum TWIHS frequency */

/* Macros to convert a I2C pin to a PIO open-drain output */

#define I2C_INPUT       (PIO_INPUT | PIO_CFG_PULLUP)
#define I2C_OUTPUT      (PIO_OUTPUT | PIO_CFG_OPENDRAIN | PIO_OUTPUT_SET)

#define MKI2C_INPUT(p)  (((p) & (PIO_PORT_MASK | PIO_PIN_MASK)) | I2C_INPUT)
#define MKI2C_OUTPUT(p) (((p) & (PIO_PORT_MASK | PIO_PIN_MASK)) | I2C_OUTPUT)

/* Debug ***********************************************************************/
/* CONFIG_DEBUG_I2C + CONFIG_DEBUG_FEATURES enables general I2C debug output. */

#ifdef CONFIG_DEBUG_I2C
#  define i2cerr    err
#  define i2cinfo   info
#  define i2cllerr  llerr
#  define i2cllinfo llinfo
#else
#  define i2cerr(x...)
#  define i2cinfo(x...)
#  define i2cllerr(x...)
#  define i2cllinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* Invariant attributes of a TWIHS bus */

struct twi_attr_s
{
  uint8_t             twi;        /* TWIHS device number (for debug output) */
  uint8_t             pid;        /* TWIHS peripheral ID */
  uint16_t            irq;        /* IRQ number for this TWIHS bus */
  gpio_pinset_t       sclcfg;     /* TWIHS CK pin configuration (SCL in I2C-ese) */
  gpio_pinset_t       sdacfg;     /* TWIHS D pin configuration (SDA in I2C-ese) */
  uintptr_t           base;       /* Base address of TWIHS registers */
  xcpt_t              handler;    /* TWIHS interrupt handler */
};

/* State of a TWIHS bus */

struct twi_dev_s
{
  struct i2c_master_s dev;        /* Generic I2C device */
  const struct twi_attr_s *attr;  /* Invariant attributes of TWIHS device */
  struct i2c_msg_s    *msg;       /* Message list */
  uint32_t            twiclk;     /* TWIHS input clock frequency */
  uint32_t            frequency;  /* TWIHS transfer clock frequency */
  bool                initd;      /* True :device has been initialized */
  uint8_t             msgc;       /* Number of message in the message list */

  sem_t               exclsem;    /* Only one thread can access at a time */
  sem_t               waitsem;    /* Wait for TWIHS transfer completion */
  WDOG_ID             timeout;    /* Watchdog to recover from bus hangs */
  volatile int        result;     /* The result of the transfer */
  volatile int        xfrd;       /* Number of bytes transfers */

  /* Debug stuff */

#ifdef CONFIG_SAMV7_TWIHSHS_REGDEBUG
   bool               wrlast;     /* Last was a write */
   uint32_t           addrlast;   /* Last address */
   uint32_t           vallast;    /* Last value */
   int                ntimes;     /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helper functions */

static void twi_takesem(sem_t *sem);
#define     twi_givesem(sem) (sem_post(sem))

#ifdef CONFIG_SAMV7_TWIHSHS_REGDEBUG
static bool twi_checkreg(struct twi_dev_s *priv, bool wr,
              uint32_t value, uintptr_t address);
static uint32_t twi_getabs(struct twi_dev_s *priv, uintptr_t address);
static void twi_putabs(struct twi_dev_s *priv, uintptr_t address,
              uint32_t value);
#else
# define    twi_checkreg(priv,wr,value,address) (false)
# define    twi_putabs(p,a,v) putreg32(v,a)
# define    twi_getabs(p,a) getreg32(a)
#endif

static inline uint32_t twi_getrel(struct twi_dev_s *priv,
          unsigned int offset);
static inline void twi_putrel(struct twi_dev_s *priv, unsigned int offset,
          uint32_t value);

/* I2C transfer helper functions */

static int twi_wait(struct twi_dev_s *priv, unsigned int size);
static void twi_wakeup(struct twi_dev_s *priv, int result);
static int twi_interrupt(struct twi_dev_s *priv);
#ifdef CONFIG_SAMV7_TWIHS0
static int twi0_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAMV7_TWIHS1
static int twi1_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_SAMV7_TWIHS2
static int twi2_interrupt(int irq, FAR void *context);
#endif
static void twi_timeout(int argc, uint32_t arg, ...);

static void twi_startread(struct twi_dev_s *priv, struct i2c_msg_s *msg);
static void twi_startwrite(struct twi_dev_s *priv, struct i2c_msg_s *msg);
static void twi_startmessage(struct twi_dev_s *priv, struct i2c_msg_s *msg);

/* I2C device operations */

static int twi_transfer(FAR struct i2c_master_s *dev,
          FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int  twi_reset(FAR struct i2c_master_s * dev);
#endif

/* Initialization */

static void twi_setfrequency(struct twi_dev_s *priv, uint32_t frequency);
static void twi_hw_initialize(struct twi_dev_s *priv, uint32_t frequency);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SAMV7_TWIHS0
static const struct twi_attr_s g_twi0attr =
{
  .twi     = 0,
  .pid     = SAM_PID_TWIHS0,
  .irq     = SAM_IRQ_TWIHS0,
  .sclcfg  = GPIO_TWIHS0_CK,
  .sdacfg  = GPIO_TWIHS0_D,
  .base    = SAM_TWIHS0_BASE,
  .handler = twi0_interrupt,
};

static struct twi_dev_s g_twi0;
#endif

#ifdef CONFIG_SAMV7_TWIHS1
static const struct twi_attr_s g_twi1attr =
{
  .twi     = 1,
  .pid     = SAM_PID_TWIHS1,
  .irq     = SAM_IRQ_TWIHS1,
  .sclcfg  = GPIO_TWIHS1_CK,
  .sdacfg  = GPIO_TWIHS1_D,
  .base    = SAM_TWIHS1_BASE,
  .handler = twi1_interrupt,
};

static struct twi_dev_s g_twi1;
#endif

#ifdef CONFIG_SAMV7_TWIHS2
static const struct twi_attr_s g_twi2attr =
{
  .twi     = 2,
  .pid     = SAM_PID_TWIHS2,
  .irq     = SAM_IRQ_TWIHS2,
  .sclcfg  = GPIO_TWIHS2_CK,
  .sdacfg  = GPIO_TWIHS2_D,
  .base    = SAM_TWIHS2_BASE,
  .handler = twi2_interrupt,
};

static struct twi_dev_s g_twi2;
#endif

static const struct i2c_ops_s g_twiops =
{
  .transfer = twi_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = twi_reset
#endif
};

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: twi_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wake-ups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SDIO device driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void twi_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: twi_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   false: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_TWIHSHS_REGDEBUG
static bool twi_checkreg(struct twi_dev_s *priv, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      value   == priv->vallast &&  /* Same value? */
      address == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          llerr("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = value;
      priv->addrlast = address;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: twi_getabs
 *
 * Description:
 *  Read any 32-bit register using an absolute
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_TWIHSHS_REGDEBUG
static uint32_t twi_getabs(struct twi_dev_s *priv, uintptr_t address)
{
  uint32_t value = getreg32(address);

  if (twi_checkreg(priv, false, value, address))
    {
      llerr("%08x->%08x\n", address, value);
    }

  return value;
}
#endif

/****************************************************************************
 * Name: twi_putabs
 *
 * Description:
 *  Write to any 32-bit register using an absolute address
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_TWIHSHS_REGDEBUG
static void twi_putabs(struct twi_dev_s *priv, uintptr_t address,
                       uint32_t value)
{
  if (twi_checkreg(priv, true, value, address))
    {
      llerr("%08x<-%08x\n", address, value);
    }

  putreg32(value, address);
}
#endif

/****************************************************************************
 * Name: twi_getrel
 *
 * Description:
 *  Read a TWIHS register using an offset relative to the TWIHS base address
 *
 ****************************************************************************/

static inline uint32_t twi_getrel(struct twi_dev_s *priv, unsigned int offset)
{
  return twi_getabs(priv, priv->attr->base + offset);
}

/****************************************************************************
 * Name: twi_putrel
 *
 * Description:
 *  Write a value to a TWIHS register using an offset relative to the TWIHS base
 *  address.
 *
 ****************************************************************************/

static inline void twi_putrel(struct twi_dev_s *priv, unsigned int offset,
                              uint32_t value)
{
  twi_putabs(priv, priv->attr->base + offset, value);
}

/****************************************************************************
 * I2C transfer helper functions
 ****************************************************************************/

/****************************************************************************
 * Name: twi_wait
 *
 * Description:
 *   Perform a I2C transfer start
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static int twi_wait(struct twi_dev_s *priv, unsigned int size)
{
  uint32_t timeout;

  /* Calculate a timeout value based on the size of the transfer
   *
   *   ticks = msec-per-byte * bytes / msec-per-tick
   *
   * There is no concern about arithmetic overflow for reasonable transfer sizes.
   */

  timeout = MSEC2TICK(TWIHS_TIMEOUT_MSPB);
  if (timeout < 1)
    {
      timeout = 1;
    }

  /* Then start the timeout.  This timeout is needed to avoid hangs if/when an
   * TWIHS transfer stalls.
   */

  wd_start(priv->timeout, (timeout * size), twi_timeout, 1, (uint32_t)priv);

  /* Wait for either the TWIHS transfer or the timeout to complete */

  do
    {
      i2cinfo("TWIHS%d Waiting...\n", priv->attr->twi);
      twi_takesem(&priv->waitsem);
      i2cinfo("TWIHS%d Awakened with result: %d\n",
              priv->attr->twi, priv->result);
    }
  while (priv->result == -EBUSY);

  /* We get here via twi_wakeup.  The watchdog timer has been disabled and
   * all further interrupts for the TWIHS have been disabled.
   */

  return priv->result;
}

/****************************************************************************
 * Name: twi_wakeup
 *
 * Description:
 *   A terminal event has occurred.  Wake-up the waiting thread
 *
 ****************************************************************************/

static void twi_wakeup(struct twi_dev_s *priv, int result)
{
  /* Cancel any pending timeout */

  wd_cancel(priv->timeout);

  /* Disable any further TWIHS interrupts */

  twi_putrel(priv, SAM_TWIHS_IDR_OFFSET, TWIHS_INT_ALL);

  /* Wake up the waiting thread with the result of the transfer */

  priv->result = result;
  twi_givesem(&priv->waitsem);
}

/****************************************************************************
 * Name: twi_interrupt
 *
 * Description:
 *   The TWIHS Interrupt Handler
 *
 ****************************************************************************/

static int twi_interrupt(struct twi_dev_s *priv)
{
  struct i2c_msg_s *msg;
  uint32_t sr;
  uint32_t imr;
  uint32_t pending;
  uint32_t regval;

  /* Retrieve masked interrupt status */

  sr      = twi_getrel(priv, SAM_TWIHS_SR_OFFSET);
  imr     = twi_getrel(priv, SAM_TWIHS_IMR_OFFSET);
  pending = sr & imr;

  i2cllinfo("TWIHS%d pending: %08x\n", priv->attr->twi, pending);

  /* Byte received */

  msg = priv->msg;
  if ((pending & TWIHS_INT_RXRDY) != 0)
    {
      msg->buffer[priv->xfrd] = twi_getrel(priv, SAM_TWIHS_RHR_OFFSET);
      priv->xfrd++;

      /* Check for transfer complete */

      if (priv->xfrd >= msg->length)
        {
          struct i2c_msg_s *next = (msg + 1);

          /* Is there another message to after this one?  Does it require a
           * restart?
           */

          if (priv->msgc <= 1 || (next->flags & I2C_M_NORESTART) == 0)
            {
              /* The transfer is complete.  Disable the RXRDY interrupt and
               * enable the TXCOMP interrupt
               */

              twi_putrel(priv, SAM_TWIHS_IDR_OFFSET, TWIHS_INT_RXRDY);
              twi_putrel(priv, SAM_TWIHS_IER_OFFSET, TWIHS_INT_TXCOMP);
            }
          else
            {
              /* No.. just switch to the next message and continue receiving.
               * On the next RXRDY, we will continue with the first byt of the
               * next message.
               */

              DEBUGASSERT((next->flags & I2C_M_READ) != 0);
              priv->msg = next;
              priv->msgc--;
              priv->xfrd = 0;
            }
        }

      /* Not yet complete, but will the next be the last byte? */

      else if (priv->xfrd == (msg->length - 1))
        {
          struct i2c_msg_s *next = (msg + 1);

          /* Is there another message to after this one?  Does it require a
           * restart?
           */

          if (priv->msgc <= 1 || (next->flags & I2C_M_NORESTART) == 0)
            {
              /* This is the last message OR a restart is required before
               * the next mesage.  Send the stop signal.
               */

              twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_STOP);
            }
        }
    }

  /* Check for errors.  We must check for errors *before* checking TXRDY or
   * TXCMP because the error can be signaled in combination with TXRDY or
   * TXCOMP.
   */

  else if ((pending & TWIHS_INT_ERRORS) != 0)
    {
      /* Wake up the thread with an I/O error indication */

      i2cllerr("ERROR: TWIHS%d pending: %08x\n", priv->attr->twi, pending);
      twi_wakeup(priv, -EIO);
    }

  /* Byte sent */

  else if ((pending & TWIHS_INT_TXRDY) != 0)
    {
      /* Transfer finished? */

      if (priv->xfrd >= msg->length)
        {
          struct i2c_msg_s *next = (msg + 1);

          /* Is there another message to after this one?  Does it require a
           * restart?
           */

          if (priv->msgc <= 1 || (next->flags & I2C_M_NORESTART) == 0)
            {
              /* The transfer is complete.  Disable the TXRDY interrupt and
               * enable the TXCOMP interrupt
               */

              twi_putrel(priv, SAM_TWIHS_IDR_OFFSET, TWIHS_INT_TXRDY);
              twi_putrel(priv, SAM_TWIHS_IER_OFFSET, TWIHS_INT_TXCOMP);

              /* Send the STOP condition */

              regval  = twi_getrel(priv, SAM_TWIHS_CR_OFFSET);
              regval |= TWIHS_CR_STOP;
              twi_putrel(priv, SAM_TWIHS_CR_OFFSET, regval);
            }
          else
            {
              /* No.. just switch to the next message and continue sending.  */

              DEBUGASSERT((next->flags & I2C_M_READ) == 0);
              priv->msg = next;
              priv->msgc--;

              twi_putrel(priv, SAM_TWIHS_THR_OFFSET, next->buffer[0]);
              priv->xfrd = 1;
            }
        }

      /* No, there are more bytes remaining to be sent */

      else
        {
          twi_putrel(priv, SAM_TWIHS_THR_OFFSET, msg->buffer[priv->xfrd]);
          priv->xfrd++;
        }
    }

  /* Transfer complete,  Occurs on message only if the STOP bit was set on
   * the previously sent byte.
   */

  else if ((pending & TWIHS_INT_TXCOMP) != 0)
    {
      twi_putrel(priv, SAM_TWIHS_IDR_OFFSET, TWIHS_INT_TXCOMP);

      /* Is there another message to send? */

      if (priv->msgc > 1)
        {
          /* Yes... start the next message */

          priv->msg++;
          priv->msgc--;
          twi_startmessage(priv, priv->msg);
        }
      else
        {
          /* No.. we made it to the end of the message list with no errors.
           * Cancel any timeout and wake up the waiting thread with a
           * success indication.
           */

          twi_wakeup(priv, OK);
        }
    }

  return OK;
}

#ifdef CONFIG_SAMV7_TWIHS0
static int twi0_interrupt(int irq, FAR void *context)
{
  return twi_interrupt(&g_twi0);
}
#endif

#ifdef CONFIG_SAMV7_TWIHS1
static int twi1_interrupt(int irq, FAR void *context)
{
  return twi_interrupt(&g_twi1);
}
#endif

#ifdef CONFIG_SAMV7_TWIHS2
static int twi2_interrupt(int irq, FAR void *context)
{
  return twi_interrupt(&g_twi2);
}
#endif

/****************************************************************************
 * Name: twi_timeout
 *
 * Description:
 *   Watchdog timer for timeout of TWIHS operation
 *
 * Assumptions:
 *   Called from the timer interrupt handler with interrupts disabled.
 *
 ****************************************************************************/

static void twi_timeout(int argc, uint32_t arg, ...)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)arg;

  i2cllerr("ERROR: TWIHS%d Timeout!\n", priv->attr->twi);
  twi_wakeup(priv, -ETIMEDOUT);
}

/****************************************************************************
 * Name: twi_startread
 *
 * Description:
 *   Start the next read message
 *
 ****************************************************************************/

static void twi_startread(struct twi_dev_s *priv, struct i2c_msg_s *msg)
{
  /* Setup for the transfer */

  priv->result = -EBUSY;
  priv->xfrd   = 0;

  /* Set STOP signal if only one byte is sent */

  if (msg->length == 1)
    {
      twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_STOP);
    }

  /* Set slave address and number of internal address bytes. */

  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET, 0);
  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET, TWIHS_MMR_IADRSZ_NONE | TWIHS_MMR_MREAD |
                   TWIHS_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  twi_putrel(priv, SAM_TWIHS_IADR_OFFSET, 0);

  /* Enable read interrupt and send the START condition */

  twi_putrel(priv, SAM_TWIHS_IER_OFFSET, TWIHS_INT_RXRDY | TWIHS_INT_ERRORS);
  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_START);
}

/****************************************************************************
 * Name: twi_startwrite
 *
 * Description:
 *   Start the next write message
 *
 ****************************************************************************/

static void twi_startwrite(struct twi_dev_s *priv, struct i2c_msg_s *msg)
{
  /* Setup for the transfer */

  priv->result = -EBUSY;
  priv->xfrd   = 0;

  /* Set slave address and number of internal address bytes. */

  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET, 0);
  twi_putrel(priv, SAM_TWIHS_MMR_OFFSET, TWIHS_MMR_IADRSZ_NONE | TWIHS_MMR_DADR(msg->addr));

  /* Set internal address bytes (not used) */

  twi_putrel(priv, SAM_TWIHS_IADR_OFFSET, 0);

  /* Write first byte to send. */

  twi_putrel(priv, SAM_TWIHS_THR_OFFSET, msg->buffer[priv->xfrd++]);

  /* Enable write interrupt */

  twi_putrel(priv, SAM_TWIHS_IER_OFFSET, TWIHS_INT_TXRDY | TWIHS_INT_ERRORS);
}

/****************************************************************************
 * Name: twi_startmessage
 *
 * Description:
 *   Start the next write message
 *
 ****************************************************************************/

static void twi_startmessage(struct twi_dev_s *priv, struct i2c_msg_s *msg)
{
  if ((msg->flags & I2C_M_READ) != 0)
    {
      twi_startread(priv, msg);
    }
  else
    {
      twi_startwrite(priv, msg);
    }
}

/****************************************************************************
 * I2C device operations
 ****************************************************************************/

/****************************************************************************
 * Name: twi_transfer
 *
 * Description:
 *   Receive a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 * Returned Value:
 *   Returns zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int twi_transfer(FAR struct i2c_master_s *dev,
                        FAR struct i2c_msg_s *msgs, int count)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  irqstate_t flags;
  unsigned int size;
  int i;
  int ret;

  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);
  i2cinfo("TWIHS%d count: %d\n", priv->attr->twi, count);

  /* Calculate the total transfer size so that we can calculate a reasonable
   * timeout value.
   */

  size = 0;
  for (i = 0; i < count; i++)
    {
      size += msgs[i].length;
    }

  DEBUGASSERT(size > 0);

  /* Get exclusive access to the device */

  twi_takesem(&priv->exclsem);

  /* Setup the message transfer */

  priv->msg  = msgs;
  priv->msgc = count;

 /* Configure the I2C frequency.
  * REVISIT: Note that the frequency is set only on the first message.
  * This could be extended to support different transfer frequencies for
  * each message segment.
  */

  twi_setfrequency(priv, msgs->frequency);

  /* Initiate the transfer.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  flags = enter_critical_section();
  twi_startmessage(priv, msgs);

  /* And wait for the transfers to complete.  Interrupts will be re-enabled
   * while we are waiting.
   */

  ret = twi_wait(priv, size);
  if (ret < 0)
    {
      i2cerr("ERROR: Transfer failed: %d\n", ret);
    }

  leave_critical_section(flags);
  twi_givesem(&priv->exclsem);
  return ret;
}

/************************************************************************************
 * Name: twi_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
static int twi_reset(FAR struct i2c_master_s *dev)
{
  struct twi_dev_s *priv = (struct twi_dev_s *)dev;
  unsigned int clockcnt;
  unsigned int stretchcnt;
  uint32_t sclpin;
  uint32_t sdapin;
  int ret;

  ASSERT(priv);

  /* Get exclusive access to the TWIHS device */

  twi_takesem(&priv->exclsem);

  /* Disable TWIHS interrupts */

  up_disable_irq(priv->attr->irq);

  /* Use PIO configuration to un-wedge the bus.
   *
   * Reconfigure both pins as open drain outputs with initial output value
   * "high" (i.e., floating since these are open-drain outputs).
   */

  sclpin = MKI2C_OUTPUT(priv->attr->sclcfg);
  sdapin = MKI2C_OUTPUT(priv->attr->sdacfg);

  sam_configgpio(sclpin);
  sam_configgpio(sdapin);

  /* Peripheral clocking must be enabled in order to read valid data from
   * the output pin (clocking is enabled automatically for pins configured
   * as inputs).
   */

  sam_pio_forceclk(sclpin, true);
  sam_pio_forceclk(sdapin, true);

  /* Clock the bus until any slaves currently driving it low let it float.
   * Reading from the output will return the actual sensed level on the
   * SDA pin (not the level that we wrote).
   */

  clockcnt = 0;
  while (sam_pioread(sdapin) == false)
    {
      /* Give up if we have tried too hard */

      if (clockcnt++ > 10)
        {
          ret = -ETIMEDOUT;
          goto errout_with_lock;
        }

      /* Sniff to make sure that clock stretching has finished.  SCL should
       * be floating high here unless something is driving it low.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretchcnt = 0;
      while (sam_pioread(sclpin) == false)
        {
          /* Give up if we have tried too hard */

          if (stretchcnt++ > 10)
            {
              ret = -EAGAIN;
              goto errout_with_lock;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      sam_piowrite(sclpin, false);
      up_udelay(10);

      /* Drive SCL high (floating) again */

      sam_piowrite(sclpin, true);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  sam_piowrite(sdapin, false);
  up_udelay(10);
  sam_piowrite(sclpin, false);
  up_udelay(10);

  sam_piowrite(sclpin, true);
  up_udelay(10);
  sam_piowrite(sdapin, true);
  up_udelay(10);

  /* Clocking is no longer forced */

  sam_pio_forceclk(sclpin, false);
  sam_pio_forceclk(sdapin, false);

  /* Re-initialize the port hardware */

  twi_hw_initialize(priv, priv->frequency);
  ret = OK;

errout_with_lock:

  /* Release our lock on the bus */

  twi_givesem(&priv->exclsem);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Initialization
 ****************************************************************************/

/****************************************************************************
 * Name: twi_enableclk
 *
 * Description:
 *   Enable clocking on the selected TWIHS
 *
 ****************************************************************************/

static void twi_enableclk(struct twi_dev_s *priv)
{
  int pid;

  /* Get the peripheral ID associated with the TWIHS device port and enable
   * clocking to the TWIHS block.
   */

  pid = priv->attr->pid;
  if (pid < 32)
    {
      sam_enableperiph0(pid);
    }
  else
    {
      sam_enableperiph1(pid);
    }
}

/****************************************************************************
 * Name: twi_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void twi_setfrequency(struct twi_dev_s *priv, uint32_t frequency)
{
  unsigned int ckdiv;
  unsigned int cldiv;
  uint32_t regval;

  if (frequency != priv->frequency)
    {
      /* Configure TWIHS output clocking, trying each value of CKDIV {0..7} */

      for (ckdiv = 0; ckdiv < 8; ckdiv++)
        {
          /* Calculate the CLDIV value using the current CKDIV guess */

          cldiv = ((priv->twiclk / (frequency << 1)) - 4) / (1 << ckdiv);

          /* Is CLDIV in range? */

          if (cldiv <= 255)
            {
              /* Yes, break out and use it */

              break;
            }
        }

      /* Then setup the TWIHS Clock Waveform Generator Register, using the same
       * value for CLDIV and CHDIV (for 1:1 duty).
       */

      twi_putrel(priv, SAM_TWIHS_CWGR_OFFSET, 0);

      regval = ((uint32_t)ckdiv << TWIHS_CWGR_CKDIV_SHIFT) |
               ((uint32_t)cldiv << TWIHS_CWGR_CHDIV_SHIFT) |
               ((uint32_t)cldiv << TWIHS_CWGR_CLDIV_SHIFT);
      twi_putrel(priv, SAM_TWIHS_CWGR_OFFSET, regval);

      /* Save the requested frequency */

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: twi_hw_initialize
 *
 * Description:
 *   Initialize/Re-initialize the TWIHS peripheral.  This logic performs only
 *   repeatable initialization after either (1) the one-time initialization, or
 *   (2) after each bus reset.
 *
 ****************************************************************************/

static void twi_hw_initialize(struct twi_dev_s *priv, uint32_t frequency)
{
  irqstate_t flags = enter_critical_section();
  uint32_t regval;
  uint32_t mck;

  i2cinfo("TWIHS%d Initializing\n", priv->attr->twi);

  /* Configure PIO pins */

  sam_configgpio(priv->attr->sclcfg);
  sam_configgpio(priv->attr->sdacfg);

  /* Enable peripheral clocking */

  twi_enableclk(priv);

  /* SVEN: TWIHS Slave Mode Enabled */

  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_SVEN);

  /* Reset the TWIHS */

  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_SWRST);
  (void)twi_getrel(priv, SAM_TWIHS_RHR_OFFSET);

  /* TWIHS Slave Mode Disabled, TWIHS Master Mode Disabled. */

  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_SVDIS);
  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_MSDIS);

  /* Set master mode */

  twi_putrel(priv, SAM_TWIHS_CR_OFFSET, TWIHS_CR_MSEN);

  /* Determine the maximum valid frequency setting */

  mck = BOARD_MCK_FREQUENCY;

#ifdef SAMV7_HAVE_PMC_PCR_DIV
  /* Select the optimal value for the PCR DIV field */

  DEBUGASSERT((mck >> 3) <= TWIHS_MAX_FREQUENCY);
  if (mck <= TWIHS_MAX_FREQUENCY)
    {
      priv->twiclk = mck;
      regval       = PMC_PCR_DIV1;
    }
  else if ((mck >> 1) <= TWIHS_MAX_FREQUENCY)
    {
      priv->twiclk = (mck >> 1);
      regval       = PMC_PCR_DIV2;
    }
  else if ((mck >> 2) <= TWIHS_MAX_FREQUENCY)
    {
      priv->twiclk = (mck >> 2);
      regval       = PMC_PCR_DIV4;
    }
  else /* if ((mck >> 3) <= TWIHS_MAX_FREQUENCY) */
    {
      priv->twiclk = (mck >> 3);
      regval       = PMC_PCR_DIV8;
    }

#else
  /* No DIV field in the PCR register */

  priv->twiclk     = mck;
  regval           = 0;

#endif /* SAMV7_HAVE_PMC_PCR_DIV */

  /* Set the TWIHS peripheral input clock to the maximum, valid frequency */

  regval |= PMC_PCR_PID(priv->attr->pid) | PMC_PCR_CMD | PMC_PCR_EN;
  twi_putabs(priv, SAM_PMC_PCR, regval);

  /* Set the initial TWIHS data transfer frequency */

  priv->frequency  = 0;
  twi_setfrequency(priv, frequency);

  /* Enable Interrupts */

  up_enable_irq(priv->attr->irq);
  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_i2cbus_initialize
 *
 * Description:
 *   Initialize a TWIHS device for I2C operation
 *
 ****************************************************************************/

struct i2c_master_s *sam_i2cbus_initialize(int bus)
{
  struct twi_dev_s *priv;
  uint32_t frequency;
  irqstate_t flags;
  int ret;

  i2cinfo("Initializing TWIHS%d\n", bus);

#ifdef CONFIG_SAMV7_TWIHS0
  if (bus == 0)
    {
      /* Select up TWIHS0 and setup invariant attributes */

      priv       = &g_twi0;
      priv->attr = &g_twi0attr;

      /* Select the (initial) TWIHS frequency */

      frequency = CONFIG_SAMV7_TWIHS0_FREQUENCY;
    }
  else
#endif
#ifdef CONFIG_SAMV7_TWIHS1
  if (bus == 1)
    {
      /* Select up TWIHS1 and setup invariant attributes */

      priv       = &g_twi1;
      priv->attr = &g_twi1attr;

      /* Select the (initial) TWIHS frequency */

      frequency = CONFIG_SAMV7_TWIHS1_FREQUENCY;
    }
  else
#endif
#ifdef CONFIG_SAMV7_TWIHS2
  if (bus == 2)
    {
      /* Select up TWIHS2 and setup invariant attributes */

      priv       = &g_twi2;
      priv->attr = &g_twi2attr;

      /* Select the (initial) TWIHS frequency */

      frequency = CONFIG_SAMV7_TWIHS2_FREQUENCY;
    }
  else
#endif
    {
      i2cerr("ERROR: Unsupported bus: TWIHS%d\n", bus);
      return NULL;
    }

  /* Perform one-time TWIHS initialization */

  flags = enter_critical_section();

  /* Has the device already been initialized? */

  if (!priv->initd)
    {
      /* Allocate a watchdog timer */

      priv->timeout = wd_create();
      if (priv->timeout == NULL)
        {
          ierr("ERROR: Failed to allocate a timer\n");
          goto errout_with_irq;
        }

      /* Attach Interrupt Handler */

      ret = irq_attach(priv->attr->irq, priv->attr->handler);
      if (ret < 0)
        {
          ierr("ERROR: Failed to attach irq %d\n", priv->attr->irq);
          goto errout_with_wdog;
        }

      /* Initialize the TWIHS driver structure */

      priv->dev.ops = &g_twiops;

      (void)sem_init(&priv->exclsem, 0, 1);
      (void)sem_init(&priv->waitsem, 0, 0);

      /* Perform repeatable TWIHS hardware initialization */

      twi_hw_initialize(priv, frequency);

      /* Now it has been initialized */

      priv->initd = true;
    }

  leave_critical_section(flags);
  return &priv->dev;

errout_with_wdog:
  wd_delete(priv->timeout);
  priv->timeout = NULL;

errout_with_irq:
  leave_critical_section(flags);
  return NULL;
}

/****************************************************************************
 * Name: sam_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C device
 *
 ****************************************************************************/

int sam_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  struct twi_dev_s *priv = (struct twi_dev_s *) dev;
  irqstate_t flags;

  i2cinfo("TWIHS%d Un-initializing\n", priv->attr->twi);

  /* Disable TWIHS interrupts */

  flags = enter_critical_section();
  up_disable_irq(priv->attr->irq);

  /* Reset data structures */

  sem_destroy(&priv->exclsem);
  sem_destroy(&priv->waitsem);

  /* Free the watchdog timer */

  wd_delete(priv->timeout);
  priv->timeout = NULL;

  /* Detach Interrupt Handler */

  (void)irq_detach(priv->attr->irq);

  priv->initd = false;
  leave_critical_section(flags);
  return OK;
}

#endif /* CONFIG_SAMV7_TWIHS0 || CONFIG_SAMV7_TWIHS1 || CONFIG_SAMV7_TWIHS2 */
