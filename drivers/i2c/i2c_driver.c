/****************************************************************************
 * drivers/i2c/i2c_driver.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_I2C_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device naming ************************************************************/

#define DEVNAME_FMT    "/dev/i2c%d"
#define DEVNAME_FMTLEN (8 + 3 + 1)

/* Debug ********************************************************************/
/* CONFIG_DEBUG_I2C + CONFIG_DEBUG_FEATURES enables general I2C debug output. */

#ifdef CONFIG_DEBUG_I2C
#  define i2cerr err
#  define i2cinfo info
#else
#  define i2cerr(x...)
#  define i2cinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Driver state structure */

struct i2c_driver_s
{
  FAR struct i2c_master_s *i2c;  /* Contained I2C lower half driver */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  sem_t exclsem;                 /* Mutual exclusion */
  int16_t crefs;                 /* Number of open references */
  bool unlinked;                 /* True, driver has been unlinked */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     i2cdrvr_open(FAR struct file *filep);
static int     i2cdrvr_close(FAR struct file *filep);
static ssize_t i2cdrvr_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t i2cdrvr_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     i2cdrvr_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     i2cdrvr_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations i2cdrvr_fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  i2cdrvr_open,    /* open */
  i2cdrvr_close,   /* close */
#else
  0,               /* open */
  0,               /* close */
#endif
  i2cdrvr_read,    /* read */
  i2cdrvr_write,   /* write */
  0,               /* seek */
  i2cdrvr_ioctl    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0              /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , i2cdrvr_unlink /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2cdrvr_open
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int i2cdrvr_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct i2c_driver_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct i2c_driver_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the I2C driver state structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      int errcode = errno;
      DEBUGASSERT(errcode < 0);
      return -errcode;
    }

  /* Increment the count of open references on the RTC driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  sem_post(&priv->exclsem);
  return OK;
}
#endif

/****************************************************************************
 * Name: i2cdrvr_close
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int i2cdrvr_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct i2c_driver_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct i2c_driver_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the I2C driver state structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      int errcode = errno;
      DEBUGASSERT(errcode < 0);
      return -errcode;
    }

  /* Decrement the count of open references on the RTC driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then commit Hara-Kiri now.
   */

  if (priv->crefs <= 0 && priv->unlinked)
    {
      sem_destroy(&priv->exclsem);
      kmm_free(priv);
      return OK;
    }

  sem_post(&priv->exclsem);
  return OK;
}
#endif

/****************************************************************************
 * Name: i2cdrvr_read
 ****************************************************************************/

static ssize_t i2cdrvr_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: i2cdrvr_write
 ****************************************************************************/

static ssize_t i2cdrvr_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  return len; /* Say that everything was written */
}

/****************************************************************************
 * Name: i2cdrvr_ioctl
 ****************************************************************************/

static int i2cdrvr_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct i2c_driver_s *priv;
  FAR struct i2c_transfer_s *transfer;
  int ret;

  i2cinfo("cmd=%d arg=%lu\n", cmd, arg);

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct i2c_driver_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the I2C driver state structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      int errcode = errno;
      DEBUGASSERT(errcode < 0);
      return -errcode;
    }

  /* Process the IOCTL command */

  switch (cmd)
    {
      /* Command:      I2CIOC_TRANSFER
       * Description:  Perform an I2C transfer
       * Argument:     A reference to an instance of struct i2c_transfer_s.
       * Dependencies: CONFIG_I2C_DRIVER
       */

      case I2CIOC_TRANSFER:
        {
          /* Get the reference to the i2c_transfer_s structure */

          transfer = (FAR struct i2c_transfer_s *)((uintptr_t)arg);
          DEBUGASSERT(transfer != NULL);

          /* Perform the transfer */

          ret = I2C_TRANSFER(priv->i2c, transfer->msgv, transfer->msgc);
        }
        break;

#ifdef CONFIG_I2C_RESET
      /* Command:      I2CIOC_RESET
       * Description:  Perform an I2C bus reset in an attempt to break loose
       *               stuck I2C devices.
       * Argument:     None
       * Dependencies: CONFIG_I2C_DRIVER && CONFIG_I2C_RESET
       */

      case I2CIOC_RESET:
        {
          ret = I2C_RESET(priv->i2c);
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        break;
    }

  sem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: i2cdrvr_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int i2cdrvr_unlink(FAR struct inode *inode)
{
  FAR struct i2c_driver_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct i2c_driver_s *)inode->i_private;

  /* Get exclusive access to the I2C driver state structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      int errcode = errno;
      DEBUGASSERT(errcode < 0);
      return -errcode;
    }

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      sem_destroy(&priv->exclsem);
      kmm_free(priv);
      return OK;
    }

  /* No... just mark the driver as unlinked and free the resouces when the
   * last client closes their reference to the driver.
   */

  priv->unlinked = true;
  sem_post(&priv->exclsem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_register
 *
 * Description:
 *   Create and register the I2C character driver.
 *
 *   The I2C character driver is a simple character driver that supports I2C
 *   transfers.  The intent of this driver is to support I2C testing.  It is
 *   not suitable for use in any real driver application.
 *
 * Input Parameters:
 *   i2c - An instance of the lower half I2C driver
 *   bus - The I2C bus number.  This will be used as the I2C device minor
 *     number.  The I2C character device will be registered as /dev/i2cN
 *     where N is the minor number
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int i2c_register(FAR struct i2c_master_s *i2c, int bus)
{
  FAR struct i2c_driver_s *priv;
  char devname[DEVNAME_FMTLEN];
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL && (unsigned)bus < 1000);

  /* Allocate a I2C character device structure */

  priv = (FAR struct i2c_driver_s *)kmm_zalloc(sizeof(struct i2c_driver_s));
  if (priv)
    {
      /* Initialize the I2C character device structure */

      priv->i2c = i2c;
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
      sem_init(&priv->exclsem, 0, 1);
#endif

      /* Create the character device name */

      snprintf(devname, DEVNAME_FMTLEN, DEVNAME_FMT, bus);
      ret = register_driver(devname, &i2cdrvr_fops, 0666, priv);
      if (ret < 0)
        {
          /* Free the device structure if we failed to create the character
           * device.
           */

          kmm_free(priv);
        }

      /* Return the result of the registration */

      return OK;
    }


  return -ENOMEM;
}

#endif /* CONFIG_I2C_DRIVER */