/****************************************************************************
 * drivers/sensors/mpl115a.c
 * Character driver for the Freescale MPL115A1 Barometer Sensor
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/mpl115a.h>

#if defined(CONFIG_SPI) && defined(CONFIG_MPL115A)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

struct mpl115a_dev_s
{
  FAR struct spi_dev_s *spi; /* SPI interface */
  int16_t mpl115a_cal_a0;
  int16_t mpl115a_cal_b1;
  int16_t mpl115a_cal_b2;
  int16_t mpl115a_cal_c12;
  uint16_t mpl115a_temperature;
  uint16_t mpl115a_pressure;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void mpl115a_configspi(FAR struct spi_dev_s *spi);
static uint8_t mpl115a_getreg8(FAR struct mpl115a_dev_s *priv, uint8_t regaddr);
static void mpl115a_updatecaldata(FAR struct mpl115a_dev_s *priv);
static void mpl115a_read_press_temp(FAR struct mpl115a_dev_s *priv);
static int mpl115a_getpressure(FAR struct mpl115a_dev_s *priv);

/* Character driver methods */

static int     mpl115a_open(FAR struct file *filep);
static int     mpl115a_close(FAR struct file *filep);
static ssize_t mpl115a_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t mpl115a_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mpl115afops =
{
  mpl115a_open,   /* open */
  mpl115a_close,  /* close */
  mpl115a_read,   /* read */
  mpl115a_write,  /* write */
  0,              /* seek */
  0,              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,              /* poll */
#endif
  0               /* unlink */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void mpl115a_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the MPL115A */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  (void)SPI_HWFEATURES(spi, 0);
  (void)SPI_SETFREQUENCY(spi, MPL115A_SPI_MAXFREQUENCY);
}

/****************************************************************************
 * Name: mpl115a_getreg8
 *
 * Description:
 *   Read from an 8-bit MPL115A register
 *
 ****************************************************************************/

static uint8_t mpl115a_getreg8(FAR struct mpl115a_dev_s *priv, uint8_t regaddr)
{
  uint8_t regval;

  /* If SPI bus is shared then lock and configure it */

  (void)SPI_LOCK(priv->spi, true);
  mpl115a_configspi(priv->spi);

  /* Select the MPL115A */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER, true);

  /* Send register to read and get the next byte */

  (void)SPI_SEND(priv->spi, regaddr);
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Deselect the MPL115A */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER, false);

  /* Unlock bus */

  (void)SPI_LOCK(priv->spi, false);

#ifdef CONFIG_MPL115A_REGDEBUG
  err("%02x->%02x\n", regaddr, regval);
#endif
  return regval;
}

/****************************************************************************
 * Name: mpl115a_updatecaldata
 *
 * Description:
 *   Update Calibration Coefficient Data
 *
 ****************************************************************************/

static void mpl115a_updatecaldata(FAR struct mpl115a_dev_s *priv)
{
  /* Get a0 coefficient */

  priv->mpl115a_cal_a0 = mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_A0_MSB << 1)) << 8;
  priv->mpl115a_cal_a0 |= mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_A0_LSB << 1));
  sninfo("a0 = %d\n", priv->mpl115a_cal_a0);

  /* Get b1 coefficient */

  priv->mpl115a_cal_b1 = mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_B1_MSB << 1)) << 8;
  priv->mpl115a_cal_b1 |= mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_B1_LSB << 1));
  sninfo("b1 = %d\n", priv->mpl115a_cal_b1);

  /* Get b2 coefficient */

  priv->mpl115a_cal_b2 = mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_B2_MSB << 1)) << 8;
  priv->mpl115a_cal_b2 |= mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_B2_LSB << 1));
  sninfo("b2 = %d\n", priv->mpl115a_cal_b2);

  /* Get c12 coefficient */

  priv->mpl115a_cal_c12 = mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_C12_MSB << 1)) << 8;
  priv->mpl115a_cal_c12 |= mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_C12_LSB << 1));
  sninfo("c12 = %d\n", priv->mpl115a_cal_c12);
}

/****************************************************************************
 * Name: mpl115a_read_press_temp
 *
 * Description:
 *   Read raw pressure and temperature from MPL115A and store it in the
 *   mpl115a_dev_s structure.
 *
 ****************************************************************************/

static void mpl115a_read_press_temp(FAR struct mpl115a_dev_s *priv)
{
  /* Start a new conversion */

  mpl115a_getreg8(priv, (MPL115A_CONVERT << 1));

  /* Delay 5ms */

  usleep(5000);

  priv->mpl115a_pressure = mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_PADC_MSB << 1)) << 8;
  priv->mpl115a_pressure |= mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_PADC_LSB << 1));
  priv->mpl115a_pressure >>= 6; /* Padc is 10bit unsigned */

  sninfo("Pressure = %d\n", priv->mpl115a_pressure);

  priv->mpl115a_temperature = mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_TADC_MSB << 1)) << 8;
  priv->mpl115a_temperature |= mpl115a_getreg8(priv, MPL115A_BASE_CMD | (MPL115A_TADC_LSB << 1));
  priv->mpl115a_temperature >>= 6; /* Tadc is 10bit unsigned */

  sninfo("Temperature = %d\n", priv->mpl115a_temperature);
}

/****************************************************************************
 * Name: mpl115a_getpressure
 *
 * Description:
 *   Calculate the Barometric Pressure using the temperature compensated
 *   See Freescale AN3785 and MPL115A1 data sheet for details
 *
 ****************************************************************************/

static int mpl115a_getpressure(FAR struct mpl115a_dev_s *priv)
{
  int32_t c12x2, a1, a1x1, y1, a2x2, pcomp;
  uint16_t padc, tadc, pressure;

  /* Check if coefficient data were read correctly */

  if ((priv->mpl115a_cal_a0 == 0) || (priv->mpl115a_cal_b1 == 0) ||
      (priv->mpl115a_cal_b2 == 0) || (priv->mpl115a_cal_c12 == 0))
    {
      mpl115a_updatecaldata(priv);
    }

  /* Read temperature and pressure */

  mpl115a_read_press_temp(priv);
  padc = priv->mpl115a_pressure;
  tadc = priv->mpl115a_temperature;

  /* These code are from Freescale AN3785 */

  c12x2 = ((int32_t)priv->mpl115a_cal_c12 * tadc) >> 11;
  a1 = (int32_t) (priv->mpl115a_cal_b1 + c12x2);
  a1x1 = a1 * padc;
  y1 = (((int32_t)priv->mpl115a_cal_a0) << 10) + a1x1;
  a2x2 = (((int32_t)priv->mpl115a_cal_b2) * tadc) >> 1;
  pcomp = (y1 + a2x2) >> 9;

  /* Calculate the pressure in 1/16 kPa from compensated */

  pressure = (((((int32_t)pcomp) * 1041) >> 14) + 800);

  /* Note that the final pressure value is reported with 4 bit fractional
   * This may be eliminated by right shifting the result 4 bits.
   */

  sninfo("Final Pressure = %d\n", pressure >> 4);
  return pressure;
}

/****************************************************************************
 * Name: mpl115a_open
 *
 * Description:
 *   This function is called whenever the MPL115A1 device is opened.
 *
 ****************************************************************************/

static int mpl115a_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: mpl115a_close
 *
 * Description:
 *   This routine is called when the LM-75 device is closed.
 *
 ****************************************************************************/

static int mpl115a_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: mpl115a_read
 ****************************************************************************/

static ssize_t mpl115a_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct mpl115a_dev_s *priv  = inode->i_private;
  FAR uint16_t             *press = (FAR uint16_t *) buffer;

  if (!buffer)
    {
      snerr("ERROR: Buffer is null\n");
      return -1;
    }

  if (buflen != 2)
    {
      snerr("ERROR: You can't read something other than 16 bits (2 bytes)\n");
      return -1;
    }

  /* Get the pressure compensated */

  *press = mpl115a_getpressure(priv);

  /* Return size of uint16_t (2 bytes) */

  return 2;
}

/****************************************************************************
 * Name: mpl115a_write
 ****************************************************************************/

static ssize_t mpl115a_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpl115a_register
 *
 * Description:
 *   Register the MPL115A character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             MPL115A
 *   addr    - The I2C address of the LM-75.  The base I2C address of the
 *             MPL115A is 0x48.  Bits 0-3 can be controlled to get 8 unique
 *             addresses from 0x48 through 0x4f.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mpl115a_register(FAR const char *devpath, FAR struct spi_dev_s *spi)
{
  FAR struct mpl115a_dev_s *priv;
  int ret;

  /* Initialize the MPL115A device structure */

  priv = (FAR struct mpl115a_dev_s *)kmm_malloc(sizeof(struct mpl115a_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi             = spi;
  priv->mpl115a_cal_a0  = 0;
  priv->mpl115a_cal_b1  = 0;
  priv->mpl115a_cal_b2  = 0;
  priv->mpl115a_cal_c12 = 0;

  /* Read the coefficient value */

  mpl115a_updatecaldata(priv);

  /* Register the character driver */

  ret = register_driver(devpath, &g_mpl115afops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SPI && CONFIG_MPL115A */
