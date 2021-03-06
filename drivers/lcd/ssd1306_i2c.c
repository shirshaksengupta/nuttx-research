/****************************************************************************
 * drivers/lcd/ssd1306_i2c.c
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis. All rights reserved.
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

#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/ssd1306.h>

#include "ssd1306.h"

#if defined(CONFIG_LCD_SSD1306) && defined(CONFIG_LCD_SSD1306_I2C)

/****************************************************************************
 * Name: ssd1306_sendbyte
 *
 * Description:
 *   Write an 8-bit value into SSD1306
 *
 ****************************************************************************/

void ssd1306_sendbyte(FAR struct ssd1306_dev_s *priv, uint8_t regval)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - SSD1306_Reg_Address - SSD1306_Write_Data - STOP
   */

  struct i2c_msg_s msg;
  uint8_t txbuffer[1];
  int ret;

#ifdef CONFIG_LCD_SSD1306_REGDEBUG
  llerr("-> 0x%02x\n", regval);
#endif

  /* Setup to the data to be transferred.  Two bytes:  The SSD1306 register
   * address followed by one byte of data.
   */

  txbuffer[0] = regval;

  /* Setup 8-bit SSD1306 address write message */

  msg.frequency = CONFIG_SSD1306_I2CFREQ;  /* I2C frequency */
  msg.addr      = priv->addr;              /* 7-bit address */
  msg.flags     = 0;                       /* Write transaction, beginning with START */
  msg.buffer    = txbuffer;                /* Transfer from this address */
  msg.length    = 1;                       /* Send one byte following the address
                                            * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: ssd1306_sendblk
 *
 * Description:
 *   Write an array of bytes to SSD1306
 *
 ****************************************************************************/

void ssd1306_sendblk(FAR struct ssd1306_dev_s *priv, uint8_t *data, uint8_t len)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - SSD1306_Reg_Address - SSD1306_Write_Data - STOP
   */

  struct i2c_msg_s msg;
  int ret;

  /* Setup 8-bit SSD1306 address write message */

  msg.frequency = CONFIG_SSD1306_I2CFREQ;  /* I2C frequency */
  msg.addr      = priv->addr;              /* 7-bit address */
  msg.flags     = 0;                       /* Write transaction, beginning with START */
  msg.buffer    = data;                    /* Transfer from this address */
  msg.length    = len;                     /* Send one byte following the address
                                            * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }
}

#endif /* CONFIG_LCD_SSD1306 &7 CONFIG_LCD_SSD1306_I2C */
