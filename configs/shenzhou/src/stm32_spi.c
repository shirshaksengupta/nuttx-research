/************************************************************************************
 * configs/shenzhou/src/stm32_spi.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32.h"
#include "shenzhou.h"

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI3)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG_FEATURES too) */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_SPI
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spierr  llerr
#  ifdef CONFIG_DEBUG_INFO
#    define spiinfo llerr
#  else
#    define spiinfo(x...)
#  endif
#else
#  define spierr(x...)
#  define spiinfo(x...)
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Shenzhou board.
 *
 ************************************************************************************/

void weak_function stm32_spidev_initialize(void)
{
  /* NOTE: Clocking for SPI1 and/or SPI3 was already provided in stm32_rcc.c.
   *       Configurations of SPI pins is performed in stm32_spi.c.
   *       Here, we only initialize chip select pins unique to the board
   *       architecture.
   */

  /* SPI1 connects to the SD CARD and to the SPI FLASH */

#ifdef CONFIG_STM32_SPI1
  stm32_configgpio(GPIO_SD_CS);       /* SD card chip select */
  stm32_configgpio(GPIO_SD_CD);       /* SD card detect */
  stm32_configgpio(GPIO_FLASH_CS);    /* FLASH chip select */
#endif

  /* SPI3 connects to TFT LCD module and the RF24L01 2.4G wireless module */

#ifdef CONFIG_STM32_SPI3
  stm32_configgpio(GPIO_TP_CS);       /* Touchscreen chip select */
  stm32_configgpio(GPIO_LCDDF_CS);    /* Data flash chip select (on the LCD module) */
  stm32_configgpio(GPIO_LCDSD_CS);    /* SD chip select (on the LCD module) */
  stm32_configgpio(GPIO_WIRELESS_CS); /* Wireless chip select */
#endif
}

/****************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including stm32_spibus_initialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  /* SPI1 connects to the SD CARD and to the SPI FLASH */

  if (devid == SPIDEV_MMCSD)
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_SD_CS, !selected);
    }
  else if (devid == SPIDEV_FLASH)
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_FLASH_CS, !selected);
    }
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  /* The card detect pin is pulled up so that we detect the presence of a card
   * by see a low value on the input pin.
   */

  if (stm32_gpioread(GPIO_SD_CD))
    {
      return 0;
    }

  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_STM32_SPI3
void stm32_spi3select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  /* SPI3 connects to TFT LCD (for touchscreen and SD) and the RF24L01 2.4G
   * wireless module.
   */

  if (devid == SPIDEV_TOUCHSCREEN)
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_TP_CS, !selected);
    }
  else if (devid == SPIDEV_MMCSD)
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_LCDDF_CS, !selected);
    }
  else if (devid == SPIDEV_FLASH)
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_LCDSD_CS, !selected);
    }
  else if (devid == SPIDEV_WIRELESS)
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_WIRELESS_CS, !selected);
    }
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}
#endif

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI3 */
