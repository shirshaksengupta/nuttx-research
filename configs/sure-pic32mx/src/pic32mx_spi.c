/************************************************************************************
 * configs/sure-pic32mx/src/pic32mx_spi.c
 *
 *   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
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
#include "pic32mx.h"
#include "sure-pic32mx.h"

#if defined(CONFIG_PIC32MX_SPI2)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifdef CONFIG_ARCH_DBDP11215

/* The Sure DB_DP11215 PIC32 Storage Demo Board has an SD slot connected on SPI2:
 *
 *  SCK2/PMA5/CN8/RG6    SCK    SD connector SCK, FLASH (U1) SCK*
 *  SDI2/PMA4/CN9/RG7    SDI    SD connector DO, FLASH (U1) SO*
 *  SDO2/PMA3/CN10/RG8   SDO    SD connector DI, FLASH (U1) SI*
 *
 * Chip Select.  Pulled up on-board
 *  TDO/AN11/PMA12/RB11  SD_CS  SD connector CS
 *
 * Status inputs.  All pulled up on-board
 *
 *  TCK/AN12/PMA11/RB12  SD_CD  SD connector CD
 *  TDI/AN13/PMA10/RB13  SD_WD  SD connector WD
 */

#  define PIC32_HAVE_SD 1

#  define GPIO_SD_CS (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTB|GPIO_PIN11)
#  define GPIO_SD_CD (GPIO_INPUT|GPIO_INT|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_SD_WD (GPIO_INPUT|GPIO_PORTB|GPIO_PIN13)

/* The Sure DB_DP11215 PIC32 Storage Demo Board has pads an SOIC (Flash or
 * EEPROM) connected on SPI2, however, U4 is not populated on my board.
 *
 *
 *  TMS/AN10/CVREFOUT/PMA13/RB10  UTIL_WP        FLASH (U1) WP
 *  SS2/PMA2/CN11/RG9             UTIL_CS        FLASH (U1) CS
 */

#  undef PIC32_HAVE_SOIC

#  define GPIO_SOIC_WP (GPIO_INPUT|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_SOIC_CS (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTG|GPIO_PIN0)

/* Change notification numbers -- Not available for SD_CD. */

#endif

#ifdef CONFIG_ARCH_DBDP11212

/* The Sure DB-DP11212 PIC32 General Purpose Demo Board does not have an
 * SD slot.
 */

#  undef PIC32_HAVE_SD

/* The Sure DB-DP11212 PIC32 General Purpose Demo Board has an SOIC (Flash or
 * EEPROM) connected on SPI2:
 *
 *  TMS/AN10/PMA13/RB10   UTIL_WP FLASH (U4) WP
 *  TDO/AN11/PMA12/RB11   UTIL_CS FLASH (U4) CS
 */

#  define PIC32_HAVE_SOIC 1

#  define GPIO_SOIC_WP (GPIO_INPUT|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_SOIC_CS (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTB|GPIO_PIN11)
#endif

/* The following enable debug output from this file.
 *
 * CONFIG_DEBUG_SPI && CONFIG_DEBUG_FEATURES - Define to enable basic SPI debug
 * CONFIG_DEBUG_INFO - Define to enable verbose SPI debug
 */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_SPI
#  undef CONFIG_DEBUG_INFO
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
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: pic32mx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Sure PIC32MX board.
 *
 ************************************************************************************/

void weak_function pic32mx_spidev_initialize(void)
{
  /* Configure the SPI2 chip select (CS) GPIO output, and the card detect (CD) and
   * write protect (WP) inputs.
   */

#ifdef PIC32_HAVE_SD
  pic32mx_configgpio(GPIO_SD_CS);
  pic32mx_configgpio(GPIO_SD_CD);
  pic32mx_configgpio(GPIO_SD_WD);
#endif

#ifdef PIC32_HAVE_SOIC
  pic32mx_configgpio(GPIO_SOIC_WP);
  pic32mx_configgpio(GPIO_SOIC_CS);
#endif
}

/************************************************************************************
 * Name:  pic32mx_spi2select and pic32mx_spi2status
 *
 * Description:
 *   The external functions, pic32mx_spi2select and pic32mx_spi2status
 *   must be provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including pic32mx_spibus_initialize())
 *   are provided by common PIC32MX logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in pic32mx_boardinitialize() to configure SPI/SPI chip select
 *      pins.
 *   2. Provide pic32mx_spi2select() and pic32mx_spi2status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to pic32mx_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by pic32mx_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_SPI2
void pic32mx_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  /* The SD card chip select is pulled high and active low */

#ifdef PIC32_HAVE_SD
  if (devid == SPIDEV_MMCSD)
    {
      pic32mx_gpiowrite(GPIO_SD_CS, !selected);
    }
#endif

#ifdef PIC32_HAVE_SOIC
  if (devid == SPIDEV_FLASH)
    {
      pic32mx_gpiowrite(GPIO_SOIC_CS, !selected);
    }
#endif
}

uint8_t pic32mx_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  uint8_t ret = 0;

  /* Card detect is pull up on-board.  If a low value is sensed then the
   * card must be present.
   */

#ifdef PIC32_HAVE_SD
  if (devid == SPIDEV_MMCSD)
    {
      if (!pic32mx_gpioread(GPIO_SD_CD))
        {
          ret = SPI_STATUS_PRESENT;

          /* It seems that a high value indicates the card is write
           * protected.
           */

          if (pic32mx_gpioread(GPIO_SD_WD))
            {
              ret |= SPI_STATUS_WRPROTECTED;
            }
        }
    }
#endif

#ifdef PIC32_HAVE_SOIC
  if (devid == SPIDEV_FLASH)
    {
      ret = SPI_STATUS_PRESENT;

      /* Write protect is indicated with a low value. */

      if (pic32mx_gpioread(GPIO_SOIC_WP))
        {
          ret |= SPI_STATUS_WRPROTECTED;
        }
    }
#endif

  spiinfo("Returning %d\n", ret);
  return ret;
}
#endif
#endif /* CONFIG_PIC32MX_SPI2 */
