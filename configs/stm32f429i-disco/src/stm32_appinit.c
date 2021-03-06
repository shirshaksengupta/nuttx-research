/****************************************************************************
 * config/stm32f429i-disco/src/stm32_appinit.c
 *
 *   Copyright (C) 2012, 2015-2016 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#ifdef CONFIG_STM32_SPI4
#  include <nuttx/mmcsd.h>
#endif

#ifdef CONFIG_MTD_SST25XX
#  include <nuttx/mtd/mtd.h>
#endif

#ifdef CONFIG_SYSTEM_USBMONITOR
#  include <apps/usbmonitor.h>
#endif

#ifndef CONFIG_STM32F429I_DISCO_FLASH_MINOR
#define CONFIG_STM32F429I_DISCO_FLASH_MINOR 0
#endif

#ifdef CONFIG_STM32F429I_DISCO_FLASH_CONFIG_PART
#ifdef CONFIG_PLATFORM_CONFIGDATA
#  include <nuttx/configdata.h>
#endif
#endif

#ifdef CONFIG_STM32_OTGHS
#  include "stm32_usbhost.h"
#endif

#include "stm32.h"
#include "stm32f429i-disco.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1

/* Can't support USB host or device features if USB OTG HS is not enabled */

#ifndef CONFIG_STM32_OTGHS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_SYSTEM_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 *   CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_LIB_BOARDCTL=n :
 *     Called from board_initialize().
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#if defined(CONFIG_STM32_SPI4)
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  FAR struct mtd_geometry_s geo;
#endif

#if defined(CONFIG_MTD_PARTITION_NAMES)
  FAR const char *partname = CONFIG_STM32F429I_DISCO_FLASH_PART_NAMES;
#endif

#if defined(CONFIG_MTD) && defined(CONFIG_MTD_SST25XX)
  int ret;
#elif defined(HAVE_USBHOST) || defined(HAVE_USBMONITOR)
  int ret;
#endif

  /* Configure SPI-based devices */

#ifdef CONFIG_STM32_SPI4
  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port 4\n");

  spi = stm32_spibus_initialize(4);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port 4\n");
      return -ENODEV;
    }

  syslog(LOG_INFO, "Successfully initialized SPI port 4\n");

  /* Now bind the SPI interface to the SST25F064 SPI FLASH driver.  This
   * is a FLASH device that has been added external to the board (i.e.
   * the board does not ship from STM with any on-board FLASH.
   */

#if defined(CONFIG_MTD) && defined(CONFIG_MTD_SST25XX)
  syslog(LOG_INFO, "Bind SPI to the SPI flash driver\n");

  mtd = sst25xx_initialize(spi);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI port 4 to the SPI FLASH driver\n");
    }
  else
    {
      syslog(LOG_INFO, "Successfully bound SPI port 4 to the SPI FLASH driver\n");

      /* Get the geometry of the FLASH device */

      ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
      if (ret < 0)
        {
          ferr("ERROR: mtd->ioctl failed: %d\n", ret);
          return ret;
        }

#ifdef CONFIG_STM32F429I_DISCO_FLASH_PART
      {
        int partno;
        int partsize;
        int partoffset;
        int partszbytes;
        int erasesize;
        const char *partstring = CONFIG_STM32F429I_DISCO_FLASH_PART_LIST;
        const char *ptr;
        FAR struct mtd_dev_s *mtd_part;
        char  partref[4];

        /* Now create a partition on the FLASH device */

        partno = 0;
        ptr = partstring;
        partoffset = 0;

        /* Get the Flash erase size */

        erasesize = geo.erasesize;

        while (*ptr != '\0')
          {
            /* Get the partition size */

            partsize = atoi(ptr);
            partszbytes = (partsize << 10); /* partsize is defined in KB */

            /* Check if partition size is bigger then erase block */

            if (partszbytes < erasesize)
              {
                ferr("ERROR: Partition size is lesser than erasesize!\n");
                return -1;
              }

            /* Check if partition size is multiple of erase block */

            if ((partszbytes % erasesize) != 0)
              {
                ferr("ERROR: Partition size is not multiple of erasesize!\n");
                return -1;
              }

            mtd_part    = mtd_partition(mtd, partoffset, partszbytes / erasesize);
            partoffset += partszbytes / erasesize;

#ifdef CONFIG_STM32F429I_DISCO_FLASH_CONFIG_PART
            /* Test if this is the config partition */

            if (CONFIG_STM32F429I_DISCO_FLASH_CONFIG_PART_NUMBER == partno)
              {
                /* Register the partition as the config device */

                mtdconfig_register(mtd_part);
              }
            else
#endif
              {
                /* Now initialize a SMART Flash block device and bind it
                 * to the MTD device.
                 */

#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
                sprintf(partref, "p%d", partno);
                smart_initialize(CONFIG_STM32F429I_DISCO_FLASH_MINOR, mtd_part, partref);
#endif
              }

#if defined(CONFIG_MTD_PARTITION_NAMES)
            /* Set the partition name */

            if (mtd_part == NULL)
              {
                ferr("ERROR: failed to create partition %s\n", partname);
                return -1;
              }

            mtd_setpartitionname(mtd_part, partname);

            /* Now skip to next name.  We don't need to split the string here
             * because the MTD partition logic will only display names up to
             * the comma, thus allowing us to use a single static name
             * in the code.
             */

            while (*partname != ',' && *partname != '\0')
              {
                /* Skip to next ',' */

                partname++;
              }

            if (*partname == ',')
              {
                partname++;
              }
#endif

            /* Update the pointer to point to the next size in the list */

            while ((*ptr >= '0') && (*ptr <= '9'))
              {
                ptr++;
              }

            if (*ptr == ',')
              {
                ptr++;
              }

            /* Increment the part number */

            partno++;
          }
      }
#else /* CONFIG_STM32F429I_DISCO_FLASH_PART */

      /* Configure the device with no partition support */

      smart_initialize(CONFIG_STM32F429I_DISCO_FLASH_MINOR, mtd, NULL);

#endif /* CONFIG_STM32F429I_DISCO_FLASH_PART */
    }

#endif /* CONFIG_MTD */
#endif /* CONFIG_STM32_SPI4 */

  /* Create a RAM MTD device if configured */

#if defined(CONFIG_RAMMTD) && defined(CONFIG_STM32F429I_DISCO_RAMMTD)
  {
    uint8_t *start = (uint8_t *) kmm_malloc(CONFIG_STM32F429I_DISCO_RAMMTD_SIZE * 1024);
    mtd = rammtd_initialize(start, CONFIG_STM32F429I_DISCO_RAMMTD_SIZE * 1024);
    mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);

    /* Now initialize a SMART Flash block device and bind it to the MTD device */

#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
    smart_initialize(CONFIG_STM32F429I_DISCO_RAMMTD_MINOR, mtd, NULL);
#endif
  }

#endif /* CONFIG_RAMMTD && CONFIG_STM32F429I_DISCO_RAMMTD */

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_initialize() starts a thread
   * will monitor for USB connection and disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start(0, NULL);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

  return OK;
}
