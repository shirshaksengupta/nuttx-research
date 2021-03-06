/************************************************************************************
 * configs/nucleo-144/src/nucleo-144.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Mark Olsson <post@markolsson.se>
 *            David Sidrane <david_s5@nscdg.com>
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

#ifndef __CONFIGS_NUCLEO_144_SRC_NUCLEO_144_H
#define __CONFIGS_NUCLEO_144_SRC_NUCLEO_144_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Nucleo-144 GPIO Pin Definitions **************************************************/
/* LED
 *
 * The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED, LD2 a
 * Blue LED and LD3 a Red LED, that can be controlled by software. The following definitions assume
 * the default Solder Bridges are installed.
 */

#define GPIO_LD1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                        GPIO_PORTB | GPIO_PIN0)
#define GPIO_LD2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                        GPIO_PORTB | GPIO_PIN7)
#define GPIO_LD3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                        GPIO_PORTB | GPIO_PIN14)

#define GPIO_LED_GREEN GPIO_LD1
#define GPIO_LED_BLUE  GPIO_LD2
#define GPIO_LED_RED   GPIO_LD3

#define LED_DRIVER_PATH "/dev/userleds"

/* BUTTONS
 *
 * The Blue pushbutton B1, labeled "User", is connected to GPIO PC13.  A high value
 * will be sensed when the button is depressed.
 * Note:
 *    1) That the EXTI is included in the definition to enable an interrupt on this
 *       IO.
 *    2) The following definitions assume the default Solder Bridges are installed.
 */

#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)

/* SPI ***************************************************************************
 *
 */
#define GPIO_SPI_CS    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_SET)

#define GPIO_SPI1_CS0   (GPIO_SPI_CS | GPIO_PORTA | GPIO_PIN15)
#define GPIO_SPI1_CS1   (GPIO_SPI_CS | GPIO_PORTC | GPIO_PIN13)
#define GPIO_SPI1_CS2   (GPIO_SPI_CS | GPIO_PORTC | GPIO_PIN13)
#define GPIO_SPI1_CS3   (GPIO_SPI_CS | GPIO_PORTC | GPIO_PIN14)
#define GPIO_SPI2_CS0   (GPIO_SPI_CS | GPIO_PORTD | GPIO_PIN7)
#define GPIO_SPI2_CS1   (GPIO_SPI_CS | GPIO_PORTC | GPIO_PIN15)
#define GPIO_SPI2_CS2   (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN2)
#define GPIO_SPI2_CS3   (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN3)
#define GPIO_SPI3_CS0   (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN4)
#define GPIO_SPI3_CS1   (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN5)
#define GPIO_SPI3_CS2   (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN6)
#define GPIO_SPI3_CS3   (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN7)

/* Logical SPI Chip Selects used to index */

#define NUCLEO_SPI_BUS1_CS0  0
#define NUCLEO_SPI_BUS1_CS1  1
#define NUCLEO_SPI_BUS1_CS2  2
#define NUCLEO_SPI_BUS1_CS3  3
#define NUCLEO_SPI_BUS2_CS0  4
#define NUCLEO_SPI_BUS2_CS1  5
#define NUCLEO_SPI_BUS2_CS2  6
#define NUCLEO_SPI_BUS2_CS3  7
#define NUCLEO_SPI_BUS3_CS0  8
#define NUCLEO_SPI_BUS3_CS1  9
#define NUCLEO_SPI_BUS3_CS2  10
#define NUCLEO_SPI_BUS3_CS3  11

#if defined(CONFIG_STM32_SDIO)
#define GPIO_SDIO_NCD (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTB | GPIO_PIN15)
#endif

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-144 board.
 *
 ************************************************************************************/

#if defined(CONFIG_SPI)
void stm32_spidev_initialize(void);
#endif

/************************************************************************************
 * Name: stm32_spidev_bus_test
 *
 * Description:
 *   Called to create the defined SPI buses and test them by initializing them
 *   and sending the NUCLEO_SPI_TEST (no chip select).
 *
 ************************************************************************************/

#if defined(NUCLEO_SPI_TEST)
int stm32_spidev_bus_test(void);
#endif

/************************************************************************************
 * Name: stm32_dma_alloc_init
 *
 * Description:
 *   Called to create a FAT DMA allocator
 *
 * Returned Value:
 *   0 on success or -ENOMEM
 *
 ************************************************************************************/

void stm32_dma_alloc_init(void);

#if defined (CONFIG_FAT_DMAMEMORY)
int stm32_dma_alloc_init(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_NUCLEO_144_SRC_NUCLEO_144_H */
