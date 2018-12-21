/************************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ************************************************************************************/
/************************************************************************************
 * arch/arm/src/stm32l4x3/src/stm32l4_boot.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Librae <librae8226@gmail.com>
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

#include <tinyara/config.h>

#include <debug.h>

#include <tinyara/arch.h>
#include <tinyara/board.h>
#include <tinyara/spi/spi.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32l4xr.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
#ifdef CONFIG_BRCM_WLAN
extern void wlan_gpio_config(void);
extern void stm32l4_brcm_wlan_driver_initialize(void);
#endif

/************************************************************************************
 * Name: stm32l4_board_initialize
 *
 * Description:
 *   All STM32L4 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32l4_board_initialize(void)
{
	/* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
	board_autoled_initialize();
#endif

	/* Configure SPI chip selects if 1) SP2 is not disabled, and 2) the weak function
	 * stm32l4_spiinitialize() has been brought into the link.
	 */

#if defined(CONFIG_STM32L4_SPI1) || defined(CONFIG_STM32L4_SPI2) || defined(CONFIG_STM32L4_SPI3)
	stm32l4_spiinitialize();
#endif

	/* Initialize USB is 1) USBDEV is selected, 2) the USB controller is not
	 * disabled, and 3) the weak function stm32l4_usbinitialize() has been brought
	 * into the build.
	 */

#if defined(CONFIG_USBDEV) && defined(CONFIG_STM32L4_USB)
	stm32l4_usbinitialize();
#endif
}

/***************************************************************************
 * Name: board_gpio_initialize
 *
 * Description:
 *  Expose board dependent GPIOs
 ****************************************************************************/
static void board_gpio_initialize(void)
{
#ifdef CONFIG_GPIO
	int i;
	struct gpio_lowerhalf_s *lower;

	struct {
		uint8_t minor;
		uint16_t pincfg;
	} pins[] = {
		{
			36, GPIO_XGPIO4
		},		/* STM32L4_PORTC, PIN4, 36 */
		{
			37, GPIO_XGPIO5
		},		/* STM32L4_PORTC, PIN5, 37 */
		{
			15, GPIO_LD2
		},		/* STM32L4_GPIOA, PIN15, 15 */
	};

	for (i = 0; i < sizeof(pins) / sizeof(*pins); i++) {
		lower = stm32l4_gpio_lowerhalf(pins[i].pincfg);
		gpio_register(pins[i].minor, lower);
	}

	/* Initialize GPIO */
	stm32l4_gpioinit();

	stm32l4_configgpio(GPIO_XGPIO4);
	stm32l4_configgpio(GPIO_XGPIO5);

	/* Load switch off */
	stm32l4_gpiowrite(GPIO_XGPIO4, false);
	stm32l4_gpiowrite(GPIO_XGPIO5, false);

	/* User LED */
	stm32l4_configgpio(GPIO_LD2);	
	stm32l4_gpiowrite(GPIO_LD2, false);

#endif							/* CONFIG_GPIO */
}

/****************************************************************************
 * Name: board_i2c_initialize
 *
 * Description:
 *  Expose board dependent I2Cs
 ****************************************************************************/
static void board_i2c_initialize(void)
{
#if defined(CONFIG_I2C) && defined(CONFIG_STM32L4_I2C)
	stm32l4_i2c_register(0);
	stm32l4_i2c_register(1);
#endif
}

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_intiialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{
	/* Initialize application */
	board_app_initialize();

	board_gpio_initialize();
	board_i2c_initialize();

#if defined(CONFIG_CORE_SDIO) && !defined(CONFIG_BRCM_WLAN)
	struct stm32_dev_s *p;
	/* Initialize the SDIO block driver */
	p = sdio_initialize(0);

	if (p == NULL) {
		lldbg("Failed to initialize SDIO driver\n");
		return;
	}
	int ret = OK;
	ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, g_sdio);
	if (ret != OK) {
		syslog(LOG_ERR, "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
		return ret;
	}
#endif

}
#endif
