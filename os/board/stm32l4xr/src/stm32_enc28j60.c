/****************************************************************************
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
 ****************************************************************************/
/****************************************************************************
 * configs/fire-stm32v2/src/stm32_enc28j60.c
 *
 *   Copyright (C) 2012, 2017 Gregory Nutt. All rights reserved.
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

/* ENC28J60
 *
 * --- ------ -------------- -----------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -----------------------------------------------------
 *
 * 17  PA11    PA11-SPI1-NSS   10Mbit ENC28J60
 * 11  PB13    PB13-SPI2-SCK   10Mbit ENC28J60
 * 13  PB14    PB14-SPI2-MISO  10Mbit ENC28J60
 * 15  PB15    PB15-SPI2-MOSI  10Mbit ENC28J60
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>
#include <stdio.h>
#include <debug.h>
#include <syslog.h>
#include <errno.h>

#include <tinyara/spi/spi.h>
#include <tinyara/net/enc28j60.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32l4_spi.h"

#include "stm32l4xr.h"

#ifdef CONFIG_ENC28J60

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* ENC28J60
 *
 * --- ------ -------------- -----------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -----------------------------------------------------
 * 17  PA11    PA11-SPI1-NSS   10Mbit ENC28J60
 * 11  PB13    PB13-SPI2-SCK   10Mbit ENC28J60
 * 13  PB14    PB14-SPI2-MISO  10Mbit ENC28J60
 * 15  PB15    PB15-SPI2-MOSI  10Mbit ENC28J60
 * 19  PA8     PA8-ARD-D9      10Mbit EN28J60 Reset
 * 21  PB6     PB6-ARD-D8      10Mbit EN28J60 Interrupt
 */

/* ENC28J60 is on SPI2 */

#ifndef CONFIG_STM32L4_SPI2
#error "Need CONFIG_STM32L4_SPI2 in the configuration"
#endif

/* SPI Assumptions **********************************************************/

#define ENC28J60_SPI_PORTNO 2	/* On SPI2 */
#define ENC28J60_DEVNO      0	/* Only one ENC28J60 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lower_s {
	const struct enc_lower_s lower;	/* Low-level MCU interface */
	xcpt_t handler;				/* ENC28J60 interrupt handler */
	FAR void *arg;				/* Argument that accompanies the interrupt */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int up_attach(FAR const struct enc_lower_s *lower, xcpt_t handler, FAR void *arg);
static void up_enable(FAR const struct enc_lower_s *lower);
static void up_disable(FAR const struct enc_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The ENC28J60 normal provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanixm for controlling
 * the ENC28J60 GPIO interrupt.
 */

static struct stm32_lower_s g_enclower = {
	.lower = {
		.attach = up_attach,
		.enable = up_enable,
		.disable = up_disable
	},
	.handler = NULL,
	.arg = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct enc_lower_s methods
 ****************************************************************************/

static int up_attach(FAR const struct enc_lower_s *lower, xcpt_t handler, FAR void *arg)
{
	FAR struct stm32_lower_s *priv = (FAR struct stm32_lower_s *)lower;

	/* Just save the handler for use when the interrupt is enabled */

	priv->handler = handler;
	priv->arg = arg;
	return OK;
}

static void up_enable(FAR const struct enc_lower_s *lower)
{
	FAR struct stm32_lower_s *priv = (FAR struct stm32_lower_s *)lower;

	DEBUGASSERT(priv->handler);
	(void)stm32l4_gpiosetevent(GPIO_ENC28J60_INTR, false, true, true, priv->handler, priv->arg);
}

/* REVISIT:  Since the interrupt is completely torn down, not just disabled,
 * in interrupt requests that occurs while the interrupt is disabled will be
 * lost.
 */

static void up_disable(FAR const struct enc_lower_s *lower)
{
	(void)stm32l4_gpiosetevent(GPIO_ENC28J60_INTR, false, true, true, NULL, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_netinitialize
 ****************************************************************************/

void up_netinitialize(void)
{
	FAR struct spi_dev_s *spi;
	int ret;

	/* Assumptions:
	 * 1) ENC28J60 pins were configured in up_spi.c early in the boot-up phase.
	 * 2) Clocking for the SPI1 peripheral was also provided earlier in boot-up.
	 */

	spi = stm32l4_spibus_initialize(ENC28J60_SPI_PORTNO);
	if (!spi) {
		syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n", ENC28J60_SPI_PORTNO);
		return;
	}

	/* Take ENC28J60 out of reset (active low) */
	stm32l4_gpiowrite(GPIO_ENC28J60_RESET, true);

	/* Bind the SPI port to the ENC28J60 driver */
	ret = enc_initialize(spi, &g_enclower.lower, ENC28J60_DEVNO);
	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: Failed to bind SPI port %d ENC28J60 device %d: %d\n", ENC28J60_SPI_PORTNO, ENC28J60_DEVNO, ret);
		return;
	}

	syslog(LOG_INFO, "Bound SPI port %d to ENC28J60 device %d\n", ENC28J60_SPI_PORTNO, ENC28J60_DEVNO);
}

#endif							/* CONFIG_ENC28J60 */
