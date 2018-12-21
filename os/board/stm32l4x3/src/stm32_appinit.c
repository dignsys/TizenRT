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
 * arch/arm/src/stm32l4x3/src/stm32l4_appinit.c
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
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <sys/mount.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>

#include <tinyara/arch.h>
#include <tinyara/board.h>
#include <tinyara/sdio.h>
#include <tinyara/mmcsd.h>

#include <stm32l4.h>
#include <stm32l4_uart.h>

#include <arch/board/board.h>
#include <arch/board/userled.h>

#include "stm32l4x3.h"

#ifdef HAVE_RTC_DRIVER
#include <tinyara/rtc.h>
#include "stm32l4_rtc.h"
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
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the
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

int board_app_initialize(void)
{
#ifdef HAVE_RTC_DRIVER
	FAR struct rtc_lowerhalf_s *rtclower;
#endif
#ifdef CONFIG_SENSORS_QENCODER
	int index;
	char buf[9];
#endif
	int ret;

	(void)ret;

#ifdef HAVE_PROC
	/* Mount the proc filesystem */

	syslog(LOG_INFO, "Mounting procfs to /proc\n");

	ret = mount(NULL, PROCFS_MOUNTPOUNT, "procfs", 0, NULL);
	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: Failed to mount the PROC filesystem: %d (%d)\n", ret, errno);
		return ret;
	}
#endif

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
	/* Register the LED driver */

	ret = userled_lower_initialize(LED_DRIVER_PATH);
	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
	}
#endif

#ifdef HAVE_RTC_DRIVER
	/* Instantiate the STM32L4 lower-half RTC driver */

	rtclower = stm32l4_rtc_lowerhalf();
	if (!rtclower) {
		syslog(LOG_ERR, "ERROR: Failed to instantiate the RTC lower-half driver\n");
		return -ENOMEM;
	} else {
		/* Bind the lower half driver and register the combined RTC driver
		 * as /dev/rtc0
		 */

		ret = rtc_initialize(0, rtclower);
		if (ret < 0) {
			syslog(LOG_ERR, "ERROR: Failed to bind/register the RTC driver: %d\n", ret);
			return ret;
		}
	}
#endif

#ifdef CONFIG_PWM
	/* Initialize PWM and register the PWM device. */

	ret = stm32l4_pwm_setup();
	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: stm32l4_pwm_setup() failed: %d\n", ret);
	}
#endif

#ifdef CONFIG_ADC
	/* Initialize ADC and register the ADC driver. */

	ret = stm32l4_adc_setup();
	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: stm32l4_adc_setup failed: %d\n", ret);
	}
#endif

#ifdef CONFIG_TIMER
	/* Initialize and register the timer driver */

	ret = board_timer_driver_initialize("/dev/timer0", 2);
	if (ret != OK) {
		syslog(LOG_ERR, "ERROR: Failed to register the timer driver: %d\n", ret);
		return ret;
	}
#endif

#ifdef CONFIG_SENSORS_QENCODER

	/* Initialize and register the qencoder driver */

	index = 0;

#ifdef CONFIG_STM32L4_TIM1_QE
	sprintf(buf, "/dev/qe%d", index++);
	ret = stm32l4_qencoder_initialize(buf, 1);
	if (ret != OK) {
		syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n", ret);
		return ret;
	}
#endif

#ifdef CONFIG_STM32L4_TIM2_QE
	sprintf(buf, "/dev/qe%d", index++);
	ret = stm32l4_qencoder_initialize(buf, 2);
	if (ret != OK) {
		syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n", ret);
		return ret;
	}
#endif

#ifdef CONFIG_STM32L4_TIM3_QE
	sprintf(buf, "/dev/qe%d", index++);
	ret = stm32l4_qencoder_initialize(buf, 3);
	if (ret != OK) {
		syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n", ret);
		return ret;
	}
#endif

#ifdef CONFIG_STM32L4_TIM4_QE
	sprintf(buf, "/dev/qe%d", index++);
	ret = stm32l4_qencoder_initialize(buf, 4);
	if (ret != OK) {
		syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n", ret);
		return ret;
	}
#endif

#ifdef CONFIG_STM32L4_TIM5_QE
	sprintf(buf, "/dev/qe%d", index++);
	ret = stm32l4_qencoder_initialize(buf, 5);
	if (ret != OK) {
		syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n", ret);
		return ret;
	}
#endif

#ifdef CONFIG_STM32L4_TIM8_QE
	sprintf(buf, "/dev/qe%d", index++);
	ret = stm32l4_qencoder_initialize(buf, 8);
	if (ret != OK) {
		syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n", ret);
		return ret;
	}
#endif

#endif

#ifdef CONFIG_STM32L4X3_BUTTON
	board_button_initialize();
#endif

	UNUSED(ret);
	return OK;
}

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
	return -ENOTTY;
}
#endif

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
	if (uniqueid == 0) {
		return -EINVAL;
	}

	stm32l4_get_uniqueid(uniqueid);
	return OK;
}
#endif
