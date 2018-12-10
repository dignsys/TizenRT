/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
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
#include <tinyara/config.h>

#include <stdio.h>
#include <pthread.h>
#include <net/if.h>
#include <net/lwip/opt.h>
#include <net/lwip/netif.h>
#include <net/lwip/tcpip.h>

#include <tinyara/configdata.h>

#include "up_internal.h"
#include "up_internal.h"
#include "stm32l4.h"
#include "stm32l4xr.h"

#if defined(CONFIG_NET)

/****************************************************************************
 * Public Functions : used on WLAN driver
 ****************************************************************************/
int cyw43438_set_country(void *priv, const char *country_code);
int cyw43438_get_country(void *priv, char *country_code);
int cyw43438_set_tx_power(void *priv, int dbm);
int cyw43438_get_tx_power(void *priv);

/* define WLAN GPIO */
#define GPIO_WLAN0_RESET	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN3)
#define GPIO_WLAN0_REG_ON	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN0)
#define GPIO_WLAN0_HOST_WAKE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN1)

/****************************************************************************
 * Function: up_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in up_initialize.c.
 *   If BCM2835_NETHCONTROLLERS greater than one, then board specific logic
 *   will have to supply a version of up_netinitialize() that calls
 *   bcm2835_ethinitialize() with the appropriate interface number.
 *
 ****************************************************************************/
void up_netinitialize(void)
{
}

static void *__bring_up_wpa_supplicant(void *data)
{
	extern int wpa_supplicant_main(int argc, char * argv[]);
	static char *argv[5];

	argv[0] = "-qqq";
	argv[1] = "-t";
	argv[2] = "-i wl1";
	argv[3] = "-Cudp";
	argv[4] = NULL;

	wpa_supplicant_main(4, argv);

	return NULL;
}

static pthread_t pthread_handle;
int bring_up_wpa_supplicant(void)
{
	pthread_attr_t attr;
	struct sched_param sched_param;
	int err;
	long stack_size = CONFIG_WPA_SUPPLICANT_STACKSIZE;

	err = pthread_attr_init(&attr);
	if (err != 0) {
		return -1;
	}

	err = pthread_attr_setstacksize(&attr, (long)stack_size);
	if (err != 0) {
		goto error_exit;
	}

	sched_param.sched_priority = 100;
	err = pthread_attr_setschedparam(&attr, &sched_param);
	if (err != 0) {
		goto error_exit;
	}

	err = pthread_attr_setschedpolicy(&attr, SCHED_RR);
	if (err != 0) {
		goto error_exit;
	}

	err = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (err != 0) {
		goto error_exit;
	}

	err = pthread_create(&pthread_handle, &attr, __bring_up_wpa_supplicant, NULL);
	if (err != 0) {
		goto error_exit;
	}

	pthread_setname_np(pthread_handle, "WPA_SUPPLICANT");

	return 0;

error_exit:
	pthread_attr_destroy(&attr);
	return -1;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
int up_wlan_get_country(char *alpha2)
{
	return cyw43438_get_country(NULL, alpha2);
}

int up_wlan_set_country(char *alpha2)
{
	return cyw43438_set_country(NULL, alpha2);
}

int up_wlan_get_txpower(void)
{
	return (int)cyw43438_get_tx_power(NULL);
}

int up_wlan_set_txpower(uint8_t *dbm)
{
	return cyw43438_set_tx_power(NULL, *dbm);
}

void sdio_gpio_config(int port, int pin)
{
	stm32l4_configgpio(GPIO_ALT | GPIO_AF12 | GPIO_PULLUP | (port << GPIO_PORT_SHIFT) | (pin << GPIO_PIN_SHIFT));
}

void wlan_gpio_config(void)
{
	bool readval;

	/* set WLAN0 RESEY signal */
	stm32l4_configgpio(GPIO_WLAN0_RESET);
	stm32l4_gpiowrite(GPIO_WLAN0_RESET, 0);
	up_mdelay(10);
	stm32l4_gpiowrite(GPIO_WLAN0_RESET, 1);

	/* set WLAN0 REG_ON signal */
	stm32l4_configgpio(GPIO_WLAN0_REG_ON);	//PD0 = WL_REG_ON
	stm32l4_gpiowrite(GPIO_WLAN0_REG_ON, 0);
	up_mdelay(10);
	stm32l4_gpiowrite(GPIO_WLAN0_REG_ON, 1);
	readval = stm32l4_gpioread(GPIO_WLAN0_REG_ON);

	/* set WLAN0 HOST_WAKE signal */
	stm32l4_configgpio(GPIO_WLAN0_HOST_WAKE);
	stm32l4_gpiowrite(GPIO_WLAN0_HOST_WAKE, 1);
}

#endif	/* CONFIG_NET */
