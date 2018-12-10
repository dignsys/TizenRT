/****************************************************************************
 *
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <tinyara/config.h>
#include <apps/shell/tash.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/ether.h>

#include <tinyara/clock.h>
#include <tinyara/net/net.h>
#include <tinyara/net/ip.h>

/****************************************************************************
 * Global variable
 ****************************************************************************/
extern uint8_t g_syslog_mask;
static ip_addr_t client_ip;

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * External Function Protoype
 ****************************************************************************/
extern uint8_t doPing(ip_addr_t ip, int pingcount);
extern void my_nic_display_state(void);
extern int my_ifup(void);
extern int my_ifdown(void);
extern int my_ifconfig(void);
extern void gy30_test(void);

int log_level_change(int argc, char *argv[]);

static tash_cmdlist_t peripheral_cmds[] = {
	{"loglevel", log_level_change, TASH_EXECMD_ASYNC},
	{NULL, NULL, 0}
};


int log_level_change(int argc, char *argv[])
{
	const unsigned buffer_size = 200;
	int result = -1;

	if (argc > 1) {
		g_syslog_mask = LOG_MASK(atoi(argv[1]));
	}

	printf("log level mask = 0x%x \n", g_syslog_mask);
	return result;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int stm32l4_demo_main(int argc, char *argv[])
#endif
{
	int i = 0;
	FAR char *gwip = "192.168.1.1";
	int pingcount = 5;

	printf("\nApplication(%s) is launched@0x%08p\n", __func__, stm32l4_demo_main);

#ifdef CONFIG_TASH
	/* add tash command */
	tash_cmdlist_install(peripheral_cmds);
#endif

	printf("Start STM32L4 Demo \n");

#if 0
	sleep(1);
	printf("Testing GY30 Light sensor(I2C1_SCL_2[PB8], I2C1_SDA_1[PB7])\n");
	gy30_test();
#endif 

#if 0
	sleep(1);
	printf("Testing ADC input(ADC1_IN1[PC0])\n");
	adc_test();
#endif 

#ifdef CONFIG_ENC28J60
	printf("Testing Ethernet ENC28J60)\n");
	my_nic_display_state();
	my_ifup();
	my_ifconfig();

        client_ip.addr = inet_addr(gwip);
        printf("Ping gateway ip address %s ...\n", ipaddr_ntoa(&client_ip));
        if (doPing(client_ip, pingcount) == 0) {
                printf("Successfully pinged\n");
        } else {
                printf("Failed to ping\n");
        }
	my_ifdown();
#endif

	while (1) {
		sleep(30);
		printf("alive ! count : %d\n", i);
		i++;
	}

	return 0;
}
