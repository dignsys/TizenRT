/*
 * resource_capability_switch_main_0.c
 *
 *  Created on: Jun 26, 2018
 *      Author: build
 */

#include <stdio.h>
#include <stdbool.h>
#include "st_things/st_things.h"

#include <tinyara/gpio.h>
#include <iotbus/iotbus_gpio.h>
#include <iotbus/iotbus_error.h>

//#define	DEBUG_SWITCH

static const char* PROP_POWER = "power";
static char saved_url[1024] = {0,};

#include <fcntl.h>
#include <tinyara/gpio.h>

#ifdef CONFIG_ARCH_CHIP_STM32L4
#define AC_LOAD_SW1	36
#define AC_LOAD_SW2	37

static iotbus_gpio_context_h ac_sw1;
static iotbus_gpio_context_h ac_sw2;

static void gpio_ac_power_stop(void)
{
	iotbus_gpio_close(ac_sw1);
	iotbus_gpio_close(ac_sw2);
}

static void gpio_ac_power_on(void)
{
	iotbus_gpio_write(ac_sw1, 1);
	iotbus_gpio_write(ac_sw2, 1);
}

static void gpio_ac_power_off(void)
{
	iotbus_gpio_write(ac_sw1, 0);
	iotbus_gpio_write(ac_sw2, 0);
}

static int gpio_ac_power_start(void)
{
	iotapi_initialize();
	ac_sw1 = iotbus_gpio_open(AC_LOAD_SW1);
	iotbus_gpio_set_direction(ac_sw1, GPIO_DIRECTION_OUT);
	ac_sw2 = iotbus_gpio_open(AC_LOAD_SW2);
	iotbus_gpio_set_direction(ac_sw2, GPIO_DIRECTION_OUT);
	up_mdelay(10);
//	gpio_ac_power_off();
	return iotbus_gpio_read(ac_sw1);
}
#else
#define LED_POWER_PORT 50//=XGPIO21		(POWER GPIO) //49//BLUE LED
#define LED_POWER_PORT2 48//=XGPIO19	(POWER GPIO)
static int gpio_read(int port)
{
	char buf[4];
	char devpath[16];
	snprintf(devpath, 16, "/dev/gpio%d", port);
	int fd = open(devpath, O_RDWR);
	if (fd < 0) {
		printf("fd open fail\n");
		return -1;
	}

	ioctl(fd, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_IN);
	if (read(fd, buf, sizeof(buf)) < 0) {
		printf("read error\n");
		close(fd);
		return -1;
	}
	close(fd);

	return buf[0] == '1';
}

static void gpio_write(int port, int value)
{
	char buf[4];
	char devpath[16];
	snprintf(devpath, 16, "/dev/gpio%d", port);
	int fd = open(devpath, O_RDWR);
	if (fd < 0) {
		printf("fd open fail\n");
		return;
	}

	ioctl(fd, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);
	if (write(fd, buf, snprintf(buf, sizeof(buf), "%d", !!value)) < 0) {
		printf("write error\n");
	}
	close(fd);
}
#endif

static int g_power = 0;
char *power_status[] = {"off", "on"};

const char* get_led_power(void)
{
	return power_status[g_power];
}

int set_led_power(char *power)
{
	if (strncmp(power, "off" , strlen("off")) == 0) {
		g_power = 0;
	} else if (strncmp(power, "on", strlen("on")) == 0) {
		g_power = 1;
	} else {
		printf("input Error");
		return -1;
	}
	#ifdef CONFIG_ARCH_CHIP_STM32L4
	if(g_power)
		gpio_ac_power_on();
	else
		gpio_ac_power_off();
	#else
	gpio_write(LED_POWER_PORT, g_power);
	usleep(300 * 1000);
	gpio_write(LED_POWER_PORT2, g_power);
	#endif
	return g_power;
}

void switch_power_gpio_init(void)
{
#ifdef CONFIG_ARCH_CHIP_STM32L4
	g_power = gpio_ac_power_start();
#else
	g_power = gpio_read(LED_POWER_PORT);
//	g_power = gpio_read(LED_POWER_PORT2);
#endif
}

void power_onoff(int onoff)
{
	if( saved_url[0] ) {
		set_led_power(onoff ? "on" : "off");
		st_things_notify_observers(saved_url);
	}
}

bool handle_get_request_on_resource_capability_switch_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
#ifdef	DEBUG_SWITCH
    printf("Received a GET request on %s\n", req_msg->resource_uri);
#endif
	if(saved_url[0]==0)
		strncpy(saved_url, req_msg->resource_uri, 1024);

    if (req_msg->has_property_key(req_msg, PROP_POWER)) {
    	resp_rep->set_str_value(resp_rep, PROP_POWER, get_led_power());//"on");
    }
    return true;  // FIXME: Modify this line with the appropriate return value.
}

bool handle_set_request_on_resource_capability_switch_main_0(st_things_set_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
	char *power;
#ifdef	DEBUG_SWITCH
    printf("Received a SET request on %s\n", req_msg->resource_uri);
#endif

    if( resp_rep->get_str_value(req_msg->rep, PROP_POWER, &power) ) {
#ifdef	DEBUG_SWITCH
		printf("%s\n", power);
#endif
		set_led_power(power);
		resp_rep->set_str_value(resp_rep, PROP_POWER, get_led_power());
		st_things_notify_observers(req_msg->resource_uri);
	}
    return true;  // FIXME: Modify this line with the appropriate return value.
}
