/*
 * resource_capability_switch_main_0.c
 *
 *  Created on: Jun 26, 2018
 *      Author: build
 */

#include <stdio.h>
#include <stdbool.h>
#include "st_things/st_things.h"

//#define	DEBUG_SWITCH

static const char* PROP_POWER = "power";
static char saved_url[1024] = {0,};

#include <fcntl.h>
#include <tinyara/gpio.h>

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

#define LED_POWER_PORT 50//=XGPIO21		(POWER GPIO) //49//BLUE LED
#define LED_POWER_PORT2 48//=XGPIO19	(POWER GPIO)

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
	gpio_write(LED_POWER_PORT, g_power);
	usleep(300 * 1000);
	gpio_write(LED_POWER_PORT2, g_power);

	return g_power;
}

void switch_power_gpio_init(void)
{
	g_power = gpio_read(LED_POWER_PORT);
//	g_power = gpio_read(LED_POWER_PORT2);
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
