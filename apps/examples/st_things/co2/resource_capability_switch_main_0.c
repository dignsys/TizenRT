/*
 * resource_capability_switch_main_0.c
 *
 *  Created on: Jun 21, 2018
 *      Author: build
 */

#include <stdio.h>
#include <stdbool.h>
#include "st_things/st_things.h"

#define VALUE_STR_LEN_MAX 32

static const char* PROP_POWER = "power";
static const char* VALUE_SWITCH_ON = "on";
static const char* VALUE_SWITCH_OFF = "off";
static char g_switch[VALUE_STR_LEN_MAX] = "off";

bool handle_get_request_on_resource_capability_switch_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
#ifdef DEBUG
    printf("Received a GET request on %s\n", req_msg->resource_uri);
#endif

    if (req_msg->has_property_key(req_msg, PROP_POWER)) {
        // TODO: Write your implementation in this section.
		resp_rep->set_str_value(resp_rep, PROP_POWER, g_switch);
    }
	else
    	return false;  // FIXME: Modify this line with the appropriate return value.
	return true;
}

bool handle_set_request_on_resource_capability_switch_main_0(st_things_set_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
#ifdef DEBUG
    printf("Received a SET request on %s\n", req_msg->resource_uri);
#endif

    // TODO: Write your implementation in this section.
	char *str_value = NULL;
	req_msg->rep->get_str_value(req_msg->rep, PROP_POWER, &str_value);

	/* check validation */
	if ((0 != strncmp(str_value, VALUE_SWITCH_ON, strlen(VALUE_SWITCH_ON)))
		&& (0 != strncmp(str_value, VALUE_SWITCH_OFF, strlen(VALUE_SWITCH_OFF)))) {
		printf("Not supported value!!");
		free(str_value);
		return false;
	}
	if (0 != strncmp(str_value, g_switch, strlen(g_switch))) {
		strncpy(g_switch, str_value, VALUE_STR_LEN_MAX);
		if (0 == strncmp(g_switch, VALUE_SWITCH_ON, strlen(VALUE_SWITCH_ON))) {
//			resource_write_led(outputPin, 0);
		}
		else  {
//			resource_write_led(outputPin, 1);
		}
	}
	resp_rep->set_str_value(resp_rep, PROP_POWER, g_switch);

	st_things_notify_observers(req_msg->resource_uri);

	free(str_value);

    return true;
}
