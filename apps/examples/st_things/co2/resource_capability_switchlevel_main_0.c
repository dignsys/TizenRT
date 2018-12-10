/*
 * resource_capability_switchlevel_main_0.c
 *
 *  Created on: Jun 21, 2018
 *      Author: build
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "st_things/st_things.h"


static const char* PROP_DIMMINGSETTING = "dimmingSetting";
static int64_t g_dimm_value = 0;

bool handle_get_request_on_resource_capability_switchlevel_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
#ifdef DEBUG
    printf("Received a GET request on %s\n", req_msg->resource_uri);
#endif

    if (req_msg->has_property_key(req_msg, PROP_DIMMINGSETTING)) {
        // TODO: Write your implementation in this section.
		resp_rep->set_int_value(resp_rep, PROP_DIMMINGSETTING, g_dimm_value);
		printf("Received a GET request on value: %lld\n", g_dimm_value);
		g_dimm_value += 10;
		if (g_dimm_value >= 100) g_dimm_value = 0;
    }
    return true;  // FIXME: Modify this line with the appropriate return value.
}

bool handle_set_request_on_resource_capability_switchlevel_main_0(st_things_set_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
#ifdef DEBUG
    printf("Received a SET request on %s\n", req_msg->resource_uri);
#endif

    // TODO: Write your implementation in this section.
	int64_t ivalue;
	if (req_msg->rep->get_int_value(req_msg->rep, PROP_DIMMINGSETTING, &ivalue)) {
		g_dimm_value = ivalue;
		resp_rep->set_int_value(resp_rep, PROP_DIMMINGSETTING, g_dimm_value);
		update_co2_value();
		printf("Received a SET request on value: %lld\n", g_dimm_value);
	}
	st_things_notify_observers(req_msg->resource_uri);

    return true;  // FIXME: Modify this line with the appropriate return value.
}
