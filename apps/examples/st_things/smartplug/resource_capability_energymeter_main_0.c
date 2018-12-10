/*
 * resource_capability_energymeter_main_0.c
 *
 *  Created on: Jun 26, 2018
 *      Author: build
 */

#include <stdio.h>
#include <stdbool.h>
#include "st_things/st_things.h"

//#define	DEBUG_ENERGY

static const char* PROP_UNIT = "unit";
static const char* PROP_ENERGY = "energy";
//static char *g_energy_unit			  = "kwh";
extern int  g_energy_hour;

static char saved_url[1024] = {0,};

void update_energy_value(void)
{
//	printf("update ENERGY HOUR: %d\n", g_energy_hour);
	if( saved_url[0] )
		st_things_notify_observers(saved_url);
}

bool handle_get_request_on_resource_capability_energymeter_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
#ifdef	DEBUG_ENERGY
    printf("Received a GET request on %s\n", req_msg->resource_uri);
#endif

	if(saved_url[0]==0)
		strncpy(saved_url, req_msg->resource_uri, 1024);

    if (req_msg->has_property_key(req_msg, PROP_ENERGY)) {
		resp_rep->set_int_value(resp_rep, PROP_ENERGY, g_energy_hour);
#ifdef	DEBUG_ENERGY
		printf("GET g_energy_hour= %d\n", g_energy_hour);
#endif
    }
    else if (req_msg->has_property_key(req_msg, PROP_UNIT)) {
#ifdef	DEBUG_ENERGY
//		printf("PROP_UNIT=%s\n", g_energy_unit);
#endif
//		resp_rep->set_str_value(resp_rep, PROP_UNIT, g_energy_unit); // can be one of ["mh", "kwh", "mwh"]
    }
	else
		return false;
    return true;
}
