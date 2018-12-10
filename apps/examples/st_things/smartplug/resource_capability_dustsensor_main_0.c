/*
 * resource_capability_dustsensor_main_0.c
 *
 *  Created on: Jun 28, 2018
 *      Author: build
 */

#include <stdio.h>
#include <stdbool.h>
#include "st_things/st_things.h"

//#define	DUST_DEBUG

static const char* PROP_DUSTLEVEL = "dustLevel";
static const char* PROP_FINEDUSTLEVEL = "fineDustLevel";
extern int  g_energy_hour;
extern int  g_energy_today;
extern int  g_energy_month;

static char saved_url[1024] = {0,};

void update_energy_ext_value(void)
{
	if( saved_url[0] )
		st_things_notify_observers(saved_url);
}

bool handle_get_request_on_resource_capability_dustsensor_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
#ifdef	DUST_DEBUG
    printf("Received a GET request on %s\n", req_msg->resource_uri);
#endif

	if(saved_url[0]==0)
		strncpy(saved_url, req_msg->resource_uri, 1024);

    if (req_msg->has_property_key(req_msg, PROP_DUSTLEVEL)) {
		resp_rep->set_int_value(resp_rep, PROP_DUSTLEVEL, g_energy_today+g_energy_hour);
#ifdef	DUST_DEBUG
		printf("GET g_energy_today= %d\n", g_energy_today);
#endif
    }
    if (req_msg->has_property_key(req_msg, PROP_FINEDUSTLEVEL)) {
		resp_rep->set_int_value(resp_rep, PROP_FINEDUSTLEVEL, g_energy_month+g_energy_today+g_energy_hour);
#ifdef	DUST_DEBUG
		printf("GET g_energy_month= %d\n", g_energy_month);
#endif
    }
    return true;
}
