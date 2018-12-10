/*
 * things.c
 *
 *  Created on: Sep 11, 2018
 *      Author: build
 */

#include <tinyara/config.h>

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <st_things/st_things.h>

static const char* RES_CAPABILITY_SWITCH_MAIN_0 = "/capability/switch/main/0";
static const char* RES_CAPABILITY_COLORCONTROL_MAIN_0 = "/capability/colorControl/main/0";
static const char* RES_CAPABILITY_DUSTSENSOR_MAIN_0 = "/capability/dustSensor/main/0";
static const char* RES_CAPABILITY_ENERGYMETER_MAIN_0 = "/capability/energyMeter/main/0";
static const char* RES_CAPABILITY_POWERMETER_MAIN_0 = "/capability/powerMeter/main/0";
static const char* RES_CAPABILITY_DOORCONTROL_MAIN_0 = "/capability/doorControl/main/0";

/* OCF callback functions */
extern bool handle_reset_request(void);
extern void handle_reset_result(bool result);
extern bool handle_ownership_transfer_request(void);
extern void handle_things_status_change(st_things_status_e things_status);

/* main loop */
extern void handle_main_loop(void);

/* get and set request handlers */
extern bool handle_get_request_on_resource_capability_switch_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep);
extern bool handle_set_request_on_resource_capability_switch_main_0(st_things_set_request_message_s* req_msg, st_things_representation_s* resp_rep);
extern bool handle_get_request_on_resource_capability_colorcontrol_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep);
extern bool handle_set_request_on_resource_capability_colorcontrol_main_0(st_things_set_request_message_s* req_msg, st_things_representation_s* resp_rep);
extern bool handle_get_request_on_resource_capability_dustsensor_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep);
extern bool handle_get_request_on_resource_capability_energymeter_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep);
extern bool handle_get_request_on_resource_capability_powermeter_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep);
extern bool handle_get_request_on_resource_capability_doorcontrol_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep);
extern bool handle_set_request_on_resource_capability_doorcontrol_main_0(st_things_set_request_message_s* req_msg, st_things_representation_s* resp_rep);

bool handle_get_request(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
//	printf("Received a GET request on %s\n", req_msg->resource_uri);

    if (0 == strcmp(req_msg->resource_uri, RES_CAPABILITY_SWITCH_MAIN_0))
    {
        return handle_get_request_on_resource_capability_switch_main_0(req_msg, resp_rep);
    }
    else if (0 == strcmp(req_msg->resource_uri, RES_CAPABILITY_COLORCONTROL_MAIN_0))
    {
        return handle_get_request_on_resource_capability_colorcontrol_main_0(req_msg, resp_rep);
    }
    else if (0 == strcmp(req_msg->resource_uri, RES_CAPABILITY_DUSTSENSOR_MAIN_0))
    {
        return handle_get_request_on_resource_capability_dustsensor_main_0(req_msg, resp_rep);
    }
    else if (0 == strcmp(req_msg->resource_uri, RES_CAPABILITY_ENERGYMETER_MAIN_0))
    {
        return handle_get_request_on_resource_capability_energymeter_main_0(req_msg, resp_rep);
    }
    else if (0 == strcmp(req_msg->resource_uri, RES_CAPABILITY_POWERMETER_MAIN_0))
    {
        return handle_get_request_on_resource_capability_powermeter_main_0(req_msg, resp_rep);
    }
    else if (0 == strcmp(req_msg->resource_uri, RES_CAPABILITY_DOORCONTROL_MAIN_0))
    {
        return handle_get_request_on_resource_capability_doorcontrol_main_0(req_msg, resp_rep);
    }
    else return false;
}

bool handle_set_request(st_things_set_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
	printf("Received a SET request on %s\n", req_msg->resource_uri);

    if (0 == strcmp(req_msg->resource_uri, RES_CAPABILITY_SWITCH_MAIN_0))
    {
        return handle_set_request_on_resource_capability_switch_main_0(req_msg, resp_rep);
    }
    else if (0 == strcmp(req_msg->resource_uri, RES_CAPABILITY_COLORCONTROL_MAIN_0))
    {
        return handle_set_request_on_resource_capability_colorcontrol_main_0(req_msg, resp_rep);
    }
    else if (0 == strcmp(req_msg->resource_uri, RES_CAPABILITY_DOORCONTROL_MAIN_0))
    {
        return handle_set_request_on_resource_capability_doorcontrol_main_0(req_msg, resp_rep);
    }
    else return false;
}

int ess_process(void)
{
	iotapi_initialize();
    bool easysetup_complete = false;
    st_things_initialize("/rom/device_def.json", &easysetup_complete);

    // The way to handle user input depends on the application developers.
    if (!easysetup_complete)
    {
		switch_power_gpio_init();
        // TODO: Write your implementation in this section.
    }

    st_things_register_request_cb                     (handle_get_request, handle_set_request);
    st_things_register_reset_cb                       (handle_reset_request, handle_reset_result);
    st_things_register_user_confirm_cb                (handle_ownership_transfer_request);
    st_things_register_things_status_change_cb        (handle_things_status_change);

    st_things_start();
    printf("=====================================================\n");
    printf("                    Stack Started                    \n");
    printf("=====================================================\n");

    handle_main_loop();

    return 0;
}
