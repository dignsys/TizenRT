/*
 * common_handlers.c
 *
 *  Created on: Jun 28, 2018
 *      Author: build
 */

#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include "st_things/st_things.h"

/* callback functions */
bool handle_reset_request(void)
{
#ifdef DEBUG
    printf("Received a request for RESET.");
#endif

    // TODO: Write your implementation in this section.
    return false;  // FIXME: Modify this line with the appropriate return value.
}

void handle_reset_result(bool result)
{
#ifdef DEBUG
    printf("Reset %s.\n", result ? "succeeded" : "failed");
#endif

    // TODO: Write your implementation in this section.
}

bool handle_ownership_transfer_request(void)
{
#ifdef DEBUG
    printf("Received a request for Ownership-transfer.");
#endif

    // TODO: Write your implementation in this section.
    return true;  // FIXME: Modify this line with the appropriate return value.
}

void handle_things_status_change(st_things_status_e things_status)
{
#ifdef DEBUG
    printf("Things status is changed: %d\n", things_status);
#endif

    // TODO: Write your implementation in this section.
}

int g_quit_flag = 0;

/* main loop */
void handle_main_loop(void) {
    // TODO: Write your implementation in this section.
    while (!g_quit_flag)
    {
		power_meter_adc_test();	//not returned from here!!
        sleep(1);
    }
}
