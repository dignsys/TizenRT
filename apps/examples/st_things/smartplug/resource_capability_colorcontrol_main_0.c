/*
 * resource_capability_colorcontrol_main_0.c
 *
 *  Created on: Jul 4, 2018
 *      Author: build
 */

#include <stdio.h>
#include <stdbool.h>
#include "st_things/st_things.h"

//#define DEBUG_TIMER

static const char* PROP_SATURATION = "saturation";
static const char* PROP_CT = "ct";
//static const char* PROP_CSC = "csc";
//static const char* PROP_MAXIMUMSATURATION = "maximumsaturation";
//static const char* PROP_HUE = "hue";

#define MAX_TIMER	8

typedef struct _plug_timer {
	unsigned char	no:6;
	unsigned char	onoff:1;
	unsigned char	use:1;
	unsigned char	cycle;
	unsigned int	minute:6;
	unsigned int	hour:5;
	unsigned int	day:5;
	unsigned int	month:4;
	unsigned int	year:11;
} plug_timer_t;

static plug_timer_t plugtimer[MAX_TIMER] = {{0,},};

static char saved_url[1024] = {0,};
static int 	timer_index = 0;
static int 	timer_update_count = 0;
static int	wait_one_minute = 0;

static int is_checked_week(int i, struct tm *ptm) {
	if(plugtimer[i].cycle < 2) return 0;
	int week_mask = 1 << (ptm->tm_wday+1);
	if(week_mask & plugtimer[i].cycle)
		return 1;
	return 0;
}

void check_all_timer(struct tm *ptm)
{
	int i;
	int passed = 0;
	
	// remove all passed timer first !!
	for(i = 0; i < MAX_TIMER; i++) {
		if(!plugtimer[i].use) continue;

		passed = 0;
		if(plugtimer[i].year < ptm->tm_year)
			passed = 1;
		else if(plugtimer[i].year == ptm->tm_year) {
			if(plugtimer[i].month < ptm->tm_mon)
				passed = 1;
			else if(plugtimer[i].month == ptm->tm_mon) {
				if(plugtimer[i].day < ptm->tm_mday)
					passed = 1;
				else if(plugtimer[i].day == ptm->tm_mday) {
					if(plugtimer[i].hour < ptm->tm_hour)
						passed = 1;
					else if(plugtimer[i].hour == ptm->tm_hour) {
						if(plugtimer[i].minute < ptm->tm_min)
							passed = 1;
					}
				}
			}
		}
		if(passed) {
			// remove this timer !!
			memset(plugtimer[i], 0, sizeof(plug_timer_t));
		}
	}

#ifdef DEBUG_TIMER
//	printf("[NOW]%04d/%02d/%02d %02d:%02d\n",1900+ptm->tm_year, ptm->tm_mon+1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min);
#endif

	// check match and run timer !!
	for(i = 0; i < MAX_TIMER; i++) {
		if(!plugtimer[i].use) continue;

#ifdef DEBUG_TIMER
		printf("[%d]%04d/%02d/%02d %02d:%02d\n",i,plugtimer[i].year, plugtimer[i].month+1, plugtimer[i].day+1, plugtimer[i].hour, plugtimer[i].minute);
#endif
		if(plugtimer[i].year == (1900+ptm->tm_year) || plugtimer[i].cycle != 0) {
#ifdef DEBUG_TIMER
			printf("year ok!!");
#endif
			if(plugtimer[i].month == ptm->tm_mon || plugtimer[i].cycle != 0) {
#ifdef DEBUG_TIMER
				printf("=>month ok!!");
#endif
				if(plugtimer[i].day == ptm->tm_mday-1 || plugtimer[i].cycle == 1
						|| is_checked_week(i, ptm)) {
#ifdef DEBUG_TIMER
					printf("=>day ok!!");
#endif
					if(plugtimer[i].hour == ptm->tm_hour) {
#ifdef DEBUG_TIMER
						printf("=>hour ok!!");
#endif
						if(plugtimer[i].minute == ptm->tm_min) {
#ifdef DEBUG_TIMER
							printf("=>min ok!!");
#endif
							// run this timer !!
							if(!wait_one_minute) {
#ifdef DEBUG_TIMER
								printf("=> POWER %s", plugtimer[i].onoff?"ON":"OFF");
#endif
								power_onoff(plugtimer[i].onoff);
								if(plugtimer[i].cycle == 0) {
									// if once, remove this timer 
#ifdef DEBUG_TIMER
									printf(" & ERASE %d", i);
#endif
									//memset(plugtimer[i], 0, sizeof(plug_timer_t));
									//when use memset, app will not updated to recognize as 'NOT INITIALIZED'
									plugtimer[i].use = 0;
									timer_index = i;
									st_things_notify_observers(saved_url);
								}
								wait_one_minute = 1;
							}
						}
						else {
							// one minute passed after run power_onoff()
							wait_one_minute = 0;
						}
					}
				}
			}
#ifdef DEBUG_TIMER
			printf("\n");
#endif
		}
	}
	if(timer_update_count) {
		if(saved_url[0]) {
			while(1) {
				st_things_notify_observers(saved_url);
				timer_update_count--;
#ifdef DEBUG_TIMER
				printf("NOTIFY %d\n", timer_update_count);
#endif
				timer_index++;
				if(timer_index>=8)
					timer_index = 0;
				if(timer_update_count == 0)
					break;
				usleep(100000);	//100ms
			}
		}
	}
#ifdef DEBUG_TIMER
	{
		static int count = 0;
		if(count++ == 10) {
			count = 0;
			for(i=0; i<MAX_TIMER; i++) {
				if(i+1 == plugtimer[i].no)
					printf("[%d]%d,%d,%04x,%04d/%02d/%02d %02d:%02d\n",
							i, plugtimer[i].use, plugtimer[i].onoff, plugtimer[i].cycle,
							plugtimer[i].year, plugtimer[i].month+1, plugtimer[i].day+1, plugtimer[i].hour, plugtimer[i].minute);
//				else
//					printf("[%d] NOT INITIALIZED!!\n", i);
			}
		}
	}
#endif
}

bool handle_get_request_on_resource_capability_colorcontrol_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
#ifdef DEBUG_TIMER
    printf("Received a GET request on %s\n", req_msg->resource_uri);
#endif

	if(saved_url[0]==0)
		strncpy(saved_url, req_msg->resource_uri, 1024);

    if (req_msg->has_property_key(req_msg, PROP_SATURATION)) {
		int val = plugtimer[timer_index].no | (plugtimer[timer_index].use<<6) | (plugtimer[timer_index].onoff<<7) | (plugtimer[timer_index].cycle<<8);
		resp_rep->set_int_value(resp_rep, PROP_SATURATION, val);
#ifdef DEBUG_TIMER
		printf("GET SATURATION[%d]= 0x%08x\n", timer_index, val);
#endif
    }
    if (req_msg->has_property_key(req_msg, PROP_CT)) {
#ifdef DEBUG_TIMER
//		printf(" =>%d yyyy, %d mm, %d dd, %d hh, %d\n", 
//			plugtimer[timer_index].year, plugtimer[timer_index].month,
//			plugtimer[timer_index].day, plugtimer[timer_index].hour, plugtimer[timer_index].minute);
#endif
		int val = plugtimer[timer_index].minute
				| ((plugtimer[timer_index].hour)<<6)
				| ((plugtimer[timer_index].day)<<11)
				| ((plugtimer[timer_index].month)<<16)
				| ((plugtimer[timer_index].year)<<20);
		resp_rep->set_int_value(resp_rep, PROP_CT, val);
#ifdef DEBUG_TIMER
		printf("GET CT[%d]= 0x%08x\n", timer_index, val);
#endif
    }
    return true;
}

bool handle_set_request_on_resource_capability_colorcontrol_main_0(st_things_set_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
	int64_t receive;
	static int index = 0;
	static int updated = 0;
	int attr;
	int read_all = 0;
#ifdef DEBUG_TIMER
    printf("Received a SET request on %s\n", req_msg->resource_uri);
#endif

    if( resp_rep->get_int_value(req_msg->rep, PROP_SATURATION, &receive) ) {
    	attr = (int)receive;
    	if(attr == 0x7ff) {	// request for reading all timer
			read_all = 1;
			updated |= 1;
    	}
    	else {
			index = (attr-1) & 0x7; //MAX_TIMER
#ifdef DEBUG_TIMER
			printf("SET SATURATION[%d]= 0x%08x\n", index, attr);
#endif
			plugtimer[index].no = index+1;
			plugtimer[index].use = (attr >> 6) & 1;
			plugtimer[index].onoff = (attr >> 7) & 1;
			plugtimer[index].cycle = (attr >> 8) & 0xFF;
			updated |= 1;
    	}
	}
    if( resp_rep->get_int_value(req_msg->rep, PROP_CT, &receive) ) {
		attr = (int)receive;
		if(!read_all && attr) {
#ifdef DEBUG_TIMER
			printf("SET CT[%d]= 0x%08x\n", index, attr);
#endif
			plugtimer[index].minute = attr & 0x3F;
			plugtimer[index].hour = (attr>>6) & 0x1F;
			plugtimer[index].day = (attr>>11) & 0x1F;
			plugtimer[index].month = (attr>>16) & 0xF;
			plugtimer[index].year = (attr>>20) & 0x7FF;
#ifdef DEBUG_TIMER
//			printf(" =>%d yyyy, %d mm, %d dd, %d hh, %d\n", 
//				plugtimer[index].year, plugtimer[index].month,
//				plugtimer[index].day, plugtimer[index].hour, plugtimer[index].minute);
#endif
		}
		updated |= 2;
	}
	if(updated == 3) {
		updated = 0;
		if(read_all) {
			read_all = 0;
			timer_index = 0;
    		timer_update_count = 8;
#ifdef DEBUG_TIMER
    		printf("8 times ");
#endif
		} else {
			timer_index = index;
		}
#ifdef DEBUG_TIMER
		printf("NOTIFY!!\n");
#endif
		st_things_notify_observers(req_msg->resource_uri);
	}
    return true;
}
