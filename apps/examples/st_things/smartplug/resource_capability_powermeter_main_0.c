/*
 * resource_capability_powermeter_main_0.c
 *
 *  Created on: Jun 26, 2018
 *      Author: build
 */

#include <stdio.h>
#include <stdbool.h>
#include "st_things/st_things.h"

#include <errno.h>
#include <fcntl.h>
#include <tinyara/analog/adc.h>
#include <tinyara/analog/ioctl.h>

//#define	DEBUG_POWER

void update_power_value(void);
extern void update_energy_value(void);

static const char* PROP_UNIT = "unit";
static const char* PROP_POWER = "power";
static char *g_power_unit			= "kw";
static int 	g_power_meter        = 0;
static int  g_output_sum = 0;
//int g_energy_sum = 0;
int g_energy_hour = 0;
int g_energy_today = 0;
int g_energy_month = 0;
static int g_prev_mon = -1;
static int g_prev_day = -1;
static int g_prev_hour = -1;

#define ADC_MAX_SAMPLES	4

static char saved_url[1024] = {0,};

#define MAX_BUF_SIZE 64
void check_time(void)
{
	time_t now;
	struct tm *ptm;
	char sysinfo_str[MAX_BUF_SIZE + 1];

	now = time(NULL);
#if 0
	//ptm = (struct tm *)localtime(&now);	//RT does not supported localtime ?
#else	//tbd yhhan should be fixed!!
	ptm = (struct tm *)gmtime(&now);
	//Covert to SEOUL
	ptm->tm_hour += 9;
	if(ptm->tm_hour>=24) {
		ptm->tm_hour -= 24;
		ptm->tm_wday = (ptm->tm_wday+1) % 7;
		++ptm->tm_mday;
	}
#endif
	(void)strftime(sysinfo_str, MAX_BUF_SIZE, "%d %b %Y, %H:%M:%S", ptm);
	/* Print System Time information */
//	printf("[UTC %s]\n", sysinfo_str);
	if(ptm->tm_mon != g_prev_mon) {
		g_energy_hour = 0;
		g_energy_today = 0;
		g_energy_month = 0;
		g_prev_mon = ptm->tm_mon;
	}
	if(ptm->tm_mday != g_prev_day) {
		g_energy_month += g_energy_today;
		g_energy_hour = 0;
		g_energy_today = 0;
		g_prev_day = ptm->tm_mday;
	}
	if(ptm->tm_hour != g_prev_hour) {
		g_energy_today += g_energy_hour;
		g_energy_hour = 0;
		g_output_sum = 0;
		g_prev_hour = ptm->tm_hour;
	}

	check_all_timer(ptm);
}

//
// Sensitivity = 90 [mV/A]
// ScaleUp : 3300[mV]/1800[mV]
// mVolt[mV] = ( ADC / 4096 ) X 1800[mV]
// Current[A] = ((mVolt - 900[mV]) / Sensitivity) * ScaleUp
// Watt[W] = Current[A] * 220[V]
// 
void report_one_volts(int output)
{
    int abs_output = abs(output * 1800 * 100 / 4096 - 900 * 100); // [10mA]
    abs_output = (abs_output * 33 * 220 / 90) / 18; // [10mW]
    int mw = abs_output / 10;	//[100mW]
    int mwh;

//	check_time();

	g_output_sum += abs_output;	// mw * (5 sec)
	mwh = g_output_sum * (3600 / 5) / 10000;	// [100 Wh]

#ifdef	DEBUG_POWER
	printf("Output= %d, POWER: %d.%d mW, Energy=%d.%d kWh\n", output, mw/10, mw%10, mwh/10, mwh%10);
#endif
	if(g_energy_hour != mwh) {
		g_energy_hour = mwh;
		update_energy_value();
		update_energy_ext_value();
	}

	if(mw != g_power_meter) {
		g_power_meter = mw;
		update_power_value();
	}
}

void power_meter_adc_test(void)
{
	int fd, ret;
	struct adc_msg_s samples[ADC_MAX_SAMPLES];
	ssize_t nbytes;
	int sample_count = 0;
    int volts = 0, sample_volts = 0;

	fd = open("/dev/adc0", O_RDONLY);
	if (fd < 0) {
		printf("%s: open failed: %d\n", __func__, errno);
		return;
	}

	for (;;) {
		ret = ioctl(fd, ANIOC_TRIGGER, 0);
		if (ret < 0) {
			printf("%s: ioctl failed: %d\n", __func__, errno);
			close(fd);
			return;
		}

		nbytes = read(fd, samples, sizeof(samples));
		if (nbytes < 0) {
			if (errno != EINTR) {
				printf("%s: read failed: %d\n", __func__, errno);
				close(fd);
				return;
			}
		} else if (nbytes == 0) {
			printf("%s: No data read, Ignoring\n", __func__);
		} else {
			int nsamples = nbytes / sizeof(struct adc_msg_s);
			if (nsamples * sizeof(struct adc_msg_s) != nbytes) {
				printf("%s: read size=%ld is not a multiple of sample size=%d, Ignoring\n", __func__, (long)nbytes, sizeof(struct adc_msg_s));
			} else {
//				printf("Sample:\n");
				int i;
				for (i = 0; i < nsamples; i++) {
					if(samples[i].am_channel == 0 
						|| (samples[i].am_channel == 1 && samples[i].am_data >= 100)) {	//100 ������ ��쿡�� ����ȵ� ������ ������
						sample_count++;
						sample_volts += samples[i].am_data;
						if(sample_count == 5) {
							volts = sample_volts / sample_count;
							report_one_volts(volts);
							sample_count = 0;
							sample_volts = 0;
						}
					}
					//printf("%d: channel: %d, value: %d, nbytes=%d\n", i + 1, samples[i].am_channel, samples[i].am_data, nbytes);
				}
			}
		}
		check_time();
		sleep(1);
	}

	close(fd);
}

void update_power_value(void)
{
//	printf("update POWER value: %d\n", g_power_meter);
	if( saved_url[0] )
		st_things_notify_observers(saved_url);
}

bool handle_get_request_on_resource_capability_powermeter_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
#ifdef	DEBUG_POWER
    printf("Received a GET request on %s\n", req_msg->resource_uri);
#endif
	if(saved_url[0]==0)
		strncpy(saved_url, req_msg->resource_uri, 1024);

    if (req_msg->has_property_key(req_msg, PROP_POWER)) {
		resp_rep->set_int_value(resp_rep, PROP_POWER, g_power_meter);
#ifdef	DEBUG_POWER
		printf("g_power_meter=%d, g_power_unit=%s\n", g_power_meter, g_power_unit);
#endif
    }
    else if (req_msg->has_property_key(req_msg, PROP_UNIT)) {
#ifdef	DEBUG_POWER
		printf("POWER UNIT=%s\n", g_power_unit);
#endif
		resp_rep->set_str_value(resp_rep, PROP_UNIT, g_power_unit); // can be one of ["w", "kw", "mw"]
    }
    else
    	return false;
    return true;
}
