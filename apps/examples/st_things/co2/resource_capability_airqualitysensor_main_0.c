/*
 * resource_capability_airqualitysensor_main_0.c
 *
 *  Created on: Jun 21, 2018
 *      Author: build
 */

#include <stdio.h>
#include <stdbool.h>
#include "st_things/st_things.h"

#include <errno.h>
#include <fcntl.h>
#include <tinyara/analog/adc.h>
#include <tinyara/analog/ioctl.h>

#define	CONVERT_TO_PPM
//#define	SIMULATION_DEBUG

#define MAX_STR_LEN                                 32
#define MAX_RANGES                                  2

void update_co2_value(void);

static const char* PROP_AIRQUALITY = "airQuality";
static const char* PROP_RANGE = "range";
//static int32_t g_co2_range[MAX_STR_LEN]      = {400, 10000};
static int32_t g_co2_value = 400;
static int32_t g_log_print = 0;		// debugging
#define ADC_MAX_SAMPLES	4

static char saved_url[1024] = {0,};

#ifdef	CONVERT_TO_PPM

#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier
#define         ZERO_POINT_VOLTAGE           (0.412) // <= 0.220 //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             (0.050) // <= 0.020 //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2
double          CO2Curve[3]  =  {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};
double         	zero_point_voltage =         ZERO_POINT_VOLTAGE;
double         	reaction_voltage =           REACTION_VOLTGAE;

inline double fastPow(double a, double b) {
  union {
    double d;
    int x[2];
  } u = { a };
  u.x[1] = (int)(b * (u.x[1] - 1072632447) + 1072632447);
  u.x[0] = 0;
  return u.d;
}

int  MGGetPercentage(double volts, double *pcurve)
{
	double val;
	if ((volts/DC_GAIN )>=zero_point_voltage) {
		return -1;
	} else {
		val = fastPow(10.0, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
		printf("\nvolts/DC_GAIN = %f\n", volts/DC_GAIN);
		printf("(volts/DC_GAIN)-pcurve[1] = %f\n", (volts/DC_GAIN)-pcurve[1]);
		printf("(volts/DC_GAIN)-pcurve[1])/pcurve[2] = %f\n", ((volts/DC_GAIN)-pcurve[1])/pcurve[2]);
		printf("PPM = %f\n", val);
		return (int)val;
	}
}
#endif

void report_one_volts(int ivolts)
{
	double volts = (double)ivolts / 1024;
    int percentage;

	printf("VOLTS: %3.2f V", volts);

#ifdef	CONVERT_TO_PPM
    percentage = MGGetPercentage(volts, CO2Curve);
    printf("  CO2: ");
    if (percentage == -1) {
	    printf("<400");
    } else {
        printf("%d", percentage);
    }
    printf(" ppm");
//	percentage /= 100;	// to 1/100 ppm
//	if(percentage > 100)
//		percentage = 100;
#else
	if(ivolts <
    percentage = ivolts/50;
    printf("  CO2: ");
    printf("%d", percentage);
    printf(" %%");
#endif

	printf("\n");

	if(percentage != g_co2_value) {
		g_co2_value = percentage;
		update_co2_value();
	}
}

#ifdef	SIMULATION_DEBUG
#define START_VAL	2550
#define END_VAL		2220
#define ADD_VAL		(40/5)
static int cur_val=	START_VAL;
#endif

void co2_meter_adc_test(void)
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
					if(samples[i].am_channel == 0) {
						sample_count++;
						#ifdef	SIMULATION_DEBUG
						sample_volts += cur_val;//samples[i].am_data;
						cur_val -= ADD_VAL;
						if(cur_val < END_VAL)
							cur_val = START_VAL;
						#else
						sample_volts += samples[i].am_data;
						#endif
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
		sleep(1);
	}

	close(fd);
}

void update_co2_value(void)
{
/*
#if 0
	g_co2_value = get_adc_analog(1);	// ADC1
#else
	g_co2_value += 100;
	if (g_co2_value >= 1000)
		g_co2_value = 400;
#endif
*/
	printf("update CO2 value: %d\n", g_co2_value);
	if( saved_url[0] )
		st_things_notify_observers(saved_url);
}

bool handle_get_request_on_resource_capability_airqualitysensor_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
#ifdef DEBUG
    printf("Received a GET request on %s\n", req_msg->resource_uri);
#endif
	if(saved_url[0]==0)
		strncpy(saved_url, req_msg->resource_uri, 1024);

    if (req_msg->has_property_key(req_msg, PROP_AIRQUALITY)) {
		resp_rep->set_int_value(resp_rep, PROP_AIRQUALITY, g_co2_value);
		if ((++g_log_print % 20) == 0)
			printf("CO2 value: %d\n", g_co2_value);
    }
    else if (req_msg->has_property_key(req_msg, PROP_RANGE)) {
//		printf("PROP_RANGE=%f ~ %f\n", g_co2_range[0], g_co2_range[1]);
//		resp_rep->set_int_array_value(resp_rep, PROP_RANGE, g_co2_range, MAX_RANGES);
    }
    else
    	return false;

//	st_things_notify_observers(req_msg->resource_uri);
    return true;
}
