/****************************************************************************
 * Copyright 2018 DIGNSYS All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
****************************************************************************/

#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <net/if.h>
#include <net/lwip/opt.h>
#include <net/lwip/netif.h>
#include <net/lwip/tcpip.h>
#include <tinyara/kmalloc.h>
#include "wwd_management.h"
#include "wwd_network.h"
#include "wwd_constants.h"
#include "wwd_structures.h"
#include "wwd_internal.h"
#include "wwd_thread_internal.h"
#include "wwd_wifi.h"
#include "RTOS/wwd_rtos_interface.h"

//#define WLAN_LOW_LEVEL_TEST

static struct netif *cyw43438_wlan_netif;
static int wlan_inited = 0;

struct netif* get_cyw43438_wlan_netif(void)
{
	return cyw43438_wlan_netif;
}

static int __up_wlan_init(void)
{
	struct ip4_addr ipaddr;
	struct ip4_addr netmask;
	struct ip4_addr gw;

	ipaddr.addr = inet_addr("0.0.0.0");
	netmask.addr = inet_addr("255.255.255.255");
	gw.addr = inet_addr("0.0.0.0");

	if (!wlan_inited)	/* only for first initialize */
	{
		cyw43438_wlan_netif = (struct netif *)kmm_zalloc(sizeof(struct netif));
		if (cyw43438_wlan_netif == NULL)
			return -1;

		netif_add(cyw43438_wlan_netif, &ipaddr, &netmask, &gw, WWD_STA_INTERFACE, wwd_ethernetif_init, tcpip_input);
	}

	if (cyw43438_wlan_netif == NULL)
		return -1;

	netif_set_down(cyw43438_wlan_netif);
	netif_set_link_up(cyw43438_wlan_netif);
	netif_set_up(cyw43438_wlan_netif);
	netif_set_default(cyw43438_wlan_netif);


#if 1 /* Test code */
	printf("ifname = %c%c, %s, %02X:%02X:%02X:%02X:%02X:%02X\n",
    		cyw43438_wlan_netif->name[0],
			cyw43438_wlan_netif->name[1],
			cyw43438_wlan_netif->d_ifname,
			cyw43438_wlan_netif->d_mac.ether_addr_octet[0],
			cyw43438_wlan_netif->d_mac.ether_addr_octet[1],
			cyw43438_wlan_netif->d_mac.ether_addr_octet[2],
			cyw43438_wlan_netif->d_mac.ether_addr_octet[3],
			cyw43438_wlan_netif->d_mac.ether_addr_octet[4],
			cyw43438_wlan_netif->d_mac.ether_addr_octet[5]);
#endif
	cyw43438_wlan_netif->d_flags |= IFF_UP;

	wlan_inited++ ;

	return 0;
}

/////////////////////////////////////////////////////////////
#define NULL_MAC( a )  (((((unsigned char*)a)[0])==0)&& \
                        ((((unsigned char*)a)[1])==0)&& \
                        ((((unsigned char*)a)[2])==0)&& \
                        ((((unsigned char*)a)[3])==0)&& \
                        ((((unsigned char*)a)[4])==0)&& \
                        ((((unsigned char*)a)[5])==0))

#define CMP_MAC( a, b )  (((((unsigned char*)a)[0])==(((unsigned char*)b)[0]))&& \
                         ((((unsigned char*)a)[1])==(((unsigned char*)b)[1]))&& \
                         ((((unsigned char*)a)[2])==(((unsigned char*)b)[2]))&& \
                         ((((unsigned char*)a)[3])==(((unsigned char*)b)[3]))&& \
                         ((((unsigned char*)a)[4])==(((unsigned char*)b)[4]))&& \
                         ((((unsigned char*)a)[5])==(((unsigned char*)b)[5])))

#define CIRCULAR_RESULT_BUFF_SIZE     (40)
static wiced_scan_result_t     result_buff[CIRCULAR_RESULT_BUFF_SIZE];
static uint16_t                result_buff_write_pos = 0;
static host_semaphore_type_t   num_scan_results_semaphore;
static wiced_mac_t             bssid_list[200];
extern void print_scan_result( wiced_scan_result_t* record );
static void wiced_scan_result_handler( wiced_scan_result_t** result_ptr, void* user_data, wiced_scan_status_t status )
{
    if ( result_ptr == NULL )
    {
        /* finished */
        result_buff[result_buff_write_pos].channel = 0xff;
        host_rtos_set_semaphore( &num_scan_results_semaphore, WICED_FALSE );
        return;
    }

    wiced_scan_result_t* record = ( *result_ptr );

    /* Check the list of BSSID values which have already been printed */
    wiced_mac_t * tmp_mac = bssid_list;
    while ( NULL_MAC( tmp_mac->octet ) == WICED_FALSE )
    {
        if ( CMP_MAC( tmp_mac->octet, record->BSSID.octet ) == WICED_TRUE )
        {
            /* already seen this BSSID */
            return;
        }
        tmp_mac++;
    }

    /* New BSSID - add it to the list */
    memcpy( &tmp_mac->octet, record->BSSID.octet, sizeof(wiced_mac_t) );

    print_scan_result(record);

    /* Add the result to the list and set the pointer for the next result */
    result_buff_write_pos++;
    *result_ptr = &result_buff[result_buff_write_pos];
    if ( result_buff_write_pos >= CIRCULAR_RESULT_BUFF_SIZE )
    {
        result_buff_write_pos = 0;
    }
}

#ifdef WLAN_LOW_LEVEL_TEST

/* Private AP test */
#if 0
#define AP_SSID              "netsto"
#define AP_PASS              "0312031203"
#define AP_SEC               WICED_SECURITY_WPA2_MIXED_PSK
#else
#define AP_SSID              "black"
#define AP_PASS              ""
#define AP_SEC               WICED_SECURITY_OPEN
#endif

static const wiced_ssid_t ap_ssid =
{
	.length = sizeof(AP_SSID) - 1,
	.value  = AP_SSID,
};

wwd_result_t wiced_wifi_find_ap( const char* ssid, wiced_scan_result_t* ap_info, const uint16_t* optional_channel_list)
{
    wwd_result_t  result;

    host_rtos_init_semaphore( &num_scan_results_semaphore );

    result = ( wwd_result_t ) wwd_wifi_scan( WICED_SCAN_TYPE_ACTIVE, WICED_BSS_TYPE_ANY,
    		(wiced_ssid_t*)ssid,
			NULL,
			optional_channel_list,
			NULL,
			wiced_scan_result_handler,
			(wiced_scan_result_t**) &ap_info,
			NULL,
			WWD_STA_INTERFACE );

    if ( result != WWD_SUCCESS ) {
		printf("wwd_wifi_scan ERROR\n");
        goto exit;
	}
    if (host_rtos_get_semaphore( &num_scan_results_semaphore, 1000000, WICED_FALSE) == WWD_SUCCESS) {
		printf("wwd_wifi_scan OK\n");
    	return WWD_SUCCESS;
	}
	printf("wwd_wifi_scan TIMEOUT\n");
    return WWD_TIMEOUT;

exit:
    host_rtos_deinit_semaphore( &num_scan_results_semaphore );

    memset(&num_scan_results_semaphore, 0, sizeof(num_scan_results_semaphore));

    return WWD_BADARG;
}


static void __wpa_scan(void) 
{
    wiced_scan_result_t ap_info;

    if (wiced_wifi_find_ap(NULL, &ap_info, NULL) == WWD_SUCCESS)
        printf("Success wiced_wifi_scan\n");
}    

static void __wpa_mac_test(void)
{
	wiced_mac_t mac;

	if ( wwd_wifi_get_mac_address( &mac, WWD_STA_INTERFACE ) == WWD_SUCCESS )
		printf("Test Pass (MAC address is: %02X:%02X:%02X:%02X:%02X:%02X)\n", mac.octet[0], mac.octet[1], mac.octet[2], mac.octet[3], mac.octet[4], mac.octet[5]);
	else
		printf("Test Fail\n");
}

static void __wpa_join_test(void)
{
	printf("Try join to:%s\n",ap_ssid.value);

	if (wwd_wifi_join(
	    		&ap_ssid,
			AP_SEC,
			(uint8_t*)AP_PASS,
			sizeof(AP_PASS)-1,
			NULL) !=  WWD_SUCCESS)
		printf(("Failed to join  : " AP_SSID"\n"));
	else
		printf(("Succeed to join: " AP_SSID"\n"));
}

static void __wpa_power_test(void)
{
	uint8_t dbm;
	wwd_result_t ret;

	ret = wwd_wifi_get_tx_power(&dbm);
	if (ret != WWD_SUCCESS) {
		printf("\tFailed to wwd_wifi_get_tx_power()\n");
		return;
	}

	printf("wwd_wifi_get_tx_power():Current %d dbm\n", dbm);

	dbm = 127;
	ret = wwd_wifi_set_tx_power(dbm);
	if (ret != WWD_SUCCESS) {
		printf("\tFailed to wwd_wifi_set_tx_power()\n");
		return;
	}

	printf("wwd_wifi_set_tx_power():New % dbm\n", dbm);

	ret = wwd_wifi_get_tx_power(&dbm);
	if (ret != WWD_SUCCESS) {
		printf("\tFailed to new wwd_wifi_get_tx_power()\n");
		return;
	}

	printf("wwd_wifi_get_tx_power():New %d dbm\n", dbm);
}

int __wpa_get_bssid(void)
{
        wiced_mac_t w_bssid;

        if (wwd_wifi_get_bssid(&w_bssid) != WWD_SUCCESS)
	{
		printf("\tFailed to get bssid\n");
                return -1;
	}

	printf("\nget BSSID : %02x:%02x:%02x:%02x:%02x:%02x\n",
			w_bssid.octet[0], w_bssid.octet[1], w_bssid.octet[2],
			w_bssid.octet[3], w_bssid.octet[4], w_bssid.octet[5]);
}


static void __arp_send_test(void)
{
	uint8_t packet[] = {
		0xe8, 0x4e, 0x06, 0x49, 0xfc, 0x67,	/* Destination MAC Address */
		0xb8, 0x27, 0xeb, 0xa1, 0xb2, 0xc3,	/* Source MAC Address */
		0x08, 0x06,				/* Ethernet Protocol Type. ARP:0x0806 */
		/* Start ARP Packet */
		0x00, 0x01,				/* Hardware Type. Ethernet:0x0001 */
		0x08, 0x00,				/* Protocol Type. IPv4:0x0800 */
		0x06,					/* Hardware Address Size */
		0x04,					/* Protocol Length. IPv4:0x04 */
		0x00, 0x01,				/* Operation Code. ARP Request:0x0001, ARP Reply:0x0002 */
		0xb8, 0x27, 0xeb, 0xa1, 0xb2, 0xc3, 	/* Sender Hardware Address */
		0xc0, 0xa8, 0x63, 0x9b,			/* Sender Protocol Address */
		0xe8, 0x4e, 0x06, 0x49, 0xfc, 0x67,   	/* Target Hardware Address */
		0xc0, 0xa8, 0x63, 0x4b,			/* Target Protocol Address */
	};

	eth_send_eapol(packet, packet + 6, packet + 14, sizeof(packet) - 14, 0x0806);
}
#endif /* WLAN_LOW_LEVEL_TEST */

extern int bring_up_wpa_supplicant(void);
int stm32l4_brcm_wlan_netif_init(void)
{
	if (wwd_buffer_init() != WWD_SUCCESS)
		return -1;

	if (wwd_management_wifi_on(WICED_COUNTRY_KOREA_REPUBLIC_OF) == WWD_SUCCESS)
		printf("Success Wifi-ON CYW43438\n");
	else {
		printf("Failed Wifi-ON CYW43438\n");
		return -1;
	}

	if (__up_wlan_init())
		return -1;

	return 0;
}

int stm32l4_brcm_wlan_netif_deinit(void)
{
	netif_set_link_down(cyw43438_wlan_netif);
	if (netif_is_up(cyw43438_wlan_netif)) {
		/* set netif down before removing (call callback function) */
		netif_set_down(cyw43438_wlan_netif);
	}

	netif_remove(cyw43438_wlan_netif);

	if (wwd_buffer_deinit() != WWD_SUCCESS)
		return -1;

	if (wwd_management_wifi_off() == WWD_SUCCESS)
	{
		printf("Success Wifi-OFF CYW43438\n");
	}
	else {
		printf("Failed Wifi-OFF CYW43438\n");
		return -1;
	}

	return 0;
}

void stm32l4_brcm_wlan_driver_initialize(void)
{

#ifndef WLAN_LOW_LEVEL_TEST

	bring_up_wpa_supplicant();

#else /* WLAN_LOW_LEVEL_TEST */
	if (stm32l4_brcm_wlan_netif_init() < 0)
		printf("Failed to WLAN network interface\n");

	printf("\n\n");
	printf("======== START TEST =======\n");
	__wpa_scan();

	printf("\n\n");
	printf("======== MAC TEST =======\n");
	__wpa_mac_test();

	printf("\n\n");
	printf("======== POWER TEST =======\n");
	__wpa_power_test();

	printf("\n\n");
	printf("======== JOIN TEST =======\n");
	WWD_WLAN_KEEP_AWAKE();
	__wpa_join_test();
	WWD_WLAN_KEEP_AWAKE();

	printf("\n\n");
	printf("======== GET BSSID =======\n");
	__wpa_get_bssid();

//	printf("\n\n");
//	printf("======== SEND TEST =======\n");
//	while (1) {
//		sleep(1);
//		__arp_send_test();
//	}

	printf("======== FINISH =======\n");
#endif /* WLAN_LOW_LEVEL_TEST */
}
