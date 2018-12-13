/*
 * resource_capability_doorcontrol_main_0.c
 *
 *  Created on: Sep 11, 2018
 *      Author: build
 */

#include <stdio.h>
#include <stdbool.h>
#include "st_things/st_things.h"

#include <apps/system/fota_hal.h>
#include <json/cJSON.h>
#include <protocols/webclient.h>
#include <slsi_wifi/slsi_wifi_utils.h>
#ifdef CONFIG_BOARDCTL_RESET
#include <sys/boardctl.h>
#endif

#ifdef CONFIG_EXAMPLES_ST_THINGS_SMARTPLUG_SUPPORT_OTA

#define DEBUG_OTA
//#define DEBUG_OTA_DUMP
#define OTA_VERIFY

#define OTA_VERSION		"1.0"

static const char* PROP_DOORSTATE = "doorState";
//static char saved_url[1024] = {0,};

#define FOTA_WEBCLIENT_BUF_SIZE     4600
#define FOTA_REC_JSON_SIZE		1000

#define KEY_URL 	"url"
#define KEY_MNID 	"mnid"
#define KEY_VID 	"vid"
#define KEY_VERSION "otaver"

const char c_ca_crt_rsa[] =
	"-----BEGIN CERTIFICATE-----\r\n"
	"MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ\r\n"
	"RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD\r\n"
	"VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX\r\n"
	"DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y\r\n"
	"ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy\r\n"
	"VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr\r\n"
	"mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr\r\n"
	"IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK\r\n"
	"mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu\r\n"
	"XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy\r\n"
	"dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye\r\n"
	"jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1\r\n"
	"BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3\r\n"
	"DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92\r\n"
	"9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx\r\n"
	"jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0\r\n"
	"Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz\r\n"
	"ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS\r\n"
	"R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp\r\n"
	"-----END CERTIFICATE-----\r\n"
	"-----BEGIN CERTIFICATE-----\r\n"
	"MIIENjCCAx6gAwIBAgIBATANBgkqhkiG9w0BAQUFADBvMQswCQYDVQQGEwJTRTEU\r\n"
	"MBIGA1UEChMLQWRkVHJ1c3QgQUIxJjAkBgNVBAsTHUFkZFRydXN0IEV4dGVybmFs\r\n"
	"IFRUUCBOZXR3b3JrMSIwIAYDVQQDExlBZGRUcnVzdCBFeHRlcm5hbCBDQSBSb290\r\n"
	"MB4XDTAwMDUzMDEwNDgzOFoXDTIwMDUzMDEwNDgzOFowbzELMAkGA1UEBhMCU0Ux\r\n"
	"FDASBgNVBAoTC0FkZFRydXN0IEFCMSYwJAYDVQQLEx1BZGRUcnVzdCBFeHRlcm5h\r\n"
	"bCBUVFAgTmV0d29yazEiMCAGA1UEAxMZQWRkVHJ1c3QgRXh0ZXJuYWwgQ0EgUm9v\r\n"
	"dDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALf3GjPm8gAELTngTlvt\r\n"
	"H7xsD821+iO2zt6bETOXpClMfZOfvUq8k+0DGuOPz+VtUFrWlymUWoCwSXrbLpX9\r\n"
	"uMq/NzgtHj6RQa1wVsfwTz/oMp50ysiQVOnGXw94nZpAPA6sYapeFI+eh6FqUNzX\r\n"
	"mk6vBbOmcZSccbNQYArHE504B4YCqOmoaSYYkKtMsE8jqzpPhNjfzp/haW+710LX\r\n"
	"a0Tkx63ubUFfclpxCDezeWWkWaCUN/cALw3CknLa0Dhy2xSoRcRdKn23tNbE7qzN\r\n"
	"E0S3ySvdQwAl+mG5aWpYIxG3pzOPVnVZ9c0p10a3CitlttNCbxWyuHv77+ldU9U0\r\n"
	"WicCAwEAAaOB3DCB2TAdBgNVHQ4EFgQUrb2YejS0Jvf6xCZU7wO94CTLVBowCwYD\r\n"
	"VR0PBAQDAgEGMA8GA1UdEwEB/wQFMAMBAf8wgZkGA1UdIwSBkTCBjoAUrb2YejS0\r\n"
	"Jvf6xCZU7wO94CTLVBqhc6RxMG8xCzAJBgNVBAYTAlNFMRQwEgYDVQQKEwtBZGRU\r\n"
	"cnVzdCBBQjEmMCQGA1UECxMdQWRkVHJ1c3QgRXh0ZXJuYWwgVFRQIE5ldHdvcmsx\r\n"
	"IjAgBgNVBAMTGUFkZFRydXN0IEV4dGVybmFsIENBIFJvb3SCAQEwDQYJKoZIhvcN\r\n"
	"AQEFBQADggEBALCb4IUlwtYj4g+WBpKdQZic2YR5gdkeWxQHIzZlj7DYd7usQWxH\r\n"
	"YINRsPkyPef89iYTx4AWpb9a/IfPeHmJIZriTAcKhjW88t5RxNKWt9x+Tu5w/Rw5\r\n"
	"6wwCURQtjr0W4MHfRnXnJK3s9EK0hZNwEGe6nQY1ShjTK3rMUUKhemPR5ruhxSvC\r\n"
	"Nr4TDea9Y355e6cJDUCrat2PisP29owaQgVR1EX1n6diIWgVIEM8med8vSTYqZEX\r\n"
	"c4g/VhsxOBi0cQ+azcgOno4uG+GMmIPLHzHxREzGBHNJdmAPx/i9F4BrLunMTA5a\r\n"
	"mnkPIAou1Z5jJh5VkpTYghdae9C8x49OhgQ=\r\n"
	"-----END CERTIFICATE-----\r\n";

struct http_client_ssl_config_t g_config = {
	(char *)c_ca_crt_rsa, NULL, NULL,
	sizeof(c_ca_crt_rsa), 0, 0, WEBCLIENT_SSL_VERIFY_REQUIRED
};

static int g_https;
//static bool g_finish = false;

typedef enum {
	FOTA_DOWNLOAD_STATE_NONE = 0,
	FOTA_DOWNLOAD_STATE_JSON,
	FOTA_DOWNLOAD_STATE_BINARY,
	FOTA_DOWNLOAD_STATE_DONE
} fota_download_state_e;

static int g_update_status = 0;
static char *update_status[] = {"open", "closing", "closing", "closed"};
static int fw_reboot = 0;

static fota_download_state_e download_state;

static const char headerfield_connect[] = "Connect";
static const char headerfield_close[] = "close";
static const char headerfield_useragent[] = "User-Agent";
static const char headerfield_tinyara[] = "TinyARA";

//static const char str_wget[] = "WGET";
//static const char str_get[] = "GET";
static char *json_url = "http://192.168.99.131/firmware.json";
#define MAX_URL_SIZE 	1024
#define MAX_MNID_SIZE 	6
#define MAX_VID_SIZE 	64
//#define MAX_VERSION_SIZE 10
static char firmware_url[MAX_URL_SIZE];
static char firmware_mnid[MAX_MNID_SIZE];
static char firmware_vid[MAX_VID_SIZE];
static u32 firmware_version;
static cJSON *cjson_root;
static cJSON *cjson_url;
static cJSON *cjson_mnid;
static cJSON *cjson_vid;
static cJSON *cjson_version;

static char *json_str;
static fotahal_handle_t fotahal_handle;
static unsigned int recv_size = 0;
static unsigned int total_size = 0;
static bool	is_link_fail;

#ifdef OTA_VERIFY
static unsigned char write_check_sum = 0;
static unsigned char read_check_sum = 0;
static u8 byte_check_sum(u8 *p, int count)
{
	u8 csum = 0;
	while(count--) {
		csum += *p++;
	}
	return csum;
}
#endif

#ifdef DEBUG_OTA_DUMP
static void	dump_4bytes(u32 pos, unsigned char *p)
{
	printf("[%08X]%02X %02X %02X %02X\n",pos,*p,*(p+1),*(p+2),*(p+3));
}
#endif

char *get_ota_version(void)
{
	return OTA_VERSION;
}

/** @brief Killx function in TASH to list all available commands
 *  @ingroup tash
 */
static void kill_useless_task(FAR struct tcb_s *tcb, FAR void *arg)
{
#if defined CONFIG_ENABLE_KILL && (CONFIG_TASK_NAME_SIZE > 0)
	char *basic_task[9] = {"Idle Task", "hpwork", "lpwork", "LWIP", "tash", "appmain", "WPA", "WLAN", "Wi-Fi"};
	char *parg[2] = {"kill", "11111"};

	int i, found = 0;
	for(i=0; i<9; i++) {
		if(strncmp(tcb->name, basic_task[i],strlen(basic_task[i])) == 0) {
			found = 1;
			break;
		}
	}
	if(!found) {
		sprintf(parg[1], "%d", tcb->pid);
	#ifdef DEBUG_OTA
		printf("kill %s\n", parg[1]);
	#endif
		kdbg_kill(2, parg);
	}
#endif
}

static void callback(struct http_client_response_t *response)
{
	if (response->status != 200) {
		is_link_fail = true;
		printf("recv callback status %d\n", response->status);
		return;
	}
#ifdef DEBUG_OTA_DUMP
	dump_4bytes(recv_size, (u8*)response->entity);
#endif
	if (download_state == FOTA_DOWNLOAD_STATE_JSON) {
		strncpy(json_str + recv_size, response->entity, response->entity_len);
	} else if (download_state == FOTA_DOWNLOAD_STATE_BINARY) {
		fotahal_write(fotahal_handle, response->entity, response->entity_len);
#ifdef DEBUG_OTA_DUMP
		printf("fota write: entity=%x, len=%x, recv[%x/%x]\n", response->entity, response->entity_len, recv_size, total_size);
#else
		if(recv_size == 0) 
			printf("fota write: entity=%x, len=%x, total_size=%x\n", response->entity, response->entity_len, total_size);
		else
			printf("%d", recv_size*10/total_size);
#endif
#ifdef OTA_VERIFY
		write_check_sum += byte_check_sum((u8*)response->entity, response->entity_len);
#endif
	}
	recv_size += response->entity_len;
	total_size = response->total_len;
}

/****************************************************************************
 * Name: wget_main
 ****************************************************************************/

static int webclient_init_request(char *url, struct http_client_request_t *request)
{
	int ret = -1;

	memset(request, 0, sizeof(struct http_client_request_t));

	request->method = WGET_MODE_GET;	
	request->url = (char *)malloc(strlen(url) + 1);
	if (!request->url) {
		printf("fota malloc error(%d)\n", strlen(url) + 1);
		return ret;
	}
	strncpy(request->url, url, strlen(url));
	request->url[strlen(url)] = '\0';

#ifdef CONFIG_NET_SECURITY_TLS
	if (!strncmp(request->url, "https", 5)) {
		g_https = 1;
	} else
#endif
	if (!strncmp(request->url, "http", 4)) {
		g_https = 0;
	} else {
		return ret;
	}

	request->buflen = FOTA_WEBCLIENT_BUF_SIZE;
	ret = 0;

	return ret;
}

static int wget_from_url(char *download_url) 
{
#ifdef DEBUG_OTA
	printf("download url : %s\n", download_url);
#endif
	struct http_client_request_t request;
	struct http_keyvalue_list_t headers;
//	struct http_client_response_t response;
	struct http_client_ssl_config_t *ssl_config = NULL;

	int ret = -1;

	if (webclient_init_request(download_url, &request) != 0) {
		printf("webclient_init_request error\n");
		return 0;
	}

	ssl_config = g_https ? &g_config : NULL;

	/* before sending request,
	 * must initialize keyvalue list for request headers
	 */
	http_keyvalue_list_init(&headers);
	http_keyvalue_list_add(&headers, headerfield_connect, headerfield_close);
	http_keyvalue_list_add(&headers, headerfield_useragent, headerfield_tinyara);
	request.headers = &headers;

	/* before sending request by sync function,
	 * must initialize response structure
	 */
	if (http_client_send_request_async(&request, ssl_config, (wget_callback_t)callback)) {
		printf("fail to send request\n");
		goto release_out;
	}
	/* sleep for end request */
	while (request.async_flag > 0) {
		usleep(100000);
	}

	if (request.async_flag < 0) {
		printf("fail to send request\n");
		goto release_out;
	}

	ret = 1;

release_out:
	/* before finish of app,
	 * must release keyvalue list for request headers
	 */
	http_keyvalue_list_release(&headers);
#ifdef DEBUG_OTA
	printf("end request\n");
#endif
	return ret;
}

#if 0
static void set_wifi_mode(void)
{
	char *secmode = SLSI_WIFI_SECURITY_WPA2_AES;
	char *passphrase = "0312031203";
	char *pcmd[3] = {"ifconfig", "wl1", "dhcp"};
	char *pjoin[4] = {"wifi", "join", "netsto", "0312031203"};

/*
	if (!WiFiRegisterLinkCallback(&sw_linkUpHandler, &sw_linkDownHandler)) {
		printf("Link call back handles registered - per default!\n");
	} else {
		printf("Link call back handles registered - Cannot continue !\n");
		return ret;
	}
*/
	(void)doStartSta();
	usleep(200000);
	printf("doJoin %s, %d, %s, %s\n", "netsto", 6, secmode, passphrase);
//	(void)doJoin("netsto", 6, NULL /*bssid */ , secmode, passphrase);
	slsi_wifi_main(4, &pjoin);
	usleep(500000);
	cmd_ifconfig(3, &pcmd);
	usleep(500000);
}
#endif

static u32 ver2int(char *s)
{
	char temp[10];
	char *p;
	int major, minor, count;
	p = strchr(s, '.');
	if(p != NULL) {
		memset(temp, 0, sizeof(temp));
		count = p - s;
		if(count < 10) {
			strncpy(temp, s, count);
			temp[count] = 0;
		}
		major = atoi(temp);
		minor = atoi(p+1);
	}
	else {
		major = atoi(s);
		minor = 0;
	}
	return major*100 + minor;
}

static int check_update(void)
{
#ifdef DEBUG_OTA
	printf("http_get_json\n");
#endif
	download_state = FOTA_DOWNLOAD_STATE_JSON;
	json_str = (char *)malloc(FOTA_REC_JSON_SIZE);
	recv_size = 0;
	
	is_link_fail = false;

	// parsing json
	if (wget_from_url(json_url) < 0) {
		printf("wget_from_url error\n");
		download_state = FOTA_DOWNLOAD_STATE_NONE;
		goto error_exit;
	}

	json_str[recv_size] = 0;

	if (is_link_fail) {
		download_state = FOTA_DOWNLOAD_STATE_NONE;
		goto error_exit;
	}

#ifdef DEBUG_OTA
	printf("[recv:JSON] state : %d / recv_size : %u / total size : %u / json = %s\n", download_state, recv_size, total_size, json_str);
#endif
	if (recv_size != total_size) {
		printf("[recv:JSON] file size error\n");
		download_state = FOTA_DOWNLOAD_STATE_NONE;
		goto error_exit;
	}

	cjson_root = cJSON_Parse((const char *)json_str);
	cjson_url = cJSON_GetObjectItem(cjson_root, KEY_URL);
	cjson_mnid = cJSON_GetObjectItem(cjson_root, KEY_MNID);
	cjson_vid = cJSON_GetObjectItem(cjson_root, KEY_VID);
	cjson_version = cJSON_GetObjectItem(cjson_root, KEY_VERSION);
	if(cjson_url->valuestring == NULL || strlen(cjson_url->valuestring) >= MAX_URL_SIZE) {
		printf("cjson url error\n");
		download_state = FOTA_DOWNLOAD_STATE_NONE;
		goto error_exit2;
	}
	else if(cjson_mnid->valuestring == NULL || strlen(cjson_mnid->valuestring) >= MAX_MNID_SIZE) {
		printf("cjson mnid error\n");
		download_state = FOTA_DOWNLOAD_STATE_NONE;
		goto error_exit2;
	}
	else if(cjson_vid->valuestring == NULL || strlen(cjson_vid->valuestring) >= MAX_VID_SIZE) {
		printf("cjson vid error\n");
		download_state = FOTA_DOWNLOAD_STATE_NONE;
		goto error_exit2;
	}
	else if(cjson_version->valuestring == NULL || strlen(cjson_version->valuestring) >= MAX_MNID_SIZE) {
		printf("cjson version error\n");
		download_state = FOTA_DOWNLOAD_STATE_NONE;
		goto error_exit2;
	}
	strncpy(firmware_url, cjson_url->valuestring, MAX_URL_SIZE);
	strncpy(firmware_mnid, cjson_mnid->valuestring, MAX_MNID_SIZE);
	strncpy(firmware_vid, cjson_vid->valuestring, MAX_VID_SIZE);
	firmware_version = ver2int(cjson_version->valuestring);
#ifdef DEBUG_OTA
	printf("firmware_url = %s\n mnid=%s, vid=%s, version=%d.%d\n", firmware_url, firmware_mnid, firmware_vid, firmware_version/100, firmware_version%100);
#endif
	// check new
	{
//		extern char *dm_get_firmware_version(void);
		extern char *dm_get_vendor_id(void);
		extern char *dm_get_mnid(void);
		char *my_mnid;
		char *my_vid;
		u32 my_version;

//		my_version = dm_get_firmware_version();
		my_version = ver2int(get_ota_version());
		my_vid = dm_get_vendor_id();
		my_mnid = dm_get_mnid();
		download_state = FOTA_DOWNLOAD_STATE_NONE;
		if(strncmp(firmware_mnid, my_mnid, strlen(my_mnid)) != 0)
			goto error_exit2;
		if(strncmp(firmware_vid, my_vid, strlen(my_vid)) != 0)
			goto error_exit2;
		if(firmware_version <= my_version) {
			download_state = FOTA_DOWNLOAD_STATE_DONE;
			goto error_exit2;
		}
#ifdef DEBUG_OTA
		printf("[NEW VERSION!! %d.%d > %d.%d]\n", firmware_version/100, firmware_version%100, my_version/100, my_version%100);
#endif
		download_state = FOTA_DOWNLOAD_STATE_JSON;
	}
error_exit2:		
	if (cjson_root != NULL) {
		cJSON_Delete(cjson_root);
	}
error_exit:		
	free(json_str);
	return download_state;
}

static int run_update(void)
{
#ifdef DEBUG_OTA
	printf("http_get_firmware\n");
#endif
	download_state = FOTA_DOWNLOAD_STATE_NONE;

	if (firmware_url[0] == 0) {
		goto download_exit;
	}

	fotahal_handle = fotahal_open();
	if (fotahal_handle == NULL) {
		goto download_exit;
	}
	download_state = FOTA_DOWNLOAD_STATE_BINARY;

	recv_size = 0;

	is_link_fail = false;

	if (wget_from_url(firmware_url) < 0) {
		printf("wget_from_url error\n");

		fotahal_erase(fotahal_handle);
		download_state = FOTA_DOWNLOAD_STATE_NONE;
		goto download_exit2;
	}

	if (is_link_fail) {
		fotahal_erase(fotahal_handle);
		download_state = FOTA_DOWNLOAD_STATE_NONE;
		printf("fota is_link_fail\n");
		goto download_exit2;
	}

#ifdef DEBUG_OTA
	printf("[recv:BINARY] recv_size : %u / total size : %u\n", recv_size, total_size, total_size);
#endif

	if (recv_size != total_size) {
		printf("[recv:BINARY] file size error\n");
		fotahal_erase(fotahal_handle);
		download_state = FOTA_DOWNLOAD_STATE_NONE;
		goto download_exit2;
	}

#ifdef OTA_VERIFY
	{
		//verify
		int read_size;
		char *temp_buf;
		read_check_sum = 0;
		temp_buf = malloc(0x1000);
		if(temp_buf == NULL) {
			download_state = FOTA_DOWNLOAD_STATE_NONE;
			goto download_exit2;
		}
	
		if ((fotahal_handle = fotahal_open()) == NULL) {
			printf("%s : fotahal_open error 2\n", __func__);
			download_state = FOTA_DOWNLOAD_STATE_NONE;
			goto download_exit2;
		}

		recv_size= 0;
		while(recv_size < total_size) {
			if(total_size - recv_size < 0x1000)
				read_size = total_size - recv_size;
			else
				read_size = 0x1000;
			if (fotahal_read(fotahal_handle, temp_buf, read_size) != FOTAHAL_RETURN_SUCCESS) {
				printf("%s : fotahal_read error 2\n", __func__);
				fotahal_erase(fotahal_handle);
				free(temp_buf);
				download_state = FOTA_DOWNLOAD_STATE_NONE;
				goto download_exit2;
			}
	#ifdef DEBUG_OTA_DUMP
			dump_4bytes(recv_size, (u8*)temp_buf);
			printf("fota read: read[%x/%x]\n", recv_size, total_size);
	#else
			if(recv_size == 0) 
				printf("fota read: total_size=%x\n", total_size);
			else
				printf("%d", recv_size*10/total_size);
	#endif
			read_check_sum += byte_check_sum((u8*)temp_buf, read_size);
			recv_size += read_size;
		}
		free(temp_buf);

		if(write_check_sum != read_check_sum) {
			printf("fota verify error !!! WSUM[%02X] != RSUM[%02X]\n", write_check_sum, read_check_sum);
			fotahal_erase(fotahal_handle);
			download_state = FOTA_DOWNLOAD_STATE_NONE;
			goto download_exit2;
		}
	#ifdef DEBUG_OTA
		printf("fota verify OK !!! WSUM[%02X] == RSUM[%02X]\n", write_check_sum, read_check_sum);
	#endif
	}
#endif

	download_state = FOTA_DOWNLOAD_STATE_BINARY;

download_exit2:
	fotahal_close(fotahal_handle);

download_exit:
	return download_state;
}

static const char* get_update_status(void)
{
	return update_status[g_update_status];
}

static int set_update_status(char *status)
{
	int retry;
	if (strncmp(status, update_status[0] , strlen(update_status[0])) == 0) {	//reset
		g_update_status = 0;
	} else if (strncmp(status, update_status[1], strlen(update_status[1])) == 0) {	//check
	//	set_wifi_mode();
		things_network_connect_home_ap();
		g_update_status = check_update();
	} else if (strncmp(status, update_status[3], strlen(update_status[3])) == 0) {	//update
		g_update_status = 2;
#ifdef DEBUG_OTA
		printf("kill_useless_task\n");
#endif
		// kill all tasks except tasks for http work
//		sched_foreach(kill_useless_task, NULL);
//		usleep(500000);
	
	//	set_wifi_mode();
		things_network_connect_home_ap();

		retry = 3;
		while(retry--) {
			g_update_status = run_update();
			if(g_update_status != FOTA_DOWNLOAD_STATE_NONE)
				break;
#ifdef DEBUG_OTA
			printf("FAIL=>RETRY COUNTDOWN %d\n", retry);
#endif
		}
		fw_reboot = 1;

	} else {
		printf("input Error");
		g_update_status = 0;
	}

	return g_update_status;
}

bool handle_get_request_on_resource_capability_doorcontrol_main_0(st_things_get_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
#ifdef DEBUG_OTA
    printf("Received a GET request on %s\n", req_msg->resource_uri);
#endif
//	if(saved_url[0]==0)
//		strncpy(saved_url, req_msg->resource_uri, 1024);

    if (req_msg->has_property_key(req_msg, PROP_DOORSTATE)) {
    	resp_rep->set_str_value(resp_rep, PROP_DOORSTATE, get_update_status());
    }
    return true;
}

bool handle_set_request_on_resource_capability_doorcontrol_main_0(st_things_set_request_message_s* req_msg, st_things_representation_s* resp_rep)
{
	char *state;
#ifdef DEBUG_OTA
    printf("Received a SET request on %s\n", req_msg->resource_uri);
#endif

    if( resp_rep->get_str_value(req_msg->rep, PROP_DOORSTATE, &state) ) {
#ifdef	DEBUG_OTA
		printf(" ==> %s\n", state);
#endif
		fw_reboot = 0;
		set_update_status(state);
		resp_rep->set_str_value(resp_rep, PROP_DOORSTATE, get_update_status());
		st_things_notify_observers(req_msg->resource_uri);

		usleep(500000);

		//reboot
#ifdef CONFIG_BOARDCTL_RESET
		if(fw_reboot) {
			boardctl(BOARDIOC_RESET, EXIT_SUCCESS);
		}
#endif
	}
    return true;
}
#endif
