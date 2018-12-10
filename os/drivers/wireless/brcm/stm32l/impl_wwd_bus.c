/*
 * Broadcom Proprietary and Confidential. Copyright 2016 Broadcom
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
 * Defines WWD SDIO functions for STM32x4xx MCU
 */
#include <tinyara/config.h>
#include <tinyara/irq.h>
#include <tinyara/arch.h>
#include <string.h> /* For memcpy */

#include "wwd_bus_protocol.h"
#include "wwd_assert.h"
#include "platform/wwd_platform_interface.h"
#include "platform/wwd_sdio_interface.h"
#include "platform/wwd_bus_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "network/wwd_network_constants.h"

#include "include/stm32l4xx.h"
#include "include/stm32l4xx_sdmmc.h"
#include "include/stm32l4xx_sdio.h"

#define DEBUG_CMD52
#define DEBUG_CMD53

/******************************************************
 *             Constants
 ******************************************************/
#define MAX_TIMEOUTS                         (30)
#define SDIO_IRQ_CHANNEL                     ((u8)(16+49))	//STM32L4_IRQ_SDMMC1
#define BUS_LEVEL_MAX_RETRIES                (5)

/******************************************************
 *             Structures
 ******************************************************/

/******************************************************
 *             Variables
 ******************************************************/
host_semaphore_type_t sdio_transfer_finished_semaphore;

/******************************************************
 *             Static Function Declarations
 ******************************************************/
static int __sdio_irq_isr(int irq, void *context, void *arg);
uint32_t sdio_get_blocksize_dctrl(sdio_block_size_t block_size);
static sdio_block_size_t find_optimal_block_size(uint32_t data_size);
static uint8_t temp_dma_buffer[MAX(2 * 1024, WICED_LINK_MTU + 64)];
static uint8_t *dma_data_source;
static pthread_mutex_t __sdio_bus_mutex;
static wiced_bool_t sdio_transfer_failed;
static uint32_t current_command;

/******************************************************
 *             Extern Function Declarations
 ******************************************************/
extern void wlan_gpio_config(void);
extern void sdio_gpio_config(int port, int pin);
extern void RCC_APB2PeriphClockCmd_SDIO(int NewState);

/******************************************************
 *             Function definitions
 ******************************************************/
#ifndef  WICED_DISABLE_MCU_POWERSAVE
static void sdio_oob_irq_handler(void *arg)
{

	UNUSED_PARAMETER(arg);

	wwd_thread_notify_irq();
}
#endif	/* ifndef  WICED_DISABLE_MCU_POWERSAVE */

#ifndef WICED_DISABLE_MCU_POWERSAVE
wwd_result_t host_enable_oob_interrupt(void)
{

	/* Set GPIO_B[1:0] to input. One of them will be re-purposed as OOB interrupt */
//.    platform_gpio_init( &wifi_sdio_pins[WWD_PIN_SDIO_OOB_IRQ], INPUT_HIGH_IMPEDANCE );
//    platform_gpio_irq_enable( &wifi_sdio_pins[WWD_PIN_SDIO_OOB_IRQ], IRQ_TRIGGER_RISING_EDGE, sdio_oob_irq_handler, 0 );
	return WWD_SUCCESS;
}


uint8_t host_platform_get_oob_interrupt_pin(void)
{

	return WICED_WIFI_OOB_IRQ_GPIO_PIN;
}


#endif	/* ifndef  WICED_DISABLE_MCU_POWERSAVE */

/* Wi-Fi SDIO bus pins. Used by WICED/platform/STM32F2xx/WWD/wwd_SDIO.c */
/**
 * WLAN SDIO pins
 */
typedef enum {
	WWD_PIN_SDIO_OOB_IRQ,
	WWD_PIN_SDIO_CLK,
	WWD_PIN_SDIO_CMD,
	WWD_PIN_SDIO_D0,
	WWD_PIN_SDIO_D1,
	WWD_PIN_SDIO_D2,
	WWD_PIN_SDIO_D3,
	WWD_PIN_SDIO_MAX,
} wwd_sdio_pin_t;

typedef struct {

	uint8_t port;

	uint8_t pin_number;
} platform_gpio_t;

static const platform_gpio_t wifi_sdio_pins[] = {
	{2, 6},
	{2, 12},
	{3, 2},
	{2, 8},
	{2, 9},
	{2, 10},
	{2, 11},
};


/**
 * Initializes the SDIO Bus
 *
 * Implemented in the platform interface which is specific to the
 * platform in use.
 *
 * @return WWD_SUCCESS or Error code
 */
wwd_result_t host_platform_bus_init(void)
{
	wwd_result_t result;
	uint8_t a;


	if (pthread_mutex_init(&__sdio_bus_mutex, NULL))
	{
		return WWD_SDIO_BUS_UP_FAIL;
	}

	result = host_rtos_init_semaphore(&sdio_transfer_finished_semaphore);
	if (result != WWD_SUCCESS)
	{
		return result;
	}

	wlan_gpio_config();

	BSP_SD_MspInit();

	/* Setup GPIO pins for SDIO data & clock */
	for (a = WWD_PIN_SDIO_CLK; a < WWD_PIN_SDIO_MAX; a++)
	{
		sdio_gpio_config(wifi_sdio_pins[a].port, wifi_sdio_pins[a].pin_number);
	}

	/*!< Enable the SDIO AHB Clock */
	RCC_APB2PeriphClockCmd_SDIO(1);
	BSP_SD_Init();

	return WWD_SUCCESS;
}


/**
 * De-Initializes the SDIO
 *
 * Implemented in the platform interface which is specific to the
 * platform in use.
 * This function does the reverse of @ref host_platform_bus_init
 *
 * @return WWD_SUCCESS or Error code
 */
wwd_result_t host_platform_bus_deinit(void)
{
	wwd_result_t result;

	pthread_mutex_destroy(&__sdio_bus_mutex);
	result = host_rtos_deinit_semaphore(&sdio_transfer_finished_semaphore);

	RCC_APB2PeriphClockCmd_SDIO(0);

#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_0
	platform_gpio_deinit(&wifi_control_pins[WWD_PIN_BOOTSTRAP_0]);

#endif	

#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_1
	platform_gpio_deinit(&wifi_control_pins[WWD_PIN_BOOTSTRAP_1]);

#endif

	/* Turn off SDIO IRQ */
	up_disable_irq(SDIO_IRQ_CHANNEL);
	irq_detach(SDIO_IRQ_CHANNEL);

	return result;
}


/**
 * Performs SDIO enumeration
 *
 * This needs to be called if the WLAN chip is reset
 *
 */
wwd_result_t host_platform_sdio_enumerate(void)
{
	BSP_SD_InitCard();

	return WWD_SUCCESS;
}


wwd_result_t host_platform_unmask_sdio_interrupt(void)
{
	host_platform_bus_enable_interrupt();

	return WWD_SUCCESS;
}


/**
 * Transfers SDIO data
 *
 * Implemented in the Platform interface, which is specific to the
 * platform in use.
 * Uses this function as a generic way to transfer data
 * across an SDIO bus.
 * Please refer to the SDIO specification.
 *
 * @param direction         : Direction of transfer - Write = to Wi-Fi device,
 *                                                    Read  = from Wi-Fi device
 * @param command           : The SDIO command number
 * @param mode              : Indicates whether transfer will be byte mode or block mode
 * @param block_size        : The block size to use (if using block mode transfer)
 * @param argument          : The argument of the particular SDIO command
 * @param data              : A pointer to the data buffer used to transmit or receive
 * @param data_size         : The length of the data buffer
 * @param response_expected : Indicates if a response is expected - RESPONSE_NEEDED = Yes
 *                                                                  NO_RESPONSE     = No
 * @param response  : A pointer to a variable which will receive the SDIO response.
 *                    Can be null if the caller does not care about the response value.
 *
 * @return WWD_SUCCESS if successful, otherwise an error code
 */
static wwd_result_t __host_platform_sdio_transfer(
	wwd_bus_transfer_direction_t direction,
	sdio_command_t command,
	sdio_transfer_mode_t mode,
	sdio_block_size_t block_size,
	uint32_t argument,
	/*@null@ */ uint32_t *data,
	uint16_t data_size,
	sdio_response_needed_t response_expected,
	/*@out@ *//*@null@ */ uint32_t *response)
{
	wwd_result_t result;
	uint16_t attempts = 0;

	wiced_assert("Bad args", !((command == SDIO_CMD_53) && (data == NULL)));

	if (response != NULL)
	{
		*response = 0;
	}

restart:
	sdio_transfer_failed = WICED_FALSE;
	++attempts;

	/* Check if we've tried too many times */
	if (attempts >= (uint16_t) BUS_LEVEL_MAX_RETRIES)
	{
		result = WWD_SDIO_RETRIES_EXCEEDED;
		goto exit;
	}

	/* Prepare the data transfer register */
	current_command = command;
	if (command == SDIO_CMD_53)
	{
		/* Dodgy STM32 hack to set the CMD53 byte mode size to be the same as the block size */
		if (mode == SDIO_BYTE_MODE)
		{
			block_size = find_optimal_block_size(data_size);

			if (block_size < SDIO_512B_BLOCK)
			{
				argument = (argument & (uint32_t)(~0x1FF)) | block_size;

			}
			else
			{
				argument = (argument & (uint32_t)(~0x1FF));

			}

		}

		/* Prepare the SDIO for a data transfer */
		if (direction == BUS_READ)
		{
			dma_data_source = temp_dma_buffer;

			/* Block DMA read */
			if (BSP_SD_ReadBlocks_DMA((uint32_t *) dma_data_source, data_size, block_size) != HAL_OK)
			{
				lldbg("BSP_SD_ReadBlocks_DMA FAIL\r\n");
				HAL_Delay(10);
				goto exit;
			}

			/* send CMD53 command */
			hal_sd_cmd_53(argument, response);
		}
		else
		{
			dma_data_source = (uint8_t *)data;

			/* send CMD53 command */
			hal_sd_cmd_53(argument, response);

			/* Block DMA write */
			if (BSP_SD_WriteBlocks_DMA((uint32_t *) dma_data_source, data_size, block_size) != HAL_OK)
			{
				lldbg("BSP_SD_WriteBlocks_DMA FAIL\r\n");
				HAL_Delay(10);
				goto exit;
			}
		}

		/* Wait for the whole transfer to complete */
		result = host_rtos_get_semaphore(&sdio_transfer_finished_semaphore, (uint32_t) 50, WICED_TRUE);
		if (result != WWD_SUCCESS)
		{
			goto exit;
		}

		if (sdio_transfer_failed == WICED_TRUE)
		{
			goto restart;
		}

		if (direction == BUS_READ)
		{
			memcpy(data, dma_data_source, (size_t) data_size);
		}
	}
	else if (command == SDIO_CMD_52)
	{
		/* Send the command */
		result = hal_sd_cmd_52(argument, response);
	}

	result = WWD_SUCCESS;

exit:
	return result;
}


wwd_result_t host_platform_sdio_transfer(
	wwd_bus_transfer_direction_t direction,
	sdio_command_t command,
	sdio_transfer_mode_t mode,
	sdio_block_size_t block_size,
	uint32_t argument,
	/*@null@ */ uint32_t *data,
	uint16_t data_size,
	sdio_response_needed_t response_expected,
	/*@out@ *//*@null@ */ uint32_t *response)
{
	wwd_result_t ret;

	pthread_mutex_lock(&__sdio_bus_mutex);

	ret = __host_platform_sdio_transfer(
			  direction,
			  command,
			  mode,
			  block_size,
			  argument,
			  data,
			  data_size,
			  response_expected,
			  response
		  );

	pthread_mutex_unlock(&__sdio_bus_mutex);

	return ret;
}


void host_platform_enable_high_speed_sdio(void)
{
	BSP_SD_InitEnd();
}

static sdio_block_size_t find_optimal_block_size(uint32_t data_size)
{
	if (data_size > (uint32_t) 256)
	{
		return SDIO_512B_BLOCK;
	}

	if (data_size > (uint32_t) 128)
	{
		return SDIO_256B_BLOCK;
	}

	if (data_size > (uint32_t) 64)
	{
		return SDIO_128B_BLOCK;
	}

	if (data_size > (uint32_t) 32)
	{
		return SDIO_64B_BLOCK;
	}

	if (data_size > (uint32_t) 16)
	{
		return SDIO_32B_BLOCK;
	}

	if (data_size > (uint32_t) 8)
	{
		return SDIO_16B_BLOCK;
	}

	if (data_size > (uint32_t) 4)
	{
		return SDIO_8B_BLOCK;
	}

	if (data_size > (uint32_t) 2)
	{
		return SDIO_4B_BLOCK;
	}

	return SDIO_4B_BLOCK;
}


uint32_t sdio_get_blocksize_dctrl(sdio_block_size_t block_size)
{
	switch (block_size)
	{
	case SDIO_1B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_1B;

	case SDIO_2B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_2B;

	case SDIO_4B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_4B;

	case SDIO_8B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_8B;

	case SDIO_16B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_16B;

	case SDIO_32B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_32B;

	case SDIO_64B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_64B;

	case SDIO_128B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_128B;

	case SDIO_256B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_256B;

	case SDIO_512B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_512B;

	case SDIO_1024B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_1024B;

	case SDIO_2048B_BLOCK:
		return SDMMC_DATABLOCK_SIZE_2048B;

	default:
		return 0;

	}
}


wwd_result_t host_platform_bus_enable_interrupt(void)
{

	return WWD_SUCCESS;
}


wwd_result_t host_platform_bus_disable_interrupt(void)
{

	return WWD_SUCCESS;
}


void host_platform_bus_buffer_freed(wwd_buffer_dir_t direction)
{

	UNUSED_PARAMETER(direction);
}
