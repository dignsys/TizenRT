/**
  ******************************************************************************
  * @file    stm32l4xx_sdio.c
  * @author  MCD Application Team
  * @brief   SDMMC Low Layer HAL module driver.
  *
  *          This file provides firmware functions to manage the following
  *          functionalities of the SDMMC peripheral:
  *           + Initialization/de-initialization functions
  *           + I/O operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *
  @verbatim
  ==============================================================================
                       ##### SDMMC peripheral features #####
  ==============================================================================
    [..] The SD/SDMMC MMC card host interface (SDMMC) provides an interface between the APB2
         peripheral bus and MultiMedia cards (MMCs), SD memory cards, SDMMC cards and CE-ATA
         devices.

    [..] The SDMMC features include the following:
         (+) Full compliance with MultiMedia Card System Specification Version 4.2. Card support
             for three different data bus modes: 1-bit (default), 4-bit and 8-bit
         (+) Full compatibility with previous versions of MultiMedia Cards (forward compatibility)
         (+) Full compliance with SD Memory Card Specifications Version 2.0
         (+) Full compliance with SD I/O Card Specification Version 2.0: card support for two
             different data bus modes: 1-bit (default) and 4-bit
         (+) Full support of the CE-ATA features (full compliance with CE-ATA digital protocol
             Rev1.1)
         (+) Data transfer up to 48 MHz for the 8 bit mode
         (+) Data and command output enable signals to control external bidirectional drivers.


                           ##### How to use this driver #####
  ==============================================================================
    [..]
      This driver is a considered as a driver of service for external devices drivers
      that interfaces with the SDMMC peripheral.
      According to the device used (SD card/ MMC card / SDMMC card ...), a set of APIs
      is used in the device's driver to perform SDMMC operations and functionalities.

      This driver is almost transparent for the final user, it is only used to implement other
      functionalities of the external device.

    [..]
      (+) The SDMMC clock (SDMMCCLK = 48 MHz) is coming from a specific output (MSI, PLLUSB1CLK,
          PLLUSB2CLK). Before start working with SDMMC peripheral make sure that the
          PLL is well configured.
          The SDMMC peripheral uses two clock signals:
          (++) SDMMC adapter clock (SDMMCCLK = 48 MHz)
          (++) APB2 bus clock (PCLK2)

          -@@- PCLK2 and SDMMC_CK clock frequencies must respect the following condition:
               Frequency(PCLK2) >= (3 / 8 x Frequency(SDMMC_CK)) for STM32L496xG and STM32L4A6xG
               Frequency(PCLK2) >= (3 / 4 x Frequency(SDMMC_CK)) otherwise

      (+) Enable/Disable peripheral clock using RCC peripheral macros related to SDMMC
          peripheral.

      (+) Enable the Power ON State using the SDMMC_PowerState_ON(SDMMCx)
          function and disable it using the function SDMMC_PowerState_OFF(SDMMCx).

      (+) Enable/Disable the clock using the __SDMMC_ENABLE()/__SDMMC_DISABLE() macros.

      (+) Enable/Disable the peripheral interrupts using the macros __SDMMC_ENABLE_IT(hSDMMC, IT)
          and __SDMMC_DISABLE_IT(hSDMMC, IT) if you need to use interrupt mode.

      (+) When using the DMA mode
          (++) Configure the DMA in the MSP layer of the external device
          (++) Active the needed channel Request
          (++) Enable the DMA using __SDMMC_DMA_ENABLE() macro or Disable it using the macro
               __SDMMC_DMA_DISABLE().

      (+) To control the CPSM (Command Path State Machine) and send
          commands to the card use the SDMMC_SendCommand(SDMMCx),
          SDMMC_GetCommandResponse() and SDMMC_GetResponse() functions. First, user has
          to fill the command structure (pointer to SDMMC_CmdInitTypeDef) according
          to the selected command to be sent.
          The parameters that should be filled are:
           (++) Command Argument
           (++) Command Index
           (++) Command Response type
           (++) Command Wait
           (++) CPSM Status (Enable or Disable).

          -@@- To check if the command is well received, read the SDMMC_CMDRESP
              register using the SDMMC_GetCommandResponse().
              The SDMMC responses registers (SDMMC_RESP1 to SDMMC_RESP2), use the
              SDMMC_GetResponse() function.

      (+) To control the DPSM (Data Path State Machine) and send/receive
           data to/from the card use the SDMMC_DataConfig(), SDMMC_GetDataCounter(),
          SDMMC_ReadFIFO(), SDMMC_WriteFIFO() and SDMMC_GetFIFOCount() functions.

    *** Read Operations ***
    =======================
    [..]
      (#) First, user has to fill the data structure (pointer to
          SDMMC_DataInitTypeDef) according to the selected data type to be received.
          The parameters that should be filled are:
           (++) Data TimeOut
           (++) Data Length
           (++) Data Block size
           (++) Data Transfer direction: should be from card (To SDMMC)
           (++) Data Transfer mode
           (++) DPSM Status (Enable or Disable)

      (#) Configure the SDMMC resources to receive the data from the card
          according to selected transfer mode (Refer to Step 8, 9 and 10).

      (#) Send the selected Read command (refer to step 11).

      (#) Use the SDMMC flags/interrupts to check the transfer status.

    *** Write Operations ***
    ========================
    [..]
     (#) First, user has to fill the data structure (pointer to
         SDMMC_DataInitTypeDef) according to the selected data type to be received.
         The parameters that should be filled are:
          (++) Data TimeOut
          (++) Data Length
          (++) Data Block size
          (++) Data Transfer direction:  should be to card (To CARD)
          (++) Data Transfer mode
          (++) DPSM Status (Enable or Disable)

     (#) Configure the SDMMC resources to send the data to the card according to
         selected transfer mode.

     (#) Send the selected Write command.

     (#) Use the SDMMC flags/interrupts to check the transfer status.

    *** Command management operations ***
    =====================================
    [..]
     (#) The commands used for Read/Write/Erase operations are managed in
         separate functions.
         Each function allows to send the needed command with the related argument,
         then check the response.
         By the same approach, you could implement a command and check the response.

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <tinyara/config.h>
#include <tinyara/arch.h>
#include <tinyara/kmalloc.h>

#include "include/stm32l4xx.h"
#include "include/stm32l4xx_sdio.h"

#include "platform/wwd_platform_interface.h"
#include "platform/wwd_sdio_interface.h"
#include "RTOS/wwd_rtos_interface.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @defgroup SDMMC_LL SDMMC Low Layer
  * @brief Low layer module for SD
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
static uint32_t gRelCardAdd;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint32_t SystemCoreClock = HSI_VALUE;	/*!< System Clock Frequency (Core Clock) */

/** @defgroup STM32L4R9I_EVAL_SD_Private_Variables Private Variables
  * @{
  */
SD_HandleTypeDef hsd_eval;
void BSP_SD_MspInit(void);

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
static uint32_t SDMMC_GetCmdError(SDMMC_TypeDef *SDMMCx);
static uint32_t SDMMC_GetCmdResp1_internal(SDMMC_TypeDef *SDMMCx, uint8_t SD_CMD, uint32_t Timeout, uint32_t *response);
static uint32_t SDMMC_GetCmdResp1(SDMMC_TypeDef *SDMMCx, uint8_t SD_CMD, uint32_t Timeout);
static uint32_t SDMMC_GetCmdResp2(SDMMC_TypeDef *SDMMCx);
static uint32_t SDMMC_GetCmdResp3(SDMMC_TypeDef *SDMMCx);
static uint32_t SDMMC_GetCmdResp7(SDMMC_TypeDef *SDMMCx);
static uint32_t SDMMC_GetCmdResp6(SDMMC_TypeDef *SDMMCx, uint8_t SD_CMD, uint16_t *pRCA);
static uint32_t SDMMC_Cmd52(SDMMC_TypeDef *SDMMCx, uint32_t arg, uint32_t *response);
static uint32_t SDMMC_Cmd53(SDMMC_TypeDef *SDMMCx, uint32_t arg, uint32_t *response);

static uint32_t SD_InitCard(SD_HandleTypeDef *hsd);
static uint32_t SD_PowerON(SD_HandleTypeDef *hsd);
static uint32_t SD_SendStatus(SD_HandleTypeDef *hsd, uint32_t *pCardStatus);
static uint32_t SD_WideBus_Enable(SD_HandleTypeDef *hsd);
static uint32_t SD_WideBus_Disable(SD_HandleTypeDef *hsd);
static uint32_t SD_FindSCR(SD_HandleTypeDef *hsd, uint32_t *pSCR);
static void SD_PowerOFF(SD_HandleTypeDef *hsd);
static void SD_Write_IT(SD_HandleTypeDef *hsd);
static void SD_Read_IT(SD_HandleTypeDef *hsd);

#if !defined(STM32L4R5xx) && !defined(STM32L4R7xx) && !defined(STM32L4R9xx) && !defined(STM32L4S5xx) && !defined(STM32L4S7xx) && !defined(STM32L4S9xx)
static void SD_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void SD_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void SD_DMAError(DMA_HandleTypeDef *hdma);
static void SD_DMATxAbort(DMA_HandleTypeDef *hdma);
static void SD_DMARxAbort(DMA_HandleTypeDef *hdma);

#endif	/* !STM32L4R5xx && !STM32L4R7xx && !STM32L4R9xx && !STM32L4S5xx && !STM32L4S7xx && !STM32L4S9xx */

static void TransferComplete(void);
static void TransferError(void);

/* Exported variables --------------------------------------------------------*/
extern host_semaphore_type_t sdio_transfer_finished_semaphore;

/* Exported functions --------------------------------------------------------*/

/** @defgroup SDMMC_LL_Exported_Functions SDMMC Low Layer Exported Functions
  * @{
  */

extern uint32_t sdio_get_blocksize_dctrl(sdio_block_size_t block_size);

/**
  * @brief This function is called to increment a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in SysTick ISR.
 * @note This function is declared as __weak to be overwritten in case of other
  *      implementations in user file.
  * @retval None
  */
__IO uint32_t uwTick;
void HAL_IncTick(void)
{

	uwTick++;
}


/**
  * @brief Provide a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{

	return uwTick;
}


/**
  * @brief This function provides minimum delay (in milliseconds) based
  *        on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay  specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(uint32_t Delay)
{

	up_mdelay(Delay);
}


/** @defgroup HAL_SDMMC_LL_Group1 Initialization de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization/de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the SDMMC according to the specified
  *         parameters in the SDMMC_InitTypeDef and create the associated handle.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  Init: SDMMC initialization structure
  * @retval HAL status
  */
HAL_StatusTypeDef SDMMC_Init(SDMMC_TypeDef *SDMMCx, SDMMC_InitTypeDef Init)
{

	/* Check the parameters */
	assert_param(IS_SDMMC_ALL_INSTANCE(SDMMCx));

	assert_param(IS_SDMMC_CLOCK_EDGE(Init.ClockEdge));

#if !defined(STM32L4R5xx) && !defined(STM32L4R7xx) && !defined(STM32L4R9xx) && !defined(STM32L4S5xx) && !defined(STM32L4S7xx) && !defined(STM32L4S9xx)
	assert_param(IS_SDMMC_CLOCK_BYPASS(Init.ClockBypass));

#endif	/* !STM32L4R5xx && !STM32L4R7xx && !STM32L4R9xx && !STM32L4S5xx && !STM32L4S7xx && !STM32L4S9xx */
	assert_param(IS_SDMMC_CLOCK_POWER_SAVE(Init.ClockPowerSave));

	assert_param(IS_SDMMC_BUS_WIDE(Init.BusWide));

	assert_param(IS_SDMMC_HARDWARE_FLOW_CONTROL(Init.HardwareFlowControl));

	assert_param(IS_SDMMC_CLKDIV(Init.ClockDiv));


	/* Set SDMMC configuration parameters */
	/* Write to SDMMC CLKCR */
#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
	MODIFY_REG(SDMMCx->CLKCR, CLKCR_CLEAR_MASK, Init.ClockEdge | \
			   Init.ClockPowerSave | \
			   Init.BusWide | \
			   Init.HardwareFlowControl | \
			   Init.ClockDiv);

#else	/* 
 */
	MODIFY_REG(SDMMCx->CLKCR, CLKCR_CLEAR_MASK, Init.ClockEdge | \
			   Init.ClockBypass | \
			   Init.ClockPowerSave | \
			   Init.BusWide | \
			   Init.HardwareFlowControl | \
			   Init.ClockDiv);

#endif	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

	return HAL_OK;
}


/**
  * @brief  Initializes the SD according to the specified parameters in the
            SD_HandleTypeDef and create the associated handle.
  * @param  hsd: Pointer to the SD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_Init(SD_HandleTypeDef *hsd)
{
	/* Check the SD handle allocation */
	if (hsd == NULL)
	{
		return HAL_ERROR;
	}


	/* Check the parameters */
	assert_param(IS_SDMMC_ALL_INSTANCE(hsd->Instance));
	assert_param(IS_SDMMC_CLOCK_EDGE(hsd->Init.ClockEdge));
#if !defined(STM32L4R5xx) && !defined(STM32L4R7xx) && !defined(STM32L4R9xx) && !defined(STM32L4S5xx) && !defined(STM32L4S7xx) && !defined(STM32L4S9xx)
	assert_param(IS_SDMMC_CLOCK_BYPASS(hsd->Init.ClockBypass));
#endif	/* !STM32L4R5xx && !STM32L4R7xx && !STM32L4R9xx && !STM32L4S5xx && !STM32L4S7xx && !STM32L4S9xx */
	assert_param(IS_SDMMC_CLOCK_POWER_SAVE(hsd->Init.ClockPowerSave));
	assert_param(IS_SDMMC_BUS_WIDE(hsd->Init.BusWide));
	assert_param(IS_SDMMC_HARDWARE_FLOW_CONTROL(hsd->Init.HardwareFlowControl));
	assert_param(IS_SDMMC_CLKDIV(hsd->Init.ClockDiv));

	hsd->State = HAL_SD_STATE_BUSY;

	/* Initialize SDMMC peripheral interface with default configuration */
	HAL_Delay(10);

	if (SDMMC_Init(hsd->Instance, hsd->Init) != HAL_OK)
	{
		return HAL_ERROR;
	}

	/* Set Power State to ON */
	HAL_Delay(10);

	if (SDMMC_PowerState_ON(hsd->Instance) != HAL_OK)
	{
		return HAL_ERROR;
	}

	/* Required power up waiting time before starting the SD initialization sequence */
	HAL_Delay(2U);

	return HAL_OK;
}


/**
  * @brief  De-Initializes the SD card.
  * @param  hsd: Pointer to SD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_DeInit(SD_HandleTypeDef *hsd)
{
	/* Check the SD handle allocation */
	if (hsd == NULL)
	{
		return HAL_ERROR;
	}

	/* Check the parameters */
	assert_param(IS_SDMMC_ALL_INSTANCE(hsd->Instance));

	hsd->State = HAL_SD_STATE_BUSY;

	/* Set SD power state to off */
	SD_PowerOFF(hsd);

	hsd->ErrorCode = HAL_SD_ERROR_NONE;
	hsd->State = HAL_SD_STATE_RESET;

	return HAL_OK;
}


/**
  * @brief  Initializes the SD Card.
  * @param  hsd: Pointer to SD handle
  * @note   This function initializes the SD card. It could be used when a card
            re-initialization is needed.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_InitCard(SD_HandleTypeDef *hsd)
{
	uint32_t errorstate;

	HAL_StatusTypeDef status;
	SD_InitTypeDef Init;


	/* Default SDMMC peripheral configuration for SD card initialization */
	Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
#if !defined(STM32L4R5xx) && !defined(STM32L4R7xx) && !defined(STM32L4R9xx) && !defined(STM32L4S5xx) && !defined(STM32L4S7xx) && !defined(STM32L4S9xx)
	Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
#endif	/* !STM32L4R5xx && !STM32L4R7xx && !STM32L4R9xx && !STM32L4S5xx && !STM32L4S7xx && !STM32L4S9xx */
	Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	Init.BusWide = SDMMC_BUS_WIDE_1B;
	Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	Init.ClockDiv = SDMMC_INIT_CLK_DIV;

#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
	if (hsd->Init.Transceiver == SDMMC_TRANSCEIVER_ENABLE)
	{
		/* Set Transceiver polarity */
		hsd->Instance->POWER |= SDMMC_POWER_DIRPOL;
	}
#endif	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

	/* Initialize SDMMC peripheral interface with default configuration */
	HAL_Delay(10);
	status = SDMMC_Init(hsd->Instance, Init);
	if (status != HAL_OK)
	{
		return HAL_ERROR;
	}

#if !defined(STM32L4R5xx) && !defined(STM32L4R7xx) && !defined(STM32L4R9xx) && !defined(STM32L4S5xx) && !defined(STM32L4S7xx) && !defined(STM32L4S9xx)
	/* Disable SDMMC Clock */
	__HAL_SD_DISABLE(hsd);
#endif	/* !STM32L4R5xx && !STM32L4R7xx && !STM32L4R9xx && !STM32L4S5xx && !STM32L4S7xx && !STM32L4S9xx */

	/* Set Power State to ON */
	HAL_Delay(10);
	status = SDMMC_PowerState_ON(hsd->Instance);
	if (status != HAL_OK)
	{
		return HAL_ERROR;
	}

#if !defined(STM32L4R5xx) && !defined(STM32L4R7xx) && !defined(STM32L4R9xx) && !defined(STM32L4S5xx) && !defined(STM32L4S7xx) && !defined(STM32L4S9xx)
	/* Enable SDMMC Clock */
	__HAL_SD_ENABLE(hsd);

#endif	/* !STM32L4R5xx && !STM32L4R7xx && !STM32L4R9xx && !STM32L4S5xx && !STM32L4S7xx && !STM32L4S9xx */

	/* Required power up waiting time before starting the SD initialization sequence */
	HAL_Delay(2U);

	/* Identify card operating voltage */
	HAL_Delay(10);

	errorstate = SD_PowerON(hsd);
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		hsd->State = HAL_SD_STATE_READY;
		hsd->ErrorCode |= errorstate;

		return HAL_ERROR;
	}

	/* Card initialization */
	HAL_Delay(10);

	errorstate = SD_InitCard(hsd);
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		hsd->State = HAL_SD_STATE_READY;
		hsd->ErrorCode |= errorstate;

		return HAL_ERROR;
	}

	HAL_Delay(10);

	return HAL_OK;
}


/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_SD_Init(void)
{
	uint8_t sd_state = MSD_OK;

	/* uSD device interface configuration */
	hsd_eval.Instance = SDMMC1;
	hsd_eval.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hsd_eval.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd_eval.Init.BusWide = SDMMC_BUS_WIDE_4B;
	hsd_eval.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
	hsd_eval.Init.ClockDiv = 1;
	hsd_eval.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;

	/* HAL SD initialization */
	if (HAL_SD_Init(&hsd_eval) != HAL_OK)
	{
		sd_state = MSD_ERROR;
		printf("BSP_SD_Init ERROR!!\r\n");
	}

	return sd_state;
}

uint8_t BSP_SD_InitCard(void)
{
	/* Initialize the Card parameters */
	HAL_Delay(10);

	if (HAL_SD_InitCard(&hsd_eval) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}


uint8_t BSP_SD_InitEnd(void)
{
	SD_HandleTypeDef *hsd = &hsd_eval;

#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
	/* Configure the bus wide */
	if (HAL_SD_ConfigWideBusOperation(hsd, hsd->Init.BusWide) != HAL_OK)
	{
		HAL_Delay(10);
		return HAL_ERROR;
	}
#endif	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

	/* Initialize the error code */
	hsd->ErrorCode = HAL_SD_ERROR_NONE;

	/* Initialize the SD operation */
	hsd->Context = SD_CONTEXT_NONE;

	/* Initialize the SD state */
	hsd->State = HAL_SD_STATE_READY;

	return HAL_OK;
}


/**
  * @brief  DeInitializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_SD_DeInit(void)
{
	uint8_t sd_state = MSD_OK;

	hsd_eval.Instance = SDMMC1;

	/* HAL SD deinitialization */
	if (HAL_SD_DeInit(&hsd_eval) != HAL_OK)
	{
		sd_state = MSD_ERROR;
	}

	/* Msp SD deinitialization */
	BSP_SD_MspDeInit(&hsd_eval, NULL);

	return sd_state;
}


/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @retval SD status
  */
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t DataSize, uint32_t BlockSize)
{
	HAL_StatusTypeDef sd_state = HAL_OK;

	/* Read block(s) in DMA transfer mode */
	sd_state = HAL_SD_ReadBlocks_DMA(&hsd_eval, (uint8_t *) pData, DataSize, BlockSize);

	return sd_state;
}



/** @defgroup HAL_SDMMC_LL_Group2 IO operation functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                      ##### I/O operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the SDMMC data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Read data (word) from Rx FIFO in blocking mode (polling)
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_ReadFIFO(SDMMC_TypeDef *SDMMCx)
{
	/* Read data from Rx FIFO */
	return (SDMMCx->FIFO);
}


/**
  * @brief  Write data (word) to Tx FIFO in blocking mode (polling)
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  pWriteData: pointer to data to write
  * @retval HAL status
  */
HAL_StatusTypeDef SDMMC_WriteFIFO(SDMMC_TypeDef *SDMMCx, uint32_t *pWriteData)
{
	/* Write data to FIFO */
	SDMMCx->FIFO = *pWriteData;

	return HAL_OK;
}


/**
  * @}
  */

/** @defgroup HAL_SDMMC_LL_Group3 Peripheral Control functions
 *  @brief   management functions
 *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the SDMMC data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Set SDMMC Power state to ON.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
HAL_StatusTypeDef SDMMC_PowerState_ON(SDMMC_TypeDef *SDMMCx)
{
	/* Set power state to ON */
#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
	SDMMCx->POWER |= SDMMC_POWER_PWRCTRL;
#else	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */
	SDMMCx->POWER = SDMMC_POWER_PWRCTRL;
#endif	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

	return HAL_OK;
}


#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
/**
  * @brief  Set SDMMC Power state to Power-Cycle.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
HAL_StatusTypeDef SDMMC_PowerState_Cycle(SDMMC_TypeDef *SDMMCx)
{
	/* Set power state to Power Cycle */
	SDMMCx->POWER |= SDMMC_POWER_PWRCTRL_1;

	return HAL_OK;
}
#endif	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

/**
  * @brief  Set SDMMC Power state to OFF.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
HAL_StatusTypeDef SDMMC_PowerState_OFF(SDMMC_TypeDef *SDMMCx)
{
	/* Set power state to OFF */
#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
	SDMMCx->POWER &= ~(SDMMC_POWER_PWRCTRL);
#else	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */
	SDMMCx->POWER = (uint32_t) 0x00000000;
#endif /* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

	return HAL_OK;
}


/**
  * @brief  Get SDMMC Power state.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval Power status of the controller. The returned value can be one of the
  *         following values:
  *            - 0x00: Power OFF
  *            - 0x02: Power UP
  *            - 0x03: Power ON
  */
uint32_t SDMMC_GetPowerState(SDMMC_TypeDef *SDMMCx)
{

	return (SDMMCx->POWER & SDMMC_POWER_PWRCTRL);
}




/** @defgroup HAL_SDMMC_LL_Group4 Command management functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                   ##### Commands management functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the needed commands.

@endverbatim
  * @{
  */

/**
  * @brief  Send the Data Block Lenght command and check the response
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdBlockLength(SDMMC_TypeDef *SDMMCx, uint32_t BlockSize)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Set Block Size for Card */
	sdmmc_cmdinit.Argument = (uint32_t) BlockSize;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SET_BLOCKLEN;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_SET_BLOCKLEN, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Read Single Block command and check the response
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdReadSingleBlock(SDMMC_TypeDef *SDMMCx, uint32_t ReadAdd)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Set Block Size for Card */
	sdmmc_cmdinit.Argument = (uint32_t) ReadAdd;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_READ_SINGLE_BLOCK;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_READ_SINGLE_BLOCK, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Read Multi Block command and check the response
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdReadMultiBlock(SDMMC_TypeDef *SDMMCx, uint32_t ReadAdd)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Set Block Size for Card */
	sdmmc_cmdinit.Argument = (uint32_t) ReadAdd;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_READ_MULT_BLOCK;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_READ_MULT_BLOCK, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Write Single Block command and check the response
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdWriteSingleBlock(SDMMC_TypeDef *SDMMCx, uint32_t WriteAdd)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Set Block Size for Card */
	sdmmc_cmdinit.Argument = (uint32_t) WriteAdd;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_WRITE_SINGLE_BLOCK;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_WRITE_SINGLE_BLOCK, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Write Multi Block command and check the response
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdWriteMultiBlock(SDMMC_TypeDef *SDMMCx, uint32_t WriteAdd)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Set Block Size for Card */
	sdmmc_cmdinit.Argument = (uint32_t) WriteAdd;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_WRITE_MULT_BLOCK;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_WRITE_MULT_BLOCK, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Start Address Erase command for SD and check the response
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdSDEraseStartAdd(SDMMC_TypeDef *SDMMCx, uint32_t StartAdd)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Set Block Size for Card */
	sdmmc_cmdinit.Argument = (uint32_t) StartAdd;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SD_ERASE_GRP_START;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_SD_ERASE_GRP_START, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the End Address Erase command for SD and check the response
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdSDEraseEndAdd(SDMMC_TypeDef *SDMMCx, uint32_t EndAdd)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Set Block Size for Card */
	sdmmc_cmdinit.Argument = (uint32_t) EndAdd;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SD_ERASE_GRP_END;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_SD_ERASE_GRP_END, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Start Address Erase command and check the response
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdEraseStartAdd(SDMMC_TypeDef *SDMMCx, uint32_t StartAdd)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Set Block Size for Card */
	sdmmc_cmdinit.Argument = (uint32_t) StartAdd;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_ERASE_GRP_START;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_ERASE_GRP_START, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the End Address Erase command and check the response
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdEraseEndAdd(SDMMC_TypeDef *SDMMCx, uint32_t EndAdd)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Set Block Size for Card */
	sdmmc_cmdinit.Argument = (uint32_t) EndAdd;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_ERASE_GRP_END;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_ERASE_GRP_END, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Erase command and check the response
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdErase(SDMMC_TypeDef *SDMMCx)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Set Block Size for Card */
	sdmmc_cmdinit.Argument = 0;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_ERASE;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_ERASE, SDMMC_MAXERASETIMEOUT);

	return errorstate;
}


/**
  * @brief  Configure the SDMMC command path according to the specified parameters in
  *         SDMMC_CmdInitTypeDef structure and send the command
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  Command: pointer to a SDMMC_CmdInitTypeDef structure that contains
  *         the configuration information for the SDMMC command
  * @retval HAL status
  */
HAL_StatusTypeDef SDMMC_SendCommand(SDMMC_TypeDef *SDMMCx, SDMMC_CmdInitTypeDef *Command)
{
	/* Check the parameters */
	assert_param(IS_SDMMC_CMD_INDEX(Command->CmdIndex));
	assert_param(IS_SDMMC_RESPONSE(Command->Response));
	assert_param(IS_SDMMC_WAIT(Command->WaitForInterrupt));
	assert_param(IS_SDMMC_CPSM(Command->CPSM));

	/* Set the SDMMC Argument value */
	SDMMCx->ARG = Command->Argument;

	/* Set SDMMC command parameters */
	/* Write to SDMMC CMD register */
	MODIFY_REG(SDMMCx->CMD, CMD_CLEAR_MASK, Command->CmdIndex | \
			   Command->Response | \
			   Command->WaitForInterrupt | \
			   Command->CPSM);

	return HAL_OK;
}


/**
  * @brief  Return the command index of last command for which response received
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval Command index of the last command response received
  */
uint8_t SDMMC_GetCommandResponse(SDMMC_TypeDef *SDMMCx)
{

	return (uint8_t)(SDMMCx->RESPCMD);
}



/**
  * @brief  Return the response received from the card for the last command
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  Response: Specifies the SDMMC response register.
  *          This parameter can be one of the following values:
  *            @arg SDMMC_RESP1: Response Register 1
  *            @arg SDMMC_RESP2: Response Register 2
  *            @arg SDMMC_RESP3: Response Register 3
  *            @arg SDMMC_RESP4: Response Register 4
  * @retval The Corresponding response register value
  */
uint32_t SDMMC_GetResponse(SDMMC_TypeDef *SDMMCx, uint32_t Response)
{
	uint32_t tmp;

	/* Check the parameters */
	assert_param(IS_SDMMC_RESP(Response));

	/* Get the response */
	tmp = (uint32_t)(&(SDMMCx->RESP1)) + Response;

	return (*(__IO uint32_t *) tmp);
}


/** @addtogroup SD_Private_Functions
  * @{
  */
uint32_t hal_sd_cmd_52(uint32_t argument, uint32_t *response)
{
	uint32_t errorstate;

	errorstate = SDMMC_Cmd52(hsd_eval.Instance, argument, response);
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		return errorstate;
	}

	return HAL_OK;
}


uint32_t hal_sd_cmd_53(uint32_t argument, uint32_t *response)
{
	uint32_t errorstate;

	errorstate = SDMMC_Cmd53(hsd_eval.Instance, argument, response);
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		return errorstate;
	}

	return HAL_OK;
}



/**
  * @brief  Send the Stop Transfer command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdStopTransfer(SDMMC_TypeDef *SDMMCx)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Send CMD12 STOP_TRANSMISSION  */
	sdmmc_cmdinit.Argument = 0;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_STOP_TRANSMISSION;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_STOP_TRANSMISSION, SDMMC_STOPTRANSFERTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Select Deselect command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  addr: Address of the card to be selected
  * @retval HAL status
  */
uint32_t SDMMC_CmdSelDesel(SDMMC_TypeDef *SDMMCx, uint64_t Addr)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;
	uint32_t response;

	/* Send CMD7 SDMMC_SEL_DESEL_CARD */
	sdmmc_cmdinit.Argument = (uint32_t) Addr;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SEL_DESEL_CARD;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1_internal(SDMMCx, SDMMC_CMD_SEL_DESEL_CARD, SDMMC_CMDTIMEOUT, &response);

	return errorstate;
}


/**
  * @brief  Send the Go Idle State command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdGoIdleState(SDMMC_TypeDef *SDMMCx)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	sdmmc_cmdinit.Argument = 0;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_GO_IDLE_STATE;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_NO;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdError(SDMMCx);

	return errorstate;
}


/**
  * @brief  Send the Operating Condition command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_Cmd5(SDMMC_TypeDef *SDMMCx)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Send CMD5 */
	sdmmc_cmdinit.Argument = 0;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SDMMC_SEN_OP_COND;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_NO;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdError(SDMMCx);

	return errorstate;
}


static uint32_t SDMMC_Cmd52(SDMMC_TypeDef *SDMMCx, uint32_t arg, uint32_t *response)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	sdmmc_cmdinit.Argument = arg;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SDMMC_RW_DIRECT;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1_internal(SDMMCx, SDMMC_CMD_SDMMC_RW_DIRECT, SDMMC_CMDTIMEOUT, response);

	return errorstate;
}


static uint32_t SDMMC_Cmd53(SDMMC_TypeDef *SDMMCx, uint32_t arg, uint32_t *response)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Send CMD53 */
	sdmmc_cmdinit.Argument = arg;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SDMMC_RW_EXTENDED;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_SDMMC_RW_DIRECT, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Operating Condition command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdOperCond(SDMMC_TypeDef *SDMMCx)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Send CMD8 to verify SD card interface operating condition */
	/* Argument: - [31:12]: Reserved (shall be set to '0')
	   - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
	   - [7:0]: Check Pattern (recommended 0xAA) */
	/* CMD Response: R7 */
	sdmmc_cmdinit.Argument = SDMMC_CHECK_PATTERN;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_HS_SEND_EXT_CSD;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp7(SDMMCx);

	return errorstate;
}


/**
  * @brief  Send the Application command to verify that that the next command
  *         is an application specific com-mand rather than a standard command
  *         and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  Argument: Command Argument
  * @retval HAL status
  */
uint32_t SDMMC_CmdAppCommand(SDMMC_TypeDef *SDMMCx, uint32_t Argument)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	sdmmc_cmdinit.Argument = (uint32_t) Argument;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_APP_CMD;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	/* If there is a HAL_ERROR, it is a MMC card, else
	   it is a SD card: SD card 2.0 (voltage range mismatch)
	   or SD card 1.x */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_APP_CMD, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the command asking the accessed card to send its operating
  *         condition register (OCR)
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  Argument: Command Argument
  * @retval HAL status
  */
uint32_t SDMMC_CmdAppOperCommand(SDMMC_TypeDef *SDMMCx, uint32_t Argument)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
	sdmmc_cmdinit.Argument = Argument;
#else	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */
	sdmmc_cmdinit.Argument = SDMMC_VOLTAGE_WINDOW_SD | Argument;
#endif /* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SD_APP_OP_COND;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp3(SDMMCx);

	return errorstate;
}


/**
  * @brief  Send the Bus Width command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  BusWidth: BusWidth
  * @retval HAL status
  */
uint32_t SDMMC_CmdBusWidth(SDMMC_TypeDef *SDMMCx, uint32_t BusWidth)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	sdmmc_cmdinit.Argument = (uint32_t) BusWidth;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_APP_SD_SET_BUSWIDTH;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_APP_SD_SET_BUSWIDTH, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Send SCR command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdSendSCR(SDMMC_TypeDef *SDMMCx)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Send CMD51 SD_APP_SEND_SCR */
	sdmmc_cmdinit.Argument = 0;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SD_APP_SEND_SCR;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_SD_APP_SEND_SCR, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Send CID command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdSendCID(SDMMC_TypeDef *SDMMCx)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Send CMD2 ALL_SEND_CID */
	sdmmc_cmdinit.Argument = 0;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_ALL_SEND_CID;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_LONG;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp2(SDMMCx);

	return errorstate;
}


/**
  * @brief  Send the Send CSD command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  Argument: Command Argument
  * @retval HAL status
  */
uint32_t SDMMC_CmdSendCSD(SDMMC_TypeDef *SDMMCx, uint32_t Argument)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Send CMD9 SEND_CSD */
	sdmmc_cmdinit.Argument = Argument;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SEND_CSD;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_LONG;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp2(SDMMCx);

	return errorstate;
}


/**
  * @brief  Send the Send CSD command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  pRCA: Card RCA
  * @retval HAL status
  */
uint32_t SDMMC_CmdSetRelAdd(SDMMC_TypeDef *SDMMCx, uint16_t *pRCA)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	/* Send CMD3 SD_CMD_SET_REL_ADDR */
	sdmmc_cmdinit.Argument = 0;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SET_REL_ADDR;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp6(SDMMCx, SDMMC_CMD_SET_REL_ADDR, pRCA);

	return errorstate;
}


/**
  * @brief  Send the Status command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  Argument: Command Argument
  * @retval HAL status
  */
uint32_t SDMMC_CmdSendStatus(SDMMC_TypeDef *SDMMCx, uint32_t Argument)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	sdmmc_cmdinit.Argument = Argument;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SEND_STATUS;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_SEND_STATUS, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Send the Status register command and check the response.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdStatusRegister(SDMMC_TypeDef *SDMMCx)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	sdmmc_cmdinit.Argument = 0;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SD_APP_STATUS;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_SD_APP_STATUS, SDMMC_CMDTIMEOUT);

	return errorstate;
}


/**
  * @brief  Sends host capacity support information and activates the card's
  *         initialization process. Send SDMMC_CMD_SEND_OP_COND command
  * @param  SDIOx: Pointer to SDIO register base
  * @parame Argument: Argument used for the command
  * @retval HAL status
  */
uint32_t SDMMC_CmdOpCondition(SDMMC_TypeDef *SDMMCx, uint32_t Argument)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	sdmmc_cmdinit.Argument = Argument;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_SEND_OP_COND;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp3(SDMMCx);

	return errorstate;
}


/**
  * @brief  Checks switchable function and switch card function. SDMMC_CMD_HS_SWITCH comand
  * @param  SDIOx: Pointer to SDIO register base
  * @parame Argument: Argument used for the command
  * @retval HAL status
  */
uint32_t SDMMC_CmdSwitch(SDMMC_TypeDef *SDMMCx, uint32_t Argument)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	sdmmc_cmdinit.Argument = Argument;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_HS_SWITCH;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_HS_SWITCH, SDMMC_CMDTIMEOUT);

	return errorstate;
}


#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
/**
  * @brief  Send the command asking the accessed card to send its operating
  *         condition register (OCR)
  * @param  None
  * @retval HAL status
  */
uint32_t SDMMC_CmdVoltageSwitch(SDMMC_TypeDef *SDMMCx)
{
	SDMMC_CmdInitTypeDef sdmmc_cmdinit;
	uint32_t errorstate;

	sdmmc_cmdinit.Argument = 0x00000000;
	sdmmc_cmdinit.CmdIndex = SDMMC_CMD_VOLTAGE_SWITCH;
	sdmmc_cmdinit.Response = SDMMC_RESPONSE_SHORT;
	sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
	sdmmc_cmdinit.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

	/* Check for error conditions */
	errorstate = SDMMC_GetCmdResp1(SDMMCx, SDMMC_CMD_VOLTAGE_SWITCH, SDMMC_CMDTIMEOUT);

	return errorstate;
}


#endif	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

/**
  * @brief  Checks for error conditions for CMD0.
  * @param  hsd: SD handle
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdError(SDMMC_TypeDef *SDMMCx)
{
	/* 8 is the number of required instructions cycles for the below loop statement.
	   The SDMMC_CMDTIMEOUT is expressed in ms */
	register uint32_t count = SDMMC_CMDTIMEOUT * (SystemCoreClock / 8U / 1000U);

	do
	{
		if (count-- == 0U)
		{
			return SDMMC_ERROR_TIMEOUT;
		}
	} while (!__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CMDSENT));

	/* Clear all the static flags */
	__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_STATIC_CMD_FLAGS);

	return SDMMC_ERROR_NONE;
}


/**
  * @brief  Checks for error conditions for R1 response.
  * @param  hsd: SD handle
  * @param  SD_CMD: The sent command index
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdResp1_internal(SDMMC_TypeDef *SDMMCx, uint8_t SD_CMD, uint32_t Timeout, uint32_t *response)
{
	uint32_t response_r1;
	uint32_t flags;

	flags = SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CMDREND | SDMMC_FLAG_CTIMEOUT;

	register uint32_t count = Timeout * (SystemCoreClock / 8U / 1000U);

	do
	{
		if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CMDACT) == RESET)
		{
			break;
		}

		if (count-- == 0U) {
			return SDMMC_ERROR_TIMEOUT;
		}
	} while (!__SDMMC_GET_FLAG(SDMMCx, flags));

	if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT))
	{
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT);

		return SDMMC_ERROR_CMD_RSP_TIMEOUT;
	}
	else if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL))
	{
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL);

		return SDMMC_ERROR_CMD_CRC_FAIL;
	}


	/* Check response received is of desired command */
	if (SDMMC_GetCommandResponse(SDMMCx) != SD_CMD)
	{
		return SDMMC_ERROR_CMD_CRC_FAIL;
	}

	/* Clear all the static flags */
	__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_STATIC_CMD_FLAGS);

	/* We have received response, retrieve it for analysis  */
	response_r1 = SDMMC_GetResponse(SDMMCx, SDMMC_RESP1);
	if (response != NULL)
	{
		*response = response_r1;
	}

	if ((response_r1 & SDMMC_OCR_ERRORBITS) == SDMMC_ALLZERO)
	{
		return SDMMC_ERROR_NONE;
	}
	else if ((response_r1 & SDMMC_OCR_ADDR_OUT_OF_RANGE) == SDMMC_OCR_ADDR_OUT_OF_RANGE)
	{
		return SDMMC_ERROR_ADDR_OUT_OF_RANGE;
	}
	else if ((response_r1 & SDMMC_OCR_ADDR_MISALIGNED) == SDMMC_OCR_ADDR_MISALIGNED)
	{
		return SDMMC_ERROR_ADDR_MISALIGNED;
	}
	else if ((response_r1 & SDMMC_OCR_BLOCK_LEN_ERR) == SDMMC_OCR_BLOCK_LEN_ERR)
	{
		return SDMMC_ERROR_BLOCK_LEN_ERR;
	}
	else if ((response_r1 & SDMMC_OCR_ERASE_SEQ_ERR) == SDMMC_OCR_ERASE_SEQ_ERR)
	{
		return SDMMC_ERROR_ERASE_SEQ_ERR;
	}
	else if ((response_r1 & SDMMC_OCR_BAD_ERASE_PARAM) == SDMMC_OCR_BAD_ERASE_PARAM)
	{
		return SDMMC_ERROR_BAD_ERASE_PARAM;
	}
	else if ((response_r1 & SDMMC_OCR_WRITE_PROT_VIOLATION) == SDMMC_OCR_WRITE_PROT_VIOLATION)
	{
		return SDMMC_ERROR_WRITE_PROT_VIOLATION;
	}
	else if ((response_r1 & SDMMC_OCR_LOCK_UNLOCK_FAILED) == SDMMC_OCR_LOCK_UNLOCK_FAILED)
	{
		return SDMMC_ERROR_LOCK_UNLOCK_FAILED;
	}
	else if ((response_r1 & SDMMC_OCR_COM_CRC_FAILED) == SDMMC_OCR_COM_CRC_FAILED)
	{
		return SDMMC_ERROR_COM_CRC_FAILED;
	}
	else if ((response_r1 & SDMMC_OCR_ILLEGAL_CMD) == SDMMC_OCR_ILLEGAL_CMD)
	{
		return SDMMC_ERROR_ILLEGAL_CMD;
	}
	else if ((response_r1 & SDMMC_OCR_CARD_ECC_FAILED) == SDMMC_OCR_CARD_ECC_FAILED)
	{
		return SDMMC_ERROR_CARD_ECC_FAILED;
	}
	else if ((response_r1 & SDMMC_OCR_CC_ERROR) == SDMMC_OCR_CC_ERROR)
	{
		return SDMMC_ERROR_CC_ERR;
	}
	else if ((response_r1 & SDMMC_OCR_STREAM_READ_UNDERRUN) == SDMMC_OCR_STREAM_READ_UNDERRUN)
	{
		return SDMMC_ERROR_STREAM_READ_UNDERRUN;
	}
	else if ((response_r1 & SDMMC_OCR_STREAM_WRITE_OVERRUN) == SDMMC_OCR_STREAM_WRITE_OVERRUN)
	{
		return SDMMC_ERROR_STREAM_WRITE_OVERRUN;
	}
	else if ((response_r1 & SDMMC_OCR_CID_CSD_OVERWRITE) == SDMMC_OCR_CID_CSD_OVERWRITE)
	{
		return SDMMC_ERROR_CID_CSD_OVERWRITE;
	}
	else if ((response_r1 & SDMMC_OCR_WP_ERASE_SKIP) == SDMMC_OCR_WP_ERASE_SKIP)
	{
		return SDMMC_ERROR_WP_ERASE_SKIP;
	}
	else if ((response_r1 & SDMMC_OCR_CARD_ECC_DISABLED) == SDMMC_OCR_CARD_ECC_DISABLED)
	{
		return SDMMC_ERROR_CARD_ECC_DISABLED;
	}
	else if ((response_r1 & SDMMC_OCR_ERASE_RESET) == SDMMC_OCR_ERASE_RESET)
	{
		return SDMMC_ERROR_ERASE_RESET;
	}
	else if ((response_r1 & SDMMC_OCR_AKE_SEQ_ERROR) == SDMMC_OCR_AKE_SEQ_ERROR)
	{
		return SDMMC_ERROR_AKE_SEQ_ERR;
	}
	else
	{
		return SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
	}
}


/**
  * @brief  Checks for error conditions for R1 response.
  * @param  hsd: SD handle
  * @param  SD_CMD: The sent command index
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdResp1(SDMMC_TypeDef *SDMMCx, uint8_t SD_CMD, uint32_t Timeout)
{

	return SDMMC_GetCmdResp1_internal(SDMMCx, SD_CMD, Timeout, NULL);
}


/**
  * @brief  Checks for error conditions for R2 (CID or CSD) response.
  * @param  hsd: SD handle
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdResp2(SDMMC_TypeDef *SDMMCx)
{
	/* 8 is the number of required instructions cycles for the below loop statement.
	   The SDMMC_CMDTIMEOUT is expressed in ms */
	register uint32_t count = SDMMC_CMDTIMEOUT * (SystemCoreClock / 8U / 1000U);

	do
	{
		if (count-- == 0U)
		{
			return SDMMC_ERROR_TIMEOUT;
		}

	} while (!__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CMDREND | SDMMC_FLAG_CTIMEOUT));

	if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT))
	{
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT);

		return SDMMC_ERROR_CMD_RSP_TIMEOUT;
	}
	else if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL))
	{
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL);

		return SDMMC_ERROR_CMD_CRC_FAIL;
	}
	else
	{
		/* No error flag set */
		/* Clear all the static flags */
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_STATIC_CMD_FLAGS);
	}

	return SDMMC_ERROR_NONE;
}


/**
  * @brief  Checks for error conditions for R3 (OCR) response.
  * @param  hsd: SD handle
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdResp3(SDMMC_TypeDef *SDMMCx)
{
	/* 8 is the number of required instructions cycles for the below loop statement.
	   The SDMMC_CMDTIMEOUT is expressed in ms */
	register uint32_t count = SDMMC_CMDTIMEOUT * (SystemCoreClock / 8U / 1000U);

	do
	{
		if (count-- == 0U)
		{
			return SDMMC_ERROR_TIMEOUT;
		}
	} while (!__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CMDREND | SDMMC_FLAG_CTIMEOUT));

	if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT))
	{
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT);

		return SDMMC_ERROR_CMD_RSP_TIMEOUT;
	}
	else
	{
		/* Clear all the static flags */
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_STATIC_CMD_FLAGS);
	}

	return SDMMC_ERROR_NONE;
}


/**
  * @brief  Checks for error conditions for R6 (RCA) response.
  * @param  hsd: SD handle
  * @param  SD_CMD: The sent command index
  * @param  pRCA: Pointer to the variable that will contain the SD card relative
  *         address RCA
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdResp6(SDMMC_TypeDef *SDMMCx, uint8_t SD_CMD, uint16_t *pRCA)
{
	uint32_t response_r1;

	/* 8 is the number of required instructions cycles for the below loop statement.
	   The SDMMC_CMDTIMEOUT is expressed in ms */
	register uint32_t count = SDMMC_CMDTIMEOUT * (SystemCoreClock / 8U / 1000U);

	do
	{
		if (count-- == 0U)
		{
			return SDMMC_ERROR_TIMEOUT;
		}

	} while (!__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CMDREND | SDMMC_FLAG_CTIMEOUT));

	if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT))
	{
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT);

		return SDMMC_ERROR_CMD_RSP_TIMEOUT;
	}
	else if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL))
	{
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL);

		return SDMMC_ERROR_CMD_CRC_FAIL;
	}
	else
	{
		/* Nothing to do */
	}

	/* Check response received is of desired command */
	if (SDMMC_GetCommandResponse(SDMMCx) != SD_CMD)
	{
		return SDMMC_ERROR_CMD_CRC_FAIL;
	}

	/* Clear all the static flags */
	__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_STATIC_CMD_FLAGS);

	/* We have received response, retrieve it.  */
	response_r1 = SDMMC_GetResponse(SDMMCx, SDMMC_RESP1);
	if ((response_r1 & (SDMMC_R6_GENERAL_UNKNOWN_ERROR | SDMMC_R6_ILLEGAL_CMD | SDMMC_R6_COM_CRC_FAILED)) == SDMMC_ALLZERO)
	{
		*pRCA = (uint16_t)(response_r1 >> 16);

		return SDMMC_ERROR_NONE;
	}
	else if ((response_r1 & SDMMC_R6_ILLEGAL_CMD) == SDMMC_R6_ILLEGAL_CMD)
	{
		return SDMMC_ERROR_ILLEGAL_CMD;
	}
	else if ((response_r1 & SDMMC_R6_COM_CRC_FAILED) == SDMMC_R6_COM_CRC_FAILED)
	{
		return SDMMC_ERROR_COM_CRC_FAILED;
	}
	else
	{
		return SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
	}
}


/**
  * @brief  Checks for error conditions for R7 response.
  * @param  hsd: SD handle
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdResp7(SDMMC_TypeDef *SDMMCx)
{
	/* 8 is the number of required instructions cycles for the below loop statement.
	   The SDMMC_CMDTIMEOUT is expressed in ms */
	register uint32_t count = SDMMC_CMDTIMEOUT * (SystemCoreClock / 8U / 1000U);

	do
	{
		if (count-- == 0U)
		{
			return SDMMC_ERROR_TIMEOUT;
		}

	} while (!__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CMDREND | SDMMC_FLAG_CTIMEOUT));

	if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT))
	{
		/* Card is SD V2.0 compliant */
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT);

		return SDMMC_ERROR_CMD_RSP_TIMEOUT;
	}
	else if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL))
	{
		/* Card is SD V2.0 compliant */
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL);

		return SDMMC_ERROR_CMD_CRC_FAIL;
	}
	else
	{
		/* Nothing to do */
	}

	if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CMDREND))
	{
		/* Card is SD V2.0 compliant */
		__SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CMDREND);
	}

	return SDMMC_ERROR_NONE;

}


/**
  * @brief  Configure the SDMMC data path according to the specified
  *         parameters in the SDMMC_DataInitTypeDef.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  Data : pointer to a SDMMC_DataInitTypeDef structure
  *         that contains the configuration information for the SDMMC data.
  * @retval HAL status
  */
HAL_StatusTypeDef SDMMC_ConfigData(SDMMC_TypeDef *SDMMCx, SDMMC_DataInitTypeDef *Data)
{
	/* Check the parameters */
	assert_param(IS_SDMMC_DATA_LENGTH(Data->DataLength));
	assert_param(IS_SDMMC_BLOCK_SIZE(Data->DataBlockSize));
	assert_param(IS_SDMMC_TRANSFER_DIR(Data->TransferDir));
	assert_param(IS_SDMMC_TRANSFER_MODE(Data->TransferMode));
	assert_param(IS_SDMMC_DPSM(Data->DPSM));

	/* Set the SDMMC Data TimeOut value */
	SDMMCx->DTIMER = Data->DataTimeOut;

	/* Set the SDMMC DataLength value */
	SDMMCx->DLEN = Data->DataLength;

	/* Set the SDMMC data configuration parameters */
	/* Write to SDMMC DCTRL */
	MODIFY_REG(SDMMCx->DCTRL, DCTRL_CLEAR_MASK, Data->DataBlockSize |
			   Data->TransferDir |
			   Data->TransferMode |
			   Data->SDIO |
			   Data->DPSM);

	return HAL_OK;
}


/**
  * @brief  Returns number of remaining data bytes to be transferred.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval Number of remaining data bytes to be transferred
  */
uint32_t SDMMC_GetDataCounter(SDMMC_TypeDef *SDMMCx)
{
	return (SDMMCx->DCOUNT);
}


/**
  * @brief  Get the FIFO data
  * @param  SDMMCx: Pointer to SDMMC register base
  * @retval Data received
  */
uint32_t SDMMC_GetFIFOCount(SDMMC_TypeDef *SDMMCx)
{
	return (SDMMCx->FIFO);
}


/**
  * @brief  Sets one of the two options of inserting read wait interval.
  * @param  SDMMCx: Pointer to SDMMC register base
  * @param  SDMMC_ReadWaitMode: SDMMC Read Wait operation mode.
  *          This parameter can be:
  *            @arg SDMMC_READ_WAIT_MODE_CLK: Read Wait control by stopping SDMMCCLK
  *            @arg SDMMC_READ_WAIT_MODE_DATA2: Read Wait control using SDMMC_DATA2
  * @retval None
  */
HAL_StatusTypeDef SDMMC_SetSDMMCReadWaitMode(SDMMC_TypeDef *SDMMCx, uint32_t SDMMC_ReadWaitMode)
{
	/* Check the parameters */
	assert_param(IS_SDMMC_READWAIT_MODE(SDMMC_ReadWaitMode));

	/* Set SDMMC read wait mode */
	MODIFY_REG(SDMMCx->DCTRL, SDMMC_DCTRL_RWMOD, SDMMC_ReadWaitMode);

	return HAL_OK;
}


/**
  * @}
  */


/**
  * @brief  Reads block(s) from a specified address in a card. The Data transfer
  *         is managed by DMA mode.
  * @note   This API should be followed by a check on the card state through
  *         HAL_SD_GetCardState().
  * @note   You could also check the DMA transfer process through the SD Rx
  *         interrupt event.
  * @param  hsd: Pointer SD handle
  * @param  pData: Pointer to the buffer that will contain the received data
  * @param  BlockAdd: Block Address from where data is to be read
  * @param  NumberOfBlocks: Number of blocks to read.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t DataSize, uint32_t BlockSize)
{
	SDMMC_DataInitTypeDef config;

	if (NULL == pData)
	{
		hsd->ErrorCode |= HAL_SD_ERROR_PARAM;

		return HAL_ERROR;
	}

	if (hsd->State == HAL_SD_STATE_READY)
	{
		hsd->ErrorCode = HAL_SD_ERROR_NONE;

		hsd->State = HAL_SD_STATE_BUSY;
		hsd_eval.Context = SD_CONTEXT_DMA;

		/* Initialize data control register */
		hsd->Instance->DCTRL = 0U;
		hsd->pRxBuffPtr = pData;
		hsd->TxXferSize = (uint32_t)(((DataSize + (uint16_t) BlockSize - 1) / (uint16_t) BlockSize) * (uint16_t) BlockSize);

		/* Configure the SD DPSM (Data Path State Machine) */
		config.DataTimeOut = SDMMC_DATATIMEOUT;
		config.DataLength = hsd->TxXferSize;
		config.DataBlockSize = sdio_get_blocksize_dctrl(BlockSize);
		config.TransferDir = SDMMC_TRANSFER_DIR_TO_SDMMC;
		config.TransferMode = SDMMC_TRANSFER_MODE_BLOCK;
		config.SDIO = SDMMC_DCTRL_SDIOEN;
		config.DPSM = SDMMC_DPSM_ENABLE;

		/* Enable transfer interrupts */
		__HAL_SD_ENABLE_IT(hsd, (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_RXOVERR | SDMMC_IT_DATAEND | SDMMC_IT_SDIOIT));

		hsd->Instance->IDMACTRL = SDMMC_ENABLE_IDMA_SINGLE_BUFF;
		hsd->Instance->IDMABASE0 = (uint32_t) pData;

		(void)SDMMC_ConfigData(hsd->Instance, &config);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;

	}
}


/**
  * @brief  Writes block(s) to a specified address in a card. The Data transfer
  *         is managed by DMA mode.
  * @note   This API should be followed by a check on the card state through
  *         HAL_SD_GetCardState().
  * @note   You could also check the DMA transfer process through the SD Tx
  *         interrupt event.
  * @param  hsd: Pointer to SD handle
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  BlockAdd: Block Address where data will be written
  * @param  NumberOfBlocks: Number of blocks to write
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t DataSize, uint32_t BlockSize)
{
	SDMMC_DataInitTypeDef config;

	if (NULL == pData)
	{
		hsd->ErrorCode |= HAL_SD_ERROR_PARAM;

		return HAL_ERROR;
	}

	if (hsd->State == HAL_SD_STATE_READY)
	{
		hsd->State = HAL_SD_STATE_BUSY;
		hsd_eval.Context = SD_CONTEXT_DMA;

		/* Initialize data control register */
		hsd->Instance->DCTRL = 0U;
		hsd->pTxBuffPtr = pData;
		hsd->TxXferSize = (uint32_t)(((DataSize + (uint16_t) BlockSize - 1) / (uint16_t) BlockSize) * (uint16_t) BlockSize);

		/* Configure the SD DPSM (Data Path State Machine) */
		config.DataTimeOut = SDMMC_DATATIMEOUT;
		config.DataLength = hsd->TxXferSize;
		config.DataBlockSize = sdio_get_blocksize_dctrl(BlockSize);
		config.TransferDir = SDMMC_TRANSFER_DIR_TO_CARD;
		config.TransferMode = SDMMC_TRANSFER_MODE_BLOCK;
		config.SDIO = SDMMC_DCTRL_SDIOEN;
		config.DPSM = SDMMC_DPSM_ENABLE;

		/* Enable transfer interrupts */
		__HAL_SD_ENABLE_IT(hsd, (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_TXUNDERR | SDMMC_IT_DATAEND | SDMMC_IT_SDIOIT));

		hsd->Instance->IDMACTRL = SDMMC_ENABLE_IDMA_SINGLE_BUFF;
		hsd->Instance->IDMABASE0 = (uint32_t) pData;

		(void)SDMMC_ConfigData(hsd->Instance, &config);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}


/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @retval SD status
  */
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t DataSize, uint32_t BlockSize)
{
	HAL_StatusTypeDef sd_state = HAL_OK;

	/* Write block(s) in DMA transfer mode */
	sd_state = HAL_SD_WriteBlocks_DMA(&hsd_eval, (uint8_t *) pData, DataSize, BlockSize);

	return sd_state;
}


/**
  * @brief  Gets the current SD card data status.
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t BSP_SD_GetCardState(void)
{
	return ((HAL_SD_GetCardState(&hsd_eval) == HAL_SD_CARD_TRANSFER) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}


/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None
  */

void SDMMC1_IRQHandler(void)
{
	HAL_SD_IRQHandler(&hsd_eval);
}


/**
  * @brief  This function handles SD card interrupt request.
  * @param  hsd: Pointer to SD handle
  * @retval None
  */
void HAL_SD_IRQHandler(SD_HandleTypeDef *hsd)
{

	uint32_t errorstate;
	uint32_t context = hsd->Context;

	if (__HAL_SD_GET_FLAG(hsd, SDMMC_IT_CMDSENT | SDMMC_IT_CMDREND) != RESET) {

		__HAL_SD_CLEAR_FLAG(hsd, SDMMC_FLAG_CMDREND | SDMMC_FLAG_CMDSENT);
	}

	/* Check for SDMMC interrupt flags */
	if (__HAL_SD_GET_FLAG(hsd, SDMMC_IT_DATAEND) != RESET)
	{
		__HAL_SD_CLEAR_FLAG(hsd, SDMMC_FLAG_DATAEND);

		__HAL_SD_DISABLE_IT(hsd, SDMMC_IT_DATAEND | SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | \
							SDMMC_IT_TXUNDERR | SDMMC_IT_RXOVERR | SDMMC_IT_TXFIFOHE | \
							SDMMC_IT_RXFIFOHF);

		__HAL_SD_DISABLE_IT(hsd, SDMMC_IT_IDMABTC);
		__SDMMC_CMDTRANS_DISABLE(hsd->Instance);

		if ((context & SD_CONTEXT_DMA) != 0U)
		{
			hsd->Instance->DLEN = 0;
			hsd->Instance->DCTRL = 0;
			hsd->Instance->IDMACTRL = SDMMC_DISABLE_IDMA;

			/* Stop Transfer for Write Multi blocks or Read Multi blocks */
			if (((context & SD_CONTEXT_READ_MULTIPLE_BLOCK) != 0U) || ((context & SD_CONTEXT_WRITE_MULTIPLE_BLOCK) != 0U))
			{
				errorstate = SDMMC_CmdStopTransfer(hsd->Instance);
				if (errorstate != HAL_SD_ERROR_NONE)
				{
					hsd->ErrorCode |= errorstate;
					HAL_SD_ErrorCallback(hsd);
				}
			}

			hsd->State = HAL_SD_STATE_READY;
			hsd->Context = SD_CONTEXT_NONE;
		}
		else
		{
			/* Nothing to do */
		}

		/* release semaphore */
		host_rtos_set_semaphore(&sdio_transfer_finished_semaphore, WICED_TRUE);

	}
	else if (__HAL_SD_GET_FLAG(hsd, SDMMC_IT_TXFIFOHE) != RESET)
	{
		SD_Write_IT(hsd);
	}
	else if (__HAL_SD_GET_FLAG(hsd, SDMMC_IT_RXFIFOHF) != RESET)
	{
		SD_Read_IT(hsd);
	}
	else if (__HAL_SD_GET_FLAG(hsd, SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_RXOVERR | SDMMC_IT_TXUNDERR) != RESET)
	{
		/* Set Error code */
		if (__HAL_SD_GET_FLAG(hsd, SDMMC_IT_DCRCFAIL) != RESET)
		{
			hsd->ErrorCode |= HAL_SD_ERROR_DATA_CRC_FAIL;
		}

		if (__HAL_SD_GET_FLAG(hsd, SDMMC_IT_DTIMEOUT) != RESET)
		{
			hsd->ErrorCode |= HAL_SD_ERROR_DATA_TIMEOUT;
		}

		if (__HAL_SD_GET_FLAG(hsd, SDMMC_IT_RXOVERR) != RESET)
		{
			hsd->ErrorCode |= HAL_SD_ERROR_RX_OVERRUN;
		}

		if (__HAL_SD_GET_FLAG(hsd, SDMMC_IT_TXUNDERR) != RESET)
		{
			hsd->ErrorCode |= HAL_SD_ERROR_TX_UNDERRUN;
		}

		/* Clear All flags */
		__HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_DATA_FLAGS);

		/* Disable all interrupts */
		__HAL_SD_DISABLE_IT(hsd, SDMMC_IT_DATAEND | SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | \
							SDMMC_IT_TXUNDERR | SDMMC_IT_RXOVERR);

		__SDMMC_CMDTRANS_DISABLE(hsd->Instance);

		hsd->Instance->DCTRL |= SDMMC_DCTRL_FIFORST;
		hsd->Instance->CMD |= SDMMC_CMD_CMDSTOP;
		hsd->ErrorCode |= SDMMC_CmdStopTransfer(hsd->Instance);
		hsd->Instance->CMD &= ~(SDMMC_CMD_CMDSTOP);

		__HAL_SD_CLEAR_FLAG(hsd, SDMMC_FLAG_DABORT);

		if ((context & SD_CONTEXT_IT) != 0U)
		{
			/* Set the SD state to ready to be able to start again the process */
			hsd->State = HAL_SD_STATE_READY;
			hsd->Context = SD_CONTEXT_NONE;
			HAL_SD_ErrorCallback(hsd);
		}
		else if ((context & SD_CONTEXT_DMA) != 0U)
		{
			if (hsd->ErrorCode != HAL_SD_ERROR_NONE)
			{
				/* Disable Internal DMA */
				__HAL_SD_DISABLE_IT(hsd, SDMMC_IT_IDMABTC);

				hsd->Instance->IDMACTRL = SDMMC_DISABLE_IDMA;

				/* Set the SD state to ready to be able to start again the process */
				hsd->State = HAL_SD_STATE_READY;

				HAL_SD_ErrorCallback(hsd);
			}
		}
		else
		{
			/* Nothing to do */
		}

		host_rtos_set_semaphore(&sdio_transfer_finished_semaphore, WICED_TRUE);
	}
	else
	{
		/* Nothing to do */
	}

	if (__HAL_SD_GET_FLAG(hsd, SDMMC_STA_SDIOIT) != RESET) {

		/* clear interrupt */
		__HAL_SD_CLEAR_FLAG(hsd, SDMMC_STA_SDIOIT);

	}

	/* anyway, Notigy to WWD thread */
	if (hsd->State == HAL_SD_STATE_READY)

	{
		wwd_thread_notify_irq();
	}
}


/**
* @brief  Return the SD error code
* @param  hsd : Pointer to a SD_HandleTypeDef structure that contains
  *              the configuration information.
* @retval SD Error Code
*/
uint32_t HAL_SD_GetError(SD_HandleTypeDef *hsd)
{
	return hsd->ErrorCode;
}


/**
  * @}
  */

void BSP_SD_MspInit(void)
{
	irq_attach(SDMMCx_IRQn, (xcpt_t)SDMMC1_IRQHandler, NULL);

	up_prioritize_irq(SDMMCx_IRQn, 5);

	up_enable_irq(SDMMCx_IRQn);
}


/**
  * @brief  De-Initializes the SD MSP.
  * @retval None
  */
void BSP_SD_MspDeInit(SD_HandleTypeDef *hsd, void *Params)
{
	up_disable_irq(SDMMCx_IRQn);

	irq_detach(SDMMCx_IRQn);
}


/**
  * @brief BSP SD Abort callback
  * @retval None
  */
void BSP_SD_AbortCallback(void)
{

}


/**
  * @brief BSP Tx Transfer completed callback
  * @retval None
  */
void BSP_SD_WriteCpltCallback(void)
{

}


/**
  * @brief BSP Rx Transfer completed callback
  * @retval None
  */
void BSP_SD_ReadCpltCallback(void)
{

}


/**
  * @brief SD Abort callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{

	BSP_SD_AbortCallback();
}


/**
  * @brief SD error callbacks
  * @param hsd: Pointer SD handle
  * @retval None
  */
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{

	/* Prevent unused argument(s) compilation warning */
	UNUSED(hsd);


	/* NOTE : This function should not be modified, when the callback is needed,
	   the HAL_SD_ErrorCallback can be implemented in the user file
	 */
}



/**
  * @brief Tx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{

	BSP_SD_WriteCpltCallback();
}


/**
  * @brief Rx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{

	BSP_SD_ReadCpltCallback();
}


#define BUFFER_SIZE              32
static const uint32_t aSRC_Buffer[BUFFER_SIZE] = {
	0x01020304, 0x05060708, 0x090A0B0C, 0x0D0E0F10,
	0x11121314, 0x15161718, 0x191A1B1C, 0x1D1E1F20,
	0x21222324, 0x25262728, 0x292A2B2C, 0x2D2E2F30,
	0x31323334, 0x35363738, 0x393A3B3C, 0x3D3E3F40,
	0x41424344, 0x45464748, 0x494A4B4C, 0x4D4E4F50,
	0x51525354, 0x55565758, 0x595A5B5C, 0x5D5E5F60,
	0x61626364, 0x65666768, 0x696A6B6C, 0x6D6E6F70,
	0x71727374, 0x75767778, 0x797A7B7C, 0x7D7E7F80
};


static __IO uint32_t transferErrorDetected;	/* Set to 1 if an error transfer is detected */
static __IO uint32_t transferCompleteDetected;	/* Set to 1 if transfer is correctly completed */

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  DMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void TransferComplete(void)
{

	transferCompleteDetected = 1;

}


/**
  * @brief  DMA conversion error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
static void TransferError(void)
{
	transferErrorDetected = 1;

}


/** @addtogroup SD_Exported_Functions_Group3
 *  @brief   management functions
 *
@verbatim
  ==============================================================================
                      ##### Peripheral Control functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to control the SD card
    operations and get the related information

@endverbatim
  * @{
  */

/**
  * @brief  Returns information the information of the card which are stored on
  *         the CID register.
  * @param  hsd: Pointer to SD handle
  * @param  pCID: Pointer to a HAL_SD_CIDTypedef structure that
  *         contains all CID register parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_GetCardCID(SD_HandleTypeDef *hsd, HAL_SD_CardCIDTypedef *pCID)
{
	pCID->ManufacturerID = (uint8_t)((hsd->CID[0] & 0xFF000000U) >> 24U);

	pCID->OEM_AppliID = (uint16_t)((hsd->CID[0] & 0x00FFFF00U) >> 8U);

	pCID->ProdName1 = (((hsd->CID[0] & 0x000000FFU) << 24U) | ((hsd->CID[1] & 0xFFFFFF00U) >> 8U));

	pCID->ProdName2 = (uint8_t)(hsd->CID[1] & 0x000000FFU);

	pCID->ProdRev = (uint8_t)((hsd->CID[2] & 0xFF000000U) >> 24U);

	pCID->ProdSN = (((hsd->CID[2] & 0x00FFFFFFU) << 8U) | ((hsd->CID[3] & 0xFF000000U) >> 24U));

	pCID->Reserved1 = (uint8_t)((hsd->CID[3] & 0x00F00000U) >> 20U);

	pCID->ManufactDate = (uint16_t)((hsd->CID[3] & 0x000FFF00U) >> 8U);

	pCID->CID_CRC = (uint8_t)((hsd->CID[3] & 0x000000FEU) >> 1U);

	pCID->Reserved2 = 1U;

	return HAL_OK;
}


/**
  * @brief  Returns information the information of the card which are stored on
  *         the CSD register.
  * @param  hsd: Pointer to SD handle
  * @param  pCSD: Pointer to a HAL_SD_CardInfoTypedef structure that
  *         contains all CSD register parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_GetCardCSD(SD_HandleTypeDef *hsd, HAL_SD_CardCSDTypedef *pCSD)
{
	pCSD->CSDStruct = (uint8_t)((hsd->CSD[0] & 0xC0000000U) >> 30U);

	pCSD->SysSpecVersion = (uint8_t)((hsd->CSD[0] & 0x3C000000U) >> 26U);

	pCSD->Reserved1 = (uint8_t)((hsd->CSD[0] & 0x03000000U) >> 24U);

	pCSD->TAAC = (uint8_t)((hsd->CSD[0] & 0x00FF0000U) >> 16U);

	pCSD->NSAC = (uint8_t)((hsd->CSD[0] & 0x0000FF00U) >> 8U);

	pCSD->MaxBusClkFrec = (uint8_t)(hsd->CSD[0] & 0x000000FFU);

	pCSD->CardComdClasses = (uint16_t)((hsd->CSD[1] & 0xFFF00000U) >> 20U);

	pCSD->RdBlockLen = (uint8_t)((hsd->CSD[1] & 0x000F0000U) >> 16U);

	pCSD->PartBlockRead = (uint8_t)((hsd->CSD[1] & 0x00008000U) >> 15U);

	pCSD->WrBlockMisalign = (uint8_t)((hsd->CSD[1] & 0x00004000U) >> 14U);

	pCSD->RdBlockMisalign = (uint8_t)((hsd->CSD[1] & 0x00002000U) >> 13U);

	pCSD->DSRImpl = (uint8_t)((hsd->CSD[1] & 0x00001000U) >> 12U);

	pCSD->Reserved2 = 0U;	/*!< Reserved */

	pCSD->DeviceSize = (((hsd->CSD[1] & 0x000003FFU) << 2U) | ((hsd->CSD[2] & 0xC0000000U) >> 30U));

	pCSD->MaxRdCurrentVDDMin = (uint8_t)((hsd->CSD[2] & 0x38000000U) >> 27U);

	pCSD->MaxRdCurrentVDDMax = (uint8_t)((hsd->CSD[2] & 0x07000000U) >> 24U);

	pCSD->MaxWrCurrentVDDMin = (uint8_t)((hsd->CSD[2] & 0x00E00000U) >> 21U);

	pCSD->MaxWrCurrentVDDMax = (uint8_t)((hsd->CSD[2] & 0x001C0000U) >> 18U);

	pCSD->DeviceSizeMul = (uint8_t)((hsd->CSD[2] & 0x00038000U) >> 15U);

	pCSD->EraseGrSize = (uint8_t)((hsd->CSD[2] & 0x00004000U) >> 14U);

	pCSD->EraseGrMul = (uint8_t)((hsd->CSD[2] & 0x00003F80U) >> 7U);

	pCSD->WrProtectGrSize = (uint8_t)(hsd->CSD[2] & 0x0000007FU);

	pCSD->WrProtectGrEnable = (uint8_t)((hsd->CSD[3] & 0x80000000U) >> 31U);

	pCSD->ManDeflECC = (uint8_t)((hsd->CSD[3] & 0x60000000U) >> 29U);

	pCSD->WrSpeedFact = (uint8_t)((hsd->CSD[3] & 0x1C000000U) >> 26U);

	pCSD->MaxWrBlockLen = (uint8_t)((hsd->CSD[3] & 0x03C00000U) >> 22U);

	pCSD->WriteBlockPaPartial = (uint8_t)((hsd->CSD[3] & 0x00200000U) >> 21U);

	pCSD->Reserved3 = 0;

	pCSD->ContentProtectAppli = (uint8_t)((hsd->CSD[3] & 0x00010000U) >> 16U);

	pCSD->FileFormatGroup = (uint8_t)((hsd->CSD[3] & 0x00008000U) >> 15U);

	pCSD->CopyFlag = (uint8_t)((hsd->CSD[3] & 0x00004000U) >> 14U);

	pCSD->PermWrProtect = (uint8_t)((hsd->CSD[3] & 0x00002000U) >> 13U);

	pCSD->TempWrProtect = (uint8_t)((hsd->CSD[3] & 0x00001000U) >> 12U);

	pCSD->FileFormat = (uint8_t)((hsd->CSD[3] & 0x00000C00U) >> 10U);

	pCSD->ECC = (uint8_t)((hsd->CSD[3] & 0x00000300U) >> 8U);

	pCSD->CSD_CRC = (uint8_t)((hsd->CSD[3] & 0x000000FEU) >> 1U);

	pCSD->Reserved4 = 1;

	return HAL_OK;
}


/**
  * @brief  Enables wide bus operation for the requested card if supported by
  *         card.
  * @param  hsd: Pointer to SD handle
  * @param  WideMode: Specifies the SD card wide bus mode
  *          This parameter can be one of the following values:
  *            @arg SDMMC_BUS_WIDE_8B: 8-bit data transfer
  *            @arg SDMMC_BUS_WIDE_4B: 4-bit data transfer
  *            @arg SDMMC_BUS_WIDE_1B: 1-bit data transfer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_ConfigWideBusOperation(SD_HandleTypeDef *hsd, uint32_t WideMode)
{
	SDMMC_InitTypeDef Init;
	uint32_t errorstate;

	/* Check the parameters */
	assert_param(IS_SDMMC_BUS_WIDE(WideMode));

	/* Change State */
	hsd->State = HAL_SD_STATE_BUSY;
	{
		if (WideMode == SDMMC_BUS_WIDE_8B)
		{
			hsd->ErrorCode |= HAL_SD_ERROR_UNSUPPORTED_FEATURE;
		}
		else if (WideMode == SDMMC_BUS_WIDE_4B)
		{
			errorstate = SD_WideBus_Enable(hsd);

			hsd->ErrorCode |= errorstate;
		}
		else if (WideMode == SDMMC_BUS_WIDE_1B)
		{
			errorstate = SD_WideBus_Disable(hsd);

			hsd->ErrorCode |= errorstate;
		}
		else
		{
			/* WideMode is not a valid argument */
			hsd->ErrorCode |= HAL_SD_ERROR_PARAM;
		}
	}

	if (hsd->ErrorCode != HAL_SD_ERROR_NONE)
	{
		HAL_Delay(10);

		/* Clear all the static flags */
		__HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);
		hsd->State = HAL_SD_STATE_READY;

		return HAL_ERROR;
	}
	else
	{
		Init.ClockEdge = hsd->Init.ClockEdge;
		Init.ClockPowerSave = hsd->Init.ClockPowerSave;
		Init.BusWide = WideMode;
		Init.HardwareFlowControl = hsd->Init.HardwareFlowControl;
		Init.ClockDiv = hsd->Init.ClockDiv;
		(void)SDMMC_Init(hsd->Instance, Init);
	}

	/* Change State */
	hsd->State = HAL_SD_STATE_READY;

	return HAL_OK;
}


/**
  * @brief  Gets the current sd card data state.
  * @param  hsd: pointer to SD handle
  * @retval Card state
  */
HAL_SD_CardStateTypedef HAL_SD_GetCardState(SD_HandleTypeDef *hsd)
{
	uint32_t cardstate;
	uint32_t errorstate;
	uint32_t resp1 = 0;

	errorstate = SD_SendStatus(hsd, &resp1);
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		hsd->ErrorCode |= errorstate;
	}

	cardstate = ((resp1 >> 9U) & 0x0FU);

	return (HAL_SD_CardStateTypedef) cardstate;
}


/**
  * @brief  Abort the current transfer and disable the SD.
  * @param  hsd: pointer to a SD_HandleTypeDef structure that contains
  *                the configuration information for SD module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_Abort(SD_HandleTypeDef *hsd)
{
	HAL_SD_CardStateTypedef CardState;

#if !defined(STM32L4R5xx) && !defined(STM32L4R7xx) && !defined(STM32L4R9xx) && !defined(STM32L4S5xx) && !defined(STM32L4S7xx) && !defined(STM32L4S9xx)
	uint32_t context = hsd->Context;
#endif

	/* DIsable All interrupts */
	__HAL_SD_DISABLE_IT(hsd, SDMMC_IT_DATAEND | SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | \
						SDMMC_IT_TXUNDERR | SDMMC_IT_RXOVERR);

	/* Clear All flags */
	__HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
	/* If IDMA Context, disable Internal DMA */
	hsd->Instance->IDMACTRL = SDMMC_DISABLE_IDMA;
#else /* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */
	CLEAR_BIT(hsd->Instance->DCTRL, SDMMC_DCTRL_DTEN);

	if ((context & SD_CONTEXT_DMA) != 0)
	{
		/* Disable the SD DMA request */
		hsd->Instance->DCTRL &= (uint32_t) ~((uint32_t) SDMMC_DCTRL_DMAEN);

		/* Abort the SD DMA Tx channel */
		if (((context & SD_CONTEXT_WRITE_SINGLE_BLOCK) != 0) || ((context & SD_CONTEXT_WRITE_MULTIPLE_BLOCK) != 0))
		{
			if (HAL_DMA_Abort(hsd->hdmatx) != HAL_OK)
			{
				hsd->ErrorCode |= HAL_SD_ERROR_DMA;
			}
		}
		/* Abort the SD DMA Rx channel */
		else if (((context & SD_CONTEXT_READ_SINGLE_BLOCK) != 0) || ((context & SD_CONTEXT_READ_MULTIPLE_BLOCK) != 0))
		{
			if (HAL_DMA_Abort(hsd->hdmarx) != HAL_OK)
			{
				hsd->ErrorCode |= HAL_SD_ERROR_DMA;
			}
		}
		else
		{
			/* Nothing to do */
		}
	}
#endif	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

	hsd->State = HAL_SD_STATE_READY;
	hsd->Context = SD_CONTEXT_NONE;
	CardState = HAL_SD_GetCardState(hsd);

	if ((CardState == HAL_SD_CARD_RECEIVING) || (CardState == HAL_SD_CARD_SENDING))
	{
		hsd->ErrorCode |= SDMMC_CmdStopTransfer(hsd->Instance);
	}

	if (hsd->ErrorCode != HAL_SD_ERROR_NONE)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}


/**
  * @brief  Abort the current transfer and disable the SD (IT mode).
  * @param  hsd: pointer to a SD_HandleTypeDef structure that contains
  *                the configuration information for SD module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_Abort_IT(SD_HandleTypeDef *hsd)
{
	HAL_SD_CardStateTypedef CardState;

#if !defined(STM32L4R5xx) && !defined(STM32L4R7xx) && !defined(STM32L4R9xx) && !defined(STM32L4S5xx) && !defined(STM32L4S7xx) && !defined(STM32L4S9xx)
	uint32_t context = hsd->Context;

#endif

	/* Disable All interrupts */
	__HAL_SD_DISABLE_IT(hsd, SDMMC_IT_DATAEND | SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | \
						SDMMC_IT_TXUNDERR | SDMMC_IT_RXOVERR);

#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
	/* If IDMA Context, disable Internal DMA */
	hsd->Instance->IDMACTRL = SDMMC_DISABLE_IDMA;

	/* Clear All flags */
	__HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_DATA_FLAGS);

	CardState = HAL_SD_GetCardState(hsd);
	hsd->State = HAL_SD_STATE_READY;

	if ((CardState == HAL_SD_CARD_RECEIVING) || (CardState == HAL_SD_CARD_SENDING))
	{
		hsd->ErrorCode = SDMMC_CmdStopTransfer(hsd->Instance);
	}

	if (hsd->ErrorCode != HAL_SD_ERROR_NONE)
	{
		return HAL_ERROR;
	}
	else
	{
		HAL_SD_AbortCallback(hsd);
	}

#else	/* 
 */
	CLEAR_BIT(hsd->Instance->DCTRL, SDMMC_DCTRL_DTEN);

	if ((context & SD_CONTEXT_DMA) != 0)
	{
		/* Disable the SD DMA request */
		hsd->Instance->DCTRL &= (uint32_t) ~((uint32_t) SDMMC_DCTRL_DMAEN);

		/* Abort the SD DMA Tx channel */
		if (((context & SD_CONTEXT_WRITE_SINGLE_BLOCK) != 0) || ((context & SD_CONTEXT_WRITE_MULTIPLE_BLOCK) != 0))
		{
			hsd->hdmatx->XferAbortCallback = SD_DMATxAbort;
			if (HAL_DMA_Abort_IT(hsd->hdmatx) != HAL_OK)
			{
				hsd->hdmatx = NULL;
			}
		}

		/* Abort the SD DMA Rx channel */
		else if (((context & SD_CONTEXT_READ_SINGLE_BLOCK) != 0) || ((context & SD_CONTEXT_READ_MULTIPLE_BLOCK) != 0))
		{
			hsd->hdmarx->XferAbortCallback = SD_DMARxAbort;
			if (HAL_DMA_Abort_IT(hsd->hdmarx) != HAL_OK)
			{
				hsd->hdmarx = NULL;
			}
		}
		else
		{
			/* Nothing to do */
		}
	}

	/* No transfer ongoing on both DMA channels */
	else
	{
		/* Clear All flags */
		__HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_DATA_FLAGS);

		CardState = HAL_SD_GetCardState(hsd);
		hsd->State = HAL_SD_STATE_READY;
		hsd->Context = SD_CONTEXT_NONE;

		if ((CardState == HAL_SD_CARD_RECEIVING) || (CardState == HAL_SD_CARD_SENDING))
		{
			hsd->ErrorCode = SDMMC_CmdStopTransfer(hsd->Instance);
		}

		if (hsd->ErrorCode != HAL_SD_ERROR_NONE)
		{
			return HAL_ERROR;
		}
		else
		{
			HAL_SD_AbortCallback(hsd);
		}
	}
#endif	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

	return HAL_OK;
}


/**
  * @}
  */


/**
  * @brief  Initializes the sd card.
  * @param  hsd: Pointer to SD handle
  * @retval SD Card error state
  */
static uint32_t SD_InitCard(SD_HandleTypeDef *hsd)
{
	HAL_SD_CardCSDTypedef CSD;
	uint32_t errorstate;
	uint16_t sd_rca = 1;

	/* Check the power State */
	if (SDMMC_GetPowerState(hsd->Instance) == 0U)
	{
		/* Power off */
		return HAL_SD_ERROR_REQUEST_NOT_APPLICABLE;
	}

	/* Send CMD3 SET_REL_ADDR with argument 0 */
	/* SD Card publishes its RCA. */
	HAL_Delay(10);

	errorstate = SDMMC_CmdSetRelAdd(hsd->Instance, &sd_rca);
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		printf("SDMMC_CmdSetRelAdd ERROR!!\r\n");

		HAL_Delay(10);

		return errorstate;
	}

	/* Get the SD card RCA */
	HAL_Delay(10);

	gRelCardAdd = sd_rca;


	/* Get CSD parameters */
	if (HAL_SD_GetCardCSD(hsd, &CSD) != HAL_OK)
	{
		return HAL_SD_ERROR_UNSUPPORTED_FEATURE;
	}

	/* Select the Card */
	HAL_Delay(10);

	errorstate = SDMMC_CmdSelDesel(hsd->Instance, (uint32_t)(((uint32_t) gRelCardAdd) << 16));
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		return errorstate;
	}

#if !defined(STM32L4R5xx) && !defined(STM32L4R7xx) && !defined(STM32L4R9xx) && !defined(STM32L4S5xx) && !defined(STM32L4S7xx) && !defined(STM32L4S9xx)
	/* Configure SDMMC peripheral interface */
	(void)SDMMC_Init(hsd->Instance, hsd->Init);
#endif	/* !STM32L4R5xx && !STM32L4R7xx && !STM32L4R9xx && !STM32L4S5xx && !STM32L4S7xx && !STM32L4S9xx */

	/* All cards are initialized */
	return HAL_SD_ERROR_NONE;
}


/**
  * @brief  Enquires cards about their operating voltage and configures clock
  *         controls and stores SD information that will be needed in future
  *         in the SD handle.
  * @param  hsd: Pointer to SD handle
  * @retval error state
  */
static uint32_t SD_PowerON(SD_HandleTypeDef *hsd)
{
	uint32_t errorstate;

	/* CMD0: GO_IDLE_STATE */
	HAL_Delay(10);

	errorstate = SDMMC_CmdGoIdleState(hsd->Instance);
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		lldbg("CMD0 ERROR\r\n");
		return errorstate;
	}

	// CMD5
	HAL_Delay(10);

	errorstate = SDMMC_Cmd5(hsd->Instance);
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		lldbg("CMD5 ERROR\r\n");
		return errorstate;
	}

	return HAL_SD_ERROR_NONE;
}


/**
  * @brief  Turns the SDMMC output signals off.
  * @param  hsd: Pointer to SD handle
  * @retval None
  */
static void SD_PowerOFF(SD_HandleTypeDef *hsd)
{
	/* Set Power State to OFF */
	(void)SDMMC_PowerState_OFF(hsd->Instance);
}


/**
  * @brief  Returns the current card's status.
  * @param  hsd: Pointer to SD handle
  * @param  pCardStatus: pointer to the buffer that will contain the SD card
  *         status (Card Status register)
  * @retval error state
  */
static uint32_t SD_SendStatus(SD_HandleTypeDef *hsd, uint32_t *pCardStatus)
{
	uint32_t errorstate;

	if (pCardStatus == NULL)
	{
		return HAL_SD_ERROR_PARAM;
	}

	/* Send Status command */
	errorstate = SDMMC_CmdSendStatus(hsd->Instance, (uint32_t)(gRelCardAdd << 16));
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		return errorstate;
	}

	/* Get SD card status */
	*pCardStatus = SDMMC_GetResponse(hsd->Instance, SDMMC_RESP1);

	return HAL_SD_ERROR_NONE;
}


/**
  * @brief  Enables the SDMMC wide bus mode.
  * @param  hsd: pointer to SD handle
  * @retval error state
  */
static uint32_t SD_WideBus_Enable(SD_HandleTypeDef *hsd)
{
	if ((SDMMC_GetResponse(hsd->Instance, SDMMC_RESP1) & SDMMC_CARD_LOCKED) == SDMMC_CARD_LOCKED)
	{
		return HAL_SD_ERROR_LOCK_UNLOCK_FAILED;

	}

	return HAL_SD_ERROR_NONE;
}


/**
  * @brief  Disables the SDMMC wide bus mode.
  * @param  hsd: Pointer to SD handle
  * @retval error state
  */
static uint32_t SD_WideBus_Disable(SD_HandleTypeDef *hsd)
{
	uint32_t scr[2] = {
		0, 0
	};

	uint32_t errorstate;

	if ((SDMMC_GetResponse(hsd->Instance, SDMMC_RESP1) & SDMMC_CARD_LOCKED) == SDMMC_CARD_LOCKED)
	{
		return HAL_SD_ERROR_LOCK_UNLOCK_FAILED;
	}

	/* Get SCR Register */
	errorstate = SD_FindSCR(hsd, scr);
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		return errorstate;
	}

	/* If requested card supports 1 bit mode operation */
	if ((scr[1] & SDMMC_SINGLE_BUS_SUPPORT) != SDMMC_ALLZERO)
	{
		/* Send CMD55 APP_CMD with argument as card's RCA */
		errorstate = SDMMC_CmdAppCommand(hsd->Instance, (uint32_t)(gRelCardAdd << 16));
		if (errorstate != HAL_SD_ERROR_NONE)
		{
			return errorstate;
		}

		/* Send ACMD6 APP_CMD with argument as 0 for single bus mode */
		errorstate = SDMMC_CmdBusWidth(hsd->Instance, 0);
		if (errorstate != HAL_SD_ERROR_NONE)
		{
			return errorstate;
		}

		return HAL_SD_ERROR_NONE;
	}
	else
	{
		return HAL_SD_ERROR_REQUEST_NOT_APPLICABLE;
	}
}



/**
  * @brief  Finds the SD card SCR register value.
  * @param  hsd: Pointer to SD handle
  * @param  pSCR: pointer to the buffer that will contain the SCR value
  * @retval error state
  */
static uint32_t SD_FindSCR(SD_HandleTypeDef *hsd, uint32_t *pSCR)
{
	SDMMC_DataInitTypeDef config;
	uint32_t errorstate;
	uint32_t tickstart = HAL_GetTick();
	uint32_t index = 0;
	uint32_t tempscr[2] = {
		0, 0
	};
	uint32_t *scr = pSCR;

	/* Set Block Size To 8 Bytes */
	errorstate = SDMMC_CmdBlockLength(hsd->Instance, 8);
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		return errorstate;
	}

	/* Send CMD55 APP_CMD with argument as card's RCA */
	errorstate = SDMMC_CmdAppCommand(hsd->Instance, (uint32_t)((gRelCardAdd) << 16));
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		return errorstate;
	}

	config.DataTimeOut = SDMMC_DATATIMEOUT;
	config.DataLength = 8;
	config.DataBlockSize = SDMMC_DATABLOCK_SIZE_8B;
	config.TransferDir = SDMMC_TRANSFER_DIR_TO_SDMMC;
	config.TransferMode = SDMMC_TRANSFER_MODE_BLOCK;
	config.DPSM = SDMMC_DPSM_ENABLE;
	(void)SDMMC_ConfigData(hsd->Instance, &config);

	/* Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
	errorstate = SDMMC_CmdSendSCR(hsd->Instance);
	if (errorstate != HAL_SD_ERROR_NONE)
	{
		return errorstate;
	}

#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
	while (!__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR | SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_DBCKEND | SDMMC_FLAG_DATAEND))
	{
		if ((!__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_RXFIFOE)) && (index == 0U))
		{
			tempscr[0] = SDMMC_ReadFIFO(hsd->Instance);
			tempscr[1] = SDMMC_ReadFIFO(hsd->Instance);
			index++;
		}

		if ((HAL_GetTick() - tickstart) >= SDMMC_DATATIMEOUT)
		{
			return HAL_SD_ERROR_TIMEOUT;
		}
	}
#else	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */
	while (!__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR | SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_DBCKEND))
	{
		if (__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_RXDAVL))
		{
			*(tempscr + index) = SDMMC_ReadFIFO(hsd->Instance);
			index++;
		}

		if ((HAL_GetTick() - tickstart) >= SDMMC_DATATIMEOUT)
		{
			return HAL_SD_ERROR_TIMEOUT;
		}
	}
#endif	/* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

	if (__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_DTIMEOUT))
	{
		__HAL_SD_CLEAR_FLAG(hsd, SDMMC_FLAG_DTIMEOUT);

		return HAL_SD_ERROR_DATA_TIMEOUT;
	}
	else if (__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_DCRCFAIL))
	{
		__HAL_SD_CLEAR_FLAG(hsd, SDMMC_FLAG_DCRCFAIL);

		return HAL_SD_ERROR_DATA_CRC_FAIL;
	}
	else if (__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR))
	{
		__HAL_SD_CLEAR_FLAG(hsd, SDMMC_FLAG_RXOVERR);

		return HAL_SD_ERROR_RX_OVERRUN;
	}
	else
	{
		/* No error flag set */
		/* Clear all the static flags */
		__HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_DATA_FLAGS);

		*scr = (((tempscr[1] & SDMMC_0TO7BITS) << 24) | ((tempscr[1] & SDMMC_8TO15BITS) << 8) | \
				((tempscr[1] & SDMMC_16TO23BITS) >> 8) | ((tempscr[1] & SDMMC_24TO31BITS) >> 24));

		scr++;

		*scr = (((tempscr[0] & SDMMC_0TO7BITS) << 24) | ((tempscr[0] & SDMMC_8TO15BITS) << 8) | \
				((tempscr[0] & SDMMC_16TO23BITS) >> 8) | ((tempscr[0] & SDMMC_24TO31BITS) >> 24));

	}

	return HAL_SD_ERROR_NONE;
}


/**
  * @brief  Wrap up reading in non-blocking mode.
  * @param  hsd: pointer to a SD_HandleTypeDef structure that contains
  *              the configuration information.
  * @retval None
  */
static void SD_Read_IT(SD_HandleTypeDef *hsd)
{
	uint32_t count, data;
	uint8_t *tmp;

	tmp = hsd->pRxBuffPtr;

	/* Read data from SDMMC Rx FIFO */
	for (count = 0U; count < 8U; count++)
	{
		data = SDMMC_ReadFIFO(hsd->Instance);
		*tmp = (uint8_t)(data & 0xFFU);
		tmp++;
		*tmp = (uint8_t)((data >> 8U) & 0xFFU);
		tmp++;
		*tmp = (uint8_t)((data >> 16U) & 0xFFU);
		tmp++;
		*tmp = (uint8_t)((data >> 24U) & 0xFFU);
		tmp++;
	}

	hsd->pRxBuffPtr = tmp;
}


/**
  * @brief  Wrap up writing in non-blocking mode.
  * @param  hsd: pointer to a SD_HandleTypeDef structure that contains
  *              the configuration information.
  * @retval None
  */
static void SD_Write_IT(SD_HandleTypeDef *hsd)
{
	uint32_t count, data;
	uint8_t *tmp;

	tmp = hsd->pTxBuffPtr;

	/* Write data to SDMMC Tx FIFO */
	for (count = 0U; count < 8U; count++)
	{
		data = (uint32_t)(*tmp);
		tmp++;
		data |= ((uint32_t)(*tmp) << 8U);
		tmp++;
		data |= ((uint32_t)(*tmp) << 16U);
		tmp++;
		data |= ((uint32_t)(*tmp) << 24U);
		tmp++;
		(void)SDMMC_WriteFIFO(hsd->Instance, &data);
	}

	hsd->pTxBuffPtr = tmp;
}


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
