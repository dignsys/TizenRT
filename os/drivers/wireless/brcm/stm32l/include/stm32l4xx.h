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

/**
  ******************************************************************************
  * @file    stm32l4xx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32L4xx Device Peripheral Access Layer Header File.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32L4xx device used in the target application
  *              - To use or not the peripheral’s drivers in application code(i.e.
  *                code will be based on direct access to peripheral’s registers
  *                rather than drivers API), this option is controlled by
  *                "#define USE_HAL_DRIVER"
  *
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

/** @addtogroup stm32l4xx
  * @{
  */

#ifndef __STM32L4xx_H
#define __STM32L4xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#define STM32L4R5xx   /*!< STM32L4R5xx Devices */

/** @addtogroup Device_Included
  * @{
  */

#if defined(STM32L4R5xx)
  #include "stm32l4r5xx.h"
#else
 #error "Please select first the target STM32L4xx device used in your application (in stm32l4xx.h file)"
#endif

/**
  * @}
  */

#define assert_param(expr)	assert(expr)

/** @addtogroup Exported_types
  * @{
  */
typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

/** @addtogroup Exported_macros
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

/**
  * @brief  RCC PLL configuration structure definition
  */
typedef struct
{
  uint32_t PLLState;   /*!< The new state of the PLL.
                            This parameter can be a value of @ref RCC_PLL_Config                      */

  uint32_t PLLSource;  /*!< RCC_PLLSource: PLL entry clock source.
                            This parameter must be a value of @ref RCC_PLL_Clock_Source               */

  uint32_t PLLM;       /*!< PLLM: Division factor for PLL VCO input clock.
                            This parameter must be a number between Min_Data = 1 and Max_Data = 16 on STM32L4Rx/STM32L4Sx devices.
                            This parameter must be a number between Min_Data = 1 and Max_Data = 8 on the other devices */

  uint32_t PLLN;       /*!< PLLN: Multiplication factor for PLL VCO output clock.
                            This parameter must be a number between Min_Data = 8 and Max_Data = 86    */

#if defined(RCC_PLLP_SUPPORT)
  uint32_t PLLP;       /*!< PLLP: Division factor for SAI clock.
                            This parameter must be a value of @ref RCC_PLLP_Clock_Divider             */
#endif /* RCC_PLLP_SUPPORT */

  uint32_t PLLQ;       /*!< PLLQ: Division factor for SDMMC1, RNG and USB clocks.
                            This parameter must be a value of @ref RCC_PLLQ_Clock_Divider             */

  uint32_t PLLR;       /*!< PLLR: Division for the main system clock.
                            User have to set the PLLR parameter correctly to not exceed max frequency 120MHZ
                            on STM32L4Rx/STM32L4Sx devices else 80MHz on the other devices.
                            This parameter must be a value of @ref RCC_PLLR_Clock_Divider             */

}RCC_PLLInitTypeDef;

/**
  * @brief  RCC Internal/External Oscillator (HSE, HSI, MSI, LSE and LSI) configuration structure definition
  */
typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                      This parameter can be a value of @ref RCC_Oscillator_Type                   */

  uint32_t HSEState;             /*!< The new state of the HSE.
                                      This parameter can be a value of @ref RCC_HSE_Config                        */

  uint32_t LSEState;             /*!< The new state of the LSE.
                                      This parameter can be a value of @ref RCC_LSE_Config                        */

  uint32_t HSIState;             /*!< The new state of the HSI.
                                      This parameter can be a value of @ref RCC_HSI_Config                        */

  uint32_t HSICalibrationValue;  /*!< The calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                      This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F on STM32L43x/STM32L44x/STM32L47x/STM32L48x devices.
                                      This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x7F on the other devices */

  uint32_t LSIState;             /*!< The new state of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Config                        */
#if defined(RCC_CSR_LSIPREDIV)

  uint32_t LSIDiv;               /*!< The division factor of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Div                           */
#endif /* RCC_CSR_LSIPREDIV */

  uint32_t MSIState;             /*!< The new state of the MSI.
                                      This parameter can be a value of @ref RCC_MSI_Config */

  uint32_t MSICalibrationValue;  /*!< The calibration trimming value (default is RCC_MSICALIBRATION_DEFAULT).
                                      This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF */

  uint32_t MSIClockRange;        /*!< The MSI frequency range.
                                      This parameter can be a value of @ref RCC_MSI_Clock_Range  */

  uint32_t HSI48State;             /*!< The new state of the HSI48 (only applicable to STM32L43x/STM32L44x/STM32L49x/STM32L4Ax devices).
                                        This parameter can be a value of @ref RCC_HSI48_Config */

  RCC_PLLInitTypeDef PLL;        /*!< Main PLL structure parameters                                               */

}RCC_OscInitTypeDef;

/**
  * @brief  RCC System, AHB and APB busses clock configuration structure definition
  */
typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured.
                                       This parameter can be a value of @ref RCC_System_Clock_Type      */

  uint32_t SYSCLKSource;          /*!< The clock source used as system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_System_Clock_Source    */

  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source       */

  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

}RCC_ClkInitTypeDef;


/**
  * @brief  RCC extended clocks structure definition
  */
typedef struct
{
  uint32_t PeriphClockSelection;   /*!< The Extended Clock to be configured.
                                        This parameter can be a value of @ref RCCEx_Periph_Clock_Selection */
  uint32_t Sdmmc1ClockSelection;   /*!< Specifies SDMMC1 clock source (warning: same source for USB and RNG).
                                        This parameter can be a value of @ref RCCEx_SDMMC1_Clock_Source */
}RCC_PeriphCLKInitTypeDef;


/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL). 
  */
#define HSI_VALUE    ((uint32_t)16000000U) /*!< Value of the Internal oscillator in Hz*/

/** @defgroup RCC_Oscillator_Type Oscillator Type
  * @{
  */
#define RCC_OSCILLATORTYPE_NONE        0x00000000U   /*!< Oscillator configuration unchanged */
#define RCC_OSCILLATORTYPE_HSE         0x00000001U   /*!< HSE to configure */
#define RCC_OSCILLATORTYPE_HSI         0x00000002U   /*!< HSI to configure */
#define RCC_OSCILLATORTYPE_LSE         0x00000004U   /*!< LSE to configure */
#define RCC_OSCILLATORTYPE_LSI         0x00000008U   /*!< LSI to configure */
#define RCC_OSCILLATORTYPE_MSI         0x00000010U   /*!< MSI to configure */
#if defined(RCC_HSI48_SUPPORT)
#define RCC_OSCILLATORTYPE_HSI48       0x00000020U   /*!< HSI48 to configure */
#endif /* RCC_HSI48_SUPPORT */

/** @defgroup RCCEx_SDMMC1_Clock_Source SDMMC1 Clock Source
  * @{
  */
#if defined(RCC_HSI48_SUPPORT)
#define RCC_SDMMC1CLKSOURCE_HSI48      0x00000000U  /*!< HSI48 clock selected as SDMMC1 clock          */
#else
#define RCC_SDMMC1CLKSOURCE_NONE       0x00000000U  /*!< No clock selected as SDMMC1 clock             */
#endif /* RCC_HSI48_SUPPORT */
#define RCC_SDMMC1CLKSOURCE_PLLSAI1    RCC_CCIPR_CLK48SEL_0     /*!< PLLSAI1 "Q" clock selected as SDMMC1 clock    */
#define RCC_SDMMC1CLKSOURCE_PLL        RCC_CCIPR_CLK48SEL_1     /*!< PLL "Q" clock selected as SDMMC1 clock        */
#define RCC_SDMMC1CLKSOURCE_MSI        RCC_CCIPR_CLK48SEL       /*!< MSI clock selected as SDMMC1 clock            */
#if defined(RCC_CCIPR2_SDMMCSEL)
#define RCC_SDMMC1CLKSOURCE_PLLP       RCC_CCIPR2_SDMMCSEL      /*!< PLL "P" clock selected as SDMMC1 kernel clock */
#endif /* RCC_CCIPR2_SDMMCSEL */
/**
  * @}
  */

/** @defgroup RCC_HSI48_Config HSI48 Config
  * @{
  */
#define RCC_HSI48_OFF                  0x00000000U       /*!< HSI48 clock deactivation */
#define RCC_HSI48_ON                   RCC_CRRCR_HSI48ON /*!< HSI48 clock activation */
/**
  * @}
  */

/** @defgroup RCC_PLL_Config PLL Config
  * @{
  */
#define RCC_PLL_NONE                   0x00000000U   /*!< PLL configuration unchanged */
#define RCC_PLL_OFF                    0x00000001U   /*!< PLL deactivation */
#define RCC_PLL_ON                     0x00000002U   /*!< PLL activation */

/** @defgroup RCCEx_Periph_Clock_Selection Periph Clock Selection
  * @{
  */
#define RCC_PERIPHCLK_SDMMC1           0x00080000U

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32L4xx_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
