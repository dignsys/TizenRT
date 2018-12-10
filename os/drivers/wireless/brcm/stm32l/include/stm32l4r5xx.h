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
  * @file    stm32l4r5xx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32L4R5xx Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral’s registers hardware
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

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup stm32l4r5xx
  * @{
  */

#ifndef __STM32L4R5xx_H
#define __STM32L4R5xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

/* Exported macros -----------------------------------------------------------*/
#define HAL_MAX_DELAY      0xFFFFFFFFU

#define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) == (BIT))
#define HAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)

#define	    __I     volatile const       /*!< Defines 'read only' permissions */
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */
#define     __IM    volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM    volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM   volatile            /*! Defines 'read / write' structure member permissions */
typedef unsigned char uint8_t;
typedef unsigned int  uint32_t;

extern uint32_t SystemCoreClock;            /*!< System Clock Frequency (Core Clock) */
extern const uint8_t  AHBPrescTable[16];    /*!< AHB prescalers table values */
extern const uint8_t  APBPrescTable[8];     /*!< APB prescalers table values */
extern const uint32_t MSIRangeTable[12];    /*!< MSI ranges table values     */

typedef struct
{
  __IOM uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  __IOM uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  __IOM uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  __IM  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;

#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */
#ifdef CONFIG_ARCH_HAVE_FPU
#define __FPU_USED         1U
#else
#define __FPU_USED         0U
#endif

/**
  * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
   */
#define __CM4_REV                 0x0001  /*!< Cortex-M4 revision r0p1                       */
#define __MPU_PRESENT             1       /*!< STM32L4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4       /*!< STM32L4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1       /*!< FPU present                                   */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32L4XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
  SDMMC1_IRQn                 = (16+49),     /*!< SDMMC1 global Interrupt                                           */
} IRQn_Type;

/**
  * @}
  */

//#include "core_cm4.h"             /* Cortex-M4 processor and core peripherals */
//#include "system_stm32l4xx.h"
//#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Controller Area Network TxMailBox
  */

typedef struct
{
  __IO uint32_t TIR;  /*!< CAN TX mailbox identifier register */
  __IO uint32_t TDTR; /*!< CAN mailbox data length control and time stamp register */
  __IO uint32_t TDLR; /*!< CAN mailbox data low register */
  __IO uint32_t TDHR; /*!< CAN mailbox data high register */
} CAN_TxMailBox_TypeDef;

/**
  * @brief Controller Area Network FIFOMailBox
  */

typedef struct
{
  __IO uint32_t RIR;  /*!< CAN receive FIFO mailbox identifier register */
  __IO uint32_t RDTR; /*!< CAN receive FIFO mailbox data length control and time stamp register */
  __IO uint32_t RDLR; /*!< CAN receive FIFO mailbox data low register */
  __IO uint32_t RDHR; /*!< CAN receive FIFO mailbox data high register */
} CAN_FIFOMailBox_TypeDef;

/**
  * @brief Controller Area Network FilterRegister
  */

typedef struct
{
  __IO uint32_t FR1; /*!< CAN Filter bank register 1 */
  __IO uint32_t FR2; /*!< CAN Filter bank register 1 */
} CAN_FilterRegister_TypeDef;

/**
  * @brief Secure digital input/output Interface
  */

typedef struct
{
  __IO uint32_t POWER;          /*!< SDMMC power control register,             Address offset: 0x00 */
  __IO uint32_t CLKCR;          /*!< SDMMC clock control register,             Address offset: 0x04 */
  __IO uint32_t ARG;            /*!< SDMMC argument register,                  Address offset: 0x08 */
  __IO uint32_t CMD;            /*!< SDMMC command register,                   Address offset: 0x0C */
  __I uint32_t  RESPCMD;        /*!< SDMMC command response register,          Address offset: 0x10 */
  __I uint32_t  RESP1;          /*!< SDMMC response 1 register,                Address offset: 0x14 */
  __I uint32_t  RESP2;          /*!< SDMMC response 2 register,                Address offset: 0x18 */
  __I uint32_t  RESP3;          /*!< SDMMC response 3 register,                Address offset: 0x1C */
  __I uint32_t  RESP4;          /*!< SDMMC response 4 register,                Address offset: 0x20 */
  __IO uint32_t DTIMER;         /*!< SDMMC data timer register,                Address offset: 0x24 */
  __IO uint32_t DLEN;           /*!< SDMMC data length register,               Address offset: 0x28 */
  __IO uint32_t DCTRL;          /*!< SDMMC data control register,              Address offset: 0x2C */
  __I uint32_t  DCOUNT;         /*!< SDMMC data counter register,              Address offset: 0x30 */
  __I uint32_t  STA;            /*!< SDMMC status register,                    Address offset: 0x34 */
  __IO uint32_t ICR;            /*!< SDMMC interrupt clear register,           Address offset: 0x38 */
  __IO uint32_t MASK;           /*!< SDMMC mask register,                      Address offset: 0x3C */
  __IO uint32_t ACKTIME;        /*!< SDMMC Acknowledgement timer register,     Address offset: 0x40 */
  uint32_t      RESERVED0[3];   /*!< Reserved, 0x44 - 0x4C - 0x4C                                   */
  __IO uint32_t IDMACTRL;       /*!< SDMMC DMA control register,               Address offset: 0x50 */
  __IO uint32_t IDMABSIZE;      /*!< SDMMC DMA buffer size register,           Address offset: 0x54 */
  __IO uint32_t IDMABASE0;      /*!< SDMMC DMA buffer 0 base address register, Address offset: 0x58 */
  __IO uint32_t IDMABASE1;      /*!< SDMMC DMA buffer 1 base address register, Address offset: 0x5C */
  uint32_t      RESERVED1[8];   /*!< Reserved, 0x60-0x7C                                            */
  __IO uint32_t FIFO;           /*!< SDMMC data FIFO register,                 Address offset: 0x80 */
} SDMMC_TypeDef;

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define FLASH_BASE            (0x08000000UL) /*!< FLASH(up to 2 MB) base address */
#define SRAM1_BASE            (0x20000000UL) /*!< SRAM1(up to 192 KB) base address */
#define SRAM2_BASE            (0x10000000UL) /*!< SRAM2(64 KB) base address */
#define SRAM3_BASE            (0x20040000UL) /*!< SRAM3(384 KB) base address */
#define PERIPH_BASE           (0x40000000UL) /*!< Peripheral base address */
#define FMC_BASE              (0x60000000UL) /*!< FMC base address */
#define OCTOSPI1_BASE         (0x90000000UL) /*!< OCTOSPI1 memories accessible over AHB base address */
#define OCTOSPI2_BASE         (0x70000000UL) /*!< OCTOSPI2 memories accessible over AHB base address */

#define FMC_R_BASE            (0xA0000000UL) /*!< FMC  control registers base address */
#define OCTOSPI1_R_BASE       (0xA0001000UL) /*!< OCTOSPI1 control registers base address */
#define OCTOSPI2_R_BASE       (0xA0001400UL) /*!< OCTOSPI2 control registers base address */
#define SRAM1_BB_BASE         (0x22000000UL) /*!< SRAM1(96 KB) base address in the bit-band region */
#define PERIPH_BB_BASE        (0x42000000UL) /*!< Peripheral base address in the bit-band region */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE

#define SRAM1_SIZE_MAX        (0x00030000UL) /*!< maximum SRAM1 size (up to 192 KBytes) */
#define SRAM2_SIZE            (0x00010000UL) /*!< SRAM2 size (64 KBytes) */
#define SRAM3_SIZE            (0x00060000UL) /*!< SRAM3 size (384 KBytes) */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000UL)

/*!< AHB2 peripherals */
#define SDMMC1_BASE           (AHB2PERIPH_BASE + 0x08062400UL)

/*!< FMC Banks registers base  address */
#define FMC_Bank1_R_BASE      (FMC_R_BASE + 0x0000UL)
#define FMC_Bank1E_R_BASE     (FMC_R_BASE + 0x0104UL)
#define FMC_Bank3_R_BASE      (FMC_R_BASE + 0x0080UL)

/* Debug MCU registers base address */
#define DBGMCU_BASE           (0xE0042000UL)

#define SDMMC1              ((SDMMC_TypeDef *) SDMMC1_BASE)

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                           SDMMC Interface                                  */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SDMMC_POWER register  ******************/
#define SDMMC_POWER_PWRCTRL_Pos         (0U)
#define SDMMC_POWER_PWRCTRL_Msk         (0x3UL << SDMMC_POWER_PWRCTRL_Pos)     /*!< 0x00000003 */
#define SDMMC_POWER_PWRCTRL             SDMMC_POWER_PWRCTRL_Msk                /*!<PWRCTRL[1:0] bits (Power supply control bits) */
#define SDMMC_POWER_PWRCTRL_0           (0x1UL << SDMMC_POWER_PWRCTRL_Pos)     /*!< 0x00000001 */
#define SDMMC_POWER_PWRCTRL_1           (0x2UL << SDMMC_POWER_PWRCTRL_Pos)     /*!< 0x00000002 */
#define SDMMC_POWER_VSWITCH_Pos         (2U)
#define SDMMC_POWER_VSWITCH_Msk         (0x1UL << SDMMC_POWER_VSWITCH_Pos)     /*!< 0x00000004 */
#define SDMMC_POWER_VSWITCH             SDMMC_POWER_VSWITCH_Pos                /*!<Voltage switch sequence start */
#define SDMMC_POWER_VSWITCHEN_Pos       (3U)
#define SDMMC_POWER_VSWITCHEN_Msk       (0x1UL << SDMMC_POWER_VSWITCHEN_Pos)   /*!< 0x00000008 */
#define SDMMC_POWER_VSWITCHEN           SDMMC_POWER_VSWITCHEN_Pos              /*!<Voltage switch procedure enable */
#define SDMMC_POWER_DIRPOL_Pos          (4U)
#define SDMMC_POWER_DIRPOL_Msk          (0x1UL << SDMMC_POWER_DIRPOL_Pos)      /*!< 0x00000010 */
#define SDMMC_POWER_DIRPOL              SDMMC_POWER_DIRPOL_Pos                 /*!<Data and Command direction signals polarity selection */

/******************  Bit definition for SDMMC_CLKCR register  ******************/
#define SDMMC_CLKCR_CLKDIV_Pos          (0U)
#define SDMMC_CLKCR_CLKDIV_Msk          (0x3FFUL << SDMMC_CLKCR_CLKDIV_Pos)    /*!< 0x000003FF */
#define SDMMC_CLKCR_CLKDIV              SDMMC_CLKCR_CLKDIV_Msk                 /*!<Clock divide factor             */
#define SDMMC_CLKCR_PWRSAV_Pos          (12U)
#define SDMMC_CLKCR_PWRSAV_Msk          (0x1UL << SDMMC_CLKCR_PWRSAV_Pos)      /*!< 0x00001000 */
#define SDMMC_CLKCR_PWRSAV              SDMMC_CLKCR_PWRSAV_Msk                 /*!<Power saving configuration bit  */

#define SDMMC_CLKCR_WIDBUS_Pos          (14U)
#define SDMMC_CLKCR_WIDBUS_Msk          (0x3UL << SDMMC_CLKCR_WIDBUS_Pos)      /*!< 0x0000C000 */
#define SDMMC_CLKCR_WIDBUS              SDMMC_CLKCR_WIDBUS_Msk                 /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define SDMMC_CLKCR_WIDBUS_0            (0x1UL << SDMMC_CLKCR_WIDBUS_Pos)      /*!< 0x00000800 */
#define SDMMC_CLKCR_WIDBUS_1            (0x2UL << SDMMC_CLKCR_WIDBUS_Pos)      /*!< 0x00001000 */

#define SDMMC_CLKCR_NEGEDGE_Pos         (16U)
#define SDMMC_CLKCR_NEGEDGE_Msk         (0x1UL << SDMMC_CLKCR_NEGEDGE_Pos)     /*!< 0x00010000 */
#define SDMMC_CLKCR_NEGEDGE             SDMMC_CLKCR_NEGEDGE_Msk                /*!<SDMMC_CK dephasing selection bit */
#define SDMMC_CLKCR_HWFC_EN_Pos         (17U)
#define SDMMC_CLKCR_HWFC_EN_Msk         (0x1UL << SDMMC_CLKCR_HWFC_EN_Pos)     /*!< 0x00020000 */
#define SDMMC_CLKCR_HWFC_EN             SDMMC_CLKCR_HWFC_EN_Msk                /*!<HW Flow Control enable          */
#define SDMMC_CLKCR_DDR_Pos             (18U)
#define SDMMC_CLKCR_DDR_Msk             (0x1UL << SDMMC_CLKCR_DDR_Pos)         /*!< 0x00040000 */
#define SDMMC_CLKCR_DDR                 SDMMC_CLKCR_DDR_Msk                    /*!<Data rate signaling selection    */
#define SDMMC_CLKCR_BUSSPEED_Pos        (19U)
#define SDMMC_CLKCR_BUSSPEED_Msk        (0x1UL << SDMMC_CLKCR_BUSSPEED_Pos)    /*!< 0x00080000 */
#define SDMMC_CLKCR_BUSSPEED            SDMMC_CLKCR_BUSSPEED_Msk               /*!<Bus speed mode selection         */

#define SDMMC_CLKCR_SELCLKRX_Pos        (20U)
#define SDMMC_CLKCR_SELCLKRX_Msk        (0x3UL << SDMMC_CLKCR_SELCLKRX_Pos)    /*!< 0x00030000 */
#define SDMMC_CLKCR_SELCLKRX            SDMMC_CLKCR_SELCLKRX_Msk               /*!<SELCLKRX[1:0] bits (Receive clock selection) */
#define SDMMC_CLKCR_SELCLKRX_0          (0x1UL << SDMMC_CLKCR_SELCLKRX_Pos)    /*!< 0x00010000 */
#define SDMMC_CLKCR_SELCLKRX_1          (0x2UL << SDMMC_CLKCR_SELCLKRX_Pos)    /*!< 0x00020000 */

/*******************  Bit definition for SDMMC_ARG register  *******************/
#define SDMMC_ARG_CMDARG_Pos            (0U)
#define SDMMC_ARG_CMDARG_Msk            (0xFFFFFFFFUL << SDMMC_ARG_CMDARG_Pos) /*!< 0xFFFFFFFF */
#define SDMMC_ARG_CMDARG                SDMMC_ARG_CMDARG_Msk                   /*!<Command argument */

/*******************  Bit definition for SDMMC_CMD register  *******************/
#define SDMMC_CMD_CMDINDEX_Pos          (0U)
#define SDMMC_CMD_CMDINDEX_Msk          (0x3FUL << SDMMC_CMD_CMDINDEX_Pos)     /*!< 0x0000003F */
#define SDMMC_CMD_CMDINDEX              SDMMC_CMD_CMDINDEX_Msk                 /*!<Command Index                               */
#define SDMMC_CMD_CMDTRANS_Pos          (6U)
#define SDMMC_CMD_CMDTRANS_Msk          (0x1UL << SDMMC_CMD_CMDTRANS_Pos)      /*!< 0x00000040 */
#define SDMMC_CMD_CMDTRANS              SDMMC_CMD_CMDTRANS_Msk                 /*!<CPSM Treats command as a Data Transfer      */
#define SDMMC_CMD_CMDSTOP_Pos           (7U)
#define SDMMC_CMD_CMDSTOP_Msk           (0x1UL << SDMMC_CMD_CMDSTOP_Pos)       /*!< 0x00000080 */
#define SDMMC_CMD_CMDSTOP               SDMMC_CMD_CMDSTOP_Msk                  /*!<CPSM Treats command as a Stop               */

#define SDMMC_CMD_WAITRESP_Pos          (8U)
#define SDMMC_CMD_WAITRESP_Msk          (0x3UL << SDMMC_CMD_WAITRESP_Pos)      /*!< 0x00000300 */
#define SDMMC_CMD_WAITRESP              SDMMC_CMD_WAITRESP_Msk                 /*!<WAITRESP[1:0] bits (Wait for response bits) */
#define SDMMC_CMD_WAITRESP_0            (0x1UL << SDMMC_CMD_WAITRESP_Pos)      /*!< 0x00000100 */
#define SDMMC_CMD_WAITRESP_1            (0x2UL << SDMMC_CMD_WAITRESP_Pos)      /*!< 0x00000200 */

#define SDMMC_CMD_WAITINT_Pos           (10U)
#define SDMMC_CMD_WAITINT_Msk           (0x1UL << SDMMC_CMD_WAITINT_Pos)       /*!< 0x00000400 */
#define SDMMC_CMD_WAITINT               SDMMC_CMD_WAITINT_Msk                  /*!<CPSM Waits for Interrupt Request                               */
#define SDMMC_CMD_WAITPEND_Pos          (11U)
#define SDMMC_CMD_WAITPEND_Msk          (0x1UL << SDMMC_CMD_WAITPEND_Pos)      /*!< 0x00000800 */
#define SDMMC_CMD_WAITPEND              SDMMC_CMD_WAITPEND_Msk                 /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define SDMMC_CMD_CPSMEN_Pos            (12U)
#define SDMMC_CMD_CPSMEN_Msk            (0x1UL << SDMMC_CMD_CPSMEN_Pos)        /*!< 0x00001000 */
#define SDMMC_CMD_CPSMEN                SDMMC_CMD_CPSMEN_Msk                   /*!<Command path state machine (CPSM) Enable bit                   */
#define SDMMC_CMD_DTHOLD_Pos            (13U)
#define SDMMC_CMD_DTHOLD_Msk            (0x1UL << SDMMC_CMD_DTHOLD_Pos)        /*!< 0x00002000 */
#define SDMMC_CMD_DTHOLD                SDMMC_CMD_DTHOLD_Msk                   /*!<Hold new data block transmission and reception in the DPSM     */
#define SDMMC_CMD_BOOTMODE_Pos          (14U)
#define SDMMC_CMD_BOOTMODE_Msk          (0x1UL << SDMMC_CMD_BOOTMODE_Pos)      /*!< 0x00004000 */
#define SDMMC_CMD_BOOTMODE              SDMMC_CMD_BOOTMODE_Msk                 /*!<Boot mode                                                      */
#define SDMMC_CMD_BOOTEN_Pos            (15U)
#define SDMMC_CMD_BOOTEN_Msk            (0x1UL << SDMMC_CMD_BOOTEN_Pos)        /*!< 0x00008000 */
#define SDMMC_CMD_BOOTEN                SDMMC_CMD_BOOTEN_Msk                   /*!<Enable Boot mode procedure                                     */
#define SDMMC_CMD_CMDSUSPEND_Pos        (16U)
#define SDMMC_CMD_CMDSUSPEND_Msk        (0x1UL << SDMMC_CMD_CMDSUSPEND_Pos)    /*!< 0x00010000 */
#define SDMMC_CMD_CMDSUSPEND            SDMMC_CMD_CMDSUSPEND_Msk               /*!<CPSM treats command as a Suspend or Resume command             */

/*****************  Bit definition for SDMMC_RESPCMD register  *****************/
#define SDMMC_RESPCMD_RESPCMD_Pos       (0U)
#define SDMMC_RESPCMD_RESPCMD_Msk       (0x3FUL << SDMMC_RESPCMD_RESPCMD_Pos)  /*!< 0x0000003F */
#define SDMMC_RESPCMD_RESPCMD           SDMMC_RESPCMD_RESPCMD_Msk              /*!<Response command index */

/******************  Bit definition for SDMMC_RESP1 register  ******************/
#define SDMMC_RESP1_CARDSTATUS1_Pos     (0U)
#define SDMMC_RESP1_CARDSTATUS1_Msk     (0xFFFFFFFFUL << SDMMC_RESP1_CARDSTATUS1_Pos) /*!< 0xFFFFFFFF */
#define SDMMC_RESP1_CARDSTATUS1         SDMMC_RESP1_CARDSTATUS1_Msk            /*!<Card Status */

/******************  Bit definition for SDMMC_RESP2 register  ******************/
#define SDMMC_RESP2_CARDSTATUS2_Pos     (0U)
#define SDMMC_RESP2_CARDSTATUS2_Msk     (0xFFFFFFFFUL << SDMMC_RESP2_CARDSTATUS2_Pos) /*!< 0xFFFFFFFF */
#define SDMMC_RESP2_CARDSTATUS2         SDMMC_RESP2_CARDSTATUS2_Msk            /*!<Card Status */

/******************  Bit definition for SDMMC_RESP3 register  ******************/
#define SDMMC_RESP3_CARDSTATUS3_Pos     (0U)
#define SDMMC_RESP3_CARDSTATUS3_Msk     (0xFFFFFFFFUL << SDMMC_RESP3_CARDSTATUS3_Pos) /*!< 0xFFFFFFFF */
#define SDMMC_RESP3_CARDSTATUS3         SDMMC_RESP3_CARDSTATUS3_Msk            /*!<Card Status */

/******************  Bit definition for SDMMC_RESP4 register  ******************/
#define SDMMC_RESP4_CARDSTATUS4_Pos     (0U)
#define SDMMC_RESP4_CARDSTATUS4_Msk     (0xFFFFFFFFUL << SDMMC_RESP4_CARDSTATUS4_Pos) /*!< 0xFFFFFFFF */
#define SDMMC_RESP4_CARDSTATUS4         SDMMC_RESP4_CARDSTATUS4_Msk            /*!<Card Status */

/******************  Bit definition for SDMMC_DTIMER register  *****************/
#define SDMMC_DTIMER_DATATIME_Pos       (0U)
#define SDMMC_DTIMER_DATATIME_Msk       (0xFFFFFFFFUL << SDMMC_DTIMER_DATATIME_Pos) /*!< 0xFFFFFFFF */
#define SDMMC_DTIMER_DATATIME           SDMMC_DTIMER_DATATIME_Msk              /*!<Data timeout period. */

/******************  Bit definition for SDMMC_DLEN register  *******************/
#define SDMMC_DLEN_DATALENGTH_Pos       (0U)
#define SDMMC_DLEN_DATALENGTH_Msk       (0x1FFFFFFUL << SDMMC_DLEN_DATALENGTH_Pos) /*!< 0x01FFFFFF */
#define SDMMC_DLEN_DATALENGTH           SDMMC_DLEN_DATALENGTH_Msk              /*!<Data length value    */

/******************  Bit definition for SDMMC_DCTRL register  ******************/
#define SDMMC_DCTRL_DTEN_Pos            (0U)
#define SDMMC_DCTRL_DTEN_Msk            (0x1UL << SDMMC_DCTRL_DTEN_Pos)        /*!< 0x00000001 */
#define SDMMC_DCTRL_DTEN                SDMMC_DCTRL_DTEN_Msk                   /*!<Data transfer enabled bit         */
#define SDMMC_DCTRL_DTDIR_Pos           (1U)
#define SDMMC_DCTRL_DTDIR_Msk           (0x1UL << SDMMC_DCTRL_DTDIR_Pos)       /*!< 0x00000002 */
#define SDMMC_DCTRL_DTDIR               SDMMC_DCTRL_DTDIR_Msk                  /*!<Data transfer direction selection */

#define SDMMC_DCTRL_DTMODE_Pos          (2U)
#define SDMMC_DCTRL_DTMODE_Msk          (0x3UL << SDMMC_DCTRL_DTMODE_Pos)      /*!< 0x0000000C */
#define SDMMC_DCTRL_DTMODE              SDMMC_DCTRL_DTMODE_Msk                 /*!<Data transfer mode selection      */
#define SDMMC_DCTRL_DTMODE_0            (0x1UL << SDMMC_DCTRL_DTMODE_Pos)      /*!< 0x00000004 */
#define SDMMC_DCTRL_DTMODE_1            (0x2UL << SDMMC_DCTRL_DTMODE_Pos)      /*!< 0x00000008 */

#define SDMMC_DCTRL_DBLOCKSIZE_Pos      (4U)
#define SDMMC_DCTRL_DBLOCKSIZE_Msk      (0xFUL << SDMMC_DCTRL_DBLOCKSIZE_Pos)  /*!< 0x000000F0 */
#define SDMMC_DCTRL_DBLOCKSIZE          SDMMC_DCTRL_DBLOCKSIZE_Msk             /*!<DBLOCKSIZE[3:0] bits (Data block size) */
#define SDMMC_DCTRL_DBLOCKSIZE_0        (0x1UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)  /*!< 0x00000010 */
#define SDMMC_DCTRL_DBLOCKSIZE_1        (0x2UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)  /*!< 0x00000020 */
#define SDMMC_DCTRL_DBLOCKSIZE_2        (0x4UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)  /*!< 0x00000040 */
#define SDMMC_DCTRL_DBLOCKSIZE_3        (0x8UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)  /*!< 0x00000080 */

#define SDMMC_DCTRL_RWSTART_Pos         (8U)
#define SDMMC_DCTRL_RWSTART_Msk         (0x1UL << SDMMC_DCTRL_RWSTART_Pos)     /*!< 0x00000100 */
#define SDMMC_DCTRL_RWSTART             SDMMC_DCTRL_RWSTART_Msk                /*!<Read wait start         */
#define SDMMC_DCTRL_RWSTOP_Pos          (9U)
#define SDMMC_DCTRL_RWSTOP_Msk          (0x1UL << SDMMC_DCTRL_RWSTOP_Pos)      /*!< 0x00000200 */
#define SDMMC_DCTRL_RWSTOP              SDMMC_DCTRL_RWSTOP_Msk                 /*!<Read wait stop          */
#define SDMMC_DCTRL_RWMOD_Pos           (10U)
#define SDMMC_DCTRL_RWMOD_Msk           (0x1UL << SDMMC_DCTRL_RWMOD_Pos)       /*!< 0x00000400 */
#define SDMMC_DCTRL_RWMOD               SDMMC_DCTRL_RWMOD_Msk                  /*!<Read wait mode          */
#define SDMMC_DCTRL_SDIOEN_Pos          (11U)
#define SDMMC_DCTRL_SDIOEN_Msk          (0x1UL << SDMMC_DCTRL_SDIOEN_Pos)      /*!< 0x00000800 */
#define SDMMC_DCTRL_SDIOEN              SDMMC_DCTRL_SDIOEN_Msk                 /*!<SD I/O enable functions */
#define SDMMC_DCTRL_BOOTACKEN_Pos       (12U)
#define SDMMC_DCTRL_BOOTACKEN_Msk       (0x1UL << SDMMC_DCTRL_BOOTACKEN_Pos)   /*!< 0x00001000 */
#define SDMMC_DCTRL_BOOTACKEN           SDMMC_DCTRL_BOOTACKEN_Msk              /*!<Data transfer mode selection */
#define SDMMC_DCTRL_FIFORST_Pos         (13U)
#define SDMMC_DCTRL_FIFORST_Msk         (0x1UL << SDMMC_DCTRL_FIFORST_Pos)     /*!< 0x00002000 */
#define SDMMC_DCTRL_FIFORST             SDMMC_DCTRL_FIFORST_Msk                /*!<FIFO reset              */

/******************  Bit definition for SDMMC_DCOUNT register  *****************/
#define SDMMC_DCOUNT_DATACOUNT_Pos      (0U)
#define SDMMC_DCOUNT_DATACOUNT_Msk      (0x1FFFFFFUL << SDMMC_DCOUNT_DATACOUNT_Pos) /*!< 0x01FFFFFF */
#define SDMMC_DCOUNT_DATACOUNT          SDMMC_DCOUNT_DATACOUNT_Msk             /*!<Data count value */

/******************  Bit definition for SDMMC_STA register  ********************/
#define SDMMC_STA_CCRCFAIL_Pos          (0U)
#define SDMMC_STA_CCRCFAIL_Msk          (0x1UL << SDMMC_STA_CCRCFAIL_Pos)      /*!< 0x00000001 */
#define SDMMC_STA_CCRCFAIL              SDMMC_STA_CCRCFAIL_Msk                 /*!<Command response received (CRC check failed)  */
#define SDMMC_STA_DCRCFAIL_Pos          (1U)
#define SDMMC_STA_DCRCFAIL_Msk          (0x1UL << SDMMC_STA_DCRCFAIL_Pos)      /*!< 0x00000002 */
#define SDMMC_STA_DCRCFAIL              SDMMC_STA_DCRCFAIL_Msk                 /*!<Data block sent/received (CRC check failed)   */
#define SDMMC_STA_CTIMEOUT_Pos          (2U)
#define SDMMC_STA_CTIMEOUT_Msk          (0x1UL << SDMMC_STA_CTIMEOUT_Pos)      /*!< 0x00000004 */
#define SDMMC_STA_CTIMEOUT              SDMMC_STA_CTIMEOUT_Msk                 /*!<Command response timeout                      */
#define SDMMC_STA_DTIMEOUT_Pos          (3U)
#define SDMMC_STA_DTIMEOUT_Msk          (0x1UL << SDMMC_STA_DTIMEOUT_Pos)      /*!< 0x00000008 */
#define SDMMC_STA_DTIMEOUT              SDMMC_STA_DTIMEOUT_Msk                 /*!<Data timeout                                  */
#define SDMMC_STA_TXUNDERR_Pos          (4U)
#define SDMMC_STA_TXUNDERR_Msk          (0x1UL << SDMMC_STA_TXUNDERR_Pos)      /*!< 0x00000010 */
#define SDMMC_STA_TXUNDERR              SDMMC_STA_TXUNDERR_Msk                 /*!<Transmit FIFO underrun error                  */
#define SDMMC_STA_RXOVERR_Pos           (5U)
#define SDMMC_STA_RXOVERR_Msk           (0x1UL << SDMMC_STA_RXOVERR_Pos)       /*!< 0x00000020 */
#define SDMMC_STA_RXOVERR               SDMMC_STA_RXOVERR_Msk                  /*!<Received FIFO overrun error                   */
#define SDMMC_STA_CMDREND_Pos           (6U)
#define SDMMC_STA_CMDREND_Msk           (0x1UL << SDMMC_STA_CMDREND_Pos)       /*!< 0x00000040 */
#define SDMMC_STA_CMDREND               SDMMC_STA_CMDREND_Msk                  /*!<Command response received (CRC check passed)  */
#define SDMMC_STA_CMDSENT_Pos           (7U)
#define SDMMC_STA_CMDSENT_Msk           (0x1UL << SDMMC_STA_CMDSENT_Pos)       /*!< 0x00000080 */
#define SDMMC_STA_CMDSENT               SDMMC_STA_CMDSENT_Msk                  /*!<Command sent (no response required)           */
#define SDMMC_STA_DATAEND_Pos           (8U)
#define SDMMC_STA_DATAEND_Msk           (0x1UL << SDMMC_STA_DATAEND_Pos)       /*!< 0x00000100 */
#define SDMMC_STA_DATAEND               SDMMC_STA_DATAEND_Msk                  /*!<Data end (data counter, SDIDCOUNT, is zero)   */
#define SDMMC_STA_DHOLD_Pos             (9U)
#define SDMMC_STA_DHOLD_Msk             (0x1UL << SDMMC_STA_DHOLD_Pos)         /*!< 0x00000200 */
#define SDMMC_STA_DHOLD                 SDMMC_STA_DHOLD_Msk                    /*!<Data transfer Hold                            */
#define SDMMC_STA_DBCKEND_Pos           (10U)
#define SDMMC_STA_DBCKEND_Msk           (0x1UL << SDMMC_STA_DBCKEND_Pos)       /*!< 0x00000400 */
#define SDMMC_STA_DBCKEND               SDMMC_STA_DBCKEND_Msk                  /*!<Data block sent/received (CRC check passed)   */
#define SDMMC_STA_DABORT_Pos            (11U)
#define SDMMC_STA_DABORT_Msk            (0x1UL << SDMMC_STA_DABORT_Pos)        /*!< 0x00000800 */
#define SDMMC_STA_DABORT                SDMMC_STA_DABORT_Msk                   /*!<Data transfer aborted by CMD12                */
#define SDMMC_STA_DPSMACT_Pos           (12U)
#define SDMMC_STA_DPSMACT_Msk           (0x1UL << SDMMC_STA_DPSMACT_Pos)       /*!< 0x00001000 */
#define SDMMC_STA_DPSMACT               SDMMC_STA_DPSMACT_Msk                  /*!<Data path state machine active                */
#define SDMMC_STA_CPSMACT_Pos           (13U)
#define SDMMC_STA_CPSMACT_Msk           (0x1UL << SDMMC_STA_CPSMACT_Pos)       /*!< 0x00002000 */
#define SDMMC_STA_CPSMACT               SDMMC_STA_CPSMACT_Msk                  /*!<Command path state machine active             */
#define SDMMC_STA_TXFIFOHE_Pos          (14U)
#define SDMMC_STA_TXFIFOHE_Msk          (0x1UL << SDMMC_STA_TXFIFOHE_Pos)      /*!< 0x00004000 */
#define SDMMC_STA_TXFIFOHE              SDMMC_STA_TXFIFOHE_Msk                 /*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define SDMMC_STA_RXFIFOHF_Pos          (15U)
#define SDMMC_STA_RXFIFOHF_Msk          (0x1UL << SDMMC_STA_RXFIFOHF_Pos)      /*!< 0x00008000 */
#define SDMMC_STA_RXFIFOHF              SDMMC_STA_RXFIFOHF_Msk                 /*!<Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define SDMMC_STA_TXFIFOF_Pos           (16U)
#define SDMMC_STA_TXFIFOF_Msk           (0x1UL << SDMMC_STA_TXFIFOF_Pos)       /*!< 0x00010000 */
#define SDMMC_STA_TXFIFOF               SDMMC_STA_TXFIFOF_Msk                  /*!<Transmit FIFO full                            */
#define SDMMC_STA_RXFIFOF_Pos           (17U)
#define SDMMC_STA_RXFIFOF_Msk           (0x1UL << SDMMC_STA_RXFIFOF_Pos)       /*!< 0x00020000 */
#define SDMMC_STA_RXFIFOF               SDMMC_STA_RXFIFOF_Msk                  /*!<Receive FIFO full                             */
#define SDMMC_STA_TXFIFOE_Pos           (18U)
#define SDMMC_STA_TXFIFOE_Msk           (0x1UL << SDMMC_STA_TXFIFOE_Pos)       /*!< 0x00040000 */
#define SDMMC_STA_TXFIFOE               SDMMC_STA_TXFIFOE_Msk                  /*!<Transmit FIFO empty                           */
#define SDMMC_STA_RXFIFOE_Pos           (19U)
#define SDMMC_STA_RXFIFOE_Msk           (0x1UL << SDMMC_STA_RXFIFOE_Pos)       /*!< 0x00080000 */
#define SDMMC_STA_RXFIFOE               SDMMC_STA_RXFIFOE_Msk                  /*!<Receive FIFO empty                            */
#define SDMMC_STA_BUSYD0_Pos            (20U)
#define SDMMC_STA_BUSYD0_Msk            (0x1UL << SDMMC_STA_BUSYD0_Pos)        /*!< 0x00100000 */
#define SDMMC_STA_BUSYD0                SDMMC_STA_BUSYD0_Msk                   /*!<Inverted value of SDMMC_D0 line (Busy)        */
#define SDMMC_STA_BUSYD0END_Pos         (21U)
#define SDMMC_STA_BUSYD0END_Msk         (0x1UL << SDMMC_STA_BUSYD0END_Pos)     /*!< 0x00200000 */
#define SDMMC_STA_BUSYD0END             SDMMC_STA_BUSYD0END_Msk                /*!<End of SDMMC_D0 Busy following a CMD response detected */
#define SDMMC_STA_SDIOIT_Pos            (22U)
#define SDMMC_STA_SDIOIT_Msk            (0x1UL << SDMMC_STA_SDIOIT_Pos)        /*!< 0x00400000 */
#define SDMMC_STA_SDIOIT                SDMMC_STA_SDIOIT_Msk                   /*!<SDIO interrupt received                       */
#define SDMMC_STA_ACKFAIL_Pos           (23U)
#define SDMMC_STA_ACKFAIL_Msk           (0x1UL << SDMMC_STA_ACKFAIL_Pos)       /*!< 0x00800000 */
#define SDMMC_STA_ACKFAIL               SDMMC_STA_ACKFAIL_Msk                  /*!<Boot Acknowledgment received (BootAck check fail) */
#define SDMMC_STA_ACKTIMEOUT_Pos        (24U)
#define SDMMC_STA_ACKTIMEOUT_Msk        (0x1UL << SDMMC_STA_ACKTIMEOUT_Pos)    /*!< 0x01000000 */
#define SDMMC_STA_ACKTIMEOUT            SDMMC_STA_ACKTIMEOUT_Msk               /*!<Boot Acknowledgment timeout                   */
#define SDMMC_STA_VSWEND_Pos            (25U)
#define SDMMC_STA_VSWEND_Msk            (0x1UL << SDMMC_STA_VSWEND_Pos)        /*!< 0x02000000 */
#define SDMMC_STA_VSWEND                SDMMC_STA_VSWEND_Msk                   /*!<Voltage switch critical timing section completion */
#define SDMMC_STA_CKSTOP_Pos            (26U)
#define SDMMC_STA_CKSTOP_Msk            (0x1UL << SDMMC_STA_CKSTOP_Pos)        /*!< 0x04000000 */
#define SDMMC_STA_CKSTOP                SDMMC_STA_CKSTOP_Msk                   /*!<SDMMC_CK stopped in Voltage switch procedure  */
#define SDMMC_STA_IDMATE_Pos            (27U)
#define SDMMC_STA_IDMATE_Msk            (0x1UL << SDMMC_STA_IDMATE_Pos)        /*!< 0x08000000 */
#define SDMMC_STA_IDMATE                SDMMC_STA_IDMATE_Msk                   /*!<IDMA transfer error                           */
#define SDMMC_STA_IDMABTC_Pos           (28U)
#define SDMMC_STA_IDMABTC_Msk           (0x1UL << SDMMC_STA_IDMABTC_Pos)       /*!< 0x10000000 */
#define SDMMC_STA_IDMABTC               SDMMC_STA_IDMABTC_Msk                  /*!<IDMA buffer transfer complete                 */

/*******************  Bit definition for SDMMC_ICR register  *******************/
#define SDMMC_ICR_CCRCFAILC_Pos         (0U)
#define SDMMC_ICR_CCRCFAILC_Msk         (0x1UL << SDMMC_ICR_CCRCFAILC_Pos)     /*!< 0x00000001 */
#define SDMMC_ICR_CCRCFAILC             SDMMC_ICR_CCRCFAILC_Msk                /*!<CCRCFAIL flag clear bit */
#define SDMMC_ICR_DCRCFAILC_Pos         (1U)
#define SDMMC_ICR_DCRCFAILC_Msk         (0x1UL << SDMMC_ICR_DCRCFAILC_Pos)     /*!< 0x00000002 */
#define SDMMC_ICR_DCRCFAILC             SDMMC_ICR_DCRCFAILC_Msk                /*!<DCRCFAIL flag clear bit */
#define SDMMC_ICR_CTIMEOUTC_Pos         (2U)
#define SDMMC_ICR_CTIMEOUTC_Msk         (0x1UL << SDMMC_ICR_CTIMEOUTC_Pos)     /*!< 0x00000004 */
#define SDMMC_ICR_CTIMEOUTC             SDMMC_ICR_CTIMEOUTC_Msk                /*!<CTIMEOUT flag clear bit */
#define SDMMC_ICR_DTIMEOUTC_Pos         (3U)
#define SDMMC_ICR_DTIMEOUTC_Msk         (0x1UL << SDMMC_ICR_DTIMEOUTC_Pos)     /*!< 0x00000008 */
#define SDMMC_ICR_DTIMEOUTC             SDMMC_ICR_DTIMEOUTC_Msk                /*!<DTIMEOUT flag clear bit */
#define SDMMC_ICR_TXUNDERRC_Pos         (4U)
#define SDMMC_ICR_TXUNDERRC_Msk         (0x1UL << SDMMC_ICR_TXUNDERRC_Pos)     /*!< 0x00000010 */
#define SDMMC_ICR_TXUNDERRC             SDMMC_ICR_TXUNDERRC_Msk                /*!<TXUNDERR flag clear bit */
#define SDMMC_ICR_RXOVERRC_Pos          (5U)
#define SDMMC_ICR_RXOVERRC_Msk          (0x1UL << SDMMC_ICR_RXOVERRC_Pos)      /*!< 0x00000020 */
#define SDMMC_ICR_RXOVERRC              SDMMC_ICR_RXOVERRC_Msk                 /*!<RXOVERR flag clear bit  */
#define SDMMC_ICR_CMDRENDC_Pos          (6U)
#define SDMMC_ICR_CMDRENDC_Msk          (0x1UL << SDMMC_ICR_CMDRENDC_Pos)      /*!< 0x00000040 */
#define SDMMC_ICR_CMDRENDC              SDMMC_ICR_CMDRENDC_Msk                 /*!<CMDREND flag clear bit  */
#define SDMMC_ICR_CMDSENTC_Pos          (7U)
#define SDMMC_ICR_CMDSENTC_Msk          (0x1UL << SDMMC_ICR_CMDSENTC_Pos)      /*!< 0x00000080 */
#define SDMMC_ICR_CMDSENTC              SDMMC_ICR_CMDSENTC_Msk                 /*!<CMDSENT flag clear bit  */
#define SDMMC_ICR_DATAENDC_Pos          (8U)
#define SDMMC_ICR_DATAENDC_Msk          (0x1UL << SDMMC_ICR_DATAENDC_Pos)      /*!< 0x00000100 */
#define SDMMC_ICR_DATAENDC              SDMMC_ICR_DATAENDC_Msk                 /*!<DATAEND flag clear bit  */
#define SDMMC_ICR_DHOLDC_Pos            (9U)
#define SDMMC_ICR_DHOLDC_Msk            (0x1UL << SDMMC_ICR_DHOLDC_Pos)        /*!< 0x00000200 */
#define SDMMC_ICR_DHOLDC                SDMMC_ICR_DHOLDC_Msk                   /*!<DHOLD flag clear bit    */
#define SDMMC_ICR_DBCKENDC_Pos          (10U)
#define SDMMC_ICR_DBCKENDC_Msk          (0x1UL << SDMMC_ICR_DBCKENDC_Pos)      /*!< 0x00000400 */
#define SDMMC_ICR_DBCKENDC              SDMMC_ICR_DBCKENDC_Msk                 /*!<DBCKEND flag clear bit  */
#define SDMMC_ICR_DABORTC_Pos           (11U)
#define SDMMC_ICR_DABORTC_Msk           (0x1UL << SDMMC_ICR_DABORTC_Pos)       /*!< 0x00000800 */
#define SDMMC_ICR_DABORTC               SDMMC_ICR_DABORTC_Msk                  /*!<DABORTC flag clear bit  */
#define SDMMC_ICR_BUSYD0ENDC_Pos        (21U)
#define SDMMC_ICR_BUSYD0ENDC_Msk        (0x1UL << SDMMC_ICR_BUSYD0ENDC_Pos)    /*!< 0x00200000 */
#define SDMMC_ICR_BUSYD0ENDC            SDMMC_ICR_BUSYD0ENDC_Msk               /*!<BUSYD0ENDC flag clear bit */
#define SDMMC_ICR_SDIOITC_Pos           (22U)
#define SDMMC_ICR_SDIOITC_Msk           (0x1UL << SDMMC_ICR_SDIOITC_Pos)       /*!< 0x00400000 */
#define SDMMC_ICR_SDIOITC               SDMMC_ICR_SDIOITC_Msk                  /*!<SDIOIT flag clear bit   */
#define SDMMC_ICR_ACKFAILC_Pos          (23U)
#define SDMMC_ICR_ACKFAILC_Msk          (0x1UL << SDMMC_ICR_ACKFAILC_Pos)      /*!< 0x00800000 */
#define SDMMC_ICR_ACKFAILC              SDMMC_ICR_ACKFAILC_Msk                 /*!<ACKFAILC flag clear bit */
#define SDMMC_ICR_ACKTIMEOUTC_Pos       (24U)
#define SDMMC_ICR_ACKTIMEOUTC_Msk       (0x1UL << SDMMC_ICR_ACKTIMEOUTC_Pos)   /*!< 0x01000000 */
#define SDMMC_ICR_ACKTIMEOUTC           SDMMC_ICR_ACKTIMEOUTC_Msk              /*!<ACKTIMEOUTC flag clear bit */
#define SDMMC_ICR_VSWENDC_Pos           (25U)
#define SDMMC_ICR_VSWENDC_Msk           (0x1UL << SDMMC_ICR_VSWENDC_Pos)       /*!< 0x02000000 */
#define SDMMC_ICR_VSWENDC               SDMMC_ICR_VSWENDC_Msk                  /*!<VSWENDC flag clear bit  */
#define SDMMC_ICR_CKSTOPC_Pos           (26U)
#define SDMMC_ICR_CKSTOPC_Msk           (0x1UL << SDMMC_ICR_CKSTOPC_Pos)       /*!< 0x04000000 */
#define SDMMC_ICR_CKSTOPC               SDMMC_ICR_CKSTOPC_Msk                  /*!<CKSTOPC flag clear bit  */
#define SDMMC_ICR_IDMATEC_Pos           (27U)
#define SDMMC_ICR_IDMATEC_Msk           (0x1UL << SDMMC_ICR_IDMATEC_Pos)       /*!< 0x08000000 */
#define SDMMC_ICR_IDMATEC               SDMMC_ICR_IDMATEC_Msk                  /*!<IDMATEC flag clear bit  */
#define SDMMC_ICR_IDMABTCC_Pos          (28U)
#define SDMMC_ICR_IDMABTCC_Msk          (0x1UL << SDMMC_ICR_IDMABTCC_Pos)      /*!< 0x10000000 */
#define SDMMC_ICR_IDMABTCC              SDMMC_ICR_IDMABTCC_Msk                 /*!<IDMABTCC flag clear bit */

/******************  Bit definition for SDMMC_MASK register  *******************/
#define SDMMC_MASK_CCRCFAILIE_Pos       (0U)
#define SDMMC_MASK_CCRCFAILIE_Msk       (0x1UL << SDMMC_MASK_CCRCFAILIE_Pos)   /*!< 0x00000001 */
#define SDMMC_MASK_CCRCFAILIE           SDMMC_MASK_CCRCFAILIE_Msk              /*!<Command CRC Fail Interrupt Enable          */
#define SDMMC_MASK_DCRCFAILIE_Pos       (1U)
#define SDMMC_MASK_DCRCFAILIE_Msk       (0x1UL << SDMMC_MASK_DCRCFAILIE_Pos)   /*!< 0x00000002 */
#define SDMMC_MASK_DCRCFAILIE           SDMMC_MASK_DCRCFAILIE_Msk              /*!<Data CRC Fail Interrupt Enable             */
#define SDMMC_MASK_CTIMEOUTIE_Pos       (2U)
#define SDMMC_MASK_CTIMEOUTIE_Msk       (0x1UL << SDMMC_MASK_CTIMEOUTIE_Pos)   /*!< 0x00000004 */
#define SDMMC_MASK_CTIMEOUTIE           SDMMC_MASK_CTIMEOUTIE_Msk              /*!<Command TimeOut Interrupt Enable           */
#define SDMMC_MASK_DTIMEOUTIE_Pos       (3U)
#define SDMMC_MASK_DTIMEOUTIE_Msk       (0x1UL << SDMMC_MASK_DTIMEOUTIE_Pos)   /*!< 0x00000008 */
#define SDMMC_MASK_DTIMEOUTIE           SDMMC_MASK_DTIMEOUTIE_Msk              /*!<Data TimeOut Interrupt Enable              */
#define SDMMC_MASK_TXUNDERRIE_Pos       (4U)
#define SDMMC_MASK_TXUNDERRIE_Msk       (0x1UL << SDMMC_MASK_TXUNDERRIE_Pos)   /*!< 0x00000010 */
#define SDMMC_MASK_TXUNDERRIE           SDMMC_MASK_TXUNDERRIE_Msk              /*!<Tx FIFO UnderRun Error Interrupt Enable    */
#define SDMMC_MASK_RXOVERRIE_Pos        (5U)
#define SDMMC_MASK_RXOVERRIE_Msk        (0x1UL << SDMMC_MASK_RXOVERRIE_Pos)    /*!< 0x00000020 */
#define SDMMC_MASK_RXOVERRIE            SDMMC_MASK_RXOVERRIE_Msk               /*!<Rx FIFO OverRun Error Interrupt Enable     */
#define SDMMC_MASK_CMDRENDIE_Pos        (6U)
#define SDMMC_MASK_CMDRENDIE_Msk        (0x1UL << SDMMC_MASK_CMDRENDIE_Pos)    /*!< 0x00000040 */
#define SDMMC_MASK_CMDRENDIE            SDMMC_MASK_CMDRENDIE_Msk               /*!<Command Response Received Interrupt Enable */
#define SDMMC_MASK_CMDSENTIE_Pos        (7U)
#define SDMMC_MASK_CMDSENTIE_Msk        (0x1UL << SDMMC_MASK_CMDSENTIE_Pos)    /*!< 0x00000080 */
#define SDMMC_MASK_CMDSENTIE            SDMMC_MASK_CMDSENTIE_Msk               /*!<Command Sent Interrupt Enable              */
#define SDMMC_MASK_DATAENDIE_Pos        (8U)
#define SDMMC_MASK_DATAENDIE_Msk        (0x1UL << SDMMC_MASK_DATAENDIE_Pos)    /*!< 0x00000100 */
#define SDMMC_MASK_DATAENDIE            SDMMC_MASK_DATAENDIE_Msk               /*!<Data End Interrupt Enable                  */
#define SDMMC_MASK_DHOLDIE_Pos          (9U)
#define SDMMC_MASK_DHOLDIE_Msk          (0x1UL << SDMMC_MASK_DHOLDIE_Pos)      /*!< 0x00000200 */
#define SDMMC_MASK_DHOLDIE              SDMMC_MASK_DHOLDIE_Msk                 /*!<Data Hold Interrupt Enable                 */
#define SDMMC_MASK_DBCKENDIE_Pos        (10U)
#define SDMMC_MASK_DBCKENDIE_Msk        (0x1UL << SDMMC_MASK_DBCKENDIE_Pos)    /*!< 0x00000400 */
#define SDMMC_MASK_DBCKENDIE            SDMMC_MASK_DBCKENDIE_Msk               /*!<Data Block End Interrupt Enable            */
#define SDMMC_MASK_DABORTIE_Pos         (11U)
#define SDMMC_MASK_DABORTIE_Msk         (0x1UL << SDMMC_MASK_DABORTIE_Pos)     /*!< 0x00000800 */
#define SDMMC_MASK_DABORTIE             SDMMC_MASK_DABORTIE_Msk                /*!<Data transfer aborted Interrupt Enable     */
#define SDMMC_MASK_TXFIFOHEIE_Pos       (14U)
#define SDMMC_MASK_TXFIFOHEIE_Msk       (0x1UL << SDMMC_MASK_TXFIFOHEIE_Pos)   /*!< 0x00004000 */
#define SDMMC_MASK_TXFIFOHEIE           SDMMC_MASK_TXFIFOHEIE_Msk              /*!<Tx FIFO Half Empty interrupt Enable        */
#define SDMMC_MASK_RXFIFOHFIE_Pos       (15U)
#define SDMMC_MASK_RXFIFOHFIE_Msk       (0x1UL << SDMMC_MASK_RXFIFOHFIE_Pos)   /*!< 0x00008000 */
#define SDMMC_MASK_RXFIFOHFIE           SDMMC_MASK_RXFIFOHFIE_Msk              /*!<Rx FIFO Half Full interrupt Enable         */
#define SDMMC_MASK_RXFIFOFIE_Pos        (17U)
#define SDMMC_MASK_RXFIFOFIE_Msk        (0x1UL << SDMMC_MASK_RXFIFOFIE_Pos)    /*!< 0x00020000 */
#define SDMMC_MASK_RXFIFOFIE            SDMMC_MASK_RXFIFOFIE_Msk               /*!<Rx FIFO Full interrupt Enable              */
#define SDMMC_MASK_TXFIFOEIE_Pos        (18U)
#define SDMMC_MASK_TXFIFOEIE_Msk        (0x1UL << SDMMC_MASK_TXFIFOEIE_Pos)    /*!< 0x00040000 */
#define SDMMC_MASK_TXFIFOEIE            SDMMC_MASK_TXFIFOEIE_Msk               /*!<Tx FIFO Empty interrupt Enable             */
#define SDMMC_MASK_BUSYD0ENDIE_Pos      (21U)
#define SDMMC_MASK_BUSYD0ENDIE_Msk      (0x1UL << SDMMC_MASK_BUSYD0ENDIE_Pos)  /*!< 0x00200000 */
#define SDMMC_MASK_BUSYD0ENDIE          SDMMC_MASK_BUSYD0ENDIE_Msk             /*!<BUSYD0END interrupt Enable                 */
#define SDMMC_MASK_SDIOITIE_Pos         (22U)
#define SDMMC_MASK_SDIOITIE_Msk         (0x1UL << SDMMC_MASK_SDIOITIE_Pos)     /*!< 0x00400000 */
#define SDMMC_MASK_SDIOITIE             SDMMC_MASK_SDIOITIE_Msk                /*!<SDIO Mode Interrupt Received interrupt Enable */
#define SDMMC_MASK_ACKFAILIE_Pos        (23U)
#define SDMMC_MASK_ACKFAILIE_Msk        (0x1UL << SDMMC_MASK_ACKFAILIE_Pos)    /*!< 0x00800000 */
#define SDMMC_MASK_ACKFAILIE            SDMMC_MASK_ACKFAILIE_Msk               /*!<Acknowledgment Fail Interrupt Enable       */
#define SDMMC_MASK_ACKTIMEOUTIE_Pos     (24U)
#define SDMMC_MASK_ACKTIMEOUTIE_Msk     (0x1UL << SDMMC_MASK_ACKTIMEOUTIE_Pos) /*!< 0x01000000 */
#define SDMMC_MASK_ACKTIMEOUTIE         SDMMC_MASK_ACKTIMEOUTIE_Msk            /*!<Acknowledgment timeout Interrupt Enable    */
#define SDMMC_MASK_VSWENDIE_Pos         (25U)
#define SDMMC_MASK_VSWENDIE_Msk         (0x1UL << SDMMC_MASK_VSWENDIE_Pos)     /*!< 0x02000000 */
#define SDMMC_MASK_VSWENDIE             SDMMC_MASK_VSWENDIE_Msk                /*!<Voltage switch critical timing section completion Interrupt Enable */
#define SDMMC_MASK_CKSTOPIE_Pos         (26U)
#define SDMMC_MASK_CKSTOPIE_Msk         (0x1UL << SDMMC_MASK_CKSTOPIE_Pos)     /*!< 0x03000000 */
#define SDMMC_MASK_CKSTOPIE             SDMMC_MASK_CKSTOPIE_Msk                /*!<Voltage Switch clock stopped Interrupt Enable */
#define SDMMC_MASK_IDMABTCIE_Pos        (28U)
#define SDMMC_MASK_IDMABTCIE_Msk        (0x1UL << SDMMC_MASK_IDMABTCIE_Pos)    /*!< 0x10000000 */
#define SDMMC_MASK_IDMABTCIE            SDMMC_MASK_IDMABTCIE_Msk               /*!<IDMA buffer transfer complete Interrupt Enable */

/*****************  Bit definition for SDMMC_FIFOCNT register  *****************/
#define SDMMC_FIFOCNT_FIFOCOUNT_Pos     (0U)
#define SDMMC_FIFOCNT_FIFOCOUNT_Msk     (0xFFFFFFUL << SDMMC_FIFOCNT_FIFOCOUNT_Pos) /*!< 0x00FFFFFF */
#define SDMMC_FIFOCNT_FIFOCOUNT         SDMMC_FIFOCNT_FIFOCOUNT_Msk            /*!<Remaining number of words to be written to or read from the FIFO */

/******************  Bit definition for SDMMC_FIFO register  *******************/
#define SDMMC_FIFO_FIFODATA_Pos         (0U)
#define SDMMC_FIFO_FIFODATA_Msk         (0xFFFFFFFFUL << SDMMC_FIFO_FIFODATA_Pos) /*!< 0xFFFFFFFF */
#define SDMMC_FIFO_FIFODATA             SDMMC_FIFO_FIFODATA_Msk                /*!<Receive and transmit FIFO data */

/******************  Bit definition for SDMMC_IDMACTRL register ****************/
#define SDMMC_IDMA_IDMAEN_Pos           (0U)
#define SDMMC_IDMA_IDMAEN_Msk           (0x1UL << SDMMC_IDMA_IDMAEN_Pos)         /*!< 0x00000001 */
#define SDMMC_IDMA_IDMAEN               SDMMC_IDMA_IDMAEN_Msk                    /*!< Enable the internal DMA of the SDMMC peripheral */
#define SDMMC_IDMA_IDMABMODE_Pos        (1U)
#define SDMMC_IDMA_IDMABMODE_Msk        (0x1UL << SDMMC_IDMA_IDMABMODE_Pos)      /*!< 0x00000002 */
#define SDMMC_IDMA_IDMABMODE            SDMMC_IDMA_IDMABMODE_Msk                 /*!< Enable double buffer mode for IDMA */
#define SDMMC_IDMA_IDMABACT_Pos         (2U)
#define SDMMC_IDMA_IDMABACT_Msk         (0x1UL << SDMMC_IDMA_IDMABACT_Pos)       /*!< 0x00000004 */
#define SDMMC_IDMA_IDMABACT             SDMMC_IDMA_IDMABACT_Msk                  /*!< Uses buffer 1 when double buffer mode is selected */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface (SPI)                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_CR1_CPHA_Pos         (0U)
#define SPI_CR1_CPHA_Msk         (0x1UL << SPI_CR1_CPHA_Pos)                   /*!< 0x00000001 */
#define SPI_CR1_CPHA             SPI_CR1_CPHA_Msk                              /*!<Clock Phase      */
#define SPI_CR1_CPOL_Pos         (1U)
#define SPI_CR1_CPOL_Msk         (0x1UL << SPI_CR1_CPOL_Pos)                   /*!< 0x00000002 */
#define SPI_CR1_CPOL             SPI_CR1_CPOL_Msk                              /*!<Clock Polarity   */
#define SPI_CR1_MSTR_Pos         (2U)
#define SPI_CR1_MSTR_Msk         (0x1UL << SPI_CR1_MSTR_Pos)                   /*!< 0x00000004 */
#define SPI_CR1_MSTR             SPI_CR1_MSTR_Msk                              /*!<Master Selection */

#define SPI_CR1_BR_Pos           (3U)
#define SPI_CR1_BR_Msk           (0x7UL << SPI_CR1_BR_Pos)                     /*!< 0x00000038 */
#define SPI_CR1_BR               SPI_CR1_BR_Msk                                /*!<BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0             (0x1UL << SPI_CR1_BR_Pos)                     /*!< 0x00000008 */
#define SPI_CR1_BR_1             (0x2UL << SPI_CR1_BR_Pos)                     /*!< 0x00000010 */
#define SPI_CR1_BR_2             (0x4UL << SPI_CR1_BR_Pos)                     /*!< 0x00000020 */

#define SPI_CR1_SPE_Pos          (6U)
#define SPI_CR1_SPE_Msk          (0x1UL << SPI_CR1_SPE_Pos)                    /*!< 0x00000040 */
#define SPI_CR1_SPE              SPI_CR1_SPE_Msk                               /*!<SPI Enable                          */
#define SPI_CR1_LSBFIRST_Pos     (7U)
#define SPI_CR1_LSBFIRST_Msk     (0x1UL << SPI_CR1_LSBFIRST_Pos)               /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST         SPI_CR1_LSBFIRST_Msk                          /*!<Frame Format                        */
#define SPI_CR1_SSI_Pos          (8U)
#define SPI_CR1_SSI_Msk          (0x1UL << SPI_CR1_SSI_Pos)                    /*!< 0x00000100 */
#define SPI_CR1_SSI              SPI_CR1_SSI_Msk                               /*!<Internal slave select               */
#define SPI_CR1_SSM_Pos          (9U)
#define SPI_CR1_SSM_Msk          (0x1UL << SPI_CR1_SSM_Pos)                    /*!< 0x00000200 */
#define SPI_CR1_SSM              SPI_CR1_SSM_Msk                               /*!<Software slave management           */
#define SPI_CR1_RXONLY_Pos       (10U)
#define SPI_CR1_RXONLY_Msk       (0x1UL << SPI_CR1_RXONLY_Pos)                 /*!< 0x00000400 */
#define SPI_CR1_RXONLY           SPI_CR1_RXONLY_Msk                            /*!<Receive only                        */
#define SPI_CR1_CRCL_Pos         (11U)
#define SPI_CR1_CRCL_Msk         (0x1UL << SPI_CR1_CRCL_Pos)                   /*!< 0x00000800 */
#define SPI_CR1_CRCL             SPI_CR1_CRCL_Msk                              /*!< CRC Length */
#define SPI_CR1_CRCNEXT_Pos      (12U)
#define SPI_CR1_CRCNEXT_Msk      (0x1UL << SPI_CR1_CRCNEXT_Pos)                /*!< 0x00001000 */
#define SPI_CR1_CRCNEXT          SPI_CR1_CRCNEXT_Msk                           /*!<Transmit CRC next                   */
#define SPI_CR1_CRCEN_Pos        (13U)
#define SPI_CR1_CRCEN_Msk        (0x1UL << SPI_CR1_CRCEN_Pos)                  /*!< 0x00002000 */
#define SPI_CR1_CRCEN            SPI_CR1_CRCEN_Msk                             /*!<Hardware CRC calculation enable     */
#define SPI_CR1_BIDIOE_Pos       (14U)
#define SPI_CR1_BIDIOE_Msk       (0x1UL << SPI_CR1_BIDIOE_Pos)                 /*!< 0x00004000 */
#define SPI_CR1_BIDIOE           SPI_CR1_BIDIOE_Msk                            /*!<Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE_Pos     (15U)
#define SPI_CR1_BIDIMODE_Msk     (0x1UL << SPI_CR1_BIDIMODE_Pos)               /*!< 0x00008000 */
#define SPI_CR1_BIDIMODE         SPI_CR1_BIDIMODE_Msk                          /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_CR2_RXDMAEN_Pos      (0U)
#define SPI_CR2_RXDMAEN_Msk      (0x1UL << SPI_CR2_RXDMAEN_Pos)                /*!< 0x00000001 */
#define SPI_CR2_RXDMAEN          SPI_CR2_RXDMAEN_Msk                           /*!< Rx Buffer DMA Enable */
#define SPI_CR2_TXDMAEN_Pos      (1U)
#define SPI_CR2_TXDMAEN_Msk      (0x1UL << SPI_CR2_TXDMAEN_Pos)                /*!< 0x00000002 */
#define SPI_CR2_TXDMAEN          SPI_CR2_TXDMAEN_Msk                           /*!< Tx Buffer DMA Enable */
#define SPI_CR2_SSOE_Pos         (2U)
#define SPI_CR2_SSOE_Msk         (0x1UL << SPI_CR2_SSOE_Pos)                   /*!< 0x00000004 */
#define SPI_CR2_SSOE             SPI_CR2_SSOE_Msk                              /*!< SS Output Enable */
#define SPI_CR2_NSSP_Pos         (3U)
#define SPI_CR2_NSSP_Msk         (0x1UL << SPI_CR2_NSSP_Pos)                   /*!< 0x00000008 */
#define SPI_CR2_NSSP             SPI_CR2_NSSP_Msk                              /*!< NSS pulse management Enable */
#define SPI_CR2_FRF_Pos          (4U)
#define SPI_CR2_FRF_Msk          (0x1UL << SPI_CR2_FRF_Pos)                    /*!< 0x00000010 */
#define SPI_CR2_FRF              SPI_CR2_FRF_Msk                               /*!< Frame Format Enable */
#define SPI_CR2_ERRIE_Pos        (5U)
#define SPI_CR2_ERRIE_Msk        (0x1UL << SPI_CR2_ERRIE_Pos)                  /*!< 0x00000020 */
#define SPI_CR2_ERRIE            SPI_CR2_ERRIE_Msk                             /*!< Error Interrupt Enable */
#define SPI_CR2_RXNEIE_Pos       (6U)
#define SPI_CR2_RXNEIE_Msk       (0x1UL << SPI_CR2_RXNEIE_Pos)                 /*!< 0x00000040 */
#define SPI_CR2_RXNEIE           SPI_CR2_RXNEIE_Msk                            /*!< RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE_Pos        (7U)
#define SPI_CR2_TXEIE_Msk        (0x1UL << SPI_CR2_TXEIE_Pos)                  /*!< 0x00000080 */
#define SPI_CR2_TXEIE            SPI_CR2_TXEIE_Msk                             /*!< Tx buffer Empty Interrupt Enable */
#define SPI_CR2_DS_Pos           (8U)
#define SPI_CR2_DS_Msk           (0xFUL << SPI_CR2_DS_Pos)                     /*!< 0x00000F00 */
#define SPI_CR2_DS               SPI_CR2_DS_Msk                                /*!< DS[3:0] Data Size */
#define SPI_CR2_DS_0             (0x1UL << SPI_CR2_DS_Pos)                     /*!< 0x00000100 */
#define SPI_CR2_DS_1             (0x2UL << SPI_CR2_DS_Pos)                     /*!< 0x00000200 */
#define SPI_CR2_DS_2             (0x4UL << SPI_CR2_DS_Pos)                     /*!< 0x00000400 */
#define SPI_CR2_DS_3             (0x8UL << SPI_CR2_DS_Pos)                     /*!< 0x00000800 */
#define SPI_CR2_FRXTH_Pos        (12U)
#define SPI_CR2_FRXTH_Msk        (0x1UL << SPI_CR2_FRXTH_Pos)                  /*!< 0x00001000 */
#define SPI_CR2_FRXTH            SPI_CR2_FRXTH_Msk                             /*!< FIFO reception Threshold */
#define SPI_CR2_LDMARX_Pos       (13U)
#define SPI_CR2_LDMARX_Msk       (0x1UL << SPI_CR2_LDMARX_Pos)                 /*!< 0x00002000 */
#define SPI_CR2_LDMARX           SPI_CR2_LDMARX_Msk                            /*!< Last DMA transfer for reception */
#define SPI_CR2_LDMATX_Pos       (14U)
#define SPI_CR2_LDMATX_Msk       (0x1UL << SPI_CR2_LDMATX_Pos)                 /*!< 0x00004000 */
#define SPI_CR2_LDMATX           SPI_CR2_LDMATX_Msk                            /*!< Last DMA transfer for transmission */

/********************  Bit definition for SPI_SR register  ********************/
#define SPI_SR_RXNE_Pos          (0U)
#define SPI_SR_RXNE_Msk          (0x1UL << SPI_SR_RXNE_Pos)                    /*!< 0x00000001 */
#define SPI_SR_RXNE              SPI_SR_RXNE_Msk                               /*!< Receive buffer Not Empty */
#define SPI_SR_TXE_Pos           (1U)
#define SPI_SR_TXE_Msk           (0x1UL << SPI_SR_TXE_Pos)                     /*!< 0x00000002 */
#define SPI_SR_TXE               SPI_SR_TXE_Msk                                /*!< Transmit buffer Empty */
#define SPI_SR_CHSIDE_Pos        (2U)
#define SPI_SR_CHSIDE_Msk        (0x1UL << SPI_SR_CHSIDE_Pos)                  /*!< 0x00000004 */
#define SPI_SR_CHSIDE            SPI_SR_CHSIDE_Msk                             /*!< Channel side */
#define SPI_SR_UDR_Pos           (3U)
#define SPI_SR_UDR_Msk           (0x1UL << SPI_SR_UDR_Pos)                     /*!< 0x00000008 */
#define SPI_SR_UDR               SPI_SR_UDR_Msk                                /*!< Underrun flag */
#define SPI_SR_CRCERR_Pos        (4U)
#define SPI_SR_CRCERR_Msk        (0x1UL << SPI_SR_CRCERR_Pos)                  /*!< 0x00000010 */
#define SPI_SR_CRCERR            SPI_SR_CRCERR_Msk                             /*!< CRC Error flag */
#define SPI_SR_MODF_Pos          (5U)
#define SPI_SR_MODF_Msk          (0x1UL << SPI_SR_MODF_Pos)                    /*!< 0x00000020 */
#define SPI_SR_MODF              SPI_SR_MODF_Msk                               /*!< Mode fault */
#define SPI_SR_OVR_Pos           (6U)
#define SPI_SR_OVR_Msk           (0x1UL << SPI_SR_OVR_Pos)                     /*!< 0x00000040 */
#define SPI_SR_OVR               SPI_SR_OVR_Msk                                /*!< Overrun flag */
#define SPI_SR_BSY_Pos           (7U)
#define SPI_SR_BSY_Msk           (0x1UL << SPI_SR_BSY_Pos)                     /*!< 0x00000080 */
#define SPI_SR_BSY               SPI_SR_BSY_Msk                                /*!< Busy flag */
#define SPI_SR_FRE_Pos           (8U)
#define SPI_SR_FRE_Msk           (0x1UL << SPI_SR_FRE_Pos)                     /*!< 0x00000100 */
#define SPI_SR_FRE               SPI_SR_FRE_Msk                                /*!< TI frame format error */
#define SPI_SR_FRLVL_Pos         (9U)
#define SPI_SR_FRLVL_Msk         (0x3UL << SPI_SR_FRLVL_Pos)                   /*!< 0x00000600 */
#define SPI_SR_FRLVL             SPI_SR_FRLVL_Msk                              /*!< FIFO Reception Level */
#define SPI_SR_FRLVL_0           (0x1UL << SPI_SR_FRLVL_Pos)                   /*!< 0x00000200 */
#define SPI_SR_FRLVL_1           (0x2UL << SPI_SR_FRLVL_Pos)                   /*!< 0x00000400 */
#define SPI_SR_FTLVL_Pos         (11U)
#define SPI_SR_FTLVL_Msk         (0x3UL << SPI_SR_FTLVL_Pos)                   /*!< 0x00001800 */
#define SPI_SR_FTLVL             SPI_SR_FTLVL_Msk                              /*!< FIFO Transmission Level */
#define SPI_SR_FTLVL_0           (0x1UL << SPI_SR_FTLVL_Pos)                   /*!< 0x00000800 */
#define SPI_SR_FTLVL_1           (0x2UL << SPI_SR_FTLVL_Pos)                   /*!< 0x00001000 */

/********************  Bit definition for SPI_DR register  ********************/
#define SPI_DR_DR_Pos            (0U)
#define SPI_DR_DR_Msk            (0xFFFFUL << SPI_DR_DR_Pos)                   /*!< 0x0000FFFF */
#define SPI_DR_DR                SPI_DR_DR_Msk                                 /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define SPI_CRCPR_CRCPOLY_Pos    (0U)
#define SPI_CRCPR_CRCPOLY_Msk    (0xFFFFUL << SPI_CRCPR_CRCPOLY_Pos)           /*!< 0x0000FFFF */
#define SPI_CRCPR_CRCPOLY        SPI_CRCPR_CRCPOLY_Msk                         /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define SPI_RXCRCR_RXCRC_Pos     (0U)
#define SPI_RXCRCR_RXCRC_Msk     (0xFFFFUL << SPI_RXCRCR_RXCRC_Pos)            /*!< 0x0000FFFF */
#define SPI_RXCRCR_RXCRC         SPI_RXCRCR_RXCRC_Msk                          /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define SPI_TXCRCR_TXCRC_Pos     (0U)
#define SPI_TXCRCR_TXCRC_Msk     (0xFFFFUL << SPI_TXCRCR_TXCRC_Pos)            /*!< 0x0000FFFF */
#define SPI_TXCRCR_TXCRC         SPI_TXCRCR_TXCRC_Msk                          /*!<Tx CRC Register         */


/**
  * @}
  */

/** @addtogroup Exported_macros
  * @{
  */

/****************************** SDMMC Instances *******************************/
#define IS_SDMMC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == SDMMC1)

/******************************** SPI Instances *******************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) || \
                                       ((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))

/**
  * @}
  */


/******************************************************************************/
/*  For a painless codes migration between the STM32L4xx device product       */
/*  lines, the aliases defined below are put in place to overcome the         */
/*  differences in the interrupt handlers and IRQn definitions.               */
/*  No need to update developed interrupt code when moving across             */
/*  product lines within the same STM32L4 Family                              */
/******************************************************************************/

/* Aliases for __IRQn */
#define TIM6_IRQn                      TIM6_DAC_IRQn
#define ADC1_2_IRQn                    ADC1_IRQn
#define TIM1_TRG_COM_IRQn              TIM1_TRG_COM_TIM17_IRQn
#define TIM8_IRQn                      TIM8_UP_IRQn
#define HASH_RNG_IRQn                  RNG_IRQn
#define HASH_CRS_IRQn                  CRS_IRQn
#define DFSDM0_IRQn                    DFSDM1_FLT0_IRQn
#define DFSDM1_IRQn                    DFSDM1_FLT1_IRQn
#define DFSDM2_IRQn                    DFSDM1_FLT2_IRQn
#define DFSDM3_IRQn                    DFSDM1_FLT3_IRQn

/* Aliases for __IRQHandler */
#define TIM6_IRQHandler                TIM6_DAC_IRQHandler
#define ADC1_2_IRQHandler              ADC1_IRQHandler
#define TIM1_TRG_COM_IRQHandler        TIM1_TRG_COM_TIM17_IRQHandler
#define TIM8_IRQHandler                TIM8_UP_IRQHandler
#define HASH_RNG_IRQHandler            RNG_IRQHandler
#define HASH_CRS_IRQHandler            CRS_IRQHandler
#define DFSDM0_IRQHandler              DFSDM1_FLT0_IRQHandler
#define DFSDM1_IRQHandler              DFSDM1_FLT1_IRQHandler
#define DFSDM2_IRQHandler              DFSDM1_FLT2_IRQHandler
#define DFSDM3_IRQHandler              DFSDM1_FLT3_IRQHandler

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32L4R5xx_H */

  /**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
