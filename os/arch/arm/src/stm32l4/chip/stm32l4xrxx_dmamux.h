/************************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ************************************************************************************/
/************************************************************************************
 * arch/arm/src/stm32l4/chip/stm32l4xrxx_dmamux.h
 *
 *   Copyright (C) 2009, 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32L4_CHIP_STM32L4XRXX_DMAMUX_H
#define __ARCH_ARM_SRC_STM32L4_CHIP_STM32L4XRXX_DMAMUX_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/**
 * @name    DMAMUX1 request sources
 * @{
 */
#define STM32L4_DMAMUX1_REQ_GEN0      1
#define STM32L4_DMAMUX1_REQ_GEN1      2
#define STM32L4_DMAMUX1_REQ_GEN2      3
#define STM32L4_DMAMUX1_REQ_GEN3      4
#define STM32L4_DMAMUX1_ADC1          5
#define STM32L4_DMAMUX1_DAC1_CH1      6
#define STM32L4_DMAMUX1_DAC1_CH2      7
#define STM32L4_DMAMUX1_TIM6_UP       8
#define STM32L4_DMAMUX1_TIM7_UP       9
#define STM32L4_DMAMUX1_SPI1_RX       10
#define STM32L4_DMAMUX1_SPI1_TX       11
#define STM32L4_DMAMUX1_SPI2_RX       12
#define STM32L4_DMAMUX1_SPI2_TX       13
#define STM32L4_DMAMUX1_SPI3_RX       14
#define STM32L4_DMAMUX1_SPI3_TX       15
#define STM32L4_DMAMUX1_I2C1_RX       16
#define STM32L4_DMAMUX1_I2C1_TX       17
#define STM32L4_DMAMUX1_I2C2_RX       18
#define STM32L4_DMAMUX1_I2C2_TX       19
#define STM32L4_DMAMUX1_I2C3_RX       20
#define STM32L4_DMAMUX1_I2C3_TX       21
#define STM32L4_DMAMUX1_I2C4_RX       22
#define STM32L4_DMAMUX1_I2C4_TX       23
#define STM32L4_DMAMUX1_USART1_RX     24
#define STM32L4_DMAMUX1_USART1_TX     25
#define STM32L4_DMAMUX1_USART2_RX     26
#define STM32L4_DMAMUX1_USART2_TX     27
#define STM32L4_DMAMUX1_USART3_RX     28
#define STM32L4_DMAMUX1_USART3_TX     29
#define STM32L4_DMAMUX1_UART4_RX      30
#define STM32L4_DMAMUX1_UART4_TX      31
#define STM32L4_DMAMUX1_UART5_RX      32
#define STM32L4_DMAMUX1_UART5_TX      33
#define STM32L4_DMAMUX1_LPUART1_RX    34
#define STM32L4_DMAMUX1_LPUART1_TX    35
#define STM32L4_DMAMUX1_SAI1_A        36
#define STM32L4_DMAMUX1_SAI1_B        37
#define STM32L4_DMAMUX1_SAI2_A        38
#define STM32L4_DMAMUX1_SAI2_B        39
#define STM32L4_DMAMUX1_OCTOSPI1      40
#define STM32L4_DMAMUX1_OCTOSPI2      41
#define STM32L4_DMAMUX1_TIM1_CH1      42
#define STM32L4_DMAMUX1_TIM1_CH2      43
#define STM32L4_DMAMUX1_TIM1_CH3      44
#define STM32L4_DMAMUX1_TIM1_CH4      45
#define STM32L4_DMAMUX1_TIM1_UP       46
#define STM32L4_DMAMUX1_TIM1_TRIG     47
#define STM32L4_DMAMUX1_TIM1_COM      48
#define STM32L4_DMAMUX1_TIM8_CH1      49
#define STM32L4_DMAMUX1_TIM8_CH2      50
#define STM32L4_DMAMUX1_TIM8_CH3      51
#define STM32L4_DMAMUX1_TIM8_CH4      52
#define STM32L4_DMAMUX1_TIM8_UP       53
#define STM32L4_DMAMUX1_TIM8_TRIG     54
#define STM32L4_DMAMUX1_TIM8_COM      55
#define STM32L4_DMAMUX1_TIM2_CH1      56
#define STM32L4_DMAMUX1_TIM2_CH2      57
#define STM32L4_DMAMUX1_TIM2_CH3      58
#define STM32L4_DMAMUX1_TIM2_CH4      59
#define STM32L4_DMAMUX1_TIM2_UP       60
#define STM32L4_DMAMUX1_TIM3_CH1      61
#define STM32L4_DMAMUX1_TIM3_CH2      62
#define STM32L4_DMAMUX1_TIM3_CH3      63
#define STM32L4_DMAMUX1_TIM3_CH4      64
#define STM32L4_DMAMUX1_TIM3_UP       65
#define STM32L4_DMAMUX1_TIM3_TRIG     66
#define STM32L4_DMAMUX1_TIM4_CH1      67
#define STM32L4_DMAMUX1_TIM4_CH2      68
#define STM32L4_DMAMUX1_TIM4_CH3      69
#define STM32L4_DMAMUX1_TIM4_CH4      70
#define STM32L4_DMAMUX1_TIM4_UP       71
#define STM32L4_DMAMUX1_TIM5_CH1      72
#define STM32L4_DMAMUX1_TIM5_CH2      73
#define STM32L4_DMAMUX1_TIM5_CH3      74
#define STM32L4_DMAMUX1_TIM5_CH4      75
#define STM32L4_DMAMUX1_TIM5_UP       76
#define STM32L4_DMAMUX1_TIM5_TRIG     77
#define STM32L4_DMAMUX1_TIM15_CH1     78
#define STM32L4_DMAMUX1_TIM15_UP      79
#define STM32L4_DMAMUX1_TIM15_TRIG    80
#define STM32L4_DMAMUX1_TIM15_COM     81
#define STM32L4_DMAMUX1_TIM16_CH1     82
#define STM32L4_DMAMUX1_TIM16_UP      83
#define STM32L4_DMAMUX1_TIM17_CH1     84
#define STM32L4_DMAMUX1_TIM17_UP      85
#define STM32L4_DMAMUX1_DFSDM1_FLT0   86
#define STM32L4_DMAMUX1_DFSDM1_FLT1   87
#define STM32L4_DMAMUX1_DFSDM1_FLT2   88
#define STM32L4_DMAMUX1_DFSDM1_FLT3   89
#define STM32L4_DMAMUX1_DCMI          90
#define STM32L4_DMAMUX1_AES_IN        91
#define STM32L4_DMAMUX1_AES_OUT       92
#define STM32L4_DMAMUX1_HASH_IN       93

/* Register Offsets *****************************************************************/
#define STM32L4_DMAMUX_CCR	0x0000 /* DMAMUX CCR(channel configuration register) base offset
					* The CCR register : CC0R ~ CC13R
					*    CC0R ~ CC7R : DMA1
					*    CC8R ~ CC13R : DMA2
					*/
#define STM32L4_DMAMUX_CSR	0x0080 /* DMAMUX interrupt channel status register base offset */
#define STM32L4_DMAMUX_CFR	0x0084 /* DMAMUX interrupt clear flag register base */
#define STM32L4_DMAMUX_RG0CR	0x0100 /* DMAMUX channel generater configuration register base 0 */
#define STM32L4_DMAMUX_RG1CR	0x0104 /* DMAMUX channel generater configuration register base 1 */
#define STM32L4_DMAMUX_RG2CR	0x0108 /* DMAMUX channel generater configuration register base 2 */
#define STM32L4_DMAMUX_RG3CR	0x010C /* DMAMUX channel generater configuration register base 3 */
#define STM32L4_DMAMUX_RGSR	0x0140 /* DMAMUX channel generater interrupt status register base */
#define STM32L4_DMAMUX_RGCFR	0x0144 /* DMAMUX channel generater interrupt clear flag register base */
#define STM32L4_DMAMUX_CSR	0x0080

#endif /* __ARCH_ARM_SRC_STM32L4_CHIP_STM32L4XRXX_DMA_H */
