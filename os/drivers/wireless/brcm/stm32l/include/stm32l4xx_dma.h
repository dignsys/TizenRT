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

#ifndef __STM32F4xx_DMA_H
#define __STM32F4xx_DMA_H

#ifdef CONFIG_ARCH_CHIP_STM32L4

#define STM32L4_DMACHAN1_OFFSET      0x0000
#define STM32L4_DMACHAN2_OFFSET      0x0014
#define STM32L4_DMACHAN3_OFFSET      0x0028
#define STM32L4_DMACHAN4_OFFSET      0x003c
#define STM32L4_DMACHAN5_OFFSET      0x0050
#define STM32L4_DMACHAN6_OFFSET      0x0064
#define STM32L4_DMACHAN7_OFFSET      0x0078

#define DMA_CCR_EN                (1 << 0)  /* Bit 0: Channel enable */
#define DMA_CCR_TCIE              (1 << 1)  /* Bit 1: Transfer complete interrupt enable */
#define DMA_CCR_HTIE              (1 << 2)  /* Bit 2: Half Transfer interrupt enable */
#define DMA_CCR_TEIE              (1 << 3)  /* Bit 3: Transfer error interrupt enable */
#define DMA_CCR_DIR               (1 << 4)  /* Bit 4: Data transfer direction */
#define DMA_CCR_CIRC              (1 << 5)  /* Bit 5: Circular mode */
#define DMA_CCR_PINC              (1 << 6)  /* Bit 6: Peripheral increment mode */
#define DMA_CCR_MINC              (1 << 7)  /* Bit 7: Memory increment mode */
#define DMA_CCR_PSIZE_SHIFT       (8)       /* Bits 8-9: Peripheral size */
#define DMA_CCR_PSIZE_MASK        (3 << DMA_CCR_PSIZE_SHIFT)
#  define DMA_CCR_PSIZE_8BITS     (0 << DMA_CCR_PSIZE_SHIFT) /* 00: 8-bits */
#  define DMA_CCR_PSIZE_16BITS    (1 << DMA_CCR_PSIZE_SHIFT) /* 01: 16-bits */
#  define DMA_CCR_PSIZE_32BITS    (2 << DMA_CCR_PSIZE_SHIFT) /* 10: 32-bits */
#define DMA_CCR_MSIZE_SHIFT       (10)      /* Bits 10-11: Memory size */
#define DMA_CCR_MSIZE_MASK        (3 << DMA_CCR_MSIZE_SHIFT)
#  define DMA_CCR_MSIZE_8BITS     (0 << DMA_CCR_MSIZE_SHIFT) /* 00: 8-bits */
#  define DMA_CCR_MSIZE_16BITS    (1 << DMA_CCR_MSIZE_SHIFT) /* 01: 16-bits */
#  define DMA_CCR_MSIZE_32BITS    (2 << DMA_CCR_MSIZE_SHIFT) /* 10: 32-bits */
#define DMA_CCR_PL_SHIFT          (12)      /* Bits 12-13: Channel Priority level */
#define DMA_CCR_PL_MASK           (3 << DMA_CCR_PL_SHIFT)
#  define DMA_CCR_PRILO           (0 << DMA_CCR_PL_SHIFT) /* 00: Low */
#  define DMA_CCR_PRIMED          (1 << DMA_CCR_PL_SHIFT) /* 01: Medium */
#  define DMA_CCR_PRIHI           (2 << DMA_CCR_PL_SHIFT) /* 10: High */
#  define DMA_CCR_PRIVERYHI       (3 << DMA_CCR_PL_SHIFT) /* 11: Very high */
#define DMA_CCR_MEM2MEM           (1 << 14) /* Bit 14: Memory to memory mode */
#define DMA_CCR_ALLINTS           (DMA_CCR_TEIE|DMA_CCR_HTIE|DMA_CCR_TCIE)

#else

#define DMA_SxCR_PFCTRL                   ((uint32_t)0x00000020)
#define DMA_SxCR_TCIE                     ((uint32_t)0x00000010)
#define DMA_SxCR_EN                       ((uint32_t)0x00000001)
#define DMA_Channel_4                     ((uint32_t)0x08000000)
#define DMA_DIR_PeripheralToMemory        ((uint32_t)0x00000000)
#define DMA_DIR_MemoryToPeripheral        ((uint32_t)0x00000040) 
#define DMA_PeripheralInc_Disable         ((uint32_t)0x00000000)
#define DMA_MemoryInc_Enable              ((uint32_t)0x00000400)
#define DMA_PeripheralDataSize_Word       ((uint32_t)0x00001000)
#define DMA_MemoryDataSize_Word           ((uint32_t)0x00004000)
#define DMA_Mode_Normal                   ((uint32_t)0x00000000) 
#define DMA_Priority_VeryHigh             ((uint32_t)0x00030000)
#define DMA_FIFOMode_Enable               ((uint32_t)0x00000004)
#define DMA_FIFOThreshold_Full            ((uint32_t)0x00000003)
#define DMA_MemoryBurst_INC4              ((uint32_t)0x00800000)  
#define DMA_PeripheralBurst_INC4          ((uint32_t)0x00200000)  

#endif

#endif /*__STM32F4xx_DMA_H */

