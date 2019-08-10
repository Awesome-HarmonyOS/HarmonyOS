/*
* The Clear BSD License
* Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
* All rights reserved.
*
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted (subject to the limitations in the disclaimer below) provided
* that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "fsl_common.h"

#ifndef __GIC_PRIO_BITS
#if defined(ENABLE_RAM_VECTOR_TABLE)
uint32_t InstallIRQHandler(IRQn_Type irq, uint32_t irqHandler)
{
/* Addresses for VECTOR_TABLE and VECTOR_RAM come from the linker file */
#if defined(__CC_ARM)
    extern uint32_t Image$$VECTOR_ROM$$Base[];
    extern uint32_t Image$$VECTOR_RAM$$Base[];
    extern uint32_t Image$$RW_m_data$$Base[];

#define __VECTOR_TABLE Image$$VECTOR_ROM$$Base
#define __VECTOR_RAM Image$$VECTOR_RAM$$Base
#define __RAM_VECTOR_TABLE_SIZE (((uint32_t)Image$$RW_m_data$$Base - (uint32_t)Image$$VECTOR_RAM$$Base))
#elif defined(__ICCARM__)
    extern uint32_t __RAM_VECTOR_TABLE_SIZE[];
    extern uint32_t __VECTOR_TABLE[];
    extern uint32_t __VECTOR_RAM[];
#elif defined(__GNUC__)
    extern uint32_t __VECTOR_TABLE[];
    extern uint32_t __VECTOR_RAM[];
    extern uint32_t __RAM_VECTOR_TABLE_SIZE_BYTES[];
    uint32_t __RAM_VECTOR_TABLE_SIZE = (uint32_t)(__RAM_VECTOR_TABLE_SIZE_BYTES);
#endif /* defined(__CC_ARM) */
    uint32_t n;
    uint32_t ret;
    uint32_t irqMaskValue;

    irqMaskValue = DisableGlobalIRQ();
    if (SCB->VTOR != (uint32_t)__VECTOR_RAM)
    {
        /* Copy the vector table from ROM to RAM */
        for (n = 0; n < ((uint32_t)__RAM_VECTOR_TABLE_SIZE) / sizeof(uint32_t); n++)
        {
            __VECTOR_RAM[n] = __VECTOR_TABLE[n];
        }
        /* Point the VTOR to the position of vector table */
        SCB->VTOR = (uint32_t)__VECTOR_RAM;
    }

    ret = __VECTOR_RAM[irq + 16];
    /* make sure the __VECTOR_RAM is noncachable */
    __VECTOR_RAM[irq + 16] = irqHandler;

    EnableGlobalIRQ(irqMaskValue);

/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif

    return ret;
}
#endif /* ENABLE_RAM_VECTOR_TABLE. */
#endif /* __GIC_PRIO_BITS. */

#ifndef QN908XC_SERIES
#if (defined(FSL_FEATURE_SOC_SYSCON_COUNT) && (FSL_FEATURE_SOC_SYSCON_COUNT > 0))

void EnableDeepSleepIRQ(IRQn_Type interrupt)
{
    uint32_t intNumber = (uint32_t)interrupt;

#if (defined(FSL_FEATURE_STARTER_DISCONTINUOUS) && (FSL_FEATURE_STARTER_DISCONTINUOUS == 1))
    {
        SYSCON->STARTERP1 = 1u << intNumber;
    }
#else
    {
        uint32_t index = 0;

        while (intNumber >= 32u)
        {
            index++;
            intNumber -= 32u;
        }

        SYSCON->STARTERSET[index] = 1u << intNumber;
    }
#endif                    /* FSL_FEATURE_STARTER_DISCONTINUOUS */
    EnableIRQ(interrupt); /* also enable interrupt at NVIC */
}

void DisableDeepSleepIRQ(IRQn_Type interrupt)
{
    uint32_t intNumber = (uint32_t)interrupt;

    DisableIRQ(interrupt); /* also disable interrupt at NVIC */
#if (defined(FSL_FEATURE_STARTER_DISCONTINUOUS) && (FSL_FEATURE_STARTER_DISCONTINUOUS == 1))
    {
        SYSCON->STARTERP1 &= ~(1u << intNumber);
    }
#else
    {
        uint32_t index = 0;

        while (intNumber >= 32u)
        {
            index++;
            intNumber -= 32u;
        }

        SYSCON->STARTERCLR[index] = 1u << intNumber;
    }
#endif /* FSL_FEATURE_STARTER_DISCONTINUOUS */
}
#endif /* FSL_FEATURE_SOC_SYSCON_COUNT */

#endif /* QN908XC_SERIES */
