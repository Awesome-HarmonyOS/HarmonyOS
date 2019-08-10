/*----------------------------------------------------------------------------
 * Copyright (c) <2013-2018>, <Huawei Technologies Co., Ltd>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific prior written
 * permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
 * Notice of Export Control Law
 * ===============================================
 * Huawei LiteOS may be subject to applicable export control laws and regulations, which might
 * include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
 * applicable export control laws and regulations.
 *---------------------------------------------------------------------------*/

#include "los_hwi.h"
#include "los_memory.h"
#include "string.h"

unsigned char g_int_cnt = 0;


#if defined (__ICC430__)

static struct hwi_int_handle s_irq_table [RESET_VECTOR >> 1] = {0};

#elif defined (__TI_COMPILER_VERSION__)

static struct hwi_int_handle s_irq_table [RESET_VECTOR] = {0};

#endif

/*****************************************************************************
 Function    : osHwiInit
 Description : initialization of the hardware interrupt
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
void osHwiInit (void)
{
    return;
}

void osHwiDispatch (int vector)
{
    int irq;

    if ((vector < 0) || (vector >= RESET_VECTOR))
    {
        PRINT_ERR ("unexpected irq\n");
        return;
    }

#if defined (__ICC430__)

    irq = vector >> 1;

#elif defined (__TI_COMPILER_VERSION__)

    irq = vector;

#endif

    if (s_irq_table [irq].pfn == NULL)
    {
        PRINT_ERR ("unexpected irq\n");
        return;
    }

    s_irq_table [irq].pfn (s_irq_table [irq].arg);
}

/*****************************************************************************
 Function    : LOS_HwiCreate
 Description : create hardware interrupt
 Input       : vector --- vector number
               prio   --- not used
               pfn    --- vector handler
               arg    --- param of the vector handler
 Output      : None
 Return      : OS_SUCCESS on success or error code on failure
 *****************************************************************************/
UINT32 LOS_HwiCreate (int vector, int prio, int mode,
                      void (* pfn) (uintptr_t), uintptr_t arg)
{
    int     irq;

    (void) mode;        /* not used for now */

    if (pfn == NULL)
    {
        return OS_ERRNO_HWI_PROC_FUNC_NULL;
    }

    if ((vector < 0) || (vector >= RESET_VECTOR))
    {
        return OS_ERRNO_HWI_NUM_INVALID;
    }

#if defined (__ICC430__)

    irq = vector >> 1;

#elif defined (__TI_COMPILER_VERSION__)

    irq = vector;

#endif

    if (s_irq_table [irq].pfn != NULL)
    {
        return OS_ERRNO_HWI_ALREADY_CREATED;
    }

    s_irq_table [irq].pfn = pfn;
    s_irq_table [irq].arg = arg;

    return 0;
}


/*****************************************************************************
 Function    : LOS_HwiDelete
 Description : delete hardware interrupt
 Input       : vector --- hwi vector to delete
 Output      : None
 Return      : OS_SUCCESS on success or error code on failure
 *****************************************************************************/
UINT32 LOS_HwiDelete (int vector)
{
    if ((vector < 0) || (vector >= RESET_VECTOR))
    {
        return OS_ERRNO_HWI_NUM_INVALID;
    }


#if defined (__ICC430__)

    s_irq_table [vector >> 1].pfn = NULL;

#elif defined (__TI_COMPILER_VERSION__)

    s_irq_table [vector].pfn = NULL;

#endif

    return LOS_OK;
}

