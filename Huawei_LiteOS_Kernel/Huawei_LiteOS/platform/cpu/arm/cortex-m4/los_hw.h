/*----------------------------------------------------------------------------
 * Copyright (c) <2013-2015>, <Huawei Technologies Co., Ltd>
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

 /**@defgroup los_hw hardware
   *@ingroup kernel
 */

#ifndef _LOS_HW_H
#define _LOS_HW_H

#include "los_base.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/**
 * @ingroup los_hw
 * The initialization value of stack space.
 */
#define EMPTY_STACK                 0xCACA

/**
 * @ingroup los_hw
 * Trigger a task.
 */
#define osTaskTrap()                asm("   TRAP    #31")

/**
 * @ingroup los_hw
 * Check task schedule.
 */
#define LOS_CHECK_SCHEDULE          ((!g_usLosTaskLock))

/**
 * @ingroup los_hw
 * Define the type of a task context control block.
 */
typedef struct tagTskContext
{
    UINT32 S16;
    UINT32 S17;
    UINT32 S18;
    UINT32 S19;
    UINT32 S20;
    UINT32 S21;
    UINT32 S22;
    UINT32 S23;
    UINT32 S24;
    UINT32 S25;
    UINT32 S26;
    UINT32 S27;
    UINT32 S28;
    UINT32 S29;
    UINT32 S30;
    UINT32 S31;
    UINT32 uwR4;
    UINT32 uwR5;
    UINT32 uwR6;
    UINT32 uwR7;
    UINT32 uwR8;
    UINT32 uwR9;
    UINT32 uwR10;
    UINT32 uwR11;
    UINT32 uwPriMask;
    UINT32 uwR0;
    UINT32 uwR1;
    UINT32 uwR2;
    UINT32 uwR3;
    UINT32 uwR12;
    UINT32 uwLR;
    UINT32 uwPC;
    UINT32 uwxPSR;
    UINT32 S0;
    UINT32 S1;
    UINT32 S2;
    UINT32 S3;
    UINT32 S4;
    UINT32 S5;
    UINT32 S6;
    UINT32 S7;
    UINT32 S8;
    UINT32 S9;
    UINT32 S10;
    UINT32 S11;
    UINT32 S12;
    UINT32 S13;
    UINT32 S14;
    UINT32 S15;
    UINT32 FPSCR;
    UINT32 NO_NAME;
} TSK_CONTEXT_S;



/**
 * @ingroup  los_hw
 * @brief: Task stack initialization.
 *
 * @par Description:
 * This API is used to initialize the task stack.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param  uwTaskID     [IN] Type#UINT32: TaskID.
 * @param  uwStackSize  [IN] Type#UINT32: Total size of the stack.
 * @param  pTopStack    [IN] Type#VOID *: Top of task's stack.
 *
 * @retval: pstContext Type#TSK_CONTEXT_S *.
 * @par Dependency:
 * <ul><li>los_hw.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern VOID * osTskStackInit(UINT32 uwTaskID, UINT32 uwStackSize, VOID *pTopStack);



/**
 * @ingroup  los_hw
 * @brief: Task scheduling Function.
 *
 * @par Description:
 * This API is used to scheduling task.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param  None.
 *
 * @retval: None.
 * @par Dependency:
 * <ul><li>los_hw.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osSchedule(VOID);


/**
 * @ingroup  los_hw
 * @brief: Function to determine whether task scheduling is required.
 *
 * @par Description:
 * This API is used to Judge and entry task scheduling.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param  None.
 *
 * @retval: None.
 * @par Dependency:
 * <ul><li>los_hw.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern VOID LOS_Schedule(VOID);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */


#endif /* _LOS_HW_H */

