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

#ifndef _LOS_MEMORY_PH
#define _LOS_MEMORY_PH

#include "los_memory.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/*
* memcheck error code: the stack have not inited
* Value: 0x02000100
* Solution: do memcheck must after stack mem init
*/
#define  OS_ERRNO_MEMCHECK_NOT_INIT      LOS_ERRNO_OS_ERROR(LOS_MOD_MEM, 0x0)

/*
*  memcheck error code: the pPtr is NULL
*  Value: 0x02000101
*  Solution: don't give a NULL parameter
*/
#define  OS_ERRNO_MEMCHECK_PARA_NULL      LOS_ERRNO_OS_ERROR(LOS_MOD_MEM, 0x1)

/*
*  memcheck error code: the pPtr addr not in the suit range
*  Value: 0x02000102
*  Solution: check pPtr and comfirm it included by stack
*/
#define  OS_ERRNO_MEMCHECK_OUTSIDE      LOS_ERRNO_OS_ERROR(LOS_MOD_MEM, 0x2)

/*
*  memcheck error code: can't find the ctrl node
*  Value: 0x02000103
*  Solution: confirm the pPtr if this node has been freed or has not been alloced
*/
#define  OS_ERRNO_MEMCHECK_NO_HEAD      LOS_ERRNO_OS_ERROR(LOS_MOD_MEM, 0x3)

/*
*  memcheck error code: the para level is wrong
*  Value: 0x02000104
*  Solution: checkout the memcheck level by the func "OS_GetMemCheck_Level"
*/
#define  OS_ERRNO_MEMCHECK_WRONG_LEVEL      LOS_ERRNO_OS_ERROR(LOS_MOD_MEM, 0x4)

/*
*  memcheck error code: memcheck func not open
*  Value: 0x02000105
*  Solution: enable memcheck by the func "OS_SetMemCheck_Level"
*/
#define  OS_ERRNO_MEMCHECK_DISABLED      LOS_ERRNO_OS_ERROR(LOS_MOD_MEM, 0x5)

/**
 *@ingroup los_memory
 *@brief Initialization the memory system.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to initialization the memory system.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  None.
 *
 *@retval UINT32 Initialization result.
 *@par Dependency:
 *<ul><li>los_memory.ph: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 osMemSystemInit(VOID);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_MEMORY_PH */
