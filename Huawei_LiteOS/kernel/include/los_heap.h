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

/**@defgroup los_heap Heap
 * @ingroup kernel
 */

#ifndef _LOS_HEAP_H
#define _LOS_HEAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <los_typedef.h>
#include "los_base.h"
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
#include "los_slab.ph"
#endif

#define IS_ALIGNED(value)                       (0 == (((UINT32)(value)) & ((UINT32)(value - 1))))
#define OS_MEM_ALIGN(value, uwAlign)            (((UINT32)(value) + (UINT32)(uwAlign - 1)) & (~(UINT32)(uwAlign - 1)))
#define OS_MEM_ALIGN_FLAG                       (0x80000000)
#define OS_MEM_SET_ALIGN_FLAG(uwAlign)          (uwAlign = ((uwAlign) | OS_MEM_ALIGN_FLAG))
#define OS_MEM_GET_ALIGN_FLAG(uwAlign)          ((uwAlign) & OS_MEM_ALIGN_FLAG)
#define OS_MEM_GET_ALIGN_GAPSIZE(uwAlign)       ((uwAlign) & (~OS_MEM_ALIGN_FLAG))

#define RAM_HEAP_SIZE                           ((OS_SYS_MEM_SIZE) &~ 7)
#define RAM_HEAP_START                          (OS_SYS_MEM_ADDR)

#ifdef CONFIG_DDR_HEAP
#define DDR_HEAP_INIT()                         osHeapInit((VOID *)DDR_HEAP_START, DDR_HEAP_SIZE)
#define DDR_HEAP_ALLOC(sz)                      osHeapAllocAlign((VOID *)DDR_HEAP_START, OS_MEM_ALIGN(sz, DCACHE_LINE_SIZE), DCACHE_LINE_SIZE)
#define DDR_HEAP_FREE(p)                        osHeapFree((VOID *)DDR_HEAP_START, p)
#endif

struct LOS_HEAP_NODE {

    struct LOS_HEAP_NODE* pstPrev;
    UINT32 uwSize:30;
    UINT32 uwUsed: 1;
    UINT32 uwAlign:1;
    UINT8  ucData[];/*lint !e43*/
};

struct LOS_HEAP_MANAGER{
    struct LOS_HEAP_NODE *pstHead;
    struct LOS_HEAP_NODE *pstTail;
    UINT32 uwSize;
#if (LOSCFG_MEM_MUL_POOL == YES)
    VOID *pNextPool;
#endif
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    struct LOS_SLAB_CONTROL_HEADER stSlabCtrlHdr;
#endif
};

/**
 *@ingroup los_heap
 *@brief Initialization heap memory.
 *
 *@par Description:
 *This API is used to initialization heap memory.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pPool   [IN/OUT] A pointer pointed to the memory pool.
 *@param uwSz   [IN] Size of heap memory.
 *
 *@retval TRUE   Initialization success.
 *@retval FALSE  Initialization failed.
 *@par Dependency:
 *<ul><li>los_heap.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern BOOL osHeapInit(VOID *pPool, UINT32 uwSz);

/**
 *@ingroup los_heap
 *@brief Alloc memory block from heap memory.
 *
 *@par Description:
 *This API is used to alloc memory block from heap memory.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pPool   [IN/OUT] A pointer pointed to the memory pool.
 *@param uwSz   [IN] Size of heap memory.
 *
 *@retval VOID*
 *@par Dependency:
 *<ul><li>los_heap.h: the header file that contains the API declaration.</li></ul>
 *@see osHeapFree
 *@since Huawei LiteOS V100R001C00
 */
extern VOID* osHeapAlloc(VOID *pPool, UINT32 uwSz);

/**
 *@ingroup los_heap
 *@brief Alloc aligned memory block from heap memory.
 *
 *@par Description:
 *This API is used to alloc aligned  memory block from heap memory.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pPool   [IN/OUT] A pointer pointed to the memory pool.
 *@param uwSz   [IN] Size of heap memory.
 *@param uwBoundary   [IN] Boundary the heap needs align
 *
 *@retval VOID*
 *@par Dependency:
 *<ul><li>los_heap.h: the header file that contains the API declaration.</li></ul>
 *@see osHeapFree
 *@since Huawei LiteOS V100R001C00
 */
extern VOID* osHeapAllocAlign(VOID *pPool, UINT32 uwSz, UINT32 uwBoundary);

/**
 *@ingroup los_heap
 *@brief Free memory block from heap memory.
 *
 *@par Description:
 *This API is used to free memory block from heap memory.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pPool   [IN/OUT] A pointer pointed to the memory pool.
 *@param pPtr    [IN] Point to be freed.
 *
 *@retval BOOL TRUE  free success  FALSE  free failed
 *@par Dependency:
 *<ul><li>los_heap.h: the header file that contains the API declaration.</li></ul>
 *@see osHeapAlloc
 *@since Huawei LiteOS V100R001C00
 */
extern BOOL osHeapFree(VOID *pPool, VOID* pPtr);

/**
 *@ingroup los_memory
 *@brief Get the memory info from Heap.
 *
 *@par Description:
 *This API is used to get the memory info from Heap.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *@param None.
 *
 *@retval   UINT32  Max size of heap memory being used.
 *
 *@par Dependency:
 *<ul><li>los_heap.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS
 */
#if (LOSCFG_HEAP_MEMORY_PEAK_STATISTICS == YES)
extern UINT32 osHeapGetHeapMemoryPeak(VOID);
#endif

#ifdef __cplusplus
}
#endif


#endif

