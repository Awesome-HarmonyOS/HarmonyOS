/*----------------------------------------------------------------------------
 * Copyright (c) <2013-2017>, <Huawei Technologies Co., Ltd>
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
#ifndef _LOS_MEMORY_H
#define _LOS_MEMORY_H
#include "los_base.h"
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
#include <los_slab.ph>
#endif
#include <los_heap.ph>

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

typedef struct __s_LOS_MEM_STATUS {
    UINT32 totalSize;
    UINT32 usedSize;
    UINT32 freeSize;
    UINT32 allocCount;
    UINT32 freeCount;
}LOS_MEM_STATUS;

#if (LOSCFG_MEMORY_BESTFIT == YES)

#if (LOSCFG_PLATFORM_EXC == YES)
#define OS_MEM_ENABLE_ALLOC_CHECK
#endif
#if (LOSCFG_BASE_MEM_NODE_SIZE_CHECK == YES)
#define OS_MEM_CHECK_DEBUG
#endif

typedef VOID (*MALLOC_HOOK)(VOID);

extern MALLOC_HOOK g_MALLOC_HOOK;

/**
 *@ingroup los_memory
 *@brief Get the size of memory totally used.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to get the size of memory totally used in memory pool.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>The input pPool parameter must be initialized via func LOS_MemInit.</li>
 *</ul>
 *
 *@param  pPool           [IN] A pointer pointed to the memory pool.
 *
 *@retval #LOS_NOK        The incoming parameter pPool is NULL.
 *@retval #UINT32         The size of the memory pool used.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemTotalUsedGet(VOID *pPool);

/**
 *@ingroup los_memory
 *@brief Get the number of free memory nodes.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to get the number of free memory nodes in memory pool.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>The input pPool parameter must be initialized via func LOS_MemInit.</li>
 *</ul>
 *
 *@param  pPool           [IN] A pointer pointed to the memory pool.
 *
 *@retval #LOS_NOK        The incoming parameter pPool is NULL.
 *@retval #UINT32         The number of free memory nodes.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemFreeBlksGet(VOID *pPool);

/**
 *@ingroup los_memory
 *@brief Get the number of used memory nodes.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to get the number of used memory nodes in memory pool.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>The input pPool parameter must be initialized via func LOS_MemInit.</li>
 *</ul>
 *
 *@param  pPool           [IN] A pointer pointed to the memory pool.
 *
 *@retval #LOS_NOK        The incoming parameter pPool is NULL.
 *@retval #UINT32         The number of used memory nodes.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemUsedBlksGet(VOID *pPool);

/**
 *@ingroup los_memory
 *@brief Get the task ID of a used memory node.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to get the task ID of a used memory node.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>The input pPtr parameter must be allocated by LOS_MemAlloc or LOS_MemAllocAlign.</li>
 *<li>This interface only support obtain the task ID of a used memory node which is allocated from the system memory pool (OS_SYS_MEM_ADDR) at present.</li>
 *</ul>
 *
 *@param  pPtr               [IN] A used memory node.
 *
 *@retval #OS_INVALID        The incoming parameter pPtr is illegal.
 *@retval #UINT32            The task ID of used memory node pPtr.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemTaskIdGet(VOID *pPool);

/**
 *@ingroup los_memory
 *@brief Get the address of last node.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to get the address of last node.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>The input pPool parameter must be initialized via func LOS_MemInit.</li>
 *<li>The last node of memory pool is not the end node.</li>
 *</ul>
 *
 *@param  pPtr               [IN] A pointer pointed to the memory pool.
 *
 *@retval #LOS_NOK           The incoming parameter pPool is NULL.
 *@retval #UINT32            The address of the last used node that casts to UINT32.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemLastUsedGet(VOID *pPool);

/**
 *@ingroup los_memory
 *@brief Check the memory pool Integrity.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to check the memory pool Integrity.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>The input pPool parameter must be initialized via func LOS_MemInit.</li>
 *<li>LOS_MemIntegrityCheck will be called by malloc function when the macro of LOSCFG_BASE_MEM_NODE_INTEGRITY_CHECK is defined in LiteOS.</li>
 *<li>LOS_MemIntegrityCheck function can be called by user anytime.</li>
 *</ul>
 *
 *@param  pPool              [IN] A pointer pointed to the memory pool.
 *
 *@retval #LOS_NOK           The memory pool (pPool) is impaired.
 *@retval #LOS_OK            The memory pool (pPool) is integrated.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemIntegrityCheck(VOID *pPool);

/**
 * @ingroup los_memory
 *  Define a mem size check intensity
 *
 *  Lowest mem check.
 */
#define LOS_MEM_CHECK_LEVEL_LOW      0

/**
 * @ingroup los_memory
 * Define a mem size check intensity
 *
 * Highest mem check.
 */
#define LOS_MEM_CHECK_LEVEL_HIGH      1

/**
 * @ingroup los_memory
 * Define a mem size check intensity
 *
 * disable mem check.
 */
#define LOS_MEM_CHECK_LEVEL_DISABLE       0xff

/**
 * @ingroup los_memory
 * Define a mem size check intensity
 *
 * default intensity set mem check.
 */
#define LOS_MEM_CHECK_LEVEL_DEFAULT       LOS_MEM_CHECK_LEVEL_DISABLE

/**
 *@ingroup los_memory
 *@brief Check the size of memory node specified.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to check the size of memory node.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>The input pPool parameter must be initialized via func LOS_MemInit.</li>
 *<li>The input pPtr parameter must be allocated by LOS_MemAlloc or LOS_MemAllocAlign.</li>
 *<li>The function will be called by function specified, such as memset or memcpy.</li>
 *<li>The feature can be enabled when you set the macro value of LOSCFG_BASE_MEM_NODE_SIZE_CHECK as YES.</li>
 *<li>You had better set memory check level as LOS_MEM_CHECK_LEVEL_DISABLE when copy bin file.</li>
 *</ul>
 *
 *@param  pPool              [IN]  A pointer pointed to the memory pool.
 *@param  pPtr               [IN]  A pointer pointed to the source node.
 *@param  puwTotalSize       [OUT] A pointer to save total size, must point to valid memory.
 *@param  puwAvailSize       [OUT] A pointer to save available size, must point to valid memory.
 *
 *@retval #OS_ERRNO_MEMCHECK_DISABLED           Memcheck function does not open.
 *@retval #OS_ERRNO_MEMCHECK_NOT_INIT           Memcheck function does not init.
 *@retval #OS_ERRNO_MEMCHECK_PARA_NULL          The pool or pPtr is NULL.
 *@retval #OS_ERRNO_MEMCHECK_OUTSIDE            The pPtr address is not in the reasonable range.
 *@retval #OS_ERRNO_MEMCHECK_NO_HEAD            Can't find the control head node from pPtr.
 *@retval #OS_ERRNO_MEMCHECK_WRONG_LEVEL        The memory check level is illegal.
 *@retval #LOS_OK                                 Success to get total size and available size of the memory node (pPtr).
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MemCheckLevelSet | LOS_MemCheckLevelGet
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemNodeSizeCheck(VOID *pPool, VOID *pPtr, UINT32 *puwTotalSize, UINT32 *puwAvailSize);

/**
 *@ingroup los_memory
 *@brief Set the memory check level.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to set the memory check level.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>There are three level you can set.</li>
 *<li>The legal level are LOS_MEM_CHECK_LEVEL_LOW, LOS_MEM_CHECK_LEVEL_HIGH, LOS_MEM_CHECK_LEVEL_DISABLE.</li>
 *</ul>
 *
 *@param  ucLevel                                  [IN] The level what you want to set.
 *
 *@retval #LOS_ERRNO_MEMCHECK_WRONG_LEVEL           The memory check level what you want to set is illegal.
 *@retval #LOS_OK                                  Success to set the memory check level.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MemNodeSizeCheck | LOS_MemCheckLevelGet
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemCheckLevelSet(UINT8 ucLevel);

/**
 *@ingroup los_memory
 *@brief Get the memory check level.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to get the current memory check level.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  None
 *
 *@retval #UINT8           The current memory check level.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MemNodeSizeCheck | LOS_MemCheckLevelSet
 *@since Huawei LiteOS V100R001C00
 */
extern UINT8 LOS_MemCheckLevelGet(VOID);
#else

/**
 *@ingroup los_memory
 *@brief calculate heap information.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to calculate heap information.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>One parameter of this interface is a pointer, it should be a correct value, otherwise, the system may be abnormal.</li>
 *</ul>
 *
 *@param  status        [OUT] Type  #LOS_MEM_STATUS* Pointer to the heap status structure to be obtained.
 *@param  pPool    [IN] Pointer to the memory pool that contains the memory block to be allocated.
 *
 *@retval #LOS_OK        The heap status calculate success.
 *@retval #LOS_NOK       The heap status calculate with some error.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MemAlloc | LOS_MemRealloc | LOS_MemFree
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemStatisticsGet(VOID *pPool, LOS_MEM_STATUS *pstStatus);


/**
 *@ingroup los_memory
 *@brief calculate heap max free block size.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to calculate heap max free block size.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  None.
 *
 *@retval #UINT32        The  max free block size.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MemAlloc | LOS_MemRealloc | LOS_MemFree
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemGetMaxFreeBlkSize(VOID *pPool);
#endif

/**
 *@ingroup los_memory
 *@brief Initialize dynamic memory.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to initialize the dynamic memory of a doubly linked list.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>The uwSize parameter value should match the following two conditions : 1) Be less than or equal to the Memory pool size; 2) Be greater than the size of OS_MEM_MIN_POOL_SIZE.</li>
 *<li>Call this API when dynamic memory needs to be initialized during the startup of Huawei LiteOS.</li>
 *<li>The parameter input must be four byte-aligned.</li>
 *<li>The init area [pPool, pPool + uwSize] should not conflict with other pools.</li>
 *</ul>
 *
 *@param pPool          [IN] Starting address of memory.
 *@param uwSize         [IN] Memory size.
 *
 *@retval #LOS_NOK    The dynamic memory fails to be initialized.
 *@retval #LOS_OK     The dynamic memory is successfully initialized.
 *@par Dependency:
 *<ul>
 *<li>los_memory.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemInit(VOID *pPool, UINT32 uwSize);

/**
 *@ingroup los_memory
 *@brief Allocate dynamic memory.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to allocate a memory block of which the size is specified.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>The input pPool parameter must be initialized via func LOS_MemInit.</li>
 *<li>The size of the input parameter uwSize can not be greater than the memory pool size that specified at the second input parameter of LOS_MemInit.</li>
 *<li>The size of the input parameter uwSize must be four byte-aligned.</li>
 *</ul>
 *
 *@param  pPool    [IN] Pointer to the memory pool that contains the memory block to be allocated.
 *@param  uwSize   [IN] Size of the memory block to be allocated (unit: byte).
 *
 *@retval #NULL          The memory fails to be allocated.
 *@retval #VOID*         The memory is successfully allocated with the starting address of the allocated memory block returned.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MemRealloc | LOS_MemAllocAlign | LOS_MemFree
 *@since Huawei LiteOS V100R001C00
 */
extern VOID *LOS_MemAlloc (VOID *pPool, UINT32  size);

/**
 *@ingroup los_memory
 *@brief Free dynamic memory.
 *
 *@par Description:
 *<li>This API is used to free specified dynamic memory that has been allocated.</li>
 *@attention
 *<ul>
 *<li>The input pPool parameter must be initialized via func LOS_MemInit.</li>
 *<li>The input pMem parameter must be allocated by LOS_MemAlloc or LOS_MemAllocAlign or LOS_MemRealloc.</li>
 *</ul>
 *
 *@param  pPool  [IN] Pointer to the memory pool that contains the dynamic memory block to be freed.
 *@param  pMem   [IN] Starting address of the memory block to be freed.
 *
 *@retval #LOS_NOK          The memory block fails to be freed
 *@retval #LOS_OK           The memory block is successfully freed.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MemAlloc | LOS_MemRealloc | LOS_MemAllocAlign
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemFree(VOID *pPool, VOID *pMem);

/**
 *@ingroup los_memory
 *@brief Re-allocate a memory block.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to allocate a new memory block of which the size is specified by size if the original memory block size is insufficient. The new memory block will copy the data in the original memory block of which the address is specified by ptr. The size of the new memory block determines the maximum size of data to be copied. After the new memory block is created, the original one is freed.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>The input pPool parameter must be initialized via func LOS_MemInit.</li>
 *<li>The input pPtr parameter must be allocated by LOS_MemAlloc.</li>
 *<li>The size of the input parameter uwSize can not be greater than the memory pool size that specified at the second input parameter of LOS_MemInit.</li>
 *<li>The size of the input parameter uwSize must be aligned as follows: 1) if the pPtr is allocated by LOS_MemAlloc, it must be four byte-aligned; 2) if the pPtr is allocated by LOS_MemAllocAlign, it must be aligned with the size of the input parameter uwBoundary of LOS_MemAllocAlign.</li>
 *</ul>
 *
 *@param  pPool      [IN] Pointer to the memory pool that contains the original and new memory blocks.
 *@param  pPtr       [IN] Address of the original memory block.
 *@param  uwSize     [IN] Size of the new memory block.
 *
 *@retval #NULL          The memory fails to be re-allocated.
 *@retval #VOID*         The memory is successfully re-allocated with the starting address of the new memory block returned.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MemAlloc | LOS_MemAllocAlign | LOS_MemFree
 *@since Huawei LiteOS V100R001C00
 */
extern VOID *LOS_MemRealloc(VOID *pPool, VOID *pPtr, UINT32 uwSize);

/**
 *@ingroup los_memory
 *@brief Allocate aligned memory.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to allocate memory blocks of specified size and of which the starting addresses are aligned on a specified boundary.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>The input pPool parameter must be initialized via func LOS_MemInit.</li>
 *<li>The size of the input parameter uwSize can not be greater than the memory pool size that specified at the second input parameter of LOS_MemInit.</li>
 *<li>The alignment parameter value must be a power of 2 with the minimum value being 4.</li>
 *</ul>
 *
 *@param  pPool       [IN] Pointer to the memory pool that contains the memory blocks to be allocated.
 *@param  uwSize      [IN] Size of the memory to be allocated.
 *@param  uwBoundary  [IN] Boundary on which the memory is aligned.
 *
 *@retval #NULL          The memory fails to be allocated.
 *@retval #VOID*         The memory is successfully allocated with the starting address of the allocated memory returned.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MemAlloc | LOS_MemRealloc | LOS_MemFree
 *@since Huawei LiteOS V100R001C00
 */
extern VOID *LOS_MemAllocAlign (VOID *pPool, UINT32 uwSize, UINT32 uwBoundary);

#if (LOSCFG_MEM_MUL_POOL == YES)
/**
 *@ingroup los_memory
 *@brief Deinitialize dynamic memory.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to deinitialize the dynamic memory of a doubly linked list.</li>
 *</ul>
 *
 *@param pPool          [IN] Starting address of memory.
 *
 *@retval #LOS_NOK    The dynamic memory fails to be deinitialized.
 *@retval #LOS_OK     The dynamic memory is successfully deinitialized.
 *@par Dependency:
 *<ul>
 *<li>los_memory.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemDeInit(VOID *pPool);

/**
 *@ingroup los_memory
 *@brief Print infomation about all pools.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to print infomation about all pools.</li>
 *</ul>
 *
 *@retval #UINT32   The pool number.
 *@par Dependency:
 *<ul>
 *<li>los_memory.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemPoolList(VOID);
#endif


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_MEMORY_H */
