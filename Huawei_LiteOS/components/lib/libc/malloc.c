/************************************************************************
 * Copyright (c) <2013-2015>, <Huawei Technologies Co., Ltd>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice,this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice,this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************/

/************************************************************************
 * Notice of Export Control Law
 * ===============================================
 * Huawei LiteOS may be subject to applicable export control laws and regulations,
 * which might include those applicable to Huawei LiteOS of U.S. and the country
 * in which you are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in
 * compliance with such applicable export control laws and regulations.
 ************************************************************************/

#include "stdlib.h"
#include "los_memory.h"
#include "string.h"

#if defined(LOS_LIBC_MALLOC_ALIGN) && !defined(LOS_LIBC_MALLOC_ALIGN_SIZE)
#error "macro LOS_LIBC_MALLOC_ALIGN_SIZE undefined"
#endif


/*****************************************************************************
Function         :      free
Description      :      Deallocates the memory previously allocated by a call to calloc, malloc, or
                realloc. The argument ptr points to the space that was previously allocated.
                If ptr points to a memory block that was not allocated with calloc, malloc,
                or realloc, or is a space that has been deallocated, then the result is
                undefined.
Input            :      [1] void *ptr, pointed to the memory need to free.
Output           :      nothing.
Return           :      No value is returned.
*****************************************************************************/
void free(void *ptr)
{
    if (ptr == NULL)
        return;

    LOS_MemFree((void *)OS_SYS_MEM_ADDR, ptr);/*lint !e534*/
}


/*****************************************************************************
Function         :      malloc
Description      :      Allocates the requested memory and returns a pointer to it. The requested
                size is size bytes. The value of the space is indeterminate.
Input            :      [1] size_t size, spcified the size need to allocate.
Output           :      nothing.
Return           :      On success a pointer to the requested space is returned.
                On failure a null pointer is returned.
*****************************************************************************/
void *malloc(size_t size) /*lint !e31 !e10*/
{
    void *ptr = NULL; /*lint !e64 !e10*/

    if (size == 0)
        return NULL; /*lint !e64*/

#if defined(LOS_LIBC_MALLOC_ALIGN)
    ptr = LOS_MemAllocAlign((void *)OS_SYS_MEM_ADDR, (UINT32)size, LOS_LIBC_MALLOC_ALIGN_SIZE);
#else
    ptr = LOS_MemAlloc((void *)OS_SYS_MEM_ADDR, (UINT32)size);
#endif

    return ptr;
}


void *zalloc(size_t size) /*lint !e10*/
{
    void *ptr = malloc (size);

    if (ptr != NULL)
    {
        memset((void *)ptr, (int)0, size);
    }

    return ptr;
}


/*****************************************************************************
Function         :      calloc
Description      :    Allocates the requested memory and returns a pointer to it. The requested
                size is nitems each size bytes long (total memory requested is nitems*size).
                The space is initialized to all zero bits.
Input            :      [1] size_t nitems,
                [2] size_t size,
Output           :      nothing.
Return           :      On success a pointer to the requested space is returned.
                On failure a null pointer is returned.
*****************************************************************************/
void *calloc(size_t nitems, size_t size) /*lint !e578*/
{
    return zalloc(nitems * size);
}


/*****************************************************************************
Function         :      memalign
Description      :      allocates a block of size bytes whose address is a multiple of boundary.
               The boundary must be a power of two!
Input            :      [1] size_t size, spcified the size need to allocate.
               [2] size_t boundary, the returned memory address will be a multiple of boundary.
               This argument must be a power of two. This property is not checked by
               memalign, so misuse may result in random runtime errors.
Output           :      nothing.
Return           :      On success a pointer to the requested space is returned.
               On failure a null pointer is returned.
*****************************************************************************/
void *memalign (size_t boundary, size_t size)  /*lint !e18 !e578*/
{
    /*lint !e18 !e578*/
    void *ptr = NULL;

    if(size == 0)
        return NULL; /*lint !e64*/

    ptr = LOS_MemAllocAlign((void *)OS_SYS_MEM_ADDR, (UINT32)size, (UINT32)boundary);

    return ptr; /*lint !e64*/
}


/*****************************************************************************
Function         :      realloc
Description      :      Attempts to resize the memory block pointed to by ptr that was previously
                allocated with a call to malloc or calloc. The contents pointed to by ptr are
                unchanged. If the value of size is greater than the previous size of the
                block, then the additional bytes have an undeterminate value. If the value
                of size is less than the previous size of the block, then the difference of
                bytes at the end of the block are freed. If ptr is null, then it behaves like
                malloc. If ptr points to a memory block that was not allocated with calloc
                or malloc, or is a space that has been deallocated, then the result is
                undefined. If the new space cannot be allocated, then the contents pointed
                to by ptr are unchanged. If size is zero, then the memory block is completely
                freed.
Input            :      [1] void *ptr, pointed to the memory which need to remalloc.
                [2] size_t size, specified the size to remalloc.
Output           :      nothing.
Return           :      On success a pointer to the memory block is returned (which may be in a
                different location as before).
               On failure or if size is zero, a null pointer is returned.
*****************************************************************************/
void *realloc(void *ptr, size_t size)
{
    if (ptr == NULL)
    {
        return malloc(size); /*lint !e64*/
    }

    if (size == 0)
    {
        free(ptr);
        return NULL;
    }

    return LOS_MemRealloc((void *)OS_SYS_MEM_ADDR, (void *)ptr, (UINT32)size);
}


#if OS_SYS_NOCACHEMEM_SIZE
/*****************************************************************************
Function       :   nocache_free
Description      :   Deallocates the memory previously allocated by a call to calloc, malloc, or
             realloc. The argument ptr points to the space that was previously allocated.
             If ptr points to a memory block that was not allocated with calloc, malloc,
             or realloc, or is a space that has been deallocated, then the result is
             undefined.
Input          :   [1] void *ptr, pointed to the memory need to free.
Output          :   nothing.
Return          :   No value is returned.
*****************************************************************************/
void nocache_free(void *ptr)
{
    if(ptr == NULL)
        return;

    LOS_MemFree((void *)OS_SYS_NOCACHEMEM_ADDR, ptr);
}


/*****************************************************************************
Function       :   nocache_malloc
Description      :   Allocates the requested memory and returns a pointer to it. The requested
             size is size bytes. The value of the space is indeterminate.
Input          :   [1] size_t size, spcified the size need to allocate.
Output          :   nothing.
Return          :   On success a pointer to the requested space is returned.
             On failure a null pointer is returned.
*****************************************************************************/
void *nocache_malloc(size_t size)
{
    void *ptr = NULL;

    if(size == 0)
        return NULL;

    ptr = LOS_MemAlloc((void *)OS_SYS_NOCACHEMEM_ADDR,  (UINT32)size);

    return ptr;
}


void *nocache_zalloc(size_t size)
{
    void *ptr = nocache_malloc(size);

    if (ptr != NULL)
    {
        memset((void *)ptr, (int)0, size);
    }

    return ptr;
}


/*****************************************************************************
Function       :   nocache_calloc
Description      :   Allocates the requested memory and returns a pointer to it. The requested
             size is nitems each size bytes long (total memory requested is nitems*size).
             The space is initialized to all zero bits.
Input          :   [1] size_t nitems,
             [2] size_t size,
Output          :   nothing.
Return          :   On success a pointer to the requested space is returned.
             On failure a null pointer is returned.
*****************************************************************************/
void *nocache_calloc(size_t nitems, size_t size)
{
    return nocache_zalloc (nitems * size);
}


/*****************************************************************************
Function       :   nocache_realloc
Description      :   Attempts to resize the memory block pointed to by ptr that was previously
             allocated with a call to malloc or calloc. The contents pointed to by ptr are
             unchanged. If the value of size is greater than the previous size of the
             block, then the additional bytes have an undeterminate value. If the value
             of size is less than the previous size of the block, then the difference of
             bytes at the end of the block are freed. If ptr is null, then it behaves like
             malloc. If ptr points to a memory block that was not allocated with calloc
             or malloc, or is a space that has been deallocated, then the result is
             undefined. If the new space cannot be allocated, then the contents pointed
             to by ptr are unchanged. If size is zero, then the memory block is completely
             freed.
Input          :   [1] void *ptr, pointed to the memory which need to remalloc.
             [2] size_t size, specified the size to remalloc.
Output          :   nothing.
Return          :   On success a pointer to the memory block is returned (which may be in a
             different location as before).
             On failure or if size is zero, a null pointer is returned.
*****************************************************************************/
void *nocache_realloc(void *ptr, size_t size)
{
    if (ptr == NULL)
    {
        ptr = malloc(size);
        return ptr;
    }

    if (size == 0)
    {
        free(ptr);
        return NULL;
    }

    return LOS_MemRealloc((void *)OS_SYS_NOCACHEMEM_ADDR, (void *)ptr, (UINT32)size);
}
#endif

/* EOF malloc.c */

