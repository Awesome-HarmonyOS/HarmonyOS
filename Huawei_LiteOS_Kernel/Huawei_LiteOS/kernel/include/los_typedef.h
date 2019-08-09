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

/**@defgroup los_typedef Type define
 * @ingroup kernel
*/

#ifndef _LOS_TYPEDEF_H
#define _LOS_TYPEDEF_H

#include "los_builddef.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


#define INLINE                                              static inline

#ifndef LOS_TYPE_DEF
#define LOS_TYPE_DEF

/* type definitions */
typedef unsigned char                                       UINT8;
typedef unsigned short                                      UINT16;
typedef unsigned int                                        UINT32;
typedef signed char                                         INT8;
typedef signed short                                        INT16;
typedef signed int                                          INT32;
typedef float                                               FLOAT;
typedef double                                              DOUBLE;
typedef char                                                CHAR;

typedef unsigned int                                        BOOL;
typedef unsigned long long                                  UINT64;
typedef signed long long                                    INT64;
typedef unsigned int                                        UINTPTR;
typedef signed int                                          INTPTR;

#if 0
typedef INT32                                               ssize_t;
typedef UINT32                                              size_t;

#ifndef uint8_t
typedef unsigned char   uint8_t;
#endif
#ifndef int8_t
typedef signed char int8_t;
#endif
#ifndef uint16_t
typedef unsigned short  uint16_t;
#endif
#ifndef int16_t
typedef signed short    int16_t;
#endif
#ifndef uint32_t
typedef unsigned int    uint32_t;
#endif
#ifndef int32_t
typedef signed int  int32_t;
#endif
#ifndef int64_t
typedef long long  int64_t;
#endif
#ifndef uint64_t
typedef unsigned long long  uint64_t;
#endif
#endif

#define VOID                                                void
#endif  /*end of #ifndef LOS_TYPE_DEF*/

#ifndef FALSE
#define FALSE                                               ((BOOL)0)
#endif

#ifndef TRUE
#define TRUE                                                ((BOOL)1)
#endif

#ifndef __cplusplus

#ifndef false
#define false                                               FALSE
#endif

#ifndef true
#define true                                                TRUE
#endif

#ifndef bool
#define bool                                                BOOL
#endif

#endif /* __cplusplus */

#ifndef NULL
#define NULL                                                ((VOID *)0)
#endif

#ifdef YES
#undef YES
#endif
#define YES                                                 (1)

#ifdef  NO
#undef  NO
#endif
#define NO                                                  (0)

#define OS_NULL_BYTE                                        ((UINT8)0xFF)
#define OS_NULL_SHORT                                       ((UINT16)0xFFFF)
#define OS_NULL_INT                                         ((UINT32)0xFFFFFFFF)

#ifndef LOS_OK
#define LOS_OK                                              (0)
#endif

#ifndef LOS_NOK
#define LOS_NOK                                             (1)
#endif

#define OS_FAIL                                             (1)
#define OS_ERROR                                            (UINT32)(-1)
#define OS_INVALID                                          (UINT32)(-1)

#define asm                                                 __asm
#ifdef typeof
#undef typeof
#endif
#define typeof                                              __typeof__


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_TYPEDEF_H */
