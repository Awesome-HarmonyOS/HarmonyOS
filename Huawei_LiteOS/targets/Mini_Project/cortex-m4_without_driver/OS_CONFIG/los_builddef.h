/*----------------------------------------------------------------------------
 * Copyright (c) <2016-2018>, <Huawei Technologies Co., Ltd>
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
 * HuaweiLite OS may be subject to applicable export control laws and regulations, which might
 * include those applicable to HuaweiLite OS of U.S. and the country in which you are located.
 * Import, export and usage of HuaweiLite OS in any manner by you shall be in compliance with such
 * applicable export control laws and regulations.
 *---------------------------------------------------------------------------*/

/**@defgroup los_builddef
 * @ingroup kernel
 */

#ifndef _LOS_BUILDEF_H
#define _LOS_BUILDEF_H
#include "los_compiler.h"
#include "string.h"
#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

/**
 * @ingroup los_builddef
 * Define inline keyword
 */

/**
 * @ingroup los_builddef
 * Little endian
 */
#define OS_LITTLE_ENDIAN                                    0x1234

/**
 * @ingroup los_builddef
 * Big endian
 */
#define OS_BIG_ENDIAN                                       0x4321

/**
 * @ingroup los_builddef
 * Byte order
 */
#ifndef OS_BYTE_ORDER
#define OS_BYTE_ORDER                                       OS_LITTLE_ENDIAN
#endif

/* Define OS code data sections */
/*The indicator function is inline*/

/**
 * @ingroup los_builddef
 * Allow inline sections
 */
#ifndef LITE_OS_SEC_ALW_INLINE
#define LITE_OS_SEC_ALW_INLINE      //__attribute__((always_inline))
#endif

/**
 * @ingroup los_builddef
 * Vector table section
 */
#ifndef LITE_OS_SEC_VEC
#define LITE_OS_SEC_VEC          __attribute__ ((section(".data.vector")))
#endif

/**
 * @ingroup los_builddef
 * .Text section (Code section)
 */
#ifndef LITE_OS_SEC_TEXT
#define LITE_OS_SEC_TEXT            //__attribute__((section(".sram.text")))
#endif

/**
 * @ingroup los_builddef
 * .Text.ddr section
 */
#ifndef LITE_OS_SEC_TEXT_MINOR
#define LITE_OS_SEC_TEXT_MINOR      // __attribute__((section(".dyn.text")))
#endif

/**
 * @ingroup los_builddef
 * .Text.init section
 */
#ifndef LITE_OS_SEC_TEXT_INIT
#define LITE_OS_SEC_TEXT_INIT       //__attribute__((section(".dyn.text")))
#endif

/**
 * @ingroup los_builddef
 * Redirect .Text  section (Code section)
 */
#ifndef LITE_OS_SEC_TEXT_REDIRECT
#define LITE_OS_SEC_TEXT_REDIRECT                   LITE_OS_SEC_TEXT
#endif

/**
 * @ingroup los_builddef
 * Redirect .Text.ddr section
 */
#ifndef LITE_OS_SEC_TEXT_MINOR_REDIRECT
#define LITE_OS_SEC_TEXT_MINOR_REDIRECT                   LITE_OS_SEC_TEXT_MINOR
#endif

/**
 * @ingroup los_builddef
 * Redirect .Text.init section
 */
#ifndef LITE_OS_SEC_TEXT_INIT_REDIRECT
#define LITE_OS_SEC_TEXT_INIT_REDIRECT                   LITE_OS_SEC_TEXT_INIT
#endif

/**
 * @ingroup los_builddef
 * .Data section
 */
#ifndef LITE_OS_SEC_DATA
#define LITE_OS_SEC_DATA  //__attribute__((section(".dyn.data")))
#endif

/**
 * @ingroup los_builddef
 * .Data.init section
 */
#ifndef LITE_OS_SEC_DATA_INIT
#define LITE_OS_SEC_DATA_INIT  //__attribute__((section(".dyn.data")))
#endif

/**
 * @ingroup los_builddef
 * Not initialized variable section
 */
#ifndef LITE_OS_SEC_BSS
#define LITE_OS_SEC_BSS  //__attribute__((section(".sym.bss")))
#endif

/**
 * @ingroup los_builddef
 * .bss.ddr section
 */
#ifndef LITE_OS_SEC_BSS_MINOR
#define LITE_OS_SEC_BSS_MINOR
#endif

/**
 * @ingroup los_builddef
 * .bss.init sections
 */
#ifndef LITE_OS_SEC_BSS_INIT
#define LITE_OS_SEC_BSS_INIT
#endif

#ifndef LITE_OS_SEC_TEXT_DATA
#define LITE_OS_SEC_TEXT_DATA       //__attribute__((section(".dyn.data")))
#define LITE_OS_SEC_TEXT_BSS        //__attribute__((section(".dyn.bss")))
#define LITE_OS_SEC_TEXT_RODATA     //__attribute__((section(".dyn.rodata")))
#endif

#ifndef LITE_OS_SEC_SYMDATA
#define LITE_OS_SEC_SYMDATA         //__attribute__((section(".sym.data")))
#endif

#ifndef LITE_OS_SEC_SYMBSS
#define LITE_OS_SEC_SYMBSS          //__attribute__((section(".sym.bss")))
#endif


#ifndef LITE_OS_SEC_KEEP_DATA_DDR
#define LITE_OS_SEC_KEEP_DATA_DDR   //__attribute__((section(".keep.data.ddr")))
#endif

#ifndef LITE_OS_SEC_KEEP_TEXT_DDR
#define LITE_OS_SEC_KEEP_TEXT_DDR   //__attribute__((section(".keep.text.ddr")))
#endif

#ifndef LITE_OS_SEC_KEEP_DATA_SRAM
#define LITE_OS_SEC_KEEP_DATA_SRAM  //__attribute__((section(".keep.data.sram")))
#endif

#ifndef LITE_OS_SEC_KEEP_TEXT_SRAM
#define LITE_OS_SEC_KEEP_TEXT_SRAM  //__attribute__((section(".keep.text.sram")))
#endif

#ifndef LITE_OS_SEC_BSS_MINOR
#define LITE_OS_SEC_BSS_MINOR
#endif

/**
 * @ingroup los_builddef
 * .text.libsec section
 */
#ifndef LIB_SECURE_SEC_TEXT
#define LIB_SECURE_SEC_TEXT
#endif

/**
 * @ingroup los_builddef
 * .text.libc section
 */
#ifndef LIBC_SEC_TEXT
#define LIBC_SEC_TEXT
#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */


#endif /* _LOS_BUILDEF_H */
