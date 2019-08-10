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

#ifndef __LOS_COMPILER_H
#define __LOS_COMPILER_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


#ifdef __ICCARM__

  #ifndef   ASM
    #define ASM                     __asm
  #endif

  #ifndef   INLINE
    #define INLINE                  inline
  #endif

  #ifndef   STATIC_INLINE
    #define STATIC_INLINE           static inline
  #endif

  #ifndef   USED
    #define USED                    __root
  #endif

  #ifndef   WEAK
    #define WEAK                    __weak
  #endif

  #ifndef   CLZ
    #define CLZ                     __CLZ
  #endif

#elif defined (__CC_ARM)

  #ifndef   ASM
    #define ASM                     __asm
  #endif

  #ifndef   INLINE
    #define INLINE                  __inline
  #endif

  #ifndef   STATIC_INLINE
    #define STATIC_INLINE           static __inline
  #endif

  #ifndef   USED
    #define USED                    __attribute__((used))
  #endif

  #ifndef   WEAK
    #define WEAK                    __attribute__((weak))
  #endif

  #ifndef   CLZ
  #define   CLZ                     __clz
  #endif

#pragma anon_unions

#elif defined (__GNUC__)

  #ifndef   ASM
    #define ASM                     __asm
  #endif

  #ifndef   INLINE
    #define INLINE                  inline
  #endif

  #ifndef   STATIC_INLINE
    #define STATIC_INLINE           static inline
  #endif

  #ifndef   USED
    #define USED                    __attribute__((used))
  #endif

  #ifndef   WEAK
    #define WEAK                    __attribute__((weak))
  #endif

  #ifndef   CLZ
  #define   CLZ                     __builtin_clz
  #endif

#elif defined (__ICC430__)

#ifndef   ASM
  #define ASM                     __asm
#endif

#ifndef   INLINE
  #define INLINE                  inline
#endif

#ifndef   STATIC_INLINE
  #define STATIC_INLINE           static inline
#endif

#ifndef   USED
  #define USED
#endif

#ifndef   WEAK
  #define WEAK
#endif

#ifndef   CLZ
  extern int __clz (unsigned long);
  #define CLZ                     __clz
#endif

#elif defined (__TI_COMPILER_VERSION__)

#ifndef   ASM
  #define ASM                     __asm
#endif

#ifndef   INLINE
  #define INLINE                  inline
#endif

#ifndef   STATIC_INLINE
  #define STATIC_INLINE           static inline
#endif

#ifndef   USED
  #define USED
#endif

#ifndef   WEAK
  #define WEAK
#endif

#ifndef   CLZ
  extern int __clz (unsigned long);
  #define CLZ                     __clz
#endif

#else
  #error Unknown compiler.
#endif


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* __LOS_COMPILER_H */
