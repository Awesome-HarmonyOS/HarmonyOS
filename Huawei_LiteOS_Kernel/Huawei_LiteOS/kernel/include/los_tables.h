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

#include "los_typedef.h"

#define __string(_x) #_x
#define __xstring(_x) __string(_x)

#ifndef LOS_HAL_TABLE_WOW_BEGIN
#define LOS_HAL_TABLE_WOW_BEGIN( _label, _name )                                 \
__asm__(".section \".liteos.table." __xstring(_name) ".wow.begin\",\"aw\"\n"       \
    ".globl " __xstring(LOS_LABEL_DEFN(_label)) "\n"                         \
    ".type    " __xstring(LOS_LABEL_DEFN(_label)) ",object\n"                \
    ".p2align " __xstring(LOSARC_P2ALIGNMENT) "\n"                           \
__xstring(LOS_LABEL_DEFN(_label)) ":\n"                                      \
    ".previous\n"                                                            \
       )
#endif

#ifndef LOS_HAL_TABLE_WOW_END
#define LOS_HAL_TABLE_WOW_END( _label, _name )                                   \
__asm__(".section \".liteos.table." __xstring(_name) ".wow.finish\",\"aw\"\n"      \
    ".globl " __xstring(LOS_LABEL_DEFN(_label)) "\n"                         \
    ".type    " __xstring(LOS_LABEL_DEFN(_label)) ",object\n"                \
    ".p2align " __xstring(LOSARC_P2ALIGNMENT) "\n"                           \
__xstring(LOS_LABEL_DEFN(_label)) ":\n"                                      \
    ".previous\n"                                                            \
       )
#endif

#ifndef LOS_HAL_TABLE_SCATTER_BEGIN
#define LOS_HAL_TABLE_SCATTER_BEGIN( _label, _name )                                 \
__asm__(".section \".liteos.table." __xstring(_name) ".scatter.begin\",\"aw\"\n"       \
    ".globl " __xstring(LOS_LABEL_DEFN(_label)) "\n"                         \
    ".type    " __xstring(LOS_LABEL_DEFN(_label)) ",object\n"                \
    ".p2align " __xstring(LOSARC_P2ALIGNMENT) "\n"                           \
__xstring(LOS_LABEL_DEFN(_label)) ":\n"                                      \
    ".previous\n"                                                            \
       )
#endif

#ifndef LOS_HAL_TABLE_SCATTER_END
#define LOS_HAL_TABLE_SCATTER_END( _label, _name )                                   \
__asm__(".section \".liteos.table." __xstring(_name) ".scatter.finish\",\"aw\"\n"      \
    ".globl " __xstring(LOS_LABEL_DEFN(_label)) "\n"                         \
    ".type    " __xstring(LOS_LABEL_DEFN(_label)) ",object\n"                \
    ".p2align " __xstring(LOSARC_P2ALIGNMENT) "\n"                           \
__xstring(LOS_LABEL_DEFN(_label)) ":\n"                                      \
    ".previous\n"                                                            \
       )
#endif

#ifndef LOS_HAL_TABLE_BEGIN
#define LOS_HAL_TABLE_BEGIN( _label, _name )                                 \
__asm__(".section \".liteos.table." __xstring(_name) ".begin\",\"aw\"\n"       \
    ".globl " __xstring(LOS_LABEL_DEFN(_label)) "\n"                         \
    ".type    " __xstring(LOS_LABEL_DEFN(_label)) ",object\n"                \
    ".p2align " __xstring(LOSARC_P2ALIGNMENT) "\n"                           \
__xstring(LOS_LABEL_DEFN(_label)) ":\n"                                      \
    ".previous\n"                                                            \
       )
#endif

#ifndef LOS_HAL_TABLE_END
#define LOS_HAL_TABLE_END( _label, _name )                                   \
__asm__(".section \".liteos.table." __xstring(_name) ".finish\",\"aw\"\n"      \
    ".globl " __xstring(LOS_LABEL_DEFN(_label)) "\n"                         \
    ".type    " __xstring(LOS_LABEL_DEFN(_label)) ",object\n"                \
    ".p2align " __xstring(LOSARC_P2ALIGNMENT) "\n"                           \
__xstring(LOS_LABEL_DEFN(_label)) ":\n"                                      \
    ".previous\n"                                                            \
       )
#endif

// This macro must be applied to any types whose objects are to be placed in
// tables
#ifndef LOS_HAL_TABLE_TYPE
#define LOS_HAL_TABLE_TYPE LOSBLD_ATTRIB_ALIGN( LOSARC_ALIGNMENT )
#endif

#ifndef LOS_HAL_TABLE_EXTRA
#define LOS_HAL_TABLE_EXTRA( _name ) \
        LOSBLD_ATTRIB_SECTION(".liteos.table." __xstring(_name) ".extra")
#endif

#ifndef LOS_HAL_TABLE_WOW_ENTRY
#define LOS_HAL_TABLE_WOW_ENTRY( _name ) \
        LOSBLD_ATTRIB_SECTION(".liteos.table." __xstring(_name) ".wow.data") \
        LOSBLD_ATTRIB_USED
#endif

#ifndef LOS_HAL_TABLE_SCATTER_ENTRY
#define LOS_HAL_TABLE_SCATTER_ENTRY( _name ) \
        LOSBLD_ATTRIB_SECTION(".liteos.table." __xstring(_name) ".scatter.data") \
        LOSBLD_ATTRIB_USED
#endif

#ifndef LOS_HAL_TABLE_ENTRY
#define LOS_HAL_TABLE_ENTRY( _name ) \
        LOSBLD_ATTRIB_SECTION(".liteos.table." __xstring(_name) ".data") \
        LOSBLD_ATTRIB_USED
#endif

#ifndef LOS_HAL_TABLE_QUALIFIED_ENTRY
#define LOS_HAL_TABLE_QUALIFIED_ENTRY( _name, _qual ) \
        LOSBLD_ATTRIB_SECTION(".liteos.table." __xstring(_name) ".data." \
                              __xstring(_qual))                        \
        LOSBLD_ATTRIB_USED
#endif
