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

#ifndef _CCSMACROS_H
#define _CCSMACROS_H

#if __LARGE_DATA_MODEL__ == 1
#define PTR_SIZE        4
#define PCMP            CMPX.A
#define PMOV            MOVA
#define PSUB            SUBX.A
#else
#define PTR_SIZE        2
#define PCMP            CMP.W
#define PMOV            MOV.W
#define PSUB            SUB.W
#endif

#if __LARGE_CODE_MODEL__ == 1
#define FRET            RETA
#define FCALL           CALLA
#else
#define FRET            RET
#define FCALL           CALL
#endif

#if (__LARGE_DATA_MODEL__ == 1) || (__LARGE_CODE_MODEL__ == 1)
#define REG_SIZE        4
#define XMOV            MOVA
#define XPUSH           PUSH.A
#define XPUSHM          PUSHM.A
#define XPOP            POP.A
#define XPOPM           POPM.A
#else
#define REG_SIZE        2
#define XMOV            MOV.W
#define XPUSH           PUSH.W
#define XPUSHM          PUSHM.W
#define XPOP            POP.W
#define XPOPM           POPM.W
#endif

#endif

