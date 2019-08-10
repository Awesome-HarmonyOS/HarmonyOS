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

#include "los_multipledlinkhead.inc"

#define IF_ELSE(mask, if_do, else_do) if(uwSize & (mask)){if_do}else{else_do}
// cppcheck-suppress *
#define BIT_NUM(num) return num;
#define BIT_NONE BIT_NUM(0xfffffff)

LITE_OS_SEC_ALW_INLINE STATIC_INLINE UINT32 LOS_Log2(UINT32 uwSize)
{
    IF_ELSE(0x80000000, \
			BIT_NONE, \
            IF_ELSE(0x7fff0000, \
                    IF_ELSE(0x7f000000,\
                            IF_ELSE(0x70000000,\
                                    IF_ELSE(0x40000000,\
                                            BIT_NUM(30),\
                                            IF_ELSE(0x20000000, BIT_NUM(29), BIT_NUM(28))), \
                                    IF_ELSE(0x0c000000, \
                                            IF_ELSE(0x08000000, BIT_NUM(27), BIT_NUM(26)), \
                                            IF_ELSE(0x02000000, BIT_NUM(25), BIT_NUM(24)))), \
                            IF_ELSE(0x00f00000, \
                                    IF_ELSE(0x00c00000, \
                                            IF_ELSE(0x00800000, BIT_NUM(23), BIT_NUM(22)), \
                                            IF_ELSE(0x00200000, BIT_NUM(21), BIT_NUM(20))), \
                                    IF_ELSE(0x000c0000,\
                                            IF_ELSE(0x00080000, BIT_NUM(19), BIT_NUM(18)), \
                                            IF_ELSE(0x00020000, BIT_NUM(17), BIT_NUM(16))))), \
                    IF_ELSE(0x0000ff00, \
                            IF_ELSE(0x0000f000, \
                                    IF_ELSE(0x0000c000, \
                                            IF_ELSE(0x00008000, BIT_NUM(15), BIT_NUM(14)), \
                                            IF_ELSE(0x00002000, BIT_NUM(13), BIT_NUM(12))), \
                                    IF_ELSE(0x00000c00, \
                                            IF_ELSE(0x00000800, BIT_NUM(11), BIT_NUM(10)), \
                                            IF_ELSE(0x00000200, BIT_NUM(9), BIT_NUM(8)))), \
                            IF_ELSE(0x000000f0, \
                                    IF_ELSE(0x000000c0, \
                                            IF_ELSE(0x00000080, BIT_NUM(7), BIT_NUM(6)), \
                                            IF_ELSE(0x00000020, BIT_NUM(5), BIT_NUM(4))), \
                                    IF_ELSE(0x0000000c, \
                                            IF_ELSE(0x00000008, BIT_NUM(3), BIT_NUM(2)), \
                                            IF_ELSE(0x00000002, BIT_NUM(1), BIT_NUM(0)))))))\

}

LITE_OS_SEC_TEXT_INIT VOID LOS_DLnkInitMultiHead(VOID *pHeadAddr)
{
    LOS_MULTIPLE_DLNK_HEAD *pstHead = (LOS_MULTIPLE_DLNK_HEAD *)pHeadAddr;
    LOS_DL_LIST *pstListHead = pstHead->stListHead;
    UINT32 uwIdx;

    for (uwIdx = 0; uwIdx < OS_MULTI_DLNK_NUM; ++uwIdx, ++pstListHead)
    {
        LOS_ListInit(pstListHead);
    }
}

LITE_OS_SEC_TEXT_MINOR LOS_DL_LIST *LOS_DLnkMultiHead(VOID *pHeadAddr, UINT32 uwSize)
{
    LOS_MULTIPLE_DLNK_HEAD *pstHead = (LOS_MULTIPLE_DLNK_HEAD *)pHeadAddr;
    UINT32 uwIdx =  LOS_Log2(uwSize);

    if(uwIdx > OS_MAX_MULTI_DLNK_LOG2)
    {
        return (LOS_DL_LIST *)NULL;
    }

    if(uwIdx <= OS_MIN_MULTI_DLNK_LOG2)
    {
        uwIdx = OS_MIN_MULTI_DLNK_LOG2;
    }

    return pstHead->stListHead + (uwIdx - OS_MIN_MULTI_DLNK_LOG2);
}
