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
 * Huawei LiteOS may be subject to applicable export control laws and regulations, which might
 * include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
 * applicable export control laws and regulations.
 *---------------------------------------------------------------------------*/

#ifndef HWPATCH_ERRNO_H
#define HWPATCH_ERRNO_H

#define ERR_OK                      0
#define ERR_ILEGAL_PARAM            -0x01
#define ERR_INTERNAL                -0x02

#define ERR_DIFF_FILE_OPEN          -0x101
#define ERR_DIFF_FILE_READ          -0x102
#define ERR_DIFF_FILE_WRITE         -0x103
#define ERR_DIFF_COMPRESS           -0x104
#define ERR_DIFF_MALLOC             -0x105
#define ERR_DIFF_BSDIFF             -0x106
#define ERR_DIFF_BUFF_NOT_ENOUGHT   -0x107
#define ERR_DIFF_GET_SETS           -0x108
#define ERR_DIFF_GEN_CMD            -0x109
#define ERR_DIFF_GEN_DEPENDENCY     -0x10A
#define ERR_DIFF_TARGET_NOT_FOUND   -0x10B

#define ERR_PATCH_WRITE_BCK         -0x201
#define ERR_PATCH_READ_BCK          -0x202
#define ERR_PATCH_WRITE_IMAGE       -0x203
#define ERR_PATCH_READ_IMAGE        -0x204
#define ERR_PATCH_READ_PATCH        -0x205
#define ERR_PATCH_MALLOC            -0x206
#define ERR_PATCH_RECOVER_BLOCK     -0x207
#define ERR_PATCH_BUFF_NOT_ENOUGHT  -0x207
#define ERR_PATCH_UNCOMPRESS        -0x208
#define ERR_PATCH_CHECKSUM          -0x209
#define ERR_PATCH_RECOVER_VERIFY    -0x20A
#define ERR_PATCH_LAST_RECOVER      -0x20B
#define ERR_PATCH_FLAG_INIT         -0x20C
#define ERR_PATCH_WRONG_FLAG        -0x20D

#endif /* HWPATCH_ERRNO_H */