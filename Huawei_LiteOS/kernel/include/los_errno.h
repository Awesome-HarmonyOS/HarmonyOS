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

/**@defgroup los_errno Error code
 * @ingroup kernel
 */

#ifndef _LOS_ERRNO_H
#define _LOS_ERRNO_H

#include "los_typedef.h"
#include "los_err.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
  * @ingroup los_errno
  * OS error code flag.
  */
#define LOS_ERRNO_OS_ID                                     ((UINT32)0x00 << 16)

/**
  * @ingroup los_errno
  * Define the error level as informative.
  */
#define LOS_ERRTYPE_NORMAL                                  ((UINT32)0x00 << 24)

/**
  * @ingroup los_errno
  * Define the error level as warning.
  */
#define LOS_ERRTYPE_WARN                                    ((UINT32)0x01 << 24)

/**
  * @ingroup los_errno
  * Define the error level as critical.
  */
#define LOS_ERRTYPE_ERROR                                   ((UINT32)0x02 << 24)

/**
  * @ingroup los_errno
  * Define the error level as fatal.
  */
#define LOS_ERRTYPE_FATAL                                   ((UINT32)0x03 << 24)

/**
  * @ingroup los_errno
  * Define fatal OS errors.
  */
#define LOS_ERRNO_OS_FATAL(MID, ERRNO)   \
            (LOS_ERRTYPE_FATAL | LOS_ERRNO_OS_ID | ((UINT32)(MID) << 8) |  (ERRNO))

/**
  * @ingroup los_errno
  * Define critical OS errors.
  */
#define LOS_ERRNO_OS_ERROR(MID, ERRNO)  \
            (LOS_ERRTYPE_ERROR | LOS_ERRNO_OS_ID | ((UINT32)(MID) << 8) | (ERRNO))

/**
  * @ingroup los_errno
  * Define warning OS errors.
  */
#define LOS_ERRNO_OS_WARN(MID, ERRNO)  \
            (LOS_ERRTYPE_WARN | LOS_ERRNO_OS_ID | ((UINT32)(MID) << 8) | (ERRNO))

/**
  * @ingroup los_errno
  * Define informative OS errors.
  */
#define LOS_ERRNO_OS_NORMAL(MID, ERRNO)  \
            (LOS_ERRTYPE_NORMAL | LOS_ERRNO_OS_ID | ((UINT32)(MID) << 8) | (ERRNO))


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_ERRNO_H */
