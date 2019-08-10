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

 /**@defgroup hal_rng Ramdon Number
 * @ingroup hal
 */

#ifndef _HAL_RNG_H_
#define _HAL_RNG_H_

#include <stdint.h>
#include <stdio.h>

#if defined(__cplusplus)
extern "C" {
#endif

/**
 *@ingroup hal_rng
 *@brief config the rng.
 *
 *@par Description:
 *This API is used to config the rng.
 *@attention none.
 *
 *@param none.
 *
 *@retval none.
 *@par Dependency: none.
 *@see none.
 */
void hal_rng_config(void);

/**
 *@ingroup hal_rng
 *@brief generate a random number.
 *
 *@par Description:
 *This API is used to generate a random number.
 *@attention none.
 *
 *@param none.
 *
 *@retval #int          random number
 *@par Dependency: none.
 *@see none.
 */
int hal_rng_generate_number(void);

/**
 *@ingroup hal_rng
 *@brief generate the random data.
 *
 *@par Description:
 *This API is used to generate the random data.
 *@attention none.
 *
 *@param buf            [OUT] buffer to store the random data.
 *@param len            [IN]  the length of the random data needed to generate.
 *
 *@retval #int          0 if succeed or -1 if failed.
 *@par Dependency: none.
 *@see none.
 */
int hal_rng_generate_buffer(void* buf, size_t len);

#if defined(__cplusplus)
}
#endif

#endif  /* _HAL_RNG_H_ */
