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

#include <string.h>
#include "hal_rng.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_rng.h"

#ifdef HAL_RNG_MODULE_ENABLED

RNG_HandleTypeDef g_rng_handle;

void hal_rng_config(void)
{
    __HAL_RCC_RNG_CLK_ENABLE();
    g_rng_handle.Instance = RNG;
    (void)HAL_RNG_Init(&g_rng_handle);
}

int hal_rng_generate_number()
{
    uint32_t random_number;

    if (HAL_RNG_GenerateRandomNumber(&g_rng_handle, &random_number) != HAL_OK)
    {
        return 0U;
    }

    return (int)random_number;
}

int hal_rng_generate_buffer(void* buf, size_t len)
{
    size_t i;
    uint32_t random_number;
    uint8_t* pbuf;

    if (NULL == buf)
    {
        return -1;
    }

    pbuf = (uint8_t*)buf;

    for (i = 0; i < len; i += sizeof(uint32_t))
    {
        if (HAL_RNG_GenerateRandomNumber(&g_rng_handle, &random_number) != HAL_OK)
        {
            return -1;
        }
        memcpy(pbuf + i, &random_number,
               sizeof(uint32_t) > len - i ? len - i : sizeof(uint32_t));
    }

    return 0;
}

#endif /* HAL_RNG_MODULE_ENABLED */
