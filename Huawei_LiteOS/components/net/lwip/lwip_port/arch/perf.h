/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 * Copyright (c) <2016-2018>, <Huawei Technologies Co., Ltd>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/**********************************************************************************
 * Notice of Export Control Law
 * ===============================================
 * Huawei LiteOS may be subject to applicable export control laws and regulations, which
 * might include those applicable to Huawei LiteOS of U.S. and the country in which you
 * are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance
 * with such applicable export control laws and regulations.
 **********************************************************************************/

#ifndef __LWIP_PERF_H__
#define __LWIP_PERF_H__

#include "lwip/arch.h"
#include "lwip/opt.h"
#include "lwip/def.h"

#ifdef  __cplusplus
extern "C" {
#endif

/* PERF_START and PERF_STOP are called in code so its kept as empty here always */

#define PERF_START    /* null definition */
#define PERF_STOP(x)  /* null definition */

#if LWIP_PERF

void
perf_print(u32_t start_ms, u32_t end_ms, char *key);

#define LWIP_PERF_DECLARATION u32_t start_ms, end_ms
#define LWIP_PERF_START  start_ms = sys_now()
#define LWIP_PERF_STOP(x)  end_ms = sys_now(); \
                       perf_print(start_ms, end_ms, x)

void
perf_init(void *data);

void
perf_fini(void *data);

#endif /* LWIP_PERF */

#ifdef  __cplusplus
}
#endif

#endif /* __LWIP_PERF_H__ */

