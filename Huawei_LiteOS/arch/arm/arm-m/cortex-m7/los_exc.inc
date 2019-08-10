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
#ifndef _LOS_EXC_INC
#define _LOS_EXC_INC

#include "los_exc.ph"
#include "los_sys.ph"
#ifdef LOSCFG_LIB_LIBC
#include "string.h"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */


/* 定义异常发生的阶段 */
#define OS_EXC_IN_INIT              0
#define OS_EXC_IN_TASK              1
#define OS_EXC_IN_HWI               2

/*
 *异常类型:uwFaultAddr域是否有效 标志位
 */
#define OS_EXC_FLAG_FAULTADDR_VALID     0x01

/*
 *异常类型:异常是否发生在硬中断中标志位
 */
#define OS_EXC_FLAG_IN_HWI              0x02

#define OS_EXC_IMPRECISE_ACCESS_ADDR    0xABABABAB

#define OS_EXC_FLAG_NO_FLOAT            0x10000000


#define OS_EXC_MAX_BUF_LEN          25
#define OS_EXC_MAX_NEST_DEPTH       1

#define OS_NVIC_SHCSR               0xE000ED24
#define OS_NVIC_CCR                 0xE000ED14

#define OS_NVIC_INT_ENABLE_SIZE     0x20
#define OS_NVIC_INT_PRI_SIZE        0xF0

#define OS_NVIC_INT_PEND_SIZE       OS_NVIC_INT_ACT_SIZE
#define OS_NVIC_INT_ACT_SIZE        OS_NVIC_INT_ENABLE_SIZE


/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:总线状态寄存器入栈时发生错误
 */
#define OS_EXC_BF_STKERR           1

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:总线状态寄存器出栈时发生错误
 */
#define OS_EXC_BF_UNSTKERR         2

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:总线状态寄存器不精确的数据访问违例
 */
#define OS_EXC_BF_IMPRECISERR      3

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:总线状态寄存器精确的数据访问违例
 */
#define OS_EXC_BF_PRECISERR        4

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:总线状态寄存器取指时的访问违例
 */
#define OS_EXC_BF_IBUSERR          5

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:存储器管理状态寄存器入栈时发生错误
 */
#define OS_EXC_MF_MSTKERR          6

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:存储器管理状态寄存器出栈时发生错误
 */
#define OS_EXC_MF_MUNSTKERR        7

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:存储器管理状态寄存器数据访问违例
 */
#define OS_EXC_MF_DACCVIOL         8

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:存储器管理状态寄存器取指访问违例
 */
#define OS_EXC_MF_IACCVIOL         9


/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:用法错误，表示除法运算时除数为零
 */
#define OS_EXC_UF_DIVBYZERO        10

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:用法错误，未对齐访问导致的错误
 */
#define OS_EXC_UF_UNALIGNED        11

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:用法错误，试图执行协处理器相关指令
 */
#define OS_EXC_UF_NOCP             12

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:用法错误，在异常返回时试图非法地加载EXC_RETURN到PC
 */
#define OS_EXC_UF_INVPC            13

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:用法错误，试图切入ARM状态
 */
#define OS_EXC_UF_INVSTATE         14

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:用法错误，执行的指令其编码是未定义的——解码不能
 */
#define OS_EXC_UF_UNDEFINSTR       15

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:NMI中断
 */

#define OS_EXC_CAUSE_NMI           16

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:硬fault
 */
#define OS_EXC_CAUSE_HARDFAULT     17

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:任务处理函数退出
 */
#define OS_EXC_CAUSE_TASK_EXIT     18

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:致命错误
 */
#define OS_EXC_CAUSE_FATAL_ERR     19

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:调试事件导致的硬fault
 */
#define OS_EXC_CAUSE_DEBUGEVT      20

/**
 *@ingroup los_exc
 *Cortex-M4异常具体类型:取向量时发生的硬fault
 */
#define OS_EXC_CAUSE_VECTBL        21

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#endif /* _LOS_EXC_INC */
