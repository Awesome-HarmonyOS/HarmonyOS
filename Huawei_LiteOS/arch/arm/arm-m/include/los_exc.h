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

#ifndef _LOS_EXC_H
#define _LOS_EXC_H

#include "los_hwi.h"
#include "los_task.ph"
#include "los_queue.h"
#include "los_config.h"
#include "los_memcheck.h"
#include "los_hw.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#if (LOSCFG_PLATFORM_EXC == YES)


#define OS_EXC_IN_INIT                      0
#define OS_EXC_IN_TASK                      1
#define OS_EXC_IN_HWI                       2

#define OS_EXC_MAX_BUF_LEN                  25
#define OS_EXC_MAX_NEST_DEPTH               1

#define OS_EXC_FLAG_NO_FLOAT                0x10000000
#define OS_EXC_FLAG_FAULTADDR_VALID         0x01
#define OS_EXC_FLAG_IN_HWI                  0x02

#define OS_EXC_IMPRECISE_ACCESS_ADDR        0xABABABAB

/**
 *@ingroup los_exc
 * the struct of register files
 *
 * description: the register files that saved when exception triggered
 *
 * notes:the following register with prefix 'uw'  correspond to the registers in the cpu  data sheet.
 */
typedef struct tagExcContext
{
#if FPU_EXIST
    UINT32 S16;
    UINT32 S17;
    UINT32 S18;
    UINT32 S19;
    UINT32 S20;
    UINT32 S21;
    UINT32 S22;
    UINT32 S23;
    UINT32 S24;
    UINT32 S25;
    UINT32 S26;
    UINT32 S27;
    UINT32 S28;
    UINT32 S29;
    UINT32 S30;
    UINT32 S31;
#endif
    UINT32 uwR4;
    UINT32 uwR5;
    UINT32 uwR6;
    UINT32 uwR7;
    UINT32 uwR8;
    UINT32 uwR9;
    UINT32 uwR10;
    UINT32 uwR11;
    UINT32 uwPriMask;
    UINT32 uwSP;
    UINT32 uwR0;
    UINT32 uwR1;
    UINT32 uwR2;
    UINT32 uwR3;
    UINT32 uwR12;
    UINT32 uwLR;
    UINT32 uwPC;
    UINT32 uwxPSR;
#if FPU_EXIST
    UINT32 S0;
    UINT32 S1;
    UINT32 S2;
    UINT32 S3;
    UINT32 S4;
    UINT32 S5;
    UINT32 S6;
    UINT32 S7;
    UINT32 S8;
    UINT32 S9;
    UINT32 S10;
    UINT32 S11;
    UINT32 S12;
    UINT32 S13;
    UINT32 S14;
    UINT32 S15;
    UINT32 FPSCR;
    UINT32 NO_NAME;
#endif
}EXC_CONTEXT_S;

typedef UINT32 (*EXC_INFO_SAVE_CALLBACK)(UINT32 , VOID* );
typedef VOID (* EXC_PROC_FUNC)(UINT32, EXC_CONTEXT_S *);
VOID osExcHandleEntry(UINT32 uwExcType, UINT32 uwFaultAddr, UINT32 uwPid, EXC_CONTEXT_S *  pstExcBufAddr);

/**
 * @ingroup  los_hwi
 * @brief: Exception initialization.
 *
 * @par Description:
 * This API is used to configure the exception function vector table.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 *@param uwArraySize [IN] Memory size of exception.
 *
 * @retval: None
 * @par Dependency:
 * <ul><li>los_hwi.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
VOID osExcInit(UINT32 uwArraySize);

extern VOID NMI_Handler(VOID);
extern VOID HardFault_Handler(VOID);
extern VOID MemManage_Handler(VOID);
extern VOID BusFault_Handler(VOID);
extern VOID UsageFault_Handler(VOID);
extern VOID SVC_Handler(VOID);
extern VOID osBackTrace(VOID);
extern UINT8 m_aucTaskArray[];

/**
 *@ingroup los_exc
 *@brief Kernel panic function.
 *
 *@par Description:
 *Stack function that prints kernel panics.
 *@attention After this function is called and stack information is printed, the system will fail to respond.
 *@attention The input parameter can be NULL.
 *@param fmt [IN] Type #char* : variadic argument.
 *
 *@retval #None.
 *
 *@par Dependency:
 *los_exc.h: the header file that contains the API declaration.
 *@see None.
 *@since Huawei LiteOS V100R001C00
*/
void LOS_Panic(const char * fmt, ...);

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:总线状态寄存器入栈时发生错误
 */
#define OS_EXC_BF_STKERR           1

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:总线状态寄存器出栈时发生错误
 */
#define OS_EXC_BF_UNSTKERR         2

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:总线状态寄存器不精确的数据访问违例
 */
#define OS_EXC_BF_IMPRECISERR      3

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:总线状态寄存器精确的数据访问违例
 */
#define OS_EXC_BF_PRECISERR        4

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:总线状态寄存器取指时的访问违例
 */
#define OS_EXC_BF_IBUSERR          5

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:总线错误，浮点惰性压栈错误，具有浮点单元的cortex-m4及cortex-m7中存在
 */
#define OS_EXC_BF_LSPERR           6

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:存储器管理状态寄存器入栈时发生错误
 */
#define OS_EXC_MF_MSTKERR          7

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:存储器管理状态寄存器出栈时发生错误
 */
#define OS_EXC_MF_MUNSTKERR        8

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:存储器管理状态寄存器数据访问违例
 */
#define OS_EXC_MF_DACCVIOL         9

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:存储器管理状态寄存器取指访问违例
 */
#define OS_EXC_MF_IACCVIOL         10

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:存储器管理错误，浮点惰性压栈错误，具有浮点单元的cortex-m4及cortex-m7中存在
 */
#define OS_EXC_MF_MLSPERR          11

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:用法错误，表示除法运算时除数为零
 */
#define OS_EXC_UF_DIVBYZERO        12

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:用法错误，未对齐访问导致的错误
 */
#define OS_EXC_UF_UNALIGNED        13

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:用法错误，试图执行协处理器相关指令
 */
#define OS_EXC_UF_NOCP             14

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:用法错误，在异常返回时试图非法地加载EXC_RETURN到PC
 */
#define OS_EXC_UF_INVPC            15

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:用法错误，试图切入ARM状态
 */
#define OS_EXC_UF_INVSTATE         16

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:用法错误，执行的指令其编码是未定义的——解码不能
 */
#define OS_EXC_UF_UNDEFINSTR       17

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:NMI中断
 */
#define OS_EXC_CAUSE_NMI           18

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:硬fault
 */
#define OS_EXC_CAUSE_HARDFAULT     19

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:存储器管理fault
 */
#define OS_EXC_CAUSE_MEMFAULT      20

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:总线fault
 */
#define OS_EXC_CAUSE_BUSFAULT      21

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:使用fault
 */
#define OS_EXC_CAUSE_USAGEFAULT    22

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:SVC调用
 */
#define OS_EXC_CAUSE_SVC           23

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:调试事件导致的硬fault
 */
#define OS_EXC_CAUSE_DEBUGEVT      24

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:取向量时发生的硬fault
 */
#define OS_EXC_CAUSE_VECTBL        25

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:任务处理函数退出
 */
#define OS_EXC_CAUSE_TASK_EXIT     26

/**
 *@ingroup los_exc
 *Cortex-M异常具体类型:致命错误
 */
#define OS_EXC_CAUSE_FATAL_ERR     27

/**
 *@ingroup los_exc
 * 异常信息结构体
 *
 * 描述:Cortex-M平台下的异常触发时保存的异常信息
 *
 */
typedef struct tagExcInfo
{
    UINT16 usPhase;              /**< 异常发生阶段： 0表示异常发生在初始化中，1表示异常发生在任务中，2表示异常发生在中断中 */
    UINT16 usType;               /**< 异常类型，分高低两字节，出异常时高低字节分别对照上面列出的异常编号 */
    UINT32 uwFaultAddr;          /**< 若为精确地址访问错误表示异常发生时的错误访问地址 */
    UINT32 uwThrdPid;            /**< 在中断中发生异常，表示中断号。在任务中发生异常，表示任务id，如果发生在初始化中，则为0xffffffff */
    UINT16 usNestCnt;            /**< 异常嵌套个数，目前仅支持第一次进入异常时执行注册的钩子函数 */
    UINT16 usFpuContext;         /**< 是否保存浮点寄存器，1表示需要保存浮点寄存器 */
    UINT32 uwCallStackDepth;     /**< 异常时调用栈分析深度 */
    UINT32 uwCallStack[LOSCFG_EXC_CALL_STACK_ANALYSIS_MAX_DEPTH];  /**< 函数调用地址 */
    EXC_CONTEXT_S *pstContext;   /**< 异常发生时的寄存器值，由usFpuContext决定寄存器中是否包含浮点寄存器的值 */
}EXC_INFO_S;

extern UINT32 g_uwCurNestCount;
extern UINT32 g_vuwIntCount;
extern EXC_INFO_S m_stExcInfo;

VOID osExcInfoDisplay(EXC_INFO_S *pstExc);

extern OS_TASK_SWITCH_INFO g_astTskSwitchInfo;
extern UINT8 g_uwExcTbl[32];

#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#endif /* _LOS_EXC_H */

