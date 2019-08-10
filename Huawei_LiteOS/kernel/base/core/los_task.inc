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

#ifndef _LOS_TASK_INC
#define _LOS_TASK_INC

#include "los_task.ph"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


#define EVALUATE_L(UWNUM, VALUE)  \
            UWNUM = (((UWNUM) & 0xf8000000) | (VALUE))

#define EVALUATE_H(UWNUM, VALUE)  \
            UWNUM = (((UWNUM) & 0x07ffffff)| ((VALUE) << 27))

#define UWROLLNUMSUB(UWNUM1,UWNUM2)  \
            UWNUM1 = ((UWNUM1 & 0xf8000000) | (UWROLLNUM(UWNUM1) - UWROLLNUM(UWNUM2)))

#define UWROLLNUMADD(UWNUM1,UWNUM2)  \
            UWNUM1 = ((UWNUM1 & 0xf8000000) | (UWROLLNUM(UWNUM1) + UWROLLNUM(UWNUM2)))

#define UWROLLNUM(UWNUM) ((UWNUM) & 0x07ffffff)

#define UWSORTINDEX(UWNUM) (UWNUM >> 27)

#define UWROLLNUMDEC(UWNUM)  \
            UWNUM = (UWNUM - 1)

#define OS_TSK_SORTLINK_LEN                     32
#define OS_TSK_SORTLINK_LOGLEN                  5
#define OS_TSK_SORTLINK_MASK                    (OS_TSK_SORTLINK_LEN - 1)

#define OS_CHECK_TASK_BLOCK                     ((OS_TASK_STATUS_DELAY | OS_TASK_STATUS_PEND | OS_TASK_STATUS_SUSPEND | OS_TASK_STATUS_EVENT | OS_TASK_STATUS_PEND_QUEUE))

#define OS_TASK_ID_CHECK(uwTaskID)              LOS_ASSERT_COND(OS_TSK_GET_INDEX(uwTaskID) < g_uwTskMaxNum)
#define OS_CHECK_TSK_PID_NOIDLE(uwTaskID)       (OS_TSK_GET_INDEX(uwTaskID) >= g_uwTskMaxNum)

typedef struct tagTskSortLinkAttr
{
    LOS_DL_LIST     *pstSortLink;
    UINT16          usCursor;
    UINT16          usUnUsed;
} TSK_SORTLINK_ATTRIBUTE_S;

/**
 * @ingroup los_task
 * Task stack information structure.
 *
 */
typedef struct tagStackInfo
{
    UINT32 uwTop;                                  /**<Task stack top                  */
    UINT32 uwBottom;                               /**<Task stack bottom                  */
    UINT32 uwSP;                                   /**<Task stack pointer                */
    UINT32 uwCurrUsed;                             /**<Current task stack usage      */
    UINT32 uwPeakUsed;                             /**<Task stack usage peak            */
    BOOL   bOvf;                                   /**<Flag that indicates whether a task stack overflow occurs        */
} STACK_INFO_S;

/**
 * @ingroup  los_task
 * @brief Add task to sorted delay list.
 *
 * @par Description:
 * This API is used to add task to sorted delay list.
 *
 * @attention
 * <ul>
 * <li>The pstTaskCB should be a correct pointer to task control block structure.</li>
 * </ul>
 *
 * @param  pstTaskCB [IN] Type #LOS_TASK_CB * pointer to task control block structure.
 * @param  uwTimeout  [IN] Type #UINT32 wait time, ticks.
 *
 * @retval  None.
 * @par Dependency:
 * <ul><li>los_task.inc: the header file that contains the API declaration.</li></ul>
 * @see osTimerListDelete
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osTaskAdd2TimerList(LOS_TASK_CB *pstTaskCB, UINT32 uwTimeout);

/**
 * @ingroup  los_task
 * @brief delete task from sorted delay list.
 *
 * @par Description:
 * This API is used to delete task from sorted delay list.
 *
 * @attention
 * <ul>
 * <li>The pstTaskCB should be a correct pointer to task control block structure.</li>
 * </ul>
 *
 * @param  pstTaskCB [IN] Type #LOS_TASK_CB * pointer to task control block structure.
 *
 * @retval  None.
 * @par Dependency:
 * <ul><li>los_task.inc: the header file that contains the API declaration.</li></ul>
 * @see osTaskAdd2TimerList
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osTimerListDelete(LOS_TASK_CB *pstTaskCB);

/**
 * @ingroup  los_task
 * @brief Convert task status to string.
 *
 * @par Description:
 * This API is used to convert task status to string.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  usTaskStatus [IN] Type #UINT16 task status.
 *
 * @retval  UINT8 *  String.
 * @par Dependency:
 * <ul><li>los_task.inc: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT8 *osConvertTskStatus(UINT16 usTaskStatus);

/**
 * @ingroup  los_task
 * @brief Get all task information.
 *
 * @par Description:
 * This API is used to get all task information.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  None.
 *
 * @retval  UINT32  All task information.
 * @par Dependency:
 * <ul><li>los_task.inc: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osGetAllTskInfo(VOID);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_TASK_INC */
