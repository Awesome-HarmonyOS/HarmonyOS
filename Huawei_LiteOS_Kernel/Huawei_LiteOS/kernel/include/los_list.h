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

 /** @defgroup los_list Doubly linked list
 * @ingroup kernel
 */

#ifndef _LOS_LIST_H
#define _LOS_LIST_H

#include "los_base.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 *@ingroup los_list
 *Structure of a node in a doubly linked list.
 */
typedef struct LOS_DL_LIST
{
    struct LOS_DL_LIST *pstPrev;            /**< Current node's pointer to the previous node*/
    struct LOS_DL_LIST *pstNext;            /**< Current node's pointer to the next node*/
} LOS_DL_LIST;

/**
 *@ingroup los_list
 *@brief Initialize a doubly linked list.
 *
 *@par Description:
 *This API is used to initialize a doubly linked list.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pstList    [IN] Node in a doubly linked list.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
LITE_OS_SEC_ALW_INLINE INLINE VOID LOS_ListInit(LOS_DL_LIST *pstList)
{
    pstList->pstNext = pstList;
    pstList->pstPrev = pstList;
}

/**
 *@ingroup los_list
 *@brief Point to the next node pointed to by the current node.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to point to the next node pointed to by the current node.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pstObject  [IN] Node in the doubly linked list.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
#define LOS_DL_LIST_FIRST(pstObject) ((pstObject)->pstNext)

/**
 *@ingroup los_list
 *@brief Insert a new node to a doubly linked list.
 *
 *@par Description:
 *This API is used to insert a new node to a doubly linked list.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pstList    [IN]    Position where the new node is inserted.
 *@param pstNode    [IN]   New node to be inserted.
 *
 *@retval None
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
LITE_OS_SEC_ALW_INLINE INLINE VOID LOS_ListAdd(LOS_DL_LIST *pstList, LOS_DL_LIST *pstNode)
{
    pstNode->pstNext = pstList->pstNext;
    pstNode->pstPrev = pstList;
    pstList->pstNext->pstPrev = pstNode;
    pstList->pstNext = pstNode;
}

/**
 *@ingroup los_list
 *@brief Insert a node to the tail of a doubly linked list.
 *
 *@par Description:
 *This API is used to insert a new node to the tail of a doubly linked list. pstListObject and pstNewNode must point to valid memory.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pstList     [IN] Doubly linked list where the new node is inserted.
 *@param pstNode     [IN] New node to be inserted.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_ListAdd
 *@since Huawei LiteOS V100R001C00
 */
LITE_OS_SEC_ALW_INLINE INLINE VOID LOS_ListTailInsert(LOS_DL_LIST *pstList, LOS_DL_LIST *pstNode)
{
    LOS_ListAdd(pstList->pstPrev, pstNode);
}

/**
 *@ingroup los_list
 *@brief Delete a specified node from a doubly linked list.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to delete a specified node from a doubly linked list.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pstNode    [IN] Node to be deleted.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_ListAdd
 *@since Huawei LiteOS V100R001C00
 */
LITE_OS_SEC_ALW_INLINE INLINE VOID LOS_ListDelete(LOS_DL_LIST *pstNode)
{
    pstNode->pstNext->pstPrev = pstNode->pstPrev;
    pstNode->pstPrev->pstNext = pstNode->pstNext;
    pstNode->pstNext = (LOS_DL_LIST *)NULL;
    pstNode->pstPrev = (LOS_DL_LIST *)NULL;
}

/**
 *@ingroup los_list
 *@brief Identify whether a specified doubly linked list is empty.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to return whether a doubly linked list is empty.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pstNode  [IN] Node in the doubly linked list.
 *
 *@retval TRUE The doubly linked list is empty.
 *@retval FALSE The doubly linked list is not empty.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
LITE_OS_SEC_ALW_INLINE INLINE BOOL LOS_ListEmpty(LOS_DL_LIST *pstNode)
{
    return (BOOL)(pstNode->pstNext == pstNode);
}

/**
 * @ingroup los_list
 * @brief Obtain the offset of a field to a structure address.
 *
 *@par  Description:
 *This API is used to obtain the offset of a field to a structure address.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param type   [IN] Structure name.
 *@param field  [IN] Name of the field of which the offset is to be measured.
 *
 *@retval Offset of the field to the structure address.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
#define OFFSET_OF_FIELD(type, field)    ((UINT32)&(((type *)0)->field))

/**
 *@ingroup los_list
 *@brief Obtain the pointer to a doubly linked list in a structure.
 *
 *@par Description:
 *This API is used to obtain the pointer to a doubly linked list in a structure.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param type    [IN] Structure name.
 *@param member  [IN] Member name of the doubly linked list in the structure.
 *
 *@retval Pointer to the doubly linked list in the structure.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
#define LOS_OFF_SET_OF(type, member) ((long)&((type *)0)->member)

/**
 *@ingroup los_list
 *@brief Obtain the pointer to a structure that contains a doubly linked list.
 *
 *@par Description:
 *This API is used to obtain the pointer to a structure that contains a doubly linked list.
 *<ul>
 *<li>None.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param item    [IN] Current node's pointer to the next node.
 *@param type    [IN] Structure name.
 *@param member  [IN] Member name of the doubly linked list in the structure.
 *
 *@retval Pointer to the structure that contains the doubly linked list.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
#define LOS_DL_LIST_ENTRY(item, type, member) \
    ((type *)((char *)item - LOS_OFF_SET_OF(type, member))) \

/**
 *@ingroup los_list
 *@brief Iterate over a doubly linked list of given type.
 *
 *@par Description:
 *This API is used to iterate over a doubly linked list of given type.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param item           [IN] Pointer to the structure that contains the doubly linked list that is to be traversed.
 *@param list           [IN] Pointer to the doubly linked list to be traversed.
 *@param type           [IN] Structure name.
 *@param member         [IN] Member name of the doubly linked list in the structure.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
#define LOS_DL_LIST_FOR_EACH_ENTRY(item, list, type, member) \
    for (item = LOS_DL_LIST_ENTRY((list)->pstNext, type, member); \
        &item->member != (list); \
        item = LOS_DL_LIST_ENTRY(item->member.pstNext, type, member))

/**
 *@ingroup los_list
 *@brief iterate over a doubly linked list safe against removal of list entry.
 *
 *@par Description:
 *This API is used to iterate over a doubly linked list safe against removal of list entry.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param item           [IN] Pointer to the structure that contains the doubly linked list that is to be traversed.
 *@param next           [IN] Save the next node.
 *@param list           [IN] Pointer to the doubly linked list to be traversed.
 *@param type           [IN] Structure name.
 *@param member         [IN] Member name of the doubly linked list in the structure.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
#define LOS_DL_LIST_FOR_EACH_ENTRY_SAFE(item, next, list, type, member)            \
    for (item = LOS_DL_LIST_ENTRY((list)->pstNext, type, member), \
        next = LOS_DL_LIST_ENTRY(item->member->pstNext, type, member); \
        &item->member != (list); \
        item = next, item = LOS_DL_LIST_ENTRY(item->member.pstNext, type, member))

LITE_OS_SEC_ALW_INLINE INLINE VOID osListDel(LOS_DL_LIST *pstPrevNode, LOS_DL_LIST *pstNextNode)
{
    pstNextNode->pstPrev = pstPrevNode;
    pstPrevNode->pstNext = pstNextNode;
}

/**
 *@ingroup los_list
 *@brief Delete initialize a doubly linked list.
 *
 *@par Description:
 *This API is used to delete initialize a doubly linked list.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pstList    [IN] Node in a doubly linked list.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
LITE_OS_SEC_ALW_INLINE INLINE VOID LOS_ListDelInit(LOS_DL_LIST *pstList)
{
    osListDel(pstList->pstPrev, pstList->pstNext);
    LOS_ListInit(pstList);
}

/**
 *@ingroup los_list
 *@brief iterate over a doubly linked list.
 *
 *@par Description:
 *This API is used to iterate over a doubly linked list.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param item           [IN] Pointer to the structure that contains the doubly linked list that is to be traversed.
 *@param list           [IN] Pointer to the doubly linked list to be traversed.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
#define LOS_DL_LIST_FOR_EACH(item, list)   \
    for ((item) = (list)->pstNext; \
        (item) != (list); \
        (item) = (item)->pstNext)

/**
 *@ingroup los_list
 *@brief Iterate over a doubly linked list safe against removal of list entry.
 *
 *@par Description:
 *This API is used to iterate over a doubly linked list safe against removal of list entry.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param item           [IN] Pointer to the structure that contains the doubly linked list that is to be traversed.
 *@param next           [IN] Save the next node.
 *@param list           [IN] Pointer to the doubly linked list to be traversed.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
#define LOS_DL_LIST_FOR_EACH_SAFE(item, next, list) \
    for (item = (list)->pstNext, next = item->pstNext; item != (list); \
        item = next, next = item->pstNext)

/**
 *@ingroup los_list
 *@brief Initialize a double linked list.
 *
 *@par Description:
 *This API is used to Initialize a double linked list.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param list           [IN] Pointer to the doubly linked list to be traversed.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_list.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
#define LOS_DL_LIST_HEAD(list) \
            LOS_DL_LIST list = { &(list), &(list) }


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_LIST_H */
