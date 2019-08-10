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

#ifndef _ATINY_LIST_H

#define _ATINY_LIST_H
#define ATINY_INLINE static inline


#ifdef __cplusplus
#if __cplusplus

extern "C" {

#endif /* __cplusplus */

#endif /* __cplusplus */


/**

 *@ingroup atiny_list

 *Structure of a node in a doubly linked list.

 */

typedef struct atiny_dl_list
{
    struct atiny_dl_list *prev;            /**< Current node's pointer to the previous node*/
    struct atiny_dl_list *next;            /**< Current node's pointer to the next node*/

} atiny_dl_list;


ATINY_INLINE void atiny_list_init(atiny_dl_list *list)
{
    list->next = list;
    list->prev = list;
}

#define ATINY_DL_LIST_FIRST(object) ((object)->next)

ATINY_INLINE void atiny_list_add(atiny_dl_list *list, atiny_dl_list *node)
{
    node->next = list->next;
    node->prev = list;
    list->next->prev = node;
    list->next = node;
}

ATINY_INLINE void atiny_list_insert_tail(atiny_dl_list *list, atiny_dl_list *node)
{
    atiny_list_add(list->prev, node);
}

ATINY_INLINE atiny_dl_list * atiny_list_get_head(atiny_dl_list *header)
{
    return header->next;
}

ATINY_INLINE void atiny_list_delete(atiny_dl_list *node)
{
    node->next->prev = node->prev;
    node->prev->next = node->next;
    node->next = (atiny_dl_list *)NULL;
    node->prev = (atiny_dl_list *)NULL;
}

ATINY_INLINE int atiny_list_empty(atiny_dl_list *node)
{
    return (node->next == node);
}

#define ATINY_OFFSET_OF_FIELD(type, field)    ((UINT32)&(((type *)0)->field))
#define ATINY_OFF_SET_OF(type, member) ((long)&((type *)0)->member)   /*lint -e(413) */
#define ATINY_FIELD_TO_STRUCT(field_addr, type, member) \
    ((type *)((char *)(field_addr) - ATINY_OFF_SET_OF(type, member)))

#define ATINY_DL_LIST_ENTRY(item, type, member)\
    ((type *)((char *)item - ATINY_OFF_SET_OF(type, member)))\


#define ATINY_DL_LIST_FOR_EACH_ENTRY(item, list, type, member)\
    for (item = ATINY_DL_LIST_ENTRY((list)->next, type, member);\
        &item->member != (list);\
        item = ATINY_DL_LIST_ENTRY(item->member.next, type, member))


#define ATINY_DL_LIST_FOR_EACH_ENTRY_SAFE(item, next, list, type, member)\
    for (item = ATINY_DL_LIST_ENTRY((list)->next, type, member),\
        next = ATINY_DL_LIST_ENTRY(item->member->next, type, member);\
        &item->member != (list);\
        item = next, item = ATINY_DL_LIST_ENTRY(item->member.next, type, member))

ATINY_INLINE void ATINY_ListDel(atiny_dl_list *pstPrevNode, atiny_dl_list *pstNextNode)
{
    pstNextNode->prev = pstPrevNode;
    pstPrevNode->next = pstNextNode;
}

ATINY_INLINE void ATINY_ListDelInit(atiny_dl_list *pstList)
{
    ATINY_ListDel(pstList->prev, pstList->next);
    atiny_list_init(pstList);
}

#define ATINY_DL_LIST_FOR_EACH(item, list)\
    for ((item) = (list)->next;\
        (item) != (list);\
        (item) = (item)->next)

#define ATINY_DL_LIST_FOR_EACH_SAFE(item, next, list)\
    for (item = (list)->next, next = item->next; item != (list);\
        item = next, next = item->next)

#define ATINY_DL_LIST_HEAD(list)\
            atiny_dl_list list = { &(list), &(list) }

#ifdef __cplusplus

#if __cplusplus

}

#endif /* __cplusplus */
#endif /* __cplusplus */
#endif /* _ATINY_LIST_H */
