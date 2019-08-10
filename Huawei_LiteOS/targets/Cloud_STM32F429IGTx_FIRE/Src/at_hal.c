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

#if defined(WITH_AT_FRAMEWORK)

#include "at_hal.h"
#include "stm32f4xx_hal.h"

extern at_task at;

UART_HandleTypeDef at_usart;

static USART_TypeDef *s_pUSART = USART2;
static uint32_t s_uwIRQn = USART2_IRQn;

//uint32_t list_mux;
uint8_t buff_full = 0;
static uint32_t g_disscard_cnt = 0;

uint32_t wi       = 0;
uint32_t pre_ri   = 0;/*only save cur msg start*/
uint32_t ri       = 0;


static void at_usart_adapter(uint32_t port)
{
    switch ( port )
    {
    case 1 :
        s_pUSART = USART1;
        s_uwIRQn = USART1_IRQn;
        break;
    case 2 :
        s_pUSART = USART2;
        s_uwIRQn = USART2_IRQn;
        break;
    case 3 :
        s_pUSART = USART3;
        s_uwIRQn = USART3_IRQn;
        break;
    default:
        break;
    }
}



void at_irq_handler(void)
{
    recv_buff recv_buf;
    at_config *at_user_conf = at_get_config();

    if(__HAL_UART_GET_FLAG(&at_usart, UART_FLAG_RXNE) != RESET)
    {
        at.recv_buf[wi++] = (uint8_t)(at_usart.Instance->DR & 0x00FF);
        if(wi == ri)buff_full = 1;
        if (wi >= at_user_conf->user_buf_len)wi = 0;
    }
    else if (__HAL_UART_GET_FLAG(&at_usart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&at_usart);
        /*
        Ring Buffer ri------------------------>wi

         __________________________________________________
         |      msg0           |  msg1        |   msg2    |           
         ri(pre_ri0)        pre_ri1         pre_ri2     wi(pre_ri3)
         __________________________________________________ 

         read_resp ---->ri= pre_ri1----------->---------->ri=wi=pre_ri3(end)  
        */
        recv_buf.ori = pre_ri;
        recv_buf.end = wi;

        pre_ri = recv_buf.end;
        recv_buf.msg_type = AT_USART_RX;

        if(LOS_QueueWriteCopy(at.rid, &recv_buf, sizeof(recv_buff), 0) != LOS_OK)
        {
            g_disscard_cnt++;
        }
    }
}

int32_t at_usart_init(void)
{
    UART_HandleTypeDef *usart = &at_usart;
    at_config *at_user_conf = at_get_config();

    at_usart_adapter(at_user_conf->usart_port);

    usart->Instance = s_pUSART;
    usart->Init.BaudRate = at_user_conf->buardrate;

    usart->Init.WordLength = UART_WORDLENGTH_8B;
    usart->Init.StopBits = UART_STOPBITS_1;
    usart->Init.Parity = UART_PARITY_NONE;
    usart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    usart->Init.Mode = UART_MODE_RX | UART_MODE_TX;
    if(HAL_UART_Init(usart) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_UART_CLEAR_FLAG(usart, UART_FLAG_TC);
    LOS_HwiCreate(s_uwIRQn, 0, 0, at_irq_handler, 0);
    __HAL_UART_ENABLE_IT(usart, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(usart, UART_IT_RXNE);

    return AT_OK;
}

void at_usart_deinit(void)
{
    UART_HandleTypeDef *husart = &at_usart;
    __HAL_UART_DISABLE(husart);
    __HAL_UART_DISABLE_IT(husart, UART_IT_IDLE);
    __HAL_UART_DISABLE_IT(husart, UART_IT_RXNE);

}

void at_transmit(uint8_t *cmd, int32_t len, int flag)
{
    at_config *at_user_conf = at_get_config();
    
    char *line_end = at_user_conf->line_end;
    (void)HAL_UART_Transmit(&at_usart, (uint8_t *)cmd, len, 0xffff);
    if(flag == 1)
    {
        (void)HAL_UART_Transmit(&at_usart, (uint8_t *)line_end, strlen(at_user_conf->line_end), 0xffff);
    }
}

int read_resp(uint8_t *buf, recv_buff* recv_buf)
{
    uint32_t len = 0;

    uint32_t tmp_len = 0;

    at_config *at_user_conf = at_get_config();

    if (NULL == buf)
    {
        return -1;
    }
    if(1 == buff_full)
    {
        AT_LOG("buf maybe full,buff_full is %d",buff_full);
    }
    //AT_LOG("wi is %d, ri is %d,pre_ri is %d, end(%d),ori(%d),buff_full is %d",
    //    wi,ri,pre_ri,recv_buf->end,recv_buf->ori,buff_full);
    
    if (recv_buf->end == recv_buf->ori)
    {
        len = 0;
        goto END;
    }

    if (recv_buf->end > recv_buf->ori)
    {
        len = recv_buf->end - recv_buf->ori;
        memcpy(buf, &at.recv_buf[recv_buf->ori], len);
    }
    else
    {
        tmp_len = at_user_conf->user_buf_len - recv_buf->ori;
        memcpy(buf, &at.recv_buf[recv_buf->ori], tmp_len);
        memcpy(buf + tmp_len, at.recv_buf, recv_buf->end);
        len = recv_buf->end + tmp_len;
    }

    ri = recv_buf->end;
END:
    
    return len;
}

void write_at_task_msg(at_msg_type_e type)
{
    recv_buff recv_buf;
    int ret;

    memset(&recv_buf,  0, sizeof(recv_buf));
    recv_buf.msg_type = type;

    ret = LOS_QueueWriteCopy(at.rid, &recv_buf, sizeof(recv_buff), 0);
    if(ret != LOS_OK)
    {
        g_disscard_cnt++;
    }
}

#endif
