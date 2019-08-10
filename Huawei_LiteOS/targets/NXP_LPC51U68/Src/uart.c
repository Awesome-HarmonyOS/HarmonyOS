/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "fsl_usart.h"
#include "pin_mux.h"
#include <stdbool.h>
#include "osport.h"


#define BOARD_UART          USART0
#define BOARD_UART_BAUDRATE 115200      //if you use the sim800c,maybe it is 9600
#define BOARD_UART_CLK_FREQ CLOCK_GetFreq(kCLOCK_Flexcomm0)
#define BOARD_UART_IRQ      FLEXCOMM0_IRQn
#define CN_UART_RINGBUF_LEN 512 
static unsigned char gUartRcvBuf[CN_UART_RINGBUF_LEN];
static tagRingBuf  gUartRcvRing;

#define uart_irqhandler FLEXCOMM0_IRQHandler

void uart_irqhandler(void)
{
    uint8_t data;
    volatile uint32_t status;

    status =  USART_GetStatusFlags(BOARD_UART);  
    if(status&kUSART_RxError)
    {
        /* Clear rx error state. */
        BOARD_UART->FIFOSTAT |= USART_FIFOSTAT_RXERR_MASK;
        /* clear rxFIFO */
        BOARD_UART->FIFOCFG |= USART_FIFOCFG_EMPTYRX_MASK;
    }
    if(status & kUSART_RxFifoNotEmptyFlag)
    {
        data = USART_ReadByte(BOARD_UART);
        ring_write(&gUartRcvRing,&data,1);
    }
}
void uart_init(void)
{
    uint32_t srcFreq = 0;
    usart_config_t usart_config;
    USART_GetDefaultConfig(&usart_config);
    srcFreq = BOARD_UART_CLK_FREQ;
    usart_config.baudRate_Bps = BOARD_UART_BAUDRATE;
    usart_config.enableTx = true;
    usart_config.enableRx = true;
    USART_Init(BOARD_UART, &usart_config, srcFreq);
    ring_init(&gUartRcvRing,gUartRcvBuf,CN_UART_RINGBUF_LEN,0,0);  
   /* Enable RX interrupt. */
    USART_EnableInterrupts(BOARD_UART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    //USART_EnableInterrupts(BOARD_UART, kUSART_RxLevelInterruptEnable);
    EnableIRQ(BOARD_UART_IRQ);
    
    return ;
}
int uart_read(unsigned char *buf,int len,int timeout)
{
    int ret = 0;
    do{
        ret = ring_datalen(&gUartRcvRing);
        task_sleepms(1);
    }while((ret < len)&&(timeout-- > 0));
    if(ret > 0)
    {
        ret = ring_read(&gUartRcvRing,buf,len);
    }
    return ret;
}
int uart_write(unsigned char *buf,int len,int timeout)
{
    USART_WriteBlocking(BOARD_UART,(unsigned char *)buf,len);
    return len;
}

//this task is used to test the uart
void *main_uart(unsigned int args)
{
    int ret;
    unsigned char rcvbuf[64];
    //char *wel="welcome to the uart receive mode\n\r";
	  char *wel="123";

    char *index="lpc51u58>>";
    //do the uart test here
    uart_write((unsigned char *)wel,strlen(wel),0);
    while(1)
    {
        ret = uart_read(rcvbuf,64,100);
        if(ret > 0)
        {
              uart_write((unsigned char *)index,strlen(index),0);
					     
        }
				 ret = uart_read(rcvbuf,64,100);
				if(ret > 0)
				{
		        uart_write(rcvbuf,ret,100);		
					
				}

    }
}



