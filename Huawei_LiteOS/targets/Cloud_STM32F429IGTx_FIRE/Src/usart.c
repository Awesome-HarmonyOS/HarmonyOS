/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void Debug_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    if(uartHandle->Instance == USART1)
    {
        /* USER CODE BEGIN USART1_MspInit 0 */

        /* USER CODE END USART1_MspInit 0 */
        /* USART1 clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();

        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USER CODE BEGIN USART1_MspInit 1 */

        /* USER CODE END USART1_MspInit 1 */
    }
    else if(uartHandle->Instance == USART2)
    {
        /* config USART2 clock */
        //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_AHB1Periph_GPIOA, ENABLE);
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();

        /* USART2 GPIO config */
        /* Configure USART2 Tx (PA.02) as alternate function push-pull */
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(uartHandle->Instance == USART3)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
    if(uartHandle->Instance == USART1)
    {
        /* USER CODE BEGIN USART1_MspDeInit 0 */

        /* USER CODE END USART1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();

        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

        /* USER CODE BEGIN USART1_MspDeInit 1 */

        /* USER CODE END USART1_MspDeInit 1 */
    }
}

/* define fputc */
#if defined ( __CC_ARM ) || defined ( __ICCARM__ )  /* KEIL and IAR: printf will call fputc to print */
int fputc(int ch, FILE *f)
{
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

#if USE_PPPOS

#include "osport.h"
#include "los_hwi.h"

int uart3_send(unsigned char *buf, int len);
int uart3_recv(unsigned char *buf, int len, int timeout);
tagRingBuf  gRcvRing;
#define CN_RING_lEN 1500
unsigned char gRcvBuf[CN_RING_lEN];
UART_HandleTypeDef huart3;


void uart3_irq()
{
    unsigned char data;
    unsigned int flags;

    flags = huart3.Instance->SR;
    if(flags & USART_SR_RXNE)
    {
        data = (uint8_t)(huart3.Instance->DR & (uint8_t)0x00FF);
        ring_write(&gRcvRing, &data, 1);
    }
}
void uart3_init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    ring_init(&gRcvRing, gRcvBuf, CN_RING_lEN, 0, 0);


    LOS_HwiCreate(39, 4, 0, uart3_irq, 0);
    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(huart3.Instance->CR3, USART_CR3_EIE);

    /* Enable the UART Parity Error and Data Register not empty Interrupts */
    SET_BIT(huart3.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
}



int uart3_send(unsigned char *buf, int len)
{
    int ret = 0;

    if(HAL_OK == HAL_UART_Transmit(&huart3, buf, len, 0xFFFF))
    {
        ret = len;
    }
    return ret;
}
int uart3_recv(unsigned char *buf, int len, int timeout)
{
    int datalen = 0;
    int ret = 0;
    unsigned char value;
    do
    {
        datalen = ring_datalen(&gRcvRing);
        task_sleepms(1);
    }
    while((timeout-- > 0) && (datalen < len));
    if(datalen > 0)
    {
        ret = ring_read(&gRcvRing, buf, len);
    }
    return ret;
}

void uart_init(void)
{
    uart3_init();
}
int uart_write(char *buf, int len, int timeout)
{
    return uart3_send((unsigned char *)buf, len);
}
int uart_read(char *buf, int len, int timeout)
{
    return uart3_recv((unsigned char *)buf, len, timeout);
}

#endif


#elif defined ( __GNUC__ )  /* GCC: printf will call _write to print */
__attribute__((used)) int _write(int fd, char *ptr, int len)
{
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}
#endif

/* USER CODE BEGIN 1 */



