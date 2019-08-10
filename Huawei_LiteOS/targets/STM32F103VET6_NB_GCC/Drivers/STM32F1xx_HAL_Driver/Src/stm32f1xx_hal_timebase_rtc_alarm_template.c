/**
  ******************************************************************************
  * @file    stm32f1xx_hal_timebase_rtc_alarm_template.c
  * @author  MCD Application Team
  * @brief   HAL time base based on the hardware RTC_ALARM.
  *
  *          This file override the native HAL time base functions (defined as weak)
  *          to use the RTC ALARM for time base generation:
  *           + Intializes the RTC peripheral to increment the seconds registers each 1ms
  *           + The alarm is configured to assert an interrupt when the RTC reaches 1ms
  *           + HAL_IncTick is called at each Alarm event and the time is reset to 00:00:00
  *           + HSE (default), LSE or LSI can be selected as RTC clock source
 @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
    This file must be copied to the application folder and modified as follows:
    (#) Rename it to 'stm32f1xx_hal_timebase_rtc_alarm.c'
    (#) Add this file and the RTC HAL drivers to your project and uncomment
       HAL_RTC_MODULE_ENABLED define in stm32f1xx_hal_conf.h

    [..]
    (@) HAL RTC alarm and HAL RTC wakeup drivers can’t be used with low power modes:
        The wake up capability of the RTC may be intrusive in case of prior low power mode
        configuration requiring different wake up sources.
        Application/Example behavior is no more guaranteed
    (@) The stm32f1xx_hal_timebase_tim use is recommended for the Applications/Examples
          requiring low power modes

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "stm32f1xx_hal.h"
/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_TimeBase_RTC_Alarm_Template  HAL TimeBase RTC Alarm Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Uncomment the line below to select the appropriate RTC Clock source for your application:
  + RTC_CLOCK_SOURCE_HSE: can be selected for applications requiring timing precision.
  + RTC_CLOCK_SOURCE_LSE: can be selected for applications with low constraint on timing
                          precision.
  + RTC_CLOCK_SOURCE_LSI: can be selected for applications with low constraint on timing
                          precision.
  */
#define RTC_CLOCK_SOURCE_HSE
/* #define RTC_CLOCK_SOURCE_LSE */
/* #define RTC_CLOCK_SOURCE_LSI */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef        hRTC_Handle;
/* Private function prototypes -----------------------------------------------*/
void RTC_Alarm_IRQHandler(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the RTC_ALARMA as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  __IO uint32_t counter = 0U;

  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

#ifdef RTC_CLOCK_SOURCE_LSE
  /* Configue LSE as RTC clock soucre */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
#elif defined (RTC_CLOCK_SOURCE_LSI)
  /* Configue LSI as RTC clock soucre */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
#elif defined (RTC_CLOCK_SOURCE_HSE)
  /* Configue HSE as RTC clock soucre */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;
#else
#error Please select the RTC Clock source
#endif /* RTC_CLOCK_SOURCE_LSE */

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) == HAL_OK)
    {
      /* Enable RTC Clock */
      __HAL_RCC_RTC_ENABLE();

      hRTC_Handle.Instance = RTC;
      /* Configure RTC time base to 10Khz */
      hRTC_Handle.Init.AsynchPrediv = (HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_RTC) / 10000) - 1;
      hRTC_Handle.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
      HAL_RTC_Init(&hRTC_Handle);

      /* Disable the write protection for RTC registers */
      __HAL_RTC_WRITEPROTECTION_DISABLE(&hRTC_Handle);

      /* Clear flag alarm A */
      __HAL_RTC_ALARM_CLEAR_FLAG(&hRTC_Handle, RTC_FLAG_ALRAF);

      counter = 0U;
      /* Wait till RTC ALRAF flag is set and if Time out is reached exit */
      while (__HAL_RTC_ALARM_GET_FLAG(&hRTC_Handle, RTC_FLAG_ALRAF) != RESET)
      {
        if (counter++ == SystemCoreClock / 48U) /* Timeout = ~ 1s */
        {
          return HAL_ERROR;
        }
      }

      /* Set RTC COUNTER MSB word */
      hRTC_Handle.Instance->ALRH = 0x00U;
      /* Set RTC COUNTER LSB word */
      hRTC_Handle.Instance->ALRL = 0x09U;

      /* RTC Alarm Interrupt Configuration: EXTI configuration */
      __HAL_RTC_ALARM_EXTI_ENABLE_IT();
      __HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE();

      /* Clear Second and overflow flags */
      CLEAR_BIT(hRTC_Handle.Instance->CRL, (RTC_FLAG_SEC | RTC_FLAG_OW));

      /* Set RTC COUNTER MSB word */
      hRTC_Handle.Instance->CNTH = 0x00U;
      /* Set RTC COUNTER LSB word */
      hRTC_Handle.Instance->CNTL = 0x00U;

      /* Configure the Alarm interrupt */
      __HAL_RTC_ALARM_ENABLE_IT(&hRTC_Handle, RTC_IT_ALRA);

      /* Enable the write protection for RTC registers */
      __HAL_RTC_WRITEPROTECTION_ENABLE(&hRTC_Handle);

      /* Wait till RTC is in INIT state and if Time out is reached exit */
      counter = 0U;
      while ((hRTC_Handle.Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
      {
        if (counter++ == SystemCoreClock / 48U) /* Timeout = ~ 1s */
        {
          return HAL_ERROR;
        }
      }

      HAL_NVIC_SetPriority(RTC_Alarm_IRQn, TickPriority, 0U);
      HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling RTC ALARM interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable RTC ALARM update Interrupt */
  __HAL_RTC_ALARM_DISABLE_IT(&hRTC_Handle, RTC_IT_ALRA);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling RTC ALARM interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  __IO uint32_t counter = 0U;

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(&hRTC_Handle);

  /* Set RTC COUNTER MSB word */
  hRTC_Handle.Instance->CNTH = 0x00U;
  /* Set RTC COUNTER LSB word */
  hRTC_Handle.Instance->CNTL = 0x00U;

  /* Clear Second and overflow flags */
  CLEAR_BIT(hRTC_Handle.Instance->CRL, (RTC_FLAG_SEC | RTC_FLAG_OW | RTC_FLAG_ALRAF));

  /* Enable RTC ALARM Update interrupt */
  __HAL_RTC_ALARM_ENABLE_IT(&hRTC_Handle, RTC_IT_ALRA);

  /* Enable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_ENABLE(&hRTC_Handle);

  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while ((hRTC_Handle.Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if (counter++ == SystemCoreClock / 48U) /* Timeout = ~ 1s */
    {
      break;
    }
  }
}

/**
  * @brief  ALARM A Event Callback in non blocking mode
  * @note   This function is called  when RTC_ALARM interrupt took place, inside
  * RTC_ALARM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  __IO uint32_t counter = 0U;

  HAL_IncTick();

  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Set RTC COUNTER MSB word */
  WRITE_REG(hrtc->Instance->CNTH, 0x00U);
  /* Set RTC COUNTER LSB word */
  WRITE_REG(hrtc->Instance->CNTL, 0x00U);

  /* Clear Second and overflow flags */
  CLEAR_BIT(hrtc->Instance->CRL, (RTC_FLAG_SEC | RTC_FLAG_OW));

  /* Enable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while ((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if (counter++ == SystemCoreClock / 48U) /* Timeout = ~ 1s */
    {
      break;
    }
  }
}

/**
  * @brief  This function handles RTC ALARM interrupt request.
  * @retval None
  */
void RTC_Alarm_IRQHandler(void)
{
  HAL_RTC_AlarmIRQHandler(&hRTC_Handle);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
