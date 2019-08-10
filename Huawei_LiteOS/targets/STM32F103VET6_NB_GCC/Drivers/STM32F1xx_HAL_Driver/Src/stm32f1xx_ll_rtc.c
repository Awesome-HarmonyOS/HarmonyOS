/**
  ******************************************************************************
  * @file    stm32f1xx_ll_rtc.c
  * @author  MCD Application Team                                                                                          
  * @brief   RTC LL module driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#if defined(USE_FULL_LL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_rtc.h"
#include "stm32f1xx_ll_cortex.h"
#ifdef  USE_FULL_ASSERT
#include "stm32_assert.h"
#else
#define assert_param(expr) ((void)0U)
#endif

/** @addtogroup STM32F1xx_LL_Driver
  * @{
  */

#if defined(RTC)

/** @addtogroup RTC_LL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @addtogroup RTC_LL_Private_Constants
  * @{
  */
/* Default values used for prescaler */
#define RTC_ASYNCH_PRESC_DEFAULT     0x00007FFFU

/* Values used for timeout */
#define RTC_INITMODE_TIMEOUT         1000U /* 1s when tick set to 1ms */
#define RTC_SYNCHRO_TIMEOUT          1000U /* 1s when tick set to 1ms */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup RTC_LL_Private_Macros
  * @{
  */

#define IS_LL_RTC_ASYNCH_PREDIV(__VALUE__)   ((__VALUE__) <= 0xFFFFFU)

#define IS_LL_RTC_FORMAT(__VALUE__) (((__VALUE__) == LL_RTC_FORMAT_BIN) \
                                  || ((__VALUE__) == LL_RTC_FORMAT_BCD))

#define IS_LL_RTC_HOUR24(__HOUR__)            ((__HOUR__) <= 23U)
#define IS_LL_RTC_MINUTES(__MINUTES__)        ((__MINUTES__) <= 59U)
#define IS_LL_RTC_SECONDS(__SECONDS__)        ((__SECONDS__) <= 59U)
#define IS_LL_RTC_CALIB_OUTPUT(__OUTPUT__) (((__OUTPUT__) == LL_RTC_CALIB_OUTPUT_NONE) || \
                                            ((__OUTPUT__) == LL_RTC_CALIB_OUTPUT_RTCCLOCK) || \
                                            ((__OUTPUT__) == LL_RTC_CALIB_OUTPUT_ALARM) || \
                                            ((__OUTPUT__) == LL_RTC_CALIB_OUTPUT_SECOND)) 
/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup RTC_LL_Exported_Functions
  * @{
  */

/** @addtogroup RTC_LL_EF_Init
  * @{
  */

/**
  * @brief  De-Initializes the RTC registers to their default reset values.
  * @note   This function doesn't reset the RTC Clock source and RTC Backup Data
  *         registers.
  * @param  RTCx RTC Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC registers are de-initialized
  *          - ERROR: RTC registers are not de-initialized
  */
ErrorStatus LL_RTC_DeInit(RTC_TypeDef *RTCx)
{
  ErrorStatus status = ERROR;

  /* Check the parameter */
  assert_param(IS_RTC_ALL_INSTANCE(RTCx));

  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTCx);

  /* Set Initialization mode */
  if (LL_RTC_EnterInitMode(RTCx) != ERROR)
  {
    LL_RTC_WriteReg(RTCx,CNTL, 0x0000);
    LL_RTC_WriteReg(RTCx,CNTH, 0x0000);
    LL_RTC_WriteReg(RTCx,PRLH, 0x0000);
    LL_RTC_WriteReg(RTCx,PRLL, 0x8000);
    LL_RTC_WriteReg(RTCx,CRH,  0x0000);
    LL_RTC_WriteReg(RTCx,CRL,  0x0020);
    
    /* Reset Tamper and alternate functions configuration register */
    LL_RTC_WriteReg(BKP,RTCCR, 0x00000000U);
    LL_RTC_WriteReg(BKP,CR,    0x00000000U);
    LL_RTC_WriteReg(BKP,CSR,   0x00000000U);
    
    /* Exit Initialization Mode */
    if(LL_RTC_ExitInitMode(RTCx) == ERROR)
    {
      return ERROR;
    }
    
    /* Wait till the RTC RSF flag is set */
    status = LL_RTC_WaitForSynchro(RTCx);
    
    /* Clear RSF Flag */
    LL_RTC_ClearFlag_RS(RTCx);
  }

  /* Enable the write protection for RTC registers */
  LL_RTC_EnableWriteProtection(RTCx);

  return status;
}

/**
  * @brief  Initializes the RTC registers according to the specified parameters
  *         in RTC_InitStruct.
  * @param  RTCx RTC Instance
  * @param  RTC_InitStruct pointer to a @ref LL_RTC_InitTypeDef structure that contains
  *         the configuration information for the RTC peripheral.
  * @note   The RTC Prescaler register is write protected and can be written in
  *         initialization mode only.
  * @note   the user should call LL_RTC_StructInit()  or the structure of Prescaler
  *         need to be initialized  before RTC init()
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC registers are initialized
  *          - ERROR: RTC registers are not initialized
  */
ErrorStatus LL_RTC_Init(RTC_TypeDef *RTCx, LL_RTC_InitTypeDef *RTC_InitStruct)
{
  ErrorStatus status = ERROR;

  /* Check the parameters */
  assert_param(IS_RTC_ALL_INSTANCE(RTCx));
  assert_param(IS_LL_RTC_ASYNCH_PREDIV(RTC_InitStruct->AsynchPrescaler));
  assert_param(IS_LL_RTC_CALIB_OUTPUT(RTC_InitStruct->OutPutSource));
  /* Waiting for synchro */
  if(LL_RTC_WaitForSynchro(RTCx) != ERROR)
  {
    /* Set Initialization mode */
    if (LL_RTC_EnterInitMode(RTCx) != ERROR)
    {
      /* Clear Flag Bits */
      LL_RTC_ClearFlag_ALR(RTCx);
      LL_RTC_ClearFlag_OW(RTCx);
      LL_RTC_ClearFlag_SEC(RTCx);
      
      if(RTC_InitStruct->OutPutSource != LL_RTC_CALIB_OUTPUT_NONE)
      {
        /* Disable the selected Tamper Pin */
        LL_RTC_TAMPER_Disable(BKP);
      }
      /* Set the signal which will be routed to RTC Tamper Pin */
      LL_RTC_SetOutputSource(BKP, RTC_InitStruct->OutPutSource);
      
      /* Configure Synchronous and Asynchronous prescaler factor */
      LL_RTC_SetAsynchPrescaler(RTCx, RTC_InitStruct->AsynchPrescaler);
      
      /* Exit Initialization Mode */
      LL_RTC_ExitInitMode(RTCx);      
      
      status = SUCCESS;
    }
  }
  return status;
}

/**
  * @brief  Set each @ref LL_RTC_InitTypeDef field to default value.
  * @param  RTC_InitStruct pointer to a @ref LL_RTC_InitTypeDef structure which will be initialized.
  * @retval None
  */
void LL_RTC_StructInit(LL_RTC_InitTypeDef *RTC_InitStruct)
{
  /* Set RTC_InitStruct fields to default values */
  RTC_InitStruct->AsynchPrescaler = RTC_ASYNCH_PRESC_DEFAULT;
  RTC_InitStruct->OutPutSource    = LL_RTC_CALIB_OUTPUT_NONE;
}

/**
  * @brief  Set the RTC current time.
  * @param  RTCx RTC Instance
  * @param  RTC_Format This parameter can be one of the following values:
  *         @arg @ref LL_RTC_FORMAT_BIN
  *         @arg @ref LL_RTC_FORMAT_BCD
  * @param  RTC_TimeStruct pointer to a RTC_TimeTypeDef structure that contains
  *                        the time configuration information for the RTC.
  * @note  The user should call LL_RTC_TIME_StructInit() or the structure 
  *        of time need to be initialized  before time init()
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Time register is configured
  *          - ERROR: RTC Time register is not configured
  */
ErrorStatus LL_RTC_TIME_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, LL_RTC_TimeTypeDef *RTC_TimeStruct)
{
  ErrorStatus status = ERROR;
  uint32_t counter_time = 0U;

  /* Check the parameters */
  assert_param(IS_RTC_ALL_INSTANCE(RTCx));
  assert_param(IS_LL_RTC_FORMAT(RTC_Format));

  if (RTC_Format == LL_RTC_FORMAT_BIN)
  {
    assert_param(IS_LL_RTC_HOUR24(RTC_TimeStruct->Hours));
    assert_param(IS_LL_RTC_MINUTES(RTC_TimeStruct->Minutes));
    assert_param(IS_LL_RTC_SECONDS(RTC_TimeStruct->Seconds));
  }
  else
  {
    assert_param(IS_LL_RTC_HOUR24(__LL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Hours)));
    assert_param(IS_LL_RTC_MINUTES(__LL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Minutes)));
    assert_param(IS_LL_RTC_SECONDS(__LL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Seconds)));
  }

  /* Enter Initialization mode */
  if (LL_RTC_EnterInitMode(RTCx) != ERROR)
  {
    /* Check the input parameters format */
    if (RTC_Format != LL_RTC_FORMAT_BIN)
    {
    counter_time = (uint32_t)(((uint32_t)RTC_TimeStruct->Hours * 3600U) + \
                        ((uint32_t)RTC_TimeStruct->Minutes * 60U) + \
                        ((uint32_t)RTC_TimeStruct->Seconds));
      LL_RTC_TIME_Set(RTCx, counter_time);
    }
    else
    {
     counter_time = (((uint32_t)(__LL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Hours)) * 3600U) + \
              ((uint32_t)(__LL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Minutes)) * 60U) + \
              ((uint32_t)(__LL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Seconds))));
      LL_RTC_TIME_Set(RTCx, counter_time);
    }
    status = SUCCESS;
  }
  /* Exit Initialization mode */
  LL_RTC_ExitInitMode(RTCx);

  return status;
}

/**
  * @brief  Set each @ref LL_RTC_TimeTypeDef field to default value (Time = 00h:00min:00sec).
  * @param  RTC_TimeStruct pointer to a @ref LL_RTC_TimeTypeDef structure which will be initialized.
  * @retval None
  */
void LL_RTC_TIME_StructInit(LL_RTC_TimeTypeDef *RTC_TimeStruct)
{
  /* Time = 00h:00min:00sec */
  RTC_TimeStruct->Hours      = 0U;
  RTC_TimeStruct->Minutes    = 0U;
  RTC_TimeStruct->Seconds    = 0U;
}

/**
  * @brief  Set the RTC Alarm.
  * @param  RTCx RTC Instance
  * @param  RTC_Format This parameter can be one of the following values:
  *         @arg @ref LL_RTC_FORMAT_BIN
  *         @arg @ref LL_RTC_FORMAT_BCD
  * @param  RTC_AlarmStruct pointer to a @ref LL_RTC_AlarmTypeDef structure that
  *                         contains the alarm configuration parameters.
  * @note   the user should call LL_RTC_ALARM_StructInit()  or the structure 
  *         of Alarm need to be initialized  before Alarm init()
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ALARM registers are configured
  *          - ERROR: ALARM registers are not configured
  */
ErrorStatus LL_RTC_ALARM_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, LL_RTC_AlarmTypeDef *RTC_AlarmStruct)
{
  ErrorStatus status = ERROR;
  uint32_t counter_alarm = 0U;
  /* Check the parameters */
  assert_param(IS_RTC_ALL_INSTANCE(RTCx));
  assert_param(IS_LL_RTC_FORMAT(RTC_Format));

  if (RTC_Format == LL_RTC_FORMAT_BIN)
  {
    assert_param(IS_LL_RTC_HOUR24(RTC_AlarmStruct->AlarmTime.Hours));
    assert_param(IS_LL_RTC_MINUTES(RTC_AlarmStruct->AlarmTime.Minutes));
    assert_param(IS_LL_RTC_SECONDS(RTC_AlarmStruct->AlarmTime.Seconds));
  }
  else
  {
    assert_param(IS_LL_RTC_HOUR24(__LL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Hours)));
    assert_param(IS_LL_RTC_MINUTES(__LL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Minutes)));
    assert_param(IS_LL_RTC_SECONDS(__LL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Seconds)));
  }

  /* Enter Initialization mode */
  if (LL_RTC_EnterInitMode(RTCx) != ERROR)
  {
    /* Check the input parameters format */
    if (RTC_Format != LL_RTC_FORMAT_BIN)
    {
    counter_alarm = (uint32_t)(((uint32_t)RTC_AlarmStruct->AlarmTime.Hours * 3600U) + \
                        ((uint32_t)RTC_AlarmStruct->AlarmTime.Minutes * 60U) + \
                        ((uint32_t)RTC_AlarmStruct->AlarmTime.Seconds));
      LL_RTC_ALARM_Set(RTCx, counter_alarm);
    }
    else
    {
     counter_alarm = (((uint32_t)(__LL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Hours)) * 3600U) + \
              ((uint32_t)(__LL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Minutes)) * 60U) + \
              ((uint32_t)(__LL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Seconds))));
      LL_RTC_ALARM_Set(RTCx, counter_alarm);
    }
    status = SUCCESS;
  }
  /* Exit Initialization mode */
  LL_RTC_ExitInitMode(RTCx);

  return status;
}

/**
  * @brief  Set each @ref LL_RTC_AlarmTypeDef of ALARM field to default value (Time = 00h:00mn:00sec /
  *         Day = 1st day of the month/Mask = all fields are masked).
  * @param  RTC_AlarmStruct pointer to a @ref LL_RTC_AlarmTypeDef structure which will be initialized.
  * @retval None
  */
void LL_RTC_ALARM_StructInit(LL_RTC_AlarmTypeDef *RTC_AlarmStruct)
{
  /* Alarm Time Settings : Time = 00h:00mn:00sec */
  RTC_AlarmStruct->AlarmTime.Hours      = 0U;
  RTC_AlarmStruct->AlarmTime.Minutes    = 0U;
  RTC_AlarmStruct->AlarmTime.Seconds    = 0U;
}

/**
  * @brief  Enters the RTC Initialization mode.
  * @param  RTCx RTC Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC is in Init mode
  *          - ERROR: RTC is not in Init mode
  */
ErrorStatus LL_RTC_EnterInitMode(RTC_TypeDef *RTCx)
{
  __IO uint32_t timeout = RTC_INITMODE_TIMEOUT;
  ErrorStatus status = SUCCESS;
  uint32_t tmp = 0U;

  /* Check the parameter */
  assert_param(IS_RTC_ALL_INSTANCE(RTCx));

    /* Wait till RTC is in INIT state and if Time out is reached exit */
    tmp = LL_RTC_IsActiveFlag_RTOF(RTCx);
    while ((timeout != 0U) && (tmp != 1U))
    {
      if (LL_SYSTICK_IsActiveCounterFlag() == 1U)
      {
        timeout --;
      }
      tmp = LL_RTC_IsActiveFlag_RTOF(RTCx);
      if (timeout == 0U)
      {
        status = ERROR;
      }
    }

   /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTCx);
  
  return status;
}

/**
  * @brief  Exit the RTC Initialization mode.
  * @note   When the initialization sequence is complete, the calendar restarts
  *         counting after 4 RTCCLK cycles.
  * @param  RTCx RTC Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC exited from in Init mode
  *          - ERROR: Not applicable
  */
ErrorStatus LL_RTC_ExitInitMode(RTC_TypeDef *RTCx)
{
  __IO uint32_t timeout = RTC_INITMODE_TIMEOUT;
  ErrorStatus status = SUCCESS;
  uint32_t tmp = 0U;
  
  /* Check the parameter */
  assert_param(IS_RTC_ALL_INSTANCE(RTCx));
  
  /* Disable initialization mode */
  LL_RTC_EnableWriteProtection(RTCx);
  
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  tmp = LL_RTC_IsActiveFlag_RTOF(RTCx);
  while ((timeout != 0U) && (tmp != 1U))
  {
    if (LL_SYSTICK_IsActiveCounterFlag() == 1U)
    {
      timeout --;
    }
    tmp = LL_RTC_IsActiveFlag_RTOF(RTCx);
    if (timeout == 0U)
    {
      status = ERROR;
    }
  }
  return status;
}

/**
  * @brief  Set the Time Counter
  * @param  RTCx RTC Instance
  * @param  TimeCounter this value can be from 0 to 0xFFFFFFFF
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Counter register configured
  *          - ERROR: Not applicable
  */
ErrorStatus LL_RTC_TIME_SetCounter(RTC_TypeDef *RTCx, uint32_t TimeCounter)
{
  ErrorStatus status = ERROR;
  /* Check the parameter */
  assert_param(IS_RTC_ALL_INSTANCE(RTCx));

  /* Enter Initialization mode */
  if (LL_RTC_EnterInitMode(RTCx) != ERROR)
  {
   LL_RTC_TIME_Set(RTCx, TimeCounter);
   status = SUCCESS;
  }
  /* Exit Initialization mode */
  LL_RTC_ExitInitMode(RTCx);
  
  return status;
}

/**
  * @brief  Set Alarm Counter.
  * @param  RTCx RTC Instance
  * @param  AlarmCounter this value can be from 0 to 0xFFFFFFFF
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC exited from in Init mode
  *          - ERROR: Not applicable
  */
ErrorStatus LL_RTC_ALARM_SetCounter(RTC_TypeDef *RTCx, uint32_t AlarmCounter)
{
  ErrorStatus status = ERROR;
  /* Check the parameter */
  assert_param(IS_RTC_ALL_INSTANCE(RTCx));

  /* Enter Initialization mode */
  if (LL_RTC_EnterInitMode(RTCx) != ERROR)
  {
     LL_RTC_ALARM_Set(RTCx, AlarmCounter);
     status = SUCCESS;
  }
  /* Exit Initialization mode */
  LL_RTC_ExitInitMode(RTCx);

  return status;
}

/**
  * @brief  Waits until the RTC registers are synchronized with RTC APB clock.
  * @note   The RTC Resynchronization mode is write protected, use the
  *         @ref LL_RTC_DisableWriteProtection before calling this function.
  * @param  RTCx RTC Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC registers are synchronised
  *          - ERROR: RTC registers are not synchronised
  */
ErrorStatus LL_RTC_WaitForSynchro(RTC_TypeDef *RTCx)
{
  __IO uint32_t timeout = RTC_SYNCHRO_TIMEOUT;
  ErrorStatus status = SUCCESS;
  uint32_t tmp = 0U;

  /* Check the parameter */
  assert_param(IS_RTC_ALL_INSTANCE(RTCx));

  /* Clear RSF flag */
  LL_RTC_ClearFlag_RS(RTCx);

  /* Wait the registers to be synchronised */
  tmp = LL_RTC_IsActiveFlag_RS(RTCx);
  while ((timeout != 0U) && (tmp != 0U))
  {
    if (LL_SYSTICK_IsActiveCounterFlag() == 1U)
    {
      timeout--;
    }
    tmp = LL_RTC_IsActiveFlag_RS(RTCx);
    if (timeout == 0U)
    {
      status = ERROR;
    }
  }

  return (status);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined(RTC) */

/**
  * @}
  */

#endif /* USE_FULL_LL_DRIVER */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
