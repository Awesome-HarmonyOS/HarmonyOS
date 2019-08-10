/**
  ******************************************************************************
  * @file    stm32f4xx_hal_iwdg.c
  * @author  MCD Application Team
  * @brief   IWDG HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Independent Watchdog (IWDG) peripheral:
  *           + Initialization and Start functions
  *           + IO operation functions
  *
  @verbatim
  ==============================================================================
                    ##### IWDG Generic features #####
  ==============================================================================
  [..]
    (+) The IWDG can be started by either software or hardware (configurable
        through option byte).

    (+) The IWDG is clocked by Low-Speed clock (LSI) and thus stays active even
        if the main clock fails.

    (+) Once the IWDG is started, the LSI is forced ON and both can not be
        disabled. The counter starts counting down from the reset value (0xFFF).
        When it reaches the end of count value (0x000) a reset signal is
        generated (IWDG reset).

    (+) Whenever the key value 0x0000 AAAA is written in the IWDG_KR register,
        the IWDG_RLR value is reloaded in the counter and the watchdog reset is
        prevented.

    (+) The IWDG is implemented in the VDD voltage domain that is still functional
        in STOP and STANDBY mode (IWDG reset can wake-up from STANDBY).
        IWDGRST flag in RCC_CSR register can be used to inform when an IWDG
        reset occurs.

    (+) Debug mode : When the microcontroller enters debug mode (core halted),
        the IWDG counter either continues to work normally or stops, depending
        on DBG_IWDG_STOP configuration bit in DBG module, accessible through
        __HAL_DBGMCU_FREEZE_IWDG() and __HAL_DBGMCU_UNFREEZE_IWDG() macros

    [..] Min-max timeout value @32KHz (LSI): ~125us / ~32.7s
         The IWDG timeout may vary due to LSI frequency dispersion. STM32F4xx
         devices provide the capability to measure the LSI frequency (LSI clock
         connected internally to TIM5 CH4 input capture). The measured value
         can be used to have an IWDG timeout with an acceptable accuracy.

                     ##### How to use this driver #####
  ==============================================================================
  [..]
    (#) Use IWDG using HAL_IWDG_Init() function to :
      (++) Enable instance by writing Start keyword in IWDG_KEY register. LSI
           clock is forced ON and IWDG counter starts downcounting.
      (++) Enable write access to configuration register: IWDG_PR & IWDG_RLR.
      (++) Configure the IWDG prescaler and counter reload value. This reload
           value will be loaded in the IWDG counter each time the watchdog is
           reloaded, then the IWDG will start counting down from this value.
      (++) wait for status flags to be reset"

    (#) Then the application program must refresh the IWDG counter at regular
        intervals during normal operation to prevent an MCU reset, using
        HAL_IWDG_Refresh() function.

     *** IWDG HAL driver macros list ***
     ====================================
     [..]
       Below the list of most used macros in IWDG HAL driver:
      (+) __HAL_IWDG_START: Enable the IWDG peripheral
      (+) __HAL_IWDG_RELOAD_COUNTER: Reloads IWDG counter with value defined in
          the reload register

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
  *      and/or other materials provided with the distribution
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
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

#ifdef HAL_IWDG_MODULE_ENABLED
/** @defgroup IWDG IWDG
  * @brief IWDG HAL module driver.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup IWDG_Private_Defines IWDG Private Defines
  * @{
  */
/* Status register need 5 RC LSI divided by prescaler clock to be updated. With
   higher prescaler (256), and according to HSI variation, we need to wait at
   least 6 cycles so 48 ms. */
#define HAL_IWDG_DEFAULT_TIMEOUT            48U
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup IWDG_Exported_Functions
  * @{
  */

/** @addtogroup IWDG_Exported_Functions_Group1
  *  @brief    Initialization and Start functions.
  *
@verbatim
 ===============================================================================
          ##### Initialization and Start functions #####
 ===============================================================================
 [..]  This section provides functions allowing to:
      (+) Initialize the IWDG according to the specified parameters in the
          IWDG_InitTypeDef of associated handle.
      (+) Once initialization is performed in HAL_IWDG_Init function, Watchdog
          is reloaded in order to exit function with correct time base.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the IWDG according to the specified parameters in the
  *         IWDG_InitTypeDef and start watchdog. Before exiting function,
  *         watchdog is refreshed in order to have correct time base.
  * @param  hiwdg  pointer to a IWDG_HandleTypeDef structure that contains
  *                the configuration information for the specified IWDG module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg)
{
  uint32_t tickstart;

  /* Check the IWDG handle allocation */
  if(hiwdg == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_IWDG_ALL_INSTANCE(hiwdg->Instance));
  assert_param(IS_IWDG_PRESCALER(hiwdg->Init.Prescaler));
  assert_param(IS_IWDG_RELOAD(hiwdg->Init.Reload));

  /* Enable IWDG. LSI is turned on automaticaly */
  __HAL_IWDG_START(hiwdg);

  /* Enable write access to IWDG_PR and IWDG_RLR registers by writing 0x5555 in KR */
  IWDG_ENABLE_WRITE_ACCESS(hiwdg);

  /* Write to IWDG registers the Prescaler & Reload values to work with */
  hiwdg->Instance->PR = hiwdg->Init.Prescaler;
  hiwdg->Instance->RLR = hiwdg->Init.Reload;

  /* Check pending flag, if previous update not done, return timeout */
  tickstart = HAL_GetTick();

  /* Wait for register to be updated */
  while(hiwdg->Instance->SR != RESET)
  {
    if((HAL_GetTick() - tickstart ) > HAL_IWDG_DEFAULT_TIMEOUT)
    {
      return HAL_TIMEOUT;
    }
  }

  /* Reload IWDG counter with value defined in the reload register */
  __HAL_IWDG_RELOAD_COUNTER(hiwdg);

  /* Return function status */
  return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup IWDG_Exported_Functions_Group2
  *  @brief   IO operation functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]  This section provides functions allowing to:
      (+) Refresh the IWDG.

@endverbatim
  * @{
  */

/**
  * @brief  Refresh the IWDG.
  * @param  hiwdg  pointer to a IWDG_HandleTypeDef structure that contains
  *                the configuration information for the specified IWDG module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg)
{
  /* Reload IWDG counter with value defined in the reload register */
  __HAL_IWDG_RELOAD_COUNTER(hiwdg);

  /* Return function status */
  return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_IWDG_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
