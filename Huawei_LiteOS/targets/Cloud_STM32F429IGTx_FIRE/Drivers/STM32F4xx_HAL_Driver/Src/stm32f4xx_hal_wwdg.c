/**
  ******************************************************************************
  * @file    stm32f4xx_hal_wwdg.c
  * @author  MCD Application Team
  * @brief   WWDG HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Window Watchdog (WWDG) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State functions
  @verbatim
  ==============================================================================
                      ##### WWDG specific features #####
  ==============================================================================
  [..]
    Once enabled the WWDG generates a system reset on expiry of a programmed
    time period, unless the program refreshes the counter (downcounter)
    before reaching 0x3F value (i.e. a reset is generated when the counter
    value rolls over from 0x40 to 0x3F).

    (+) An MCU reset is also generated if the counter value is refreshed
        before the counter has reached the refresh window value. This
        implies that the counter must be refreshed in a limited window.
    (+) Once enabled the WWDG cannot be disabled except by a system reset.
    (+) WWDGRST flag in RCC_CSR register can be used to inform when a WWDG
        reset occurs.
    (+) The WWDG counter input clock is derived from the APB clock divided
        by a programmable prescaler.
    (+) WWDG clock (Hz) = PCLK1 / (4096 * Prescaler)
    (+) WWDG timeout (mS) = 1000 * Counter / WWDG clock
    (+) WWDG Counter refresh is allowed between the following limits :
        (++) min time (mS) = 1000 * (Counter _ Window) / WWDG clock
        (++) max time (mS) = 1000 * (Counter _ 0x40) / WWDG clock

    (+) Min-max timeout value at 50 MHz(PCLK1): 81.9 us / 41.9 ms

    (+) The Early Wakeup Interrupt (EWI) can be used if specific safety
        operations or data logging must be performed before the actual reset is
        generated. When the downcounter reaches the value 0x40, an EWI interrupt
        is generated and the corresponding interrupt service routine (ISR) can
        be used to trigger specific actions (such as communications or data
        logging), before resetting the device.
        In some applications, the EWI interrupt can be used to manage a software
        system check and/or system recovery/graceful degradation, without
        generating a WWDG reset. In this case, the corresponding interrupt
        service routine (ISR) should reload the WWDG counter to avoid the WWDG
        reset, then trigger the required actions.
        Note:When the EWI interrupt cannot be served, e.g. due to a system lock
        in a higher priority task, the WWDG reset will eventually be generated.

    (+) Debug mode : When the microcontroller enters debug mode (core halted),
        the WWDG counter either continues to work normally or stops, depending
        on DBG_WWDG_STOP configuration bit in DBG module, accessible through
        __HAL_DBGMCU_FREEZE_WWDG() and __HAL_DBGMCU_UNFREEZE_WWDG() macros

                     ##### How to use this driver #####
  ==============================================================================
  [..]
    (+) Enable WWDG APB1 clock using __HAL_RCC_WWDG_CLK_ENABLE().

    (+) Set the WWDG prescaler, refresh window, counter value and Early Wakeup
        Interrupt mode using using HAL_WWDG_Init() function.
        This enables WWDG peripheral and the downcounter starts downcounting
        from given counter value.
        Init function can be called again to modify all watchdog parameters,
        however if EWI mode has been set once, it can't be clear until next
        reset.

    (+) The application program must refresh the WWDG counter at regular
        intervals during normal operation to prevent an MCU reset using
        HAL_WWDG_Refresh() function. This operation must occur only when
        the counter is lower than the window value already programmed.

    (+) if Early Wakeup Interrupt mode is enable an interrupt is generated when
        the counter reaches 0x40. User can add his own code in weak function
        HAL_WWDG_EarlyWakeupCallback().

     *** WWDG HAL driver macros list ***
     ==================================
     [..]
       Below the list of most used macros in WWDG HAL driver.

      (+) __HAL_WWDG_GET_IT_SOURCE: Check the selected WWDG's interrupt source.
      (+) __HAL_WWDG_GET_FLAG: Get the selected WWDG's flag status.
      (+) __HAL_WWDG_CLEAR_FLAG: Clear the WWDG's pending flags.

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
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

#ifdef HAL_WWDG_MODULE_ENABLED
/** @defgroup WWDG WWDG
  * @brief WWDG HAL module driver.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup WWDG_Exported_Functions WWDG Exported Functions
  * @{
  */

/** @defgroup WWDG_Exported_Functions_Group1 Initialization and Configuration functions
  *  @brief    Initialization and Configuration functions.
  *
@verbatim
  ==============================================================================
          ##### Initialization and Configuration functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
      (+) Initialize and start the WWDG according to the specified parameters
          in the WWDG_InitTypeDef of associated handle.
      (+) Initialize the WWDG MSP.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the WWDG according to the specified.
  *         parameters in the WWDG_InitTypeDef of  associated handle.
  * @param  hwwdg  pointer to a WWDG_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDG module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_WWDG_Init(WWDG_HandleTypeDef *hwwdg)
{
  /* Check the WWDG handle allocation */
  if(hwwdg == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_WWDG_ALL_INSTANCE(hwwdg->Instance));
  assert_param(IS_WWDG_PRESCALER(hwwdg->Init.Prescaler));
  assert_param(IS_WWDG_WINDOW(hwwdg->Init.Window));
  assert_param(IS_WWDG_COUNTER(hwwdg->Init.Counter));
  assert_param(IS_WWDG_EWI_MODE(hwwdg->Init.EWIMode));

  /* Init the low level hardware */
  HAL_WWDG_MspInit(hwwdg);

  /* Set WWDG Counter */
  WRITE_REG(hwwdg->Instance->CR, (WWDG_CR_WDGA | hwwdg->Init.Counter));

  /* Set WWDG Prescaler and Window */
  WRITE_REG(hwwdg->Instance->CFR, (hwwdg->Init.EWIMode | hwwdg->Init.Prescaler | hwwdg->Init.Window));

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initialize the WWDG MSP.
  * @param  hwwdg  pointer to a WWDG_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDG module.
  * @note   When rewriting this function in user file, mechanism may be added
  *         to avoid multiple initialize when HAL_WWDG_Init function is called
  *         again to change parameters.
  * @retval None
  */
__weak void HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hwwdg);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_WWDG_MspInit could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup WWDG_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Refresh the WWDG.
    (+) Handle WWDG interrupt request and associated function callback.

@endverbatim
  * @{
  */

/**
  * @brief  Refresh the WWDG.
  * @param  hwwdg  pointer to a WWDG_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDG module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_WWDG_Refresh(WWDG_HandleTypeDef *hwwdg)
{
  /* Write to WWDG CR the WWDG Counter value to refresh with */
  WRITE_REG(hwwdg->Instance->CR, (hwwdg->Init.Counter));

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Handle WWDG interrupt request.
  * @note   The Early Wakeup Interrupt (EWI) can be used if specific safety operations
  *         or data logging must be performed before the actual reset is generated.
  *         The EWI interrupt is enabled by calling HAL_WWDG_Init function with
  *         EWIMode set to WWDG_EWI_ENABLE.
  *         When the downcounter reaches the value 0x40, and EWI interrupt is
  *         generated and the corresponding Interrupt Service Routine (ISR) can
  *         be used to trigger specific actions (such as communications or data
  *         logging), before resetting the device.
  * @param  hwwdg  pointer to a WWDG_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDG module.
  * @retval None
  */
void HAL_WWDG_IRQHandler(WWDG_HandleTypeDef *hwwdg)
{
  /* Check if Early Wakeup Interrupt is enable */
  if(__HAL_WWDG_GET_IT_SOURCE(hwwdg, WWDG_IT_EWI) != RESET)
  {
    /* Check if WWDG Early Wakeup Interrupt occurred */
    if(__HAL_WWDG_GET_FLAG(hwwdg, WWDG_FLAG_EWIF) != RESET)
    {
      /* Clear the WWDG Early Wakeup flag */
      __HAL_WWDG_CLEAR_FLAG(hwwdg, WWDG_FLAG_EWIF);

      /* Early Wakeup callback */
      HAL_WWDG_EarlyWakeupCallback(hwwdg);
    }
  }
}

/**
  * @brief  WWDG Early Wakeup callback.
  * @param  hwwdg  pointer to a WWDG_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDG module.
  * @retval None
  */
__weak void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hwwdg);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_WWDG_EarlyWakeupCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_WWDG_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
