/**
  ******************************************************************************
  * @file    stm32f4xx_hal_ltdc_ex.c
  * @author  MCD Application Team
  * @brief   LTDC Extension HAL module driver.
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
/** @defgroup LTDCEx LTDCEx
  * @brief LTDC HAL module driver
  * @{
  */

#ifdef HAL_LTDC_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup LTDCEx_Exported_Functions LTDC Extended Exported Functions
  * @{
  */

/** @defgroup LTDCEx_Exported_Functions_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
                ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the LTDC

@endverbatim
  * @{
  */
#if defined(STM32F469xx) || defined(STM32F479xx)
/**
  * @brief  Retrieve common parameters from DSI Video mode configuration structure
  * @param  hltdc   pointer to a LTDC_HandleTypeDef structure that contains
  *                 the configuration information for the LTDC.
  * @param  VidCfg  pointer to a DSI_VidCfgTypeDef structure that contains
  *                 the DSI video mode configuration parameters
  * @note   The implementation of this function is taking into account the LTDC
  *         polarities inversion as described in the current LTDC specification
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LTDCEx_StructInitFromVideoConfig(LTDC_HandleTypeDef* hltdc, DSI_VidCfgTypeDef *VidCfg)
{
  /* Retrieve signal polarities from DSI */

  /* The following polarity is inverted:
                     LTDC_DEPOLARITY_AL <-> LTDC_DEPOLARITY_AH */

  /* Note 1 : Code in line w/ Current LTDC specification */
  hltdc->Init.DEPolarity = (VidCfg->DEPolarity == DSI_DATA_ENABLE_ACTIVE_HIGH) ? LTDC_DEPOLARITY_AL : LTDC_DEPOLARITY_AH;
  hltdc->Init.VSPolarity = (VidCfg->VSPolarity == DSI_VSYNC_ACTIVE_HIGH) ? LTDC_VSPOLARITY_AH : LTDC_VSPOLARITY_AL;
  hltdc->Init.HSPolarity = (VidCfg->HSPolarity == DSI_HSYNC_ACTIVE_HIGH) ? LTDC_HSPOLARITY_AH : LTDC_HSPOLARITY_AL;

  /* Note 2: Code to be used in case LTDC polarities inversion updated in the specification */
  /* hltdc->Init.DEPolarity = VidCfg->DEPolarity << 29;
     hltdc->Init.VSPolarity = VidCfg->VSPolarity << 29;
     hltdc->Init.HSPolarity = VidCfg->HSPolarity << 29; */

  /* Retrieve vertical timing parameters from DSI */
  hltdc->Init.VerticalSync       = VidCfg->VerticalSyncActive - 1U;
  hltdc->Init.AccumulatedVBP     = VidCfg->VerticalSyncActive + VidCfg->VerticalBackPorch - 1U;
  hltdc->Init.AccumulatedActiveH = VidCfg->VerticalSyncActive + VidCfg->VerticalBackPorch + VidCfg->VerticalActive - 1U;
  hltdc->Init.TotalHeigh         = VidCfg->VerticalSyncActive + VidCfg->VerticalBackPorch + VidCfg->VerticalActive + VidCfg->VerticalFrontPorch - 1U;

  return HAL_OK;
}

/**
  * @brief  Retrieve common parameters from DSI Adapted command mode configuration structure
  * @param  hltdc   pointer to a LTDC_HandleTypeDef structure that contains
  *                 the configuration information for the LTDC.
  * @param  CmdCfg  pointer to a DSI_CmdCfgTypeDef structure that contains
  *                 the DSI command mode configuration parameters
  * @note   The implementation of this function is taking into account the LTDC
  *         polarities inversion as described in the current LTDC specification
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LTDCEx_StructInitFromAdaptedCommandConfig(LTDC_HandleTypeDef* hltdc, DSI_CmdCfgTypeDef *CmdCfg)
{
  /* Retrieve signal polarities from DSI */

  /* The following polarities are inverted:
                     LTDC_DEPOLARITY_AL <-> LTDC_DEPOLARITY_AH
                     LTDC_VSPOLARITY_AL <-> LTDC_VSPOLARITY_AH
                     LTDC_HSPOLARITY_AL <-> LTDC_HSPOLARITY_AH)*/

  /* Note 1 : Code in line w/ Current LTDC specification */
  hltdc->Init.DEPolarity = (CmdCfg->DEPolarity == DSI_DATA_ENABLE_ACTIVE_HIGH) ? LTDC_DEPOLARITY_AL : LTDC_DEPOLARITY_AH;
  hltdc->Init.VSPolarity = (CmdCfg->VSPolarity == DSI_VSYNC_ACTIVE_HIGH) ? LTDC_VSPOLARITY_AL : LTDC_VSPOLARITY_AH;
  hltdc->Init.HSPolarity = (CmdCfg->HSPolarity == DSI_HSYNC_ACTIVE_HIGH) ? LTDC_HSPOLARITY_AL : LTDC_HSPOLARITY_AH;

  /* Note 2: Code to be used in case LTDC polarities inversion updated in the specification */
  /* hltdc->Init.DEPolarity = CmdCfg->DEPolarity << 29;
     hltdc->Init.VSPolarity = CmdCfg->VSPolarity << 29;
     hltdc->Init.HSPolarity = CmdCfg->HSPolarity << 29; */

  return HAL_OK;
}
#endif /* STM32F469xx || STM32F479xx */

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_DCMI_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
