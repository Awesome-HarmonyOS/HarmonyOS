/**
  ******************************************************************************
  * @file    stm32f4xx_hal_dma2d.c
  * @author  MCD Application Team
  * @brief   DMA2D HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the DMA2D peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State and Errors functions
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
      (#) Program the required configuration through the following parameters:
          the transfer mode, the output color mode and the output offset using
          HAL_DMA2D_Init() function.

      (#) Program the required configuration through the following parameters:
          the input color mode, the input color, the input alpha value, the alpha mode,
          and the input offset using HAL_DMA2D_ConfigLayer() function for foreground
          or/and background layer.

     *** Polling mode IO operation ***
     =================================
    [..]
       (#) Configure pdata parameter (explained hereafter), destination and data length
           and enable the transfer using HAL_DMA2D_Start().
       (#) Wait for end of transfer using HAL_DMA2D_PollForTransfer(), at this stage
           user can specify the value of timeout according to his end application.

     *** Interrupt mode IO operation ***
     ===================================
     [..]
       (#) Configure pdata parameter, destination and data length and enable
           the transfer using HAL_DMA2D_Start_IT().
       (#) Use HAL_DMA2D_IRQHandler() called under DMA2D_IRQHandler() interrupt subroutine
       (#) At the end of data transfer HAL_DMA2D_IRQHandler() function is executed and user can
           add his own function by customization of function pointer XferCpltCallback (member
           of DMA2D handle structure).
       (#) In case of error, the HAL_DMA2D_IRQHandler() function will call the callback
           XferErrorCallback.

         -@-   In Register-to-Memory transfer mode, pdata parameter is the register
               color, in Memory-to-memory or Memory-to-Memory with pixel format
               conversion pdata is the source address.

         -@-   Configure the foreground source address, the background source address,
               the destination and data length then Enable the transfer using
               HAL_DMA2D_BlendingStart() in polling mode and HAL_DMA2D_BlendingStart_IT()
               in interrupt mode

         -@-   HAL_DMA2D_BlendingStart() and HAL_DMA2D_BlendingStart_IT() functions
               are used if the memory to memory with blending transfer mode is selected.

      (#) Optionally, configure and enable the CLUT using HAL_DMA2D_CLUTLoad() in polling
          mode or HAL_DMA2D_CLUTLoad_IT() in interrupt mode.

      (#) Optionally, configure the line watermark in using the API HAL_DMA2D_ProgramLineEvent()

      (#) Optionally, configure the dead time value in the AHB clock cycle inserted between two
          consecutive accesses on the AHB master port in using the API HAL_DMA2D_ConfigDeadTime()
          and enable/disable the functionality  with the APIs HAL_DMA2D_EnableDeadTime() or
          HAL_DMA2D_DisableDeadTime().

      (#) The transfer can be suspended, resumed and aborted using the following
          functions: HAL_DMA2D_Suspend(), HAL_DMA2D_Resume(), HAL_DMA2D_Abort().

      (#) The CLUT loading can be suspended, resumed and aborted using the following
          functions: HAL_DMA2D_CLUTLoading_Suspend(), HAL_DMA2D_CLUTLoading_Resume(),
          HAL_DMA2D_CLUTLoading_Abort().

      (#) To control the DMA2D state, use the following function: HAL_DMA2D_GetState().

      (#) To read the DMA2D error code, use the following function: HAL_DMA2D_GetError().

     *** DMA2D HAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in DMA2D HAL driver :

      (+) __HAL_DMA2D_ENABLE: Enable the DMA2D peripheral.
      (+) __HAL_DMA2D_GET_FLAG: Get the DMA2D pending flags.
      (+) __HAL_DMA2D_CLEAR_FLAG: Clear the DMA2D pending flags.
      (+) __HAL_DMA2D_ENABLE_IT: Enable the specified DMA2D interrupts.
      (+) __HAL_DMA2D_DISABLE_IT: Disable the specified DMA2D interrupts.
      (+) __HAL_DMA2D_GET_IT_SOURCE: Check whether the specified DMA2D interrupt is enabled or not

     [..]
      (@) You can refer to the DMA2D HAL driver header file for more useful macros

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

/** @defgroup DMA2D  DMA2D
  * @brief DMA2D HAL module driver
  * @{
  */

#ifdef HAL_DMA2D_MODULE_ENABLED

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)

/* Private types -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup DMA2D_Private_Constants DMA2D Private Constants
  * @{
  */

/** @defgroup DMA2D_TimeOut DMA2D Time Out
  * @{
  */
#define DMA2D_TIMEOUT_ABORT           1000U  /*!<  1s  */
#define DMA2D_TIMEOUT_SUSPEND         1000U  /*!<  1s  */
/**
  * @}
  */

/** @defgroup DMA2D_Shifts DMA2D Shifts
  * @{
  */
#define DMA2D_POSITION_FGPFCCR_CS     (uint32_t)DMA2D_FGPFCCR_CS_Pos    /*!< Required left shift to set foreground CLUT size */
#define DMA2D_POSITION_BGPFCCR_CS     (uint32_t)DMA2D_BGPFCCR_CS_Pos    /*!< Required left shift to set background CLUT size */

#define DMA2D_POSITION_FGPFCCR_CCM    (uint32_t)DMA2D_FGPFCCR_CCM_Pos   /*!< Required left shift to set foreground CLUT color mode */
#define DMA2D_POSITION_BGPFCCR_CCM    (uint32_t)DMA2D_BGPFCCR_CCM_Pos   /*!< Required left shift to set background CLUT color mode */

#define DMA2D_POSITION_AMTCR_DT       (uint32_t)DMA2D_AMTCR_DT_Pos      /*!< Required left shift to set deadtime value */

#define DMA2D_POSITION_FGPFCCR_AM     (uint32_t)DMA2D_FGPFCCR_AM_Pos    /*!< Required left shift to set foreground alpha mode */
#define DMA2D_POSITION_BGPFCCR_AM     (uint32_t)DMA2D_BGPFCCR_AM_Pos    /*!< Required left shift to set background alpha mode */

#define DMA2D_POSITION_FGPFCCR_ALPHA  (uint32_t)DMA2D_FGPFCCR_ALPHA_Pos /*!< Required left shift to set foreground alpha value */
#define DMA2D_POSITION_BGPFCCR_ALPHA  (uint32_t)DMA2D_BGPFCCR_ALPHA_Pos /*!< Required left shift to set background alpha value */

#define DMA2D_POSITION_NLR_PL         (uint32_t)DMA2D_NLR_PL_Pos        /*!< Required left shift to set pixels per lines value */
/**
  * @}
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup DMA2D_Private_Functions_Prototypes
  * @{
  */
static void DMA2D_SetConfig(DMA2D_HandleTypeDef *hdma2d, uint32_t pdata, uint32_t DstAddress, uint32_t Width, uint32_t Height);
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup DMA2D_Exported_Functions DMA2D Exported Functions
  * @{
  */

/** @defgroup DMA2D_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief   Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
                ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the DMA2D
      (+) De-initialize the DMA2D

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the DMA2D according to the specified
  *         parameters in the DMA2D_InitTypeDef and create the associated handle.
  * @param  hdma2d pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_Init(DMA2D_HandleTypeDef *hdma2d)
{
  /* Check the DMA2D peripheral state */
  if(hdma2d == NULL)
  {
     return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_DMA2D_ALL_INSTANCE(hdma2d->Instance));
  assert_param(IS_DMA2D_MODE(hdma2d->Init.Mode));
  assert_param(IS_DMA2D_CMODE(hdma2d->Init.ColorMode));
  assert_param(IS_DMA2D_OFFSET(hdma2d->Init.OutputOffset));

  if(hdma2d->State == HAL_DMA2D_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hdma2d->Lock = HAL_UNLOCKED;
    /* Init the low level hardware */
    HAL_DMA2D_MspInit(hdma2d);
  }

  /* Change DMA2D peripheral state */
  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* DMA2D CR register configuration -------------------------------------------*/
  MODIFY_REG(hdma2d->Instance->CR, DMA2D_CR_MODE, hdma2d->Init.Mode);

  /* DMA2D OPFCCR register configuration ---------------------------------------*/
  MODIFY_REG(hdma2d->Instance->OPFCCR, DMA2D_OPFCCR_CM, hdma2d->Init.ColorMode);

  /* DMA2D OOR register configuration ------------------------------------------*/
  MODIFY_REG(hdma2d->Instance->OOR, DMA2D_OOR_LO, hdma2d->Init.OutputOffset);

  /* Update error code */
  hdma2d->ErrorCode = HAL_DMA2D_ERROR_NONE;

  /* Initialize the DMA2D state*/
  hdma2d->State  = HAL_DMA2D_STATE_READY;

  return HAL_OK;
}

/**
  * @brief  Deinitializes the DMA2D peripheral registers to their default reset
  *         values.
  * @param  hdma2d pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @retval None
  */

HAL_StatusTypeDef HAL_DMA2D_DeInit(DMA2D_HandleTypeDef *hdma2d)
{
  /* Check the DMA2D peripheral state */
  if(hdma2d == NULL)
  {
     return HAL_ERROR;
  }

  /* Before aborting any DMA2D transfer or CLUT loading, check
     first whether or not DMA2D clock is enabled */
  if (__HAL_RCC_DMA2D_IS_CLK_ENABLED())
  {
    /* Abort DMA2D transfer if any */
    if ((hdma2d->Instance->CR & DMA2D_CR_START) == DMA2D_CR_START)
    {
      if (HAL_DMA2D_Abort(hdma2d) != HAL_OK)
      {
        /* Issue when aborting DMA2D transfer */
        return HAL_ERROR;
      }
    }
    else
    {
      /* Abort background CLUT loading if any */
      if ((hdma2d->Instance->BGPFCCR & DMA2D_BGPFCCR_START) == DMA2D_BGPFCCR_START)
      {
        if (HAL_DMA2D_CLUTLoading_Abort(hdma2d, 0U) != HAL_OK)
        {
          /* Issue when aborting background CLUT loading */
          return HAL_ERROR;
        }
      }
      else
      {
        /* Abort foreground CLUT loading if any */
        if ((hdma2d->Instance->FGPFCCR & DMA2D_FGPFCCR_START) == DMA2D_FGPFCCR_START)
        {
          if (HAL_DMA2D_CLUTLoading_Abort(hdma2d, 1U) != HAL_OK)
          {
            /* Issue when aborting foreground CLUT loading */
            return HAL_ERROR;
          }
        }
      }
    }
  }

  /* Carry on with de-initialization of low level hardware */
  HAL_DMA2D_MspDeInit(hdma2d);

  /* Reset DMA2D control registers*/
  hdma2d->Instance->CR = 0U;
  hdma2d->Instance->FGOR = 0U;
  hdma2d->Instance->BGOR = 0U;
  hdma2d->Instance->FGPFCCR = 0U;
  hdma2d->Instance->BGPFCCR = 0U;
  hdma2d->Instance->OPFCCR = 0U;

  /* Update error code */
  hdma2d->ErrorCode = HAL_DMA2D_ERROR_NONE;

  /* Initialize the DMA2D state*/
  hdma2d->State  = HAL_DMA2D_STATE_RESET;

  /* Release Lock */
  __HAL_UNLOCK(hdma2d);

  return HAL_OK;
}

/**
  * @brief  Initializes the DMA2D MSP.
  * @param  hdma2d pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @retval None
  */
__weak void HAL_DMA2D_MspInit(DMA2D_HandleTypeDef* hdma2d)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdma2d);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_DMA2D_MspInit can be implemented in the user file.
   */
}

/**
  * @brief  DeInitializes the DMA2D MSP.
  * @param  hdma2d pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @retval None
  */
__weak void HAL_DMA2D_MspDeInit(DMA2D_HandleTypeDef* hdma2d)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdma2d);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_DMA2D_MspDeInit can be implemented in the user file.
   */
}

/**
  * @}
  */

/** @defgroup DMA2D_Exported_Functions_Group2 IO operation functions
 *  @brief   IO operation functions
 *
@verbatim
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the pdata, destination address and data size then
          start the DMA2D transfer.
      (+) Configure the source for foreground and background, destination address
          and data size then start a MultiBuffer DMA2D transfer.
      (+) Configure the pdata, destination address and data size then
          start the DMA2D transfer with interrupt.
      (+) Configure the source for foreground and background, destination address
          and data size then start a MultiBuffer DMA2D transfer with interrupt.
      (+) Abort DMA2D transfer.
      (+) Suspend DMA2D transfer.
      (+) Resume DMA2D transfer.
      (+) Enable CLUT transfer.
      (+) Configure CLUT loading then start transfer in polling mode.
      (+) Configure CLUT loading then start transfer in interrupt mode.
      (+) Abort DMA2D CLUT loading.
      (+) Suspend DMA2D CLUT loading.
      (+) Resume DMA2D CLUT loading.
      (+) Poll for transfer complete.
      (+) handle DMA2D interrupt request.
      (+) Transfer watermark callback.
      (+) CLUT Transfer Complete callback.

@endverbatim
  * @{
  */

/**
  * @brief  Start the DMA2D Transfer.
  * @param  hdma2d     Pointer to a DMA2D_HandleTypeDef structure that contains
  *                     the configuration information for the DMA2D.
  * @param  pdata      Configure the source memory Buffer address if
  *                     Memory-to-Memory or Memory-to-Memory with pixel format
  *                     conversion mode is selected, or configure
  *                     the color value if Register-to-Memory mode is selected.
  * @param  DstAddress The destination memory Buffer address.
  * @param  Width      The width of data to be transferred from source to destination (expressed in number of pixels per line).
  * @param  Height     The height of data to be transferred from source to destination (expressed in number of lines).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_Start(DMA2D_HandleTypeDef *hdma2d, uint32_t pdata, uint32_t DstAddress, uint32_t Width,  uint32_t Height)
{
  /* Check the parameters */
  assert_param(IS_DMA2D_LINE(Height));
  assert_param(IS_DMA2D_PIXEL(Width));

  /* Process locked */
  __HAL_LOCK(hdma2d);

  /* Change DMA2D peripheral state */
  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* Configure the source, destination address and the data size */
  DMA2D_SetConfig(hdma2d, pdata, DstAddress, Width, Height);

  /* Enable the Peripheral */
  __HAL_DMA2D_ENABLE(hdma2d);

  return HAL_OK;
}

/**
  * @brief  Start the DMA2D Transfer with interrupt enabled.
  * @param  hdma2d     Pointer to a DMA2D_HandleTypeDef structure that contains
  *                     the configuration information for the DMA2D.
  * @param  pdata      Configure the source memory Buffer address if
  *                     the Memory-to-Memory or Memory-to-Memory with pixel format
  *                     conversion mode is selected, or configure
  *                     the color value if Register-to-Memory mode is selected.
  * @param  DstAddress The destination memory Buffer address.
  * @param  Width      The width of data to be transferred from source to destination (expressed in number of pixels per line).
  * @param  Height     The height of data to be transferred from source to destination (expressed in number of lines).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_Start_IT(DMA2D_HandleTypeDef *hdma2d, uint32_t pdata, uint32_t DstAddress, uint32_t Width,  uint32_t Height)
{
  /* Check the parameters */
  assert_param(IS_DMA2D_LINE(Height));
  assert_param(IS_DMA2D_PIXEL(Width));

  /* Process locked */
  __HAL_LOCK(hdma2d);

  /* Change DMA2D peripheral state */
  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* Configure the source, destination address and the data size */
  DMA2D_SetConfig(hdma2d, pdata, DstAddress, Width, Height);

  /* Enable the transfer complete, transfer error and configuration error interrupts */
  __HAL_DMA2D_ENABLE_IT(hdma2d, DMA2D_IT_TC|DMA2D_IT_TE|DMA2D_IT_CE);

  /* Enable the Peripheral */
  __HAL_DMA2D_ENABLE(hdma2d);

  return HAL_OK;
}

/**
  * @brief  Start the multi-source DMA2D Transfer.
  * @param  hdma2d      Pointer to a DMA2D_HandleTypeDef structure that contains
  *                      the configuration information for the DMA2D.
  * @param  SrcAddress1 The source memory Buffer address for the foreground layer.
  * @param  SrcAddress2 The source memory Buffer address for the background layer.
  * @param  DstAddress  The destination memory Buffer address.
  * @param  Width       The width of data to be transferred from source to destination (expressed in number of pixels per line).
  * @param  Height      The height of data to be transferred from source to destination (expressed in number of lines).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_BlendingStart(DMA2D_HandleTypeDef *hdma2d, uint32_t SrcAddress1, uint32_t  SrcAddress2, uint32_t DstAddress, uint32_t Width,  uint32_t Height)
{
  /* Check the parameters */
  assert_param(IS_DMA2D_LINE(Height));
  assert_param(IS_DMA2D_PIXEL(Width));

  /* Process locked */
  __HAL_LOCK(hdma2d);

  /* Change DMA2D peripheral state */
  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* Configure DMA2D Stream source2 address */
  WRITE_REG(hdma2d->Instance->BGMAR, SrcAddress2);

  /* Configure the source, destination address and the data size */
  DMA2D_SetConfig(hdma2d, SrcAddress1, DstAddress, Width, Height);

  /* Enable the Peripheral */
  __HAL_DMA2D_ENABLE(hdma2d);

  return HAL_OK;
}

/**
  * @brief  Start the multi-source DMA2D Transfer with interrupt enabled.
  * @param  hdma2d     Pointer to a DMA2D_HandleTypeDef structure that contains
  *                     the configuration information for the DMA2D.
  * @param  SrcAddress1 The source memory Buffer address for the foreground layer.
  * @param  SrcAddress2 The source memory Buffer address for the background layer.
  * @param  DstAddress  The destination memory Buffer address.
  * @param  Width       The width of data to be transferred from source to destination (expressed in number of pixels per line).
  * @param  Height      The height of data to be transferred from source to destination (expressed in number of lines).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_BlendingStart_IT(DMA2D_HandleTypeDef *hdma2d, uint32_t SrcAddress1, uint32_t  SrcAddress2, uint32_t DstAddress, uint32_t Width,  uint32_t Height)
{
  /* Check the parameters */
  assert_param(IS_DMA2D_LINE(Height));
  assert_param(IS_DMA2D_PIXEL(Width));

  /* Process locked */
  __HAL_LOCK(hdma2d);

  /* Change DMA2D peripheral state */
  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* Configure DMA2D Stream source2 address */
  WRITE_REG(hdma2d->Instance->BGMAR, SrcAddress2);

  /* Configure the source, destination address and the data size */
  DMA2D_SetConfig(hdma2d, SrcAddress1, DstAddress, Width, Height);

  /* Enable the transfer complete, transfer error and configuration error interrupts */
  __HAL_DMA2D_ENABLE_IT(hdma2d, DMA2D_IT_TC|DMA2D_IT_TE|DMA2D_IT_CE);

  /* Enable the Peripheral */
  __HAL_DMA2D_ENABLE(hdma2d);

  return HAL_OK;
}

/**
  * @brief  Abort the DMA2D Transfer.
  * @param  hdma2d  pointer to a DMA2D_HandleTypeDef structure that contains
  *                  the configuration information for the DMA2D.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_Abort(DMA2D_HandleTypeDef *hdma2d)
{
  uint32_t tickstart = 0U;

  /* Abort the DMA2D transfer */
  /* START bit is reset to make sure not to set it again, in the event the HW clears it
     between the register read and the register write by the CPU (writing 0 has no
     effect on START bitvalue) */
   MODIFY_REG(hdma2d->Instance->CR, DMA2D_CR_ABORT|DMA2D_CR_START, DMA2D_CR_ABORT);

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Check if the DMA2D is effectively disabled */
  while((hdma2d->Instance->CR & DMA2D_CR_START) != RESET)
  {
    if((HAL_GetTick() - tickstart ) > DMA2D_TIMEOUT_ABORT)
    {
      /* Update error code */
      hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TIMEOUT;

      /* Change the DMA2D state */
      hdma2d->State = HAL_DMA2D_STATE_TIMEOUT;

      /* Process Unlocked */
      __HAL_UNLOCK(hdma2d);

      return HAL_TIMEOUT;
    }
  }

  /* Disable the Transfer Complete, Transfer Error and Configuration Error interrupts */
  __HAL_DMA2D_DISABLE_IT(hdma2d, DMA2D_IT_TC|DMA2D_IT_TE|DMA2D_IT_CE);

  /* Change the DMA2D state*/
  hdma2d->State = HAL_DMA2D_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hdma2d);

  return HAL_OK;
}

/**
  * @brief  Suspend the DMA2D Transfer.
  * @param  hdma2d pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_Suspend(DMA2D_HandleTypeDef *hdma2d)
{
  uint32_t tickstart = 0U;

  /* Suspend the DMA2D transfer */
  /* START bit is reset to make sure not to set it again, in the event the HW clears it
     between the register read and the register write by the CPU (writing 0 has no
     effect on START bitvalue) */
  MODIFY_REG(hdma2d->Instance->CR, DMA2D_CR_SUSP|DMA2D_CR_START, DMA2D_CR_SUSP);

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Check if the DMA2D is effectively suspended */
  while (((hdma2d->Instance->CR & DMA2D_CR_SUSP) != DMA2D_CR_SUSP) \
    && ((hdma2d->Instance->CR & DMA2D_CR_START) == DMA2D_CR_START))
  {
    if((HAL_GetTick() - tickstart ) > DMA2D_TIMEOUT_SUSPEND)
    {
      /* Update error code */
      hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TIMEOUT;

      /* Change the DMA2D state */
      hdma2d->State = HAL_DMA2D_STATE_TIMEOUT;

      return HAL_TIMEOUT;
    }
  }

   /* Check whether or not a transfer is actually suspended and change the DMA2D state accordingly */
  if ((hdma2d->Instance->CR & DMA2D_CR_START) != RESET)
  {
    hdma2d->State = HAL_DMA2D_STATE_SUSPEND;
  }
  else
  {
    /* Make sure SUSP bit is cleared since it is meaningless
       when no tranfer is on-going */
    CLEAR_BIT(hdma2d->Instance->CR, DMA2D_CR_SUSP);
  }

  return HAL_OK;
}

/**
  * @brief  Resume the DMA2D Transfer.
  * @param  hdma2d pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_Resume(DMA2D_HandleTypeDef *hdma2d)
{
  /* Check the SUSP and START bits */
  if((hdma2d->Instance->CR & (DMA2D_CR_SUSP | DMA2D_CR_START)) == (DMA2D_CR_SUSP | DMA2D_CR_START))
  {
    /* Ongoing transfer is suspended: change the DMA2D state before resuming */
    hdma2d->State = HAL_DMA2D_STATE_BUSY;
  }

  /* Resume the DMA2D transfer */
  /* START bit is reset to make sure not to set it again, in the event the HW clears it
     between the register read and the register write by the CPU (writing 0 has no
     effect on START bitvalue) */
  CLEAR_BIT(hdma2d->Instance->CR, (DMA2D_CR_SUSP|DMA2D_CR_START));

  return HAL_OK;
}

/**
  * @brief  Enable the DMA2D CLUT Transfer.
  * @param  hdma2d   Pointer to a DMA2D_HandleTypeDef structure that contains
  *                   the configuration information for the DMA2D.
  * @param  LayerIdx DMA2D Layer index.
  *                   This parameter can be one of the following values:
  *                   0(background) / 1(foreground)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_EnableCLUT(DMA2D_HandleTypeDef *hdma2d, uint32_t LayerIdx)
{
  /* Check the parameters */
  assert_param(IS_DMA2D_LAYER(LayerIdx));

  /* Process locked */
  __HAL_LOCK(hdma2d);

  /* Change DMA2D peripheral state */
  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  if(LayerIdx == 0U)
  {
    /* Enable the background CLUT loading */
    SET_BIT(hdma2d->Instance->BGPFCCR, DMA2D_BGPFCCR_START);
  }
  else
  {
    /* Enable the foreground CLUT loading */
    SET_BIT(hdma2d->Instance->FGPFCCR, DMA2D_FGPFCCR_START);
  }

  return HAL_OK;
}

/**
  * @brief  Start DMA2D CLUT Loading.
  * @param  hdma2d   Pointer to a DMA2D_HandleTypeDef structure that contains
  *                   the configuration information for the DMA2D.
  * @param  CLUTCfg  Pointer to a DMA2D_CLUTCfgTypeDef structure that contains
  *                   the configuration information for the color look up table.
  * @param  LayerIdx DMA2D Layer index.
  *                   This parameter can be one of the following values:
  *                   0(background) / 1(foreground)
  * @note Invoking this API is similar to calling HAL_DMA2D_ConfigCLUT() then HAL_DMA2D_EnableCLUT().
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_CLUTLoad(DMA2D_HandleTypeDef *hdma2d, DMA2D_CLUTCfgTypeDef CLUTCfg, uint32_t LayerIdx)
{
  /* Check the parameters */
  assert_param(IS_DMA2D_LAYER(LayerIdx));
  assert_param(IS_DMA2D_CLUT_CM(CLUTCfg.CLUTColorMode));
  assert_param(IS_DMA2D_CLUT_SIZE(CLUTCfg.Size));

  /* Process locked */
  __HAL_LOCK(hdma2d);

  /* Change DMA2D peripheral state */
  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* Configure the CLUT of the background DMA2D layer */
  if(LayerIdx == 0U)
  {
    /* Write background CLUT memory address */
    WRITE_REG(hdma2d->Instance->BGCMAR, (uint32_t)CLUTCfg.pCLUT);

    /* Write background CLUT size and CLUT color mode */
    MODIFY_REG(hdma2d->Instance->BGPFCCR, (DMA2D_BGPFCCR_CS | DMA2D_BGPFCCR_CCM),
            ((CLUTCfg.Size << DMA2D_POSITION_BGPFCCR_CS) | (CLUTCfg.CLUTColorMode << DMA2D_POSITION_BGPFCCR_CCM)));

    /* Enable the CLUT loading for the background */
    SET_BIT(hdma2d->Instance->BGPFCCR, DMA2D_BGPFCCR_START);
  }
  /* Configure the CLUT of the foreground DMA2D layer */
  else
  {
    /* Write foreground CLUT memory address */
    WRITE_REG(hdma2d->Instance->FGCMAR, (uint32_t)CLUTCfg.pCLUT);

    /* Write foreground CLUT size and CLUT color mode */
    MODIFY_REG(hdma2d->Instance->FGPFCCR, (DMA2D_FGPFCCR_CS | DMA2D_FGPFCCR_CCM),
              ((CLUTCfg.Size << DMA2D_POSITION_BGPFCCR_CS) | (CLUTCfg.CLUTColorMode << DMA2D_POSITION_FGPFCCR_CCM)));

 /* Enable the CLUT loading for the foreground */
    SET_BIT(hdma2d->Instance->FGPFCCR, DMA2D_FGPFCCR_START);
  }

  return HAL_OK;
}

/**
  * @brief  Start DMA2D CLUT Loading with interrupt enabled.
  * @param  hdma2d   Pointer to a DMA2D_HandleTypeDef structure that contains
  *                   the configuration information for the DMA2D.
  * @param  CLUTCfg  Pointer to a DMA2D_CLUTCfgTypeDef structure that contains
  *                   the configuration information for the color look up table.
  * @param  LayerIdx DMA2D Layer index.
  *                   This parameter can be one of the following values:
  *                   0(background) / 1(foreground)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_CLUTLoad_IT(DMA2D_HandleTypeDef *hdma2d, DMA2D_CLUTCfgTypeDef CLUTCfg, uint32_t LayerIdx)
{
  /* Check the parameters */
  assert_param(IS_DMA2D_LAYER(LayerIdx));
  assert_param(IS_DMA2D_CLUT_CM(CLUTCfg.CLUTColorMode));
  assert_param(IS_DMA2D_CLUT_SIZE(CLUTCfg.Size));

  /* Process locked */
  __HAL_LOCK(hdma2d);

  /* Change DMA2D peripheral state */
  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* Configure the CLUT of the background DMA2D layer */
  if(LayerIdx == 0U)
  {
    /* Write background CLUT memory address */
    WRITE_REG(hdma2d->Instance->BGCMAR, (uint32_t)CLUTCfg.pCLUT);

    /* Write background CLUT size and CLUT color mode */
    MODIFY_REG(hdma2d->Instance->BGPFCCR, (DMA2D_BGPFCCR_CS | DMA2D_BGPFCCR_CCM),
            ((CLUTCfg.Size << DMA2D_POSITION_BGPFCCR_CS) | (CLUTCfg.CLUTColorMode << DMA2D_POSITION_BGPFCCR_CCM)));

    /* Enable the CLUT Transfer Complete, transfer Error, configuration Error and CLUT Access Error interrupts */
    __HAL_DMA2D_ENABLE_IT(hdma2d, DMA2D_IT_CTC | DMA2D_IT_TE | DMA2D_IT_CE |DMA2D_IT_CAE);

    /* Enable the CLUT loading for the background */
    SET_BIT(hdma2d->Instance->BGPFCCR, DMA2D_BGPFCCR_START);
  }
  /* Configure the CLUT of the foreground DMA2D layer */
  else
  {
    /* Write foreground CLUT memory address */
    WRITE_REG(hdma2d->Instance->FGCMAR, (uint32_t)CLUTCfg.pCLUT);

    /* Write foreground CLUT size and CLUT color mode */
    MODIFY_REG(hdma2d->Instance->FGPFCCR, (DMA2D_FGPFCCR_CS | DMA2D_FGPFCCR_CCM),
              ((CLUTCfg.Size << DMA2D_POSITION_BGPFCCR_CS) | (CLUTCfg.CLUTColorMode << DMA2D_POSITION_FGPFCCR_CCM)));

    /* Enable the CLUT Transfer Complete, transfer Error, configuration Error and CLUT Access Error interrupts */
    __HAL_DMA2D_ENABLE_IT(hdma2d, DMA2D_IT_CTC | DMA2D_IT_TE | DMA2D_IT_CE |DMA2D_IT_CAE);

    /* Enable the CLUT loading for the foreground */
    SET_BIT(hdma2d->Instance->FGPFCCR, DMA2D_FGPFCCR_START);
  }

  return HAL_OK;
}

/**
  * @brief  Abort the DMA2D CLUT loading.
  * @param  hdma2d  Pointer to a DMA2D_HandleTypeDef structure that contains
  *                  the configuration information for the DMA2D.
  * @param  LayerIdx DMA2D Layer index.
  *                   This parameter can be one of the following values:
  *                   0(background) / 1(foreground)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_CLUTLoading_Abort(DMA2D_HandleTypeDef *hdma2d, uint32_t LayerIdx)
{
  uint32_t tickstart  = 0U;
  __IO uint32_t * reg =  &(hdma2d->Instance->BGPFCCR); /* by default, point at background register */

  /* Abort the CLUT loading */
  SET_BIT(hdma2d->Instance->CR, DMA2D_CR_ABORT);

  /* If foreground CLUT loading is considered, update local variables */
  if(LayerIdx == 1)
  {
    reg  = &(hdma2d->Instance->FGPFCCR);
  }

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Check if the CLUT loading is aborted */
  while((*reg & DMA2D_BGPFCCR_START) != RESET)
  {
    if((HAL_GetTick() - tickstart ) > DMA2D_TIMEOUT_ABORT)
    {
      /* Update error code */
      hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TIMEOUT;

      /* Change the DMA2D state */
      hdma2d->State = HAL_DMA2D_STATE_TIMEOUT;

      /* Process Unlocked */
      __HAL_UNLOCK(hdma2d);

      return HAL_TIMEOUT;
    }
  }

  /* Disable the CLUT Transfer Complete, Transfer Error, Configuration Error and CLUT Access Error interrupts */
  __HAL_DMA2D_DISABLE_IT(hdma2d, DMA2D_IT_CTC | DMA2D_IT_TE | DMA2D_IT_CE |DMA2D_IT_CAE);

  /* Change the DMA2D state*/
  hdma2d->State = HAL_DMA2D_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hdma2d);

  return HAL_OK;
}

/**
  * @brief  Suspend the DMA2D CLUT loading.
  * @param  hdma2d Pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @param  LayerIdx DMA2D Layer index.
  *                   This parameter can be one of the following values:
  *                   0(background) / 1(foreground)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_CLUTLoading_Suspend(DMA2D_HandleTypeDef *hdma2d, uint32_t LayerIdx)
{
  uint32_t tickstart = 0U;
  __IO uint32_t * reg =  &(hdma2d->Instance->BGPFCCR); /* by default, point at background register */

  /* Suspend the CLUT loading */
  SET_BIT(hdma2d->Instance->CR, DMA2D_CR_SUSP);

  /* If foreground CLUT loading is considered, update local variables */
  if(LayerIdx == 1U)
  {
    reg  = &(hdma2d->Instance->FGPFCCR);
  }

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Check if the CLUT loading is suspended */
  while (((hdma2d->Instance->CR & DMA2D_CR_SUSP) != DMA2D_CR_SUSP) \
    && ((*reg & DMA2D_BGPFCCR_START) == DMA2D_BGPFCCR_START))
  {
    if((HAL_GetTick() - tickstart ) > DMA2D_TIMEOUT_SUSPEND)
    {
      /* Update error code */
      hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TIMEOUT;

      /* Change the DMA2D state */
      hdma2d->State = HAL_DMA2D_STATE_TIMEOUT;

      return HAL_TIMEOUT;
    }
  }

   /* Check whether or not a transfer is actually suspended and change the DMA2D state accordingly */
  if ((*reg & DMA2D_BGPFCCR_START) != RESET)
  {
    hdma2d->State = HAL_DMA2D_STATE_SUSPEND;
  }
  else
  {
    /* Make sure SUSP bit is cleared since it is meaningless
       when no tranfer is on-going */
    CLEAR_BIT(hdma2d->Instance->CR, DMA2D_CR_SUSP);
  }

  return HAL_OK;
}

/**
  * @brief  Resume the DMA2D CLUT loading.
  * @param  hdma2d pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @param  LayerIdx DMA2D Layer index.
  *                   This parameter can be one of the following values:
  *                   0(background) / 1(foreground)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_CLUTLoading_Resume(DMA2D_HandleTypeDef *hdma2d, uint32_t LayerIdx)
{
  /* Check the SUSP and START bits for background or foreground CLUT loading */
  if(LayerIdx == 0U)
  {
    /* Background CLUT loading suspension check */
    if (((hdma2d->Instance->CR & DMA2D_CR_SUSP) == DMA2D_CR_SUSP)
      && ((hdma2d->Instance->BGPFCCR & DMA2D_BGPFCCR_START) == DMA2D_BGPFCCR_START))
    {
      /* Ongoing CLUT loading is suspended: change the DMA2D state before resuming */
      hdma2d->State = HAL_DMA2D_STATE_BUSY;
    }
  }
  else
  {
    /* Foreground CLUT loading suspension check */
    if (((hdma2d->Instance->CR & DMA2D_CR_SUSP) == DMA2D_CR_SUSP)
      && ((hdma2d->Instance->FGPFCCR & DMA2D_FGPFCCR_START) == DMA2D_FGPFCCR_START))
    {
      /* Ongoing CLUT loading is suspended: change the DMA2D state before resuming */
      hdma2d->State = HAL_DMA2D_STATE_BUSY;
    }
  }

  /* Resume the CLUT loading */
  CLEAR_BIT(hdma2d->Instance->CR, DMA2D_CR_SUSP);

  return HAL_OK;
}

/**
  * @brief  Polling for transfer complete or CLUT loading.
  * @param  hdma2d Pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_PollForTransfer(DMA2D_HandleTypeDef *hdma2d, uint32_t Timeout)
{
  uint32_t tickstart = 0U;
  __IO uint32_t isrflags = 0x0U;

  /* Polling for DMA2D transfer */
  if((hdma2d->Instance->CR & DMA2D_CR_START) != RESET)
  {
   /* Get tick */
   tickstart = HAL_GetTick();

    while(__HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_TC) == RESET)
    {
      isrflags = READ_REG(hdma2d->Instance->ISR);
      if ((isrflags & (DMA2D_FLAG_CE|DMA2D_FLAG_TE)) != RESET)
      {
        if ((isrflags & DMA2D_FLAG_CE) != RESET)
        {
          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_CE;
        }
        if ((isrflags & DMA2D_FLAG_TE) != RESET)
        {
          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TE;
        }
        /* Clear the transfer and configuration error flags */
        __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_CE | DMA2D_FLAG_TE);

        /* Change DMA2D state */
        hdma2d->State = HAL_DMA2D_STATE_ERROR;

        /* Process unlocked */
        __HAL_UNLOCK(hdma2d);

        return HAL_ERROR;
      }
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Update error code */
          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TIMEOUT;

          /* Change the DMA2D state */
          hdma2d->State = HAL_DMA2D_STATE_TIMEOUT;

          /* Process unlocked */
          __HAL_UNLOCK(hdma2d);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  /* Polling for CLUT loading (foreground or background) */
  if (((hdma2d->Instance->FGPFCCR & DMA2D_FGPFCCR_START) != RESET)  ||
      ((hdma2d->Instance->BGPFCCR & DMA2D_BGPFCCR_START) != RESET))
  {
    /* Get tick */
    tickstart = HAL_GetTick();

    while(__HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_CTC) == RESET)
    {
      isrflags = READ_REG(hdma2d->Instance->ISR);
      if ((isrflags & (DMA2D_FLAG_CAE|DMA2D_FLAG_CE|DMA2D_FLAG_TE)) != RESET)
      {
        if ((isrflags & DMA2D_FLAG_CAE) != RESET)
        {
          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_CAE;
        }
        if ((isrflags & DMA2D_FLAG_CE) != RESET)
        {
          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_CE;
        }
        if ((isrflags & DMA2D_FLAG_TE) != RESET)
        {
          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TE;
        }
        /* Clear the CLUT Access Error, Configuration Error and Transfer Error flags */
        __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_CAE | DMA2D_FLAG_CE | DMA2D_FLAG_TE);

        /* Change DMA2D state */
        hdma2d->State= HAL_DMA2D_STATE_ERROR;

        /* Process unlocked */
        __HAL_UNLOCK(hdma2d);

        return HAL_ERROR;
      }
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Update error code */
          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TIMEOUT;

          /* Change the DMA2D state */
          hdma2d->State= HAL_DMA2D_STATE_TIMEOUT;

          /* Process unlocked */
          __HAL_UNLOCK(hdma2d);

          return HAL_TIMEOUT;
        }
      }
    }
  }

  /* Clear the transfer complete and CLUT loading flags */
  __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_TC|DMA2D_FLAG_CTC);

  /* Change DMA2D state */
  hdma2d->State = HAL_DMA2D_STATE_READY;

  /* Process unlocked */
  __HAL_UNLOCK(hdma2d);

  return HAL_OK;
}
/**
  * @brief  Handle DMA2D interrupt request.
  * @param  hdma2d Pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @retval HAL status
  */
void HAL_DMA2D_IRQHandler(DMA2D_HandleTypeDef *hdma2d)
{
  uint32_t isrflags = READ_REG(hdma2d->Instance->ISR);
  uint32_t crflags = READ_REG(hdma2d->Instance->CR);

  /* Transfer Error Interrupt management ***************************************/
  if ((isrflags & DMA2D_FLAG_TE) != RESET)
  {
    if ((crflags & DMA2D_IT_TE) != RESET)
    {
      /* Disable the transfer Error interrupt */
      __HAL_DMA2D_DISABLE_IT(hdma2d, DMA2D_IT_TE);

      /* Update error code */
      hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TE;

      /* Clear the transfer error flag */
      __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_TE);

      /* Change DMA2D state */
      hdma2d->State = HAL_DMA2D_STATE_ERROR;

      /* Process Unlocked */
      __HAL_UNLOCK(hdma2d);

      if(hdma2d->XferErrorCallback != NULL)
      {
        /* Transfer error Callback */
        hdma2d->XferErrorCallback(hdma2d);
      }
    }
  }
  /* Configuration Error Interrupt management **********************************/
  if ((isrflags & DMA2D_FLAG_CE) != RESET)
  {
    if ((crflags & DMA2D_IT_CE) != RESET)
    {
      /* Disable the Configuration Error interrupt */
      __HAL_DMA2D_DISABLE_IT(hdma2d, DMA2D_IT_CE);

      /* Clear the Configuration error flag */
      __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_CE);

      /* Update error code */
      hdma2d->ErrorCode |= HAL_DMA2D_ERROR_CE;

      /* Change DMA2D state */
      hdma2d->State = HAL_DMA2D_STATE_ERROR;

      /* Process Unlocked */
      __HAL_UNLOCK(hdma2d);

      if(hdma2d->XferErrorCallback != NULL)
      {
        /* Transfer error Callback */
        hdma2d->XferErrorCallback(hdma2d);
      }
    }
  }
  /* CLUT access Error Interrupt management ***********************************/
  if ((isrflags & DMA2D_FLAG_CAE) != RESET)
  {
    if ((crflags & DMA2D_IT_CAE) != RESET)
    {
      /* Disable the CLUT access error interrupt */
      __HAL_DMA2D_DISABLE_IT(hdma2d, DMA2D_IT_CAE);

      /* Clear the CLUT access error flag */
      __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_CAE);

      /* Update error code */
      hdma2d->ErrorCode |= HAL_DMA2D_ERROR_CAE;

      /* Change DMA2D state */
      hdma2d->State = HAL_DMA2D_STATE_ERROR;

      /* Process Unlocked */
      __HAL_UNLOCK(hdma2d);

      if(hdma2d->XferErrorCallback != NULL)
      {
        /* Transfer error Callback */
        hdma2d->XferErrorCallback(hdma2d);
      }
    }
  }
  /* Transfer watermark Interrupt management **********************************/
  if ((isrflags & DMA2D_FLAG_TW) != RESET)
  {
    if ((crflags & DMA2D_IT_TW) != RESET)
    {
      /* Disable the transfer watermark interrupt */
      __HAL_DMA2D_DISABLE_IT(hdma2d, DMA2D_IT_TW);

      /* Clear the transfer watermark flag */
      __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_TW);

      /* Transfer watermark Callback */
      HAL_DMA2D_LineEventCallback(hdma2d);
    }
  }
  /* Transfer Complete Interrupt management ************************************/
  if ((isrflags & DMA2D_FLAG_TC) != RESET)
  {
    if ((crflags & DMA2D_IT_TC) != RESET)
    {
      /* Disable the transfer complete interrupt */
      __HAL_DMA2D_DISABLE_IT(hdma2d, DMA2D_IT_TC);

      /* Clear the transfer complete flag */
      __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_TC);

      /* Update error code */
      hdma2d->ErrorCode |= HAL_DMA2D_ERROR_NONE;

      /* Change DMA2D state */
      hdma2d->State = HAL_DMA2D_STATE_READY;

      /* Process Unlocked */
      __HAL_UNLOCK(hdma2d);

      if(hdma2d->XferCpltCallback != NULL)
      {
        /* Transfer complete Callback */
        hdma2d->XferCpltCallback(hdma2d);
      }
    }
  }
  /* CLUT Transfer Complete Interrupt management ******************************/
  if ((isrflags & DMA2D_FLAG_CTC) != RESET)
  {
    if ((crflags & DMA2D_IT_CTC) != RESET)
    {
      /* Disable the CLUT transfer complete interrupt */
      __HAL_DMA2D_DISABLE_IT(hdma2d, DMA2D_IT_CTC);

      /* Clear the CLUT transfer complete flag */
      __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_CTC);

      /* Update error code */
      hdma2d->ErrorCode |= HAL_DMA2D_ERROR_NONE;

      /* Change DMA2D state */
      hdma2d->State = HAL_DMA2D_STATE_READY;

      /* Process Unlocked */
      __HAL_UNLOCK(hdma2d);

      /* CLUT Transfer complete Callback */
      HAL_DMA2D_CLUTLoadingCpltCallback(hdma2d);
    }
  }
}

/**
  * @brief  Transfer watermark callback.
  * @param  hdma2d pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @retval None
  */
__weak void HAL_DMA2D_LineEventCallback(DMA2D_HandleTypeDef *hdma2d)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdma2d);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_DMA2D_LineEventCallback can be implemented in the user file.
   */
}

/**
  * @brief  CLUT Transfer Complete callback.
  * @param  hdma2d pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @retval None
  */
__weak void HAL_DMA2D_CLUTLoadingCpltCallback(DMA2D_HandleTypeDef *hdma2d)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdma2d);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_DMA2D_CLUTLoadingCpltCallback can be implemented in the user file.
   */
}

/**
  * @}
  */

/** @defgroup DMA2D_Exported_Functions_Group3 Peripheral Control functions
 *  @brief    Peripheral Control functions
 *
@verbatim
 ===============================================================================
                    ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the DMA2D foreground or background layer parameters.
      (+) Configure the DMA2D CLUT transfer.
      (+) Configure the line watermark
      (+) Configure the dead time value.
      (+) Enable or disable the dead time value functionality.

@endverbatim
  * @{
  */

/**
  * @brief  Configure the DMA2D Layer according to the specified
  *         parameters in the DMA2D_InitTypeDef and create the associated handle.
  * @param  hdma2d pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @param  LayerIdx DMA2D Layer index.
  *                   This parameter can be one of the following values:
  *                   0(background) / 1(foreground)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_ConfigLayer(DMA2D_HandleTypeDef *hdma2d, uint32_t LayerIdx)
{
  DMA2D_LayerCfgTypeDef *pLayerCfg = &hdma2d->LayerCfg[LayerIdx];
  uint32_t regValue = 0U;

  /* Check the parameters */
  assert_param(IS_DMA2D_LAYER(LayerIdx));
  assert_param(IS_DMA2D_OFFSET(pLayerCfg->InputOffset));
  if(hdma2d->Init.Mode != DMA2D_R2M)
  {
    assert_param(IS_DMA2D_INPUT_COLOR_MODE(pLayerCfg->InputColorMode));
    if(hdma2d->Init.Mode != DMA2D_M2M)
    {
      assert_param(IS_DMA2D_ALPHA_MODE(pLayerCfg->AlphaMode));
    }
  }

  /* Process locked */
  __HAL_LOCK(hdma2d);

  /* Change DMA2D peripheral state */
  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* DMA2D BGPFCR register configuration -----------------------------------*/
  /* Prepare the value to be written to the BGPFCCR register */

  if ((pLayerCfg->InputColorMode == DMA2D_INPUT_A4) || (pLayerCfg->InputColorMode == DMA2D_INPUT_A8))
  {
    regValue = pLayerCfg->InputColorMode | (pLayerCfg->AlphaMode << DMA2D_POSITION_BGPFCCR_AM) | (pLayerCfg->InputAlpha & DMA2D_BGPFCCR_ALPHA);
  }
  else
  {
    regValue =  pLayerCfg->InputColorMode | (pLayerCfg->AlphaMode << DMA2D_POSITION_BGPFCCR_AM) | (pLayerCfg->InputAlpha << DMA2D_POSITION_BGPFCCR_ALPHA);
  }

  /* Configure the background DMA2D layer */
  if(LayerIdx == 0)
  {
    /* Write DMA2D BGPFCCR register */
    MODIFY_REG(hdma2d->Instance->BGPFCCR, (DMA2D_BGPFCCR_CM | DMA2D_BGPFCCR_AM | DMA2D_BGPFCCR_ALPHA), regValue);

    /* DMA2D BGOR register configuration -------------------------------------*/
    WRITE_REG(hdma2d->Instance->BGOR, pLayerCfg->InputOffset);

    /* DMA2D BGCOLR register configuration -------------------------------------*/
    if ((pLayerCfg->InputColorMode == DMA2D_INPUT_A4) || (pLayerCfg->InputColorMode == DMA2D_INPUT_A8))
    {
      WRITE_REG(hdma2d->Instance->BGCOLR, pLayerCfg->InputAlpha & (DMA2D_BGCOLR_BLUE|DMA2D_BGCOLR_GREEN|DMA2D_BGCOLR_RED));
    }
  }
  /* Configure the foreground DMA2D layer */
  else
  {
     /* Write DMA2D FGPFCCR register */
    MODIFY_REG(hdma2d->Instance->FGPFCCR, (DMA2D_BGPFCCR_CM | DMA2D_BGPFCCR_AM | DMA2D_BGPFCCR_ALPHA), regValue);

    /* DMA2D FGOR register configuration -------------------------------------*/
    WRITE_REG(hdma2d->Instance->FGOR, pLayerCfg->InputOffset);

    /* DMA2D FGCOLR register configuration -------------------------------------*/
    if ((pLayerCfg->InputColorMode == DMA2D_INPUT_A4) || (pLayerCfg->InputColorMode == DMA2D_INPUT_A8))
    {
      WRITE_REG(hdma2d->Instance->FGCOLR, pLayerCfg->InputAlpha & (DMA2D_FGCOLR_BLUE|DMA2D_FGCOLR_GREEN|DMA2D_FGCOLR_RED));
    }
  }
  /* Initialize the DMA2D state*/
  hdma2d->State = HAL_DMA2D_STATE_READY;

  /* Process unlocked */
  __HAL_UNLOCK(hdma2d);

  return HAL_OK;
}

/**
  * @brief  Configure the DMA2D CLUT Transfer.
  * @param  hdma2d   Pointer to a DMA2D_HandleTypeDef structure that contains
  *                   the configuration information for the DMA2D.
  * @param  CLUTCfg  Pointer to a DMA2D_CLUTCfgTypeDef structure that contains
  *                   the configuration information for the color look up table.
  * @param  LayerIdx DMA2D Layer index.
  *                   This parameter can be one of the following values:
  *                   0(background) / 1(foreground)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_ConfigCLUT(DMA2D_HandleTypeDef *hdma2d, DMA2D_CLUTCfgTypeDef CLUTCfg, uint32_t LayerIdx)
{
  /* Check the parameters */
  assert_param(IS_DMA2D_LAYER(LayerIdx));
  assert_param(IS_DMA2D_CLUT_CM(CLUTCfg.CLUTColorMode));
  assert_param(IS_DMA2D_CLUT_SIZE(CLUTCfg.Size));

  /* Process locked */
  __HAL_LOCK(hdma2d);

  /* Change DMA2D peripheral state */
  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* Configure the CLUT of the background DMA2D layer */
  if(LayerIdx == 0U)
  {
    /* Write background CLUT memory address */
    WRITE_REG(hdma2d->Instance->BGCMAR, (uint32_t)CLUTCfg.pCLUT);

    /* Write background CLUT size and CLUT color mode */
    MODIFY_REG(hdma2d->Instance->BGPFCCR, (DMA2D_BGPFCCR_CS | DMA2D_BGPFCCR_CCM),
              ((CLUTCfg.Size << DMA2D_POSITION_BGPFCCR_CS) | (CLUTCfg.CLUTColorMode << DMA2D_POSITION_BGPFCCR_CCM)));
 }
 /* Configure the CLUT of the foreground DMA2D layer */
 else
 {
   /* Write foreground CLUT memory address */
    WRITE_REG(hdma2d->Instance->FGCMAR, (uint32_t)CLUTCfg.pCLUT);

    /* Write foreground CLUT size and CLUT color mode */
    MODIFY_REG(hdma2d->Instance->FGPFCCR, (DMA2D_FGPFCCR_CS | DMA2D_FGPFCCR_CCM),
              ((CLUTCfg.Size << DMA2D_POSITION_BGPFCCR_CS) | (CLUTCfg.CLUTColorMode << DMA2D_POSITION_FGPFCCR_CCM)));
  }

  /* Set the DMA2D state to Ready */
  hdma2d->State = HAL_DMA2D_STATE_READY;

  /* Process unlocked */
  __HAL_UNLOCK(hdma2d);

  return HAL_OK;
}

/**
  * @brief  Configure the line watermark.
  * @param  hdma2d Pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @param  Line   Line Watermark configuration (maximum 16-bit long value expected).
  * @note   HAL_DMA2D_ProgramLineEvent() API enables the transfer watermark interrupt.
  * @note   The transfer watermark interrupt is disabled once it has occurred.
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_DMA2D_ProgramLineEvent(DMA2D_HandleTypeDef *hdma2d, uint32_t Line)
{
  /* Check the parameters */
  assert_param(IS_DMA2D_LINEWATERMARK(Line));

  if (Line > DMA2D_LWR_LW)
  {
    return HAL_ERROR;
  }
  else
  {
    /* Process locked */
    __HAL_LOCK(hdma2d);

    /* Change DMA2D peripheral state */
    hdma2d->State = HAL_DMA2D_STATE_BUSY;

    /* Sets the Line watermark configuration */
    WRITE_REG(hdma2d->Instance->LWR, Line);

    /* Enable the Line interrupt */
    __HAL_DMA2D_ENABLE_IT(hdma2d, DMA2D_IT_TW);

    /* Initialize the DMA2D state */
    hdma2d->State = HAL_DMA2D_STATE_READY;

    /* Process unlocked */
    __HAL_UNLOCK(hdma2d);

    return HAL_OK;
  }
}

/**
  * @brief Enable DMA2D dead time feature.
  * @param hdma2d DMA2D handle.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_EnableDeadTime(DMA2D_HandleTypeDef *hdma2d)
{
  /* Process Locked */
  __HAL_LOCK(hdma2d);

  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* Set DMA2D_AMTCR EN bit */
  SET_BIT(hdma2d->Instance->AMTCR, DMA2D_AMTCR_EN);

  hdma2d->State = HAL_DMA2D_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hdma2d);

  return HAL_OK;
}

/**
  * @brief Disable DMA2D dead time feature.
  * @param hdma2d DMA2D handle.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_DisableDeadTime(DMA2D_HandleTypeDef *hdma2d)
{
  /* Process Locked */
  __HAL_LOCK(hdma2d);

  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* Clear DMA2D_AMTCR EN bit */
  CLEAR_BIT(hdma2d->Instance->AMTCR, DMA2D_AMTCR_EN);

  hdma2d->State = HAL_DMA2D_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hdma2d);

  return HAL_OK;
}

/**
  * @brief Configure dead time.
  * @note The dead time value represents the guaranteed minimum number of cycles between
  *       two consecutive transactions on the AHB bus.
  * @param hdma2d DMA2D handle.
  * @param DeadTime dead time value.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA2D_ConfigDeadTime(DMA2D_HandleTypeDef *hdma2d, uint8_t DeadTime)
{
  /* Process Locked */
  __HAL_LOCK(hdma2d);

  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* Set DMA2D_AMTCR DT field */
  MODIFY_REG(hdma2d->Instance->AMTCR, DMA2D_AMTCR_DT, (((uint32_t) DeadTime) << DMA2D_POSITION_AMTCR_DT));

  hdma2d->State = HAL_DMA2D_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hdma2d);

  return HAL_OK;
}

/**
  * @}
  */

/** @defgroup DMA2D_Exported_Functions_Group4 Peripheral State and Error functions
 *  @brief    Peripheral State functions
 *
@verbatim
 ===============================================================================
                  ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to :
      (+) Get the DMA2D state
      (+) Get the DMA2D error code

@endverbatim
  * @{
  */

/**
  * @brief  Return the DMA2D state
  * @param  hdma2d pointer to a DMA2D_HandleTypeDef structure that contains
  *                 the configuration information for the DMA2D.
  * @retval HAL state
  */
HAL_DMA2D_StateTypeDef HAL_DMA2D_GetState(DMA2D_HandleTypeDef *hdma2d)
{
  return hdma2d->State;
}

/**
  * @brief  Return the DMA2D error code
  * @param  hdma2d  pointer to a DMA2D_HandleTypeDef structure that contains
  *               the configuration information for DMA2D.
  * @retval DMA2D Error Code
  */
uint32_t HAL_DMA2D_GetError(DMA2D_HandleTypeDef *hdma2d)
{
  return hdma2d->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup DMA2D_Private_Functions DMA2D Private Functions
  * @{
  */

/**
  * @brief  Set the DMA2D transfer parameters.
  * @param  hdma2d     Pointer to a DMA2D_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA2D.
  * @param  pdata      The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  Width      The width of data to be transferred from source to destination.
  * @param  Height     The height of data to be transferred from source to destination.
  * @retval HAL status
  */
static void DMA2D_SetConfig(DMA2D_HandleTypeDef *hdma2d, uint32_t pdata, uint32_t DstAddress, uint32_t Width, uint32_t Height)
{
  uint32_t tmp = 0U;
  uint32_t tmp1 = 0U;
  uint32_t tmp2 = 0U;
  uint32_t tmp3 = 0U;
  uint32_t tmp4 = 0U;

  /* Configure DMA2D data size */
  MODIFY_REG(hdma2d->Instance->NLR, (DMA2D_NLR_NL|DMA2D_NLR_PL), (Height| (Width << DMA2D_POSITION_NLR_PL)));

  /* Configure DMA2D destination address */
  WRITE_REG(hdma2d->Instance->OMAR, DstAddress);

  /* Register to memory DMA2D mode selected */
  if (hdma2d->Init.Mode == DMA2D_R2M)
  {
    tmp1 = pdata & DMA2D_OCOLR_ALPHA_1;
    tmp2 = pdata & DMA2D_OCOLR_RED_1;
    tmp3 = pdata & DMA2D_OCOLR_GREEN_1;
    tmp4 = pdata & DMA2D_OCOLR_BLUE_1;

    /* Prepare the value to be written to the OCOLR register according to the color mode */
    if (hdma2d->Init.ColorMode == DMA2D_OUTPUT_ARGB8888)
    {
      tmp = (tmp3 | tmp2 | tmp1| tmp4);
    }
    else if (hdma2d->Init.ColorMode == DMA2D_OUTPUT_RGB888)
    {
      tmp = (tmp3 | tmp2 | tmp4);
    }
    else if (hdma2d->Init.ColorMode == DMA2D_OUTPUT_RGB565)
    {
      tmp2 = (tmp2 >> 19U);
      tmp3 = (tmp3 >> 10U);
      tmp4 = (tmp4 >> 3U);
      tmp  = ((tmp3 << 5U) | (tmp2 << 11U) | tmp4);
    }
    else if (hdma2d->Init.ColorMode == DMA2D_OUTPUT_ARGB1555)
    {
      tmp1 = (tmp1 >> 31U);
      tmp2 = (tmp2 >> 19U);
      tmp3 = (tmp3 >> 11U);
      tmp4 = (tmp4 >> 3U);
      tmp  = ((tmp3 << 5U) | (tmp2 << 10U) | (tmp1 << 15U) | tmp4);
    }
    else /* Dhdma2d->Init.ColorMode = DMA2D_OUTPUT_ARGB4444 */
    {
      tmp1 = (tmp1 >> 28U);
      tmp2 = (tmp2 >> 20U);
      tmp3 = (tmp3 >> 12U);
      tmp4 = (tmp4 >> 4U);
      tmp  = ((tmp3 << 4U) | (tmp2 << 8U) | (tmp1 << 12U) | tmp4);
    }
    /* Write to DMA2D OCOLR register */
    WRITE_REG(hdma2d->Instance->OCOLR, tmp);
  }
  else /* M2M, M2M_PFC or M2M_Blending DMA2D Mode */
  {
    /* Configure DMA2D source address */
    WRITE_REG(hdma2d->Instance->FGMAR, pdata);
  }
}

/**
  * @}
  */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
#endif /* HAL_DMA2D_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
