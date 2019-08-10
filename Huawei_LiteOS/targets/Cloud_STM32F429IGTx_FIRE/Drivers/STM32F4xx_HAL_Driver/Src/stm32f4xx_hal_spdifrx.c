/**
  ******************************************************************************
  * @file    stm32f4xx_hal_spdifrx.c
  * @author  MCD Application Team
  * @brief   This file provides firmware functions to manage the following
  *          functionalities of the SPDIFRX audio interface:
  *           + Initialization and Configuration
  *           + Data transfers functions
  *           + DMA transfers management
  *           + Interrupts and flags management
  @verbatim
 ===============================================================================
                      ##### How to use this driver #####
 ===============================================================================
 [..]
    The SPDIFRX HAL driver can be used as follow:

    (#) Declare SPDIFRX_HandleTypeDef handle structure.
    (#) Initialize the SPDIFRX low level resources by implement the HAL_SPDIFRX_MspInit() API:
        (##) Enable the SPDIFRX interface clock.
        (##) SPDIFRX pins configuration:
            (+++) Enable the clock for the SPDIFRX GPIOs.
            (+++) Configure these SPDIFRX pins as alternate function pull-up.
        (##) NVIC configuration if you need to use interrupt process (HAL_SPDIFRX_ReceiveControlFlow_IT() and HAL_SPDIFRX_ReceiveDataFlow_IT() API's).
            (+++) Configure the SPDIFRX interrupt priority.
            (+++) Enable the NVIC SPDIFRX IRQ handle.
        (##) DMA Configuration if you need to use DMA process (HAL_SPDIFRX_ReceiveDataFlow_DMA() and HAL_SPDIFRX_ReceiveControlFlow_DMA() API's).
            (+++) Declare a DMA handle structure for the reception of the Data Flow channel.
            (+++) Declare a DMA handle structure for the reception of the Control Flow channel.
            (+++) Enable the DMAx interface clock.
            (+++) Configure the declared DMA handle structure CtrlRx/DataRx with the required parameters.
            (+++) Configure the DMA Channel.
            (+++) Associate the initialized DMA handle to the SPDIFRX DMA CtrlRx/DataRx handle.
            (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on the
                  DMA CtrlRx/DataRx channel.

   (#) Program the input selection, re-tries number, wait for activity, channel status selection, data format, stereo mode and masking of user bits
       using HAL_SPDIFRX_Init() function.

   -@- The specific SPDIFRX interrupts (RXNE/CSRNE and Error Interrupts) will be managed using the macros
       __SPDIFRX_ENABLE_IT() and __SPDIFRX_DISABLE_IT() inside the receive process.
   -@- Make sure that ck_spdif clock is configured.

   (#) Three operation modes are available within this driver :

   *** Polling mode for reception operation (for debug purpose) ***
   ================================================================
   [..]
     (+) Receive data flow in blocking mode using HAL_SPDIFRX_ReceiveDataFlow()
         (+) Receive control flow of data in blocking mode using HAL_SPDIFRX_ReceiveControlFlow()

   *** Interrupt mode for reception operation ***
   =========================================
   [..]
     (+) Receive an amount of data (Data Flow) in non blocking mode using HAL_SPDIFRX_ReceiveDataFlow_IT()
     (+) Receive an amount of data (Control Flow) in non blocking mode using HAL_SPDIFRX_ReceiveControlFlow_IT()
     (+) At reception end of half transfer HAL_SPDIFRX_RxHalfCpltCallback is executed and user can
         add his own code by customization of function pointer HAL_SPDIFRX_RxHalfCpltCallback
     (+) At reception end of transfer HAL_SPDIFRX_RxCpltCallback is executed and user can
         add his own code by customization of function pointer HAL_SPDIFRX_RxCpltCallback
     (+) In case of transfer Error, HAL_SPDIFRX_ErrorCallback() function is executed and user can
         add his own code by customization of function pointer HAL_SPDIFRX_ErrorCallback

   *** DMA mode for reception operation ***
   ========================================
   [..]
     (+) Receive an amount of data (Data Flow) in non blocking mode (DMA) using HAL_SPDIFRX_ReceiveDataFlow_DMA()
     (+) Receive an amount of data (Control Flow) in non blocking mode (DMA) using HAL_SPDIFRX_ReceiveControlFlow_DMA()
     (+) At reception end of half transfer HAL_SPDIFRX_RxHalfCpltCallback is executed and user can
         add his own code by customization of function pointer HAL_SPDIFRX_RxHalfCpltCallback
     (+) At reception end of transfer HAL_SPDIFRX_RxCpltCallback is executed and user can
         add his own code by customization of function pointer HAL_SPDIFRX_RxCpltCallback
     (+) In case of transfer Error, HAL_SPDIFRX_ErrorCallback() function is executed and user can
         add his own code by customization of function pointer HAL_SPDIFRX_ErrorCallback
     (+) Stop the DMA Transfer using HAL_SPDIFRX_DMAStop()

   *** SPDIFRX HAL driver macros list ***
   =============================================
   [..]
     Below the list of most used macros in SPDIFRX HAL driver.
      (+) __HAL_SPDIFRX_IDLE: Disable the specified SPDIFRX peripheral (IDEL State)
      (+) __HAL_SPDIFRX_SYNC: Enable the synchronization state of the specified SPDIFRX peripheral (SYNC State)
      (+) __HAL_SPDIFRX_RCV: Enable the receive state of the specified SPDIFRX peripheral (RCV State)
      (+) __HAL_SPDIFRX_ENABLE_IT : Enable the specified SPDIFRX interrupts
      (+) __HAL_SPDIFRX_DISABLE_IT : Disable the specified SPDIFRX interrupts
      (+) __HAL_SPDIFRX_GET_FLAG: Check whether the specified SPDIFRX flag is set or not.

   [..]
      (@) You can refer to the SPDIFRX HAL driver header file for more useful macros

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
/** @defgroup SPDIFRX SPDIFRX
  * @brief SPDIFRX HAL module driver
  * @{
  */

#ifdef HAL_SPDIFRX_MODULE_ENABLED

#if defined(STM32F446xx)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SPDIFRX_TIMEOUT_VALUE  0xFFFF

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @addtogroup SPDIFRX_Private_Functions
  * @{
  */
static void  SPDIFRX_DMARxCplt(DMA_HandleTypeDef *hdma);
static void  SPDIFRX_DMARxHalfCplt(DMA_HandleTypeDef *hdma);
static void  SPDIFRX_DMACxCplt(DMA_HandleTypeDef *hdma);
static void  SPDIFRX_DMACxHalfCplt(DMA_HandleTypeDef *hdma);
static void  SPDIFRX_DMAError(DMA_HandleTypeDef *hdma);
static void  SPDIFRX_ReceiveControlFlow_IT(SPDIFRX_HandleTypeDef *hspdif);
static void  SPDIFRX_ReceiveDataFlow_IT(SPDIFRX_HandleTypeDef *hspdif);
static HAL_StatusTypeDef  SPDIFRX_WaitOnFlagUntilTimeout(SPDIFRX_HandleTypeDef *hspdif, uint32_t Flag, FlagStatus Status, uint32_t Timeout);

/**
  * @}
  */
/* Exported functions ---------------------------------------------------------*/

/** @defgroup SPDIFRX_Exported_Functions SPDIFRX Exported Functions
  * @{
  */

/** @defgroup  SPDIFRX_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
  @verbatim
  ===============================================================================
           ##### Initialization and de-initialization functions #####
  ===============================================================================
  [..] This subsection provides a set of functions allowing to initialize and
       de-initialize the SPDIFRX peripheral:

     (+) User must Implement HAL_SPDIFRX_MspInit() function in which he configures
         all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

     (+) Call the function HAL_SPDIFRX_Init() to configure the SPDIFRX peripheral with
         the selected configuration:
         (++) Input Selection (IN0, IN1,...)
         (++) Maximum allowed re-tries during synchronization phase
         (++) Wait for activity on SPDIF selected input
         (++) Channel status selection (from channel A or B)
         (++) Data format (LSB, MSB, ...)
         (++) Stereo mode
         (++) User bits masking (PT,C,U,V,...)

     (+) Call the function HAL_SPDIFRX_DeInit() to restore the default configuration
         of the selected SPDIFRXx peripheral.
  @endverbatim
  * @{
  */

/**
  * @brief Initializes the SPDIFRX according to the specified parameters
  *        in the SPDIFRX_InitTypeDef and create the associated handle.
  * @param hspdif SPDIFRX handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPDIFRX_Init(SPDIFRX_HandleTypeDef *hspdif)
{
  uint32_t tmpreg = 0U;

  /* Check the SPDIFRX handle allocation */
  if(hspdif == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the SPDIFRX parameters */
  assert_param(IS_STEREO_MODE(hspdif->Init.StereoMode));
  assert_param(IS_SPDIFRX_INPUT_SELECT(hspdif->Init.InputSelection));
  assert_param(IS_SPDIFRX_MAX_RETRIES(hspdif->Init.Retries));
  assert_param(IS_SPDIFRX_WAIT_FOR_ACTIVITY(hspdif->Init.WaitForActivity));
  assert_param(IS_SPDIFRX_CHANNEL(hspdif->Init.ChannelSelection));
  assert_param(IS_SPDIFRX_DATA_FORMAT(hspdif->Init.DataFormat));
  assert_param(IS_PREAMBLE_TYPE_MASK(hspdif->Init.PreambleTypeMask));
  assert_param(IS_CHANNEL_STATUS_MASK(hspdif->Init.ChannelStatusMask));
  assert_param(IS_VALIDITY_MASK(hspdif->Init.ValidityBitMask));
  assert_param(IS_PARITY_ERROR_MASK(hspdif->Init.ParityErrorMask));

  if(hspdif->State == HAL_SPDIFRX_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hspdif->Lock = HAL_UNLOCKED;
    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    HAL_SPDIFRX_MspInit(hspdif);
  }

     /* SPDIFRX peripheral state is BUSY*/
   hspdif->State = HAL_SPDIFRX_STATE_BUSY;

  /* Disable SPDIFRX interface (IDLE State) */
  __HAL_SPDIFRX_IDLE(hspdif);

  /* Reset the old SPDIFRX CR configuration */
  tmpreg = hspdif->Instance->CR;

  tmpreg &= ~((uint16_t) SPDIFRX_CR_RXSTEO  | SPDIFRX_CR_DRFMT  | SPDIFRX_CR_PMSK |
                         SPDIFRX_CR_VMSK | SPDIFRX_CR_CUMSK | SPDIFRX_CR_PTMSK  |
                         SPDIFRX_CR_CHSEL | SPDIFRX_CR_NBTR | SPDIFRX_CR_WFA |
                         SPDIFRX_CR_INSEL);

  /* Sets the new configuration of the SPDIFRX peripheral */
  tmpreg |= ((uint16_t) hspdif->Init.StereoMode |
                        hspdif->Init.InputSelection |
                        hspdif->Init.Retries |
                        hspdif->Init.WaitForActivity |
                        hspdif->Init.ChannelSelection |
                        hspdif->Init.DataFormat |
                        hspdif->Init.PreambleTypeMask |
                        hspdif->Init.ChannelStatusMask |
                        hspdif->Init.ValidityBitMask |
                        hspdif->Init.ParityErrorMask);

  hspdif->Instance->CR = tmpreg;

  hspdif->ErrorCode = HAL_SPDIFRX_ERROR_NONE;

    /* SPDIFRX peripheral state is READY*/
  hspdif->State = HAL_SPDIFRX_STATE_READY;

  return HAL_OK;
}

/**
  * @brief DeInitializes the SPDIFRX peripheral
  * @param hspdif SPDIFRX handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPDIFRX_DeInit(SPDIFRX_HandleTypeDef *hspdif)
{
  /* Check the SPDIFRX handle allocation */
  if(hspdif == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_SPDIFRX_ALL_INSTANCE(hspdif->Instance));

  hspdif->State = HAL_SPDIFRX_STATE_BUSY;

  /* Disable SPDIFRX interface (IDLE state) */
  __HAL_SPDIFRX_IDLE(hspdif);

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
  HAL_SPDIFRX_MspDeInit(hspdif);

  hspdif->ErrorCode = HAL_SPDIFRX_ERROR_NONE;

  /* SPDIFRX peripheral state is RESET*/
  hspdif->State = HAL_SPDIFRX_STATE_RESET;

  /* Release Lock */
  __HAL_UNLOCK(hspdif);

  return HAL_OK;
}

/**
  * @brief SPDIFRX MSP Init
  * @param hspdif SPDIFRX handle
  * @retval None
  */
__weak void HAL_SPDIFRX_MspInit(SPDIFRX_HandleTypeDef *hspdif)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspdif);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPDIFRX_MspInit could be implemented in the user file
  */
}

/**
  * @brief SPDIFRX MSP DeInit
  * @param hspdif SPDIFRX handle
  * @retval None
  */
__weak void HAL_SPDIFRX_MspDeInit(SPDIFRX_HandleTypeDef *hspdif)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspdif);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPDIFRX_MspDeInit could be implemented in the user file
  */
}

/**
  * @brief Sets the SPDIFRX  dtat format according to the specified parameters
  *        in the SPDIFRX_InitTypeDef.
  * @param hspdif SPDIFRX handle
  * @param sDataFormat SPDIFRX data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPDIFRX_SetDataFormat(SPDIFRX_HandleTypeDef *hspdif, SPDIFRX_SetDataFormatTypeDef  sDataFormat)
{
  uint32_t tmpreg = 0U;

  /* Check the SPDIFRX handle allocation */
  if(hspdif == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the SPDIFRX parameters */
  assert_param(IS_STEREO_MODE(sDataFormat.StereoMode));
  assert_param(IS_SPDIFRX_DATA_FORMAT(sDataFormat.DataFormat));
  assert_param(IS_PREAMBLE_TYPE_MASK(sDataFormat.PreambleTypeMask));
  assert_param(IS_CHANNEL_STATUS_MASK(sDataFormat.ChannelStatusMask));
  assert_param(IS_VALIDITY_MASK(sDataFormat.ValidityBitMask));
  assert_param(IS_PARITY_ERROR_MASK(sDataFormat.ParityErrorMask));

  /* Reset the old SPDIFRX CR configuration */
  tmpreg = hspdif->Instance->CR;

  if(((tmpreg & SPDIFRX_STATE_RCV) == SPDIFRX_STATE_RCV) &&
    (((tmpreg & SPDIFRX_CR_DRFMT) != sDataFormat.DataFormat) ||
    ((tmpreg & SPDIFRX_CR_RXSTEO) != sDataFormat.StereoMode)))
  {
      return HAL_ERROR;
  }

  tmpreg &= ~((uint16_t) SPDIFRX_CR_RXSTEO  | SPDIFRX_CR_DRFMT  | SPDIFRX_CR_PMSK |
                         SPDIFRX_CR_VMSK | SPDIFRX_CR_CUMSK | SPDIFRX_CR_PTMSK);

  /* Sets the new configuration of the SPDIFRX peripheral */
  tmpreg |= ((uint16_t) sDataFormat.StereoMode |
                        sDataFormat.DataFormat |
                        sDataFormat.PreambleTypeMask |
                        sDataFormat.ChannelStatusMask |
                        sDataFormat.ValidityBitMask |
                        sDataFormat.ParityErrorMask);

  hspdif->Instance->CR = tmpreg;

  return HAL_OK;
}

/**
  * @}
  */

/** @defgroup SPDIFRX_Exported_Functions_Group2 IO operation functions
  *  @brief Data transfers functions
  *
@verbatim
===============================================================================
                      ##### IO operation functions #####
===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the SPDIFRX data
    transfers.

    (#) There is two mode of transfer:
        (++) Blocking mode : The communication is performed in the polling mode.
             The status of all data processing is returned by the same function
             after finishing transfer.
        (++) No-Blocking mode : The communication is performed using Interrupts
             or DMA. These functions return the status of the transfer start-up.
             The end of the data processing will be indicated through the
             dedicated SPDIFRX IRQ when using Interrupt mode or the DMA IRQ when
             using DMA mode.

    (#) Blocking mode functions are :
        (++) HAL_SPDIFRX_ReceiveDataFlow()
        (++) HAL_SPDIFRX_ReceiveControlFlow()
                (+@) Do not use blocking mode to receive both control and data flow at the same time.

    (#) No-Blocking mode functions with Interrupt are :
        (++) HAL_SPDIFRX_ReceiveControlFlow_IT()
        (++) HAL_SPDIFRX_ReceiveDataFlow_IT()

    (#) No-Blocking mode functions with DMA are :
        (++) HAL_SPDIFRX_ReceiveControlFlow_DMA()
        (++) HAL_SPDIFRX_ReceiveDataFlow_DMA()

    (#) A set of Transfer Complete Callbacks are provided in No_Blocking mode:
        (++) HAL_SPDIFRX_RxCpltCallback()
        (++) HAL_SPDIFRX_ErrorCallback()

@endverbatim
  * @{
  */

/**
  * @brief  Receives an amount of data (Data Flow) in blocking mode.
  * @param  hspdif pointer to SPDIFRX_HandleTypeDef structure that contains
  *                 the configuration information for SPDIFRX module.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be received
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveDataFlow(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size, uint32_t Timeout)
{
  if((pData == NULL ) || (Size == 0))
  {
    return  HAL_ERROR;
  }

  if(hspdif->State == HAL_SPDIFRX_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hspdif);

    hspdif->State = HAL_SPDIFRX_STATE_BUSY;

    /* Start synchronisation */
    __HAL_SPDIFRX_SYNC(hspdif);

    /* Wait until SYNCD flag is set */
    if(SPDIFRX_WaitOnFlagUntilTimeout(hspdif, SPDIFRX_FLAG_SYNCD, RESET, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    /* Start reception */
    __HAL_SPDIFRX_RCV(hspdif);

    /* Receive data flow */
    while(Size > 0)
    {
      /* Wait until RXNE flag is set */
      if(SPDIFRX_WaitOnFlagUntilTimeout(hspdif, SPDIFRX_FLAG_RXNE, RESET, Timeout) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }

      (*pData++) = hspdif->Instance->DR;
      Size--;
    }

    /* SPDIFRX ready */
    hspdif->State = HAL_SPDIFRX_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hspdif);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receives an amount of data (Control Flow) in blocking mode.
  * @param  hspdif pointer to a SPDIFRX_HandleTypeDef structure that contains
  *                 the configuration information for SPDIFRX module.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be received
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveControlFlow(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size, uint32_t Timeout)
{
  if((pData == NULL ) || (Size == 0))
  {
    return  HAL_ERROR;
  }

  if(hspdif->State == HAL_SPDIFRX_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hspdif);

    hspdif->State = HAL_SPDIFRX_STATE_BUSY;

    /* Start synchronization */
    __HAL_SPDIFRX_SYNC(hspdif);

    /* Wait until SYNCD flag is set */
    if(SPDIFRX_WaitOnFlagUntilTimeout(hspdif, SPDIFRX_FLAG_SYNCD, RESET, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    /* Start reception */
    __HAL_SPDIFRX_RCV(hspdif);

    /* Receive control flow */
    while(Size > 0)
    {
      /* Wait until CSRNE flag is set */
      if(SPDIFRX_WaitOnFlagUntilTimeout(hspdif, SPDIFRX_FLAG_CSRNE, RESET, Timeout) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }

      (*pData++) = hspdif->Instance->CSR;
      Size--;
    }

    /* SPDIFRX ready */
    hspdif->State = HAL_SPDIFRX_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hspdif);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/**
  * @brief Receive an amount of data (Data Flow) in non-blocking mode with Interrupt
  * @param hspdif SPDIFRX handle
  * @param pData a 32-bit pointer to the Receive data buffer.
  * @param Size number of data sample to be received .
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveDataFlow_IT(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size)
{
  __IO uint32_t count = SPDIFRX_TIMEOUT_VALUE * (SystemCoreClock / 24U / 1000U);

  if((hspdif->State == HAL_SPDIFRX_STATE_READY) || (hspdif->State == HAL_SPDIFRX_STATE_BUSY_CX))
  {
    if((pData == NULL) || (Size == 0))
    {
      return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hspdif);

    hspdif->pRxBuffPtr = pData;
    hspdif->RxXferSize = Size;
    hspdif->RxXferCount = Size;

    hspdif->ErrorCode = HAL_SPDIFRX_ERROR_NONE;

    /* Check if a receive process is ongoing or not */
    hspdif->State = HAL_SPDIFRX_STATE_BUSY_RX;

    /* Enable the SPDIFRX  PE Error Interrupt */
    __HAL_SPDIFRX_ENABLE_IT(hspdif, SPDIFRX_IT_PERRIE);

    /* Enable the SPDIFRX  OVR Error Interrupt */
    __HAL_SPDIFRX_ENABLE_IT(hspdif, SPDIFRX_IT_OVRIE);

    /* Process Unlocked */
    __HAL_UNLOCK(hspdif);

    /* Enable the SPDIFRX RXNE interrupt */
    __HAL_SPDIFRX_ENABLE_IT(hspdif, SPDIFRX_IT_RXNE);

    if ((SPDIFRX->CR & SPDIFRX_CR_SPDIFEN) != SPDIFRX_STATE_SYNC || (SPDIFRX->CR & SPDIFRX_CR_SPDIFEN) != 0x00U)
    {
      /* Start synchronization */
      __HAL_SPDIFRX_SYNC(hspdif);

      /* Wait until SYNCD flag is set */
      do
      {
        if (count-- == 0U)
        {
          /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_RXNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_CSRNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_PERRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_OVRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SBLKIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SYNCDIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_IFEIE);

          hspdif->State= HAL_SPDIFRX_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspdif);

          return HAL_TIMEOUT;
        }
      }
      while (__HAL_SPDIFRX_GET_FLAG(hspdif, SPDIFRX_FLAG_SYNCD) == RESET);

      /* Start reception */
      __HAL_SPDIFRX_RCV(hspdif);
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief Receive an amount of data (Control Flow) with Interrupt
  * @param hspdif SPDIFRX handle
  * @param pData a 32-bit pointer to the Receive data buffer.
  * @param Size number of data sample (Control Flow) to be received :
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveControlFlow_IT(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size)
{
  __IO uint32_t count = SPDIFRX_TIMEOUT_VALUE * (SystemCoreClock / 24U / 1000U);

  if((hspdif->State == HAL_SPDIFRX_STATE_READY) || (hspdif->State == HAL_SPDIFRX_STATE_BUSY_RX))
  {
    if((pData == NULL ) || (Size == 0))
    {
      return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hspdif);

    hspdif->pCsBuffPtr = pData;
    hspdif->CsXferSize = Size;
    hspdif->CsXferCount = Size;

    hspdif->ErrorCode = HAL_SPDIFRX_ERROR_NONE;

    /* Check if a receive process is ongoing or not */
    hspdif->State = HAL_SPDIFRX_STATE_BUSY_CX;

    /* Enable the SPDIFRX PE Error Interrupt */
    __HAL_SPDIFRX_ENABLE_IT(hspdif, SPDIFRX_IT_PERRIE);

    /* Enable the SPDIFRX OVR Error Interrupt */
    __HAL_SPDIFRX_ENABLE_IT(hspdif, SPDIFRX_IT_OVRIE);

    /* Process Unlocked */
    __HAL_UNLOCK(hspdif);

    /* Enable the SPDIFRX CSRNE interrupt */
    __HAL_SPDIFRX_ENABLE_IT(hspdif, SPDIFRX_IT_CSRNE);

    if ((SPDIFRX->CR & SPDIFRX_CR_SPDIFEN) != SPDIFRX_STATE_SYNC || (SPDIFRX->CR & SPDIFRX_CR_SPDIFEN) != 0x00U)
    {
      /* Start synchronization */
      __HAL_SPDIFRX_SYNC(hspdif);

      /* Wait until SYNCD flag is set */
      do
      {
        if (count-- == 0U)
        {
          /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_RXNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_CSRNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_PERRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_OVRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SBLKIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SYNCDIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_IFEIE);

          hspdif->State= HAL_SPDIFRX_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspdif);

          return HAL_TIMEOUT;
        }
      }
      while (__HAL_SPDIFRX_GET_FLAG(hspdif, SPDIFRX_FLAG_SYNCD) == RESET);

      /* Start reception */
      __HAL_SPDIFRX_RCV(hspdif);
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief Receive an amount of data (Data Flow) mode with DMA
  * @param hspdif SPDIFRX handle
  * @param pData a 32-bit pointer to the Receive data buffer.
  * @param Size number of data sample to be received :
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveDataFlow_DMA(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size)
{
  __IO uint32_t count = SPDIFRX_TIMEOUT_VALUE * (SystemCoreClock / 24U / 1000U);

  if((pData == NULL) || (Size == 0))
  {
    return  HAL_ERROR;
  }

  if((hspdif->State == HAL_SPDIFRX_STATE_READY) || (hspdif->State == HAL_SPDIFRX_STATE_BUSY_CX))
  {
    hspdif->pRxBuffPtr = pData;
    hspdif->RxXferSize = Size;
    hspdif->RxXferCount = Size;

    /* Process Locked */
    __HAL_LOCK(hspdif);

    hspdif->ErrorCode = HAL_SPDIFRX_ERROR_NONE;
    hspdif->State = HAL_SPDIFRX_STATE_BUSY_RX;

    /* Set the SPDIFRX Rx DMA Half transfer complete callback */
    hspdif->hdmaDrRx->XferHalfCpltCallback = SPDIFRX_DMARxHalfCplt;

    /* Set the SPDIFRX Rx DMA transfer complete callback */
    hspdif->hdmaDrRx->XferCpltCallback = SPDIFRX_DMARxCplt;

    /* Set the DMA error callback */
    hspdif->hdmaDrRx->XferErrorCallback = SPDIFRX_DMAError;

    /* Enable the DMA request */
    HAL_DMA_Start_IT(hspdif->hdmaDrRx, (uint32_t)&hspdif->Instance->DR, (uint32_t)hspdif->pRxBuffPtr, Size);

    /* Enable RXDMAEN bit in SPDIFRX CR register for data flow reception*/
    hspdif->Instance->CR |= SPDIFRX_CR_RXDMAEN;

    if ((SPDIFRX->CR & SPDIFRX_CR_SPDIFEN) != SPDIFRX_STATE_SYNC || (SPDIFRX->CR & SPDIFRX_CR_SPDIFEN) != 0x00U)
    {
      /* Start synchronization */
      __HAL_SPDIFRX_SYNC(hspdif);

      /* Wait until SYNCD flag is set */
      do
      {
        if (count-- == 0U)
        {
          /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_RXNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_CSRNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_PERRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_OVRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SBLKIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SYNCDIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_IFEIE);

          hspdif->State= HAL_SPDIFRX_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspdif);

          return HAL_TIMEOUT;
        }
      }
      while (__HAL_SPDIFRX_GET_FLAG(hspdif, SPDIFRX_FLAG_SYNCD) == RESET);

      /* Start reception */
      __HAL_SPDIFRX_RCV(hspdif);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hspdif);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief Receive an amount of data (Control Flow) with DMA
  * @param hspdif SPDIFRX handle
  * @param pData a 32-bit pointer to the Receive data buffer.
  * @param Size number of data (Control Flow) sample to be received :
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveControlFlow_DMA(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size)
{
  __IO uint32_t count = SPDIFRX_TIMEOUT_VALUE * (SystemCoreClock / 24U / 1000U);

  if((pData == NULL) || (Size == 0))
  {
    return  HAL_ERROR;
  }

  if((hspdif->State == HAL_SPDIFRX_STATE_READY) || (hspdif->State == HAL_SPDIFRX_STATE_BUSY_RX))
  {
    hspdif->pCsBuffPtr = pData;
    hspdif->CsXferSize = Size;
    hspdif->CsXferCount = Size;

    /* Process Locked */
    __HAL_LOCK(hspdif);

    hspdif->ErrorCode = HAL_SPDIFRX_ERROR_NONE;
    hspdif->State = HAL_SPDIFRX_STATE_BUSY_CX;

    /* Set the SPDIFRX Rx DMA Half transfer complete callback */
    hspdif->hdmaCsRx->XferHalfCpltCallback = SPDIFRX_DMACxHalfCplt;

    /* Set the SPDIFRX Rx DMA transfer complete callback */
    hspdif->hdmaCsRx->XferCpltCallback = SPDIFRX_DMACxCplt;

    /* Set the DMA error callback */
    hspdif->hdmaCsRx->XferErrorCallback = SPDIFRX_DMAError;

    /* Enable the DMA request */
    HAL_DMA_Start_IT(hspdif->hdmaCsRx, (uint32_t)&hspdif->Instance->CSR, (uint32_t)hspdif->pCsBuffPtr, Size);

    /* Enable CBDMAEN bit in SPDIFRX CR register for control flow reception*/
    hspdif->Instance->CR |= SPDIFRX_CR_CBDMAEN;

    if ((SPDIFRX->CR & SPDIFRX_CR_SPDIFEN) != SPDIFRX_STATE_SYNC || (SPDIFRX->CR & SPDIFRX_CR_SPDIFEN) != 0x00U)
    {
      /* Start synchronization */
      __HAL_SPDIFRX_SYNC(hspdif);

      /* Wait until SYNCD flag is set */
      do
      {
        if (count-- == 0U)
        {
          /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_RXNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_CSRNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_PERRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_OVRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SBLKIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SYNCDIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_IFEIE);

          hspdif->State= HAL_SPDIFRX_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspdif);

          return HAL_TIMEOUT;
        }
      }
      while (__HAL_SPDIFRX_GET_FLAG(hspdif, SPDIFRX_FLAG_SYNCD) == RESET);

      /* Start reception */
      __HAL_SPDIFRX_RCV(hspdif);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hspdif);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief stop the audio stream receive from the Media.
  * @param hspdif SPDIFRX handle
  * @retval None
  */
HAL_StatusTypeDef HAL_SPDIFRX_DMAStop(SPDIFRX_HandleTypeDef *hspdif)
{
  /* Process Locked */
  __HAL_LOCK(hspdif);

  /* Disable the SPDIFRX DMA requests */
  hspdif->Instance->CR &= (uint16_t)(~SPDIFRX_CR_RXDMAEN);
  hspdif->Instance->CR &= (uint16_t)(~SPDIFRX_CR_CBDMAEN);

  /* Disable the SPDIFRX DMA channel */
  __HAL_DMA_DISABLE(hspdif->hdmaDrRx);
  __HAL_DMA_DISABLE(hspdif->hdmaCsRx);

  /* Disable SPDIFRX peripheral */
  __HAL_SPDIFRX_IDLE(hspdif);

  hspdif->State = HAL_SPDIFRX_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hspdif);

  return HAL_OK;
}

/**
  * @brief  This function handles SPDIFRX interrupt request.
  * @param  hspdif SPDIFRX handle
  * @retval HAL status
  */
void HAL_SPDIFRX_IRQHandler(SPDIFRX_HandleTypeDef *hspdif)
{
  /* SPDIFRX in mode Data Flow Reception ------------------------------------------------*/
  if((__HAL_SPDIFRX_GET_FLAG(hspdif, SPDIFRX_FLAG_RXNE) != RESET) && (__HAL_SPDIFRX_GET_IT_SOURCE(hspdif, SPDIFRX_IT_RXNE) != RESET))
  {
    __HAL_SPDIFRX_CLEAR_IT(hspdif, SPDIFRX_IT_RXNE);
    SPDIFRX_ReceiveDataFlow_IT(hspdif);
  }

  /* SPDIFRX in mode Control Flow Reception ------------------------------------------------*/
  if((__HAL_SPDIFRX_GET_FLAG(hspdif, SPDIFRX_FLAG_CSRNE) != RESET) && (__HAL_SPDIFRX_GET_IT_SOURCE(hspdif, SPDIFRX_IT_CSRNE) != RESET))
  {
    __HAL_SPDIFRX_CLEAR_IT(hspdif, SPDIFRX_IT_CSRNE);
    SPDIFRX_ReceiveControlFlow_IT(hspdif);
  }

  /* SPDIFRX Overrun error interrupt occurred ---------------------------------*/
  if((__HAL_SPDIFRX_GET_FLAG(hspdif, SPDIFRX_FLAG_OVR) != RESET) && (__HAL_SPDIFRX_GET_IT_SOURCE(hspdif, SPDIFRX_IT_OVRIE) != RESET))
  {
    __HAL_SPDIFRX_CLEAR_IT(hspdif, SPDIFRX_FLAG_OVR);

    /* Change the SPDIFRX error code */
    hspdif->ErrorCode |= HAL_SPDIFRX_ERROR_OVR;

    /* the transfer is not stopped */
    HAL_SPDIFRX_ErrorCallback(hspdif);
  }

  /* SPDIFRX Parity error interrupt occurred ---------------------------------*/
  if((__HAL_SPDIFRX_GET_FLAG(hspdif, SPDIFRX_FLAG_PERR) != RESET) && (__HAL_SPDIFRX_GET_IT_SOURCE(hspdif, SPDIFRX_IT_PERRIE) != RESET))
  {
    __HAL_SPDIFRX_CLEAR_IT(hspdif, SPDIFRX_FLAG_PERR);

    /* Change the SPDIFRX error code */
    hspdif->ErrorCode |= HAL_SPDIFRX_ERROR_PE;

    /* the transfer is not stopped */
    HAL_SPDIFRX_ErrorCallback(hspdif);
  }
}

/**
  * @brief Rx Transfer (Data flow) half completed callbacks
  * @param hspdif SPDIFRX handle
  * @retval None
  */
__weak void HAL_SPDIFRX_RxHalfCpltCallback(SPDIFRX_HandleTypeDef *hspdif)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspdif);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPDIFRX_RxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief Rx Transfer (Data flow) completed callbacks
  * @param hspdif SPDIFRX handle
  * @retval None
  */
__weak void HAL_SPDIFRX_RxCpltCallback(SPDIFRX_HandleTypeDef *hspdif)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspdif);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPDIFRX_RxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief Rx (Control flow) Transfer half completed callbacks
  * @param hspdif SPDIFRX handle
  * @retval None
  */
__weak void HAL_SPDIFRX_CxHalfCpltCallback(SPDIFRX_HandleTypeDef *hspdif)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspdif);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPDIFRX_RxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief Rx Transfer (Control flow) completed callbacks
  * @param hspdif SPDIFRX handle
  * @retval None
  */
__weak void HAL_SPDIFRX_CxCpltCallback(SPDIFRX_HandleTypeDef *hspdif)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspdif);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPDIFRX_RxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief SPDIFRX error callbacks
  * @param hspdif SPDIFRX handle
  * @retval None
  */
__weak void HAL_SPDIFRX_ErrorCallback(SPDIFRX_HandleTypeDef *hspdif)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspdif);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPDIFRX_ErrorCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup SPDIFRX_Exported_Functions_Group3 Peripheral State and Errors functions
  *  @brief   Peripheral State functions
  *
@verbatim
===============================================================================
              ##### Peripheral State and Errors functions #####
===============================================================================
[..]
     This subsection permit to get in run-time the status of the peripheral
     and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the SPDIFRX state
  * @param  hspdif  SPDIFRX handle
  * @retval HAL state
  */
HAL_SPDIFRX_StateTypeDef HAL_SPDIFRX_GetState(SPDIFRX_HandleTypeDef *hspdif)
{
  return hspdif->State;
}

/**
  * @brief  Return the SPDIFRX error code
  * @param  hspdif  SPDIFRX handle
  * @retval SPDIFRX Error Code
  */
uint32_t HAL_SPDIFRX_GetError(SPDIFRX_HandleTypeDef *hspdif)
{
  return hspdif->ErrorCode;
}

/**
  * @}
  */

/**
  * @brief DMA SPDIFRX receive process (Data flow) complete callback
  * @param hdma  DMA handle
  * @retval None
  */
static void SPDIFRX_DMARxCplt(DMA_HandleTypeDef *hdma)
{
  SPDIFRX_HandleTypeDef* hspdif = ( SPDIFRX_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Disable Rx DMA Request */
  hspdif->Instance->CR &= (uint16_t)(~SPDIFRX_CR_RXDMAEN);
  hspdif->RxXferCount = 0U;

  hspdif->State = HAL_SPDIFRX_STATE_READY;
  HAL_SPDIFRX_RxCpltCallback(hspdif);
}

/**
  * @brief DMA SPDIFRX receive process (Data flow) half complete callback
  * @param hdma  DMA handle
  * @retval None
  */
static void SPDIFRX_DMARxHalfCplt(DMA_HandleTypeDef *hdma)
{
  SPDIFRX_HandleTypeDef* hspdif = (SPDIFRX_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  HAL_SPDIFRX_RxHalfCpltCallback(hspdif);
}

/**
  * @brief DMA SPDIFRX receive process (Control flow) complete callback
  * @param hdma  DMA handle
  * @retval None
  */
static void SPDIFRX_DMACxCplt(DMA_HandleTypeDef *hdma)
{
  SPDIFRX_HandleTypeDef* hspdif = ( SPDIFRX_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Disable Cb DMA Request */
  hspdif->Instance->CR &= (uint16_t)(~SPDIFRX_CR_CBDMAEN);
  hspdif->CsXferCount = 0U;

  hspdif->State = HAL_SPDIFRX_STATE_READY;
  HAL_SPDIFRX_CxCpltCallback(hspdif);
}

/**
  * @brief DMA SPDIFRX receive process (Control flow) half complete callback
  * @param hdma  DMA handle
  * @retval None
  */
static void SPDIFRX_DMACxHalfCplt(DMA_HandleTypeDef *hdma)
{
  SPDIFRX_HandleTypeDef* hspdif = (SPDIFRX_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  HAL_SPDIFRX_CxHalfCpltCallback(hspdif);
}

/**
  * @brief DMA SPDIFRX communication error callback
  * @param hdma  DMA handle
  * @retval None
  */
static void SPDIFRX_DMAError(DMA_HandleTypeDef *hdma)
{
  SPDIFRX_HandleTypeDef* hspdif = ( SPDIFRX_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Disable Rx and Cb DMA Request */
  hspdif->Instance->CR &= (uint16_t)(~(SPDIFRX_CR_RXDMAEN | SPDIFRX_CR_CBDMAEN));
  hspdif->RxXferCount = 0U;

  hspdif->State= HAL_SPDIFRX_STATE_READY;

  /* Set the error code and execute error callback*/
  hspdif->ErrorCode |= HAL_SPDIFRX_ERROR_DMA;
  HAL_SPDIFRX_ErrorCallback(hspdif);
}

/**
  * @brief Receive an amount of data (Data Flow) with Interrupt
  * @param hspdif SPDIFRX handle
  * @retval None
  */
static void SPDIFRX_ReceiveDataFlow_IT(SPDIFRX_HandleTypeDef *hspdif)
{
  /* Receive data */
  (*hspdif->pRxBuffPtr++) = hspdif->Instance->DR;
  hspdif->RxXferCount--;

  if(hspdif->RxXferCount == 0U)
  {
    /* Disable RXNE/PE and OVR interrupts */
    __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_OVRIE | SPDIFRX_IT_PERRIE | SPDIFRX_IT_RXNE);

    hspdif->State = HAL_SPDIFRX_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hspdif);

    HAL_SPDIFRX_RxCpltCallback(hspdif);
  }
}

/**
  * @brief Receive an amount of data (Control Flow) with Interrupt
  * @param hspdif SPDIFRX handle
  * @retval None
  */
static void SPDIFRX_ReceiveControlFlow_IT(SPDIFRX_HandleTypeDef *hspdif)
{
  /* Receive data */
  (*hspdif->pCsBuffPtr++) = hspdif->Instance->CSR;
  hspdif->CsXferCount--;

  if(hspdif->CsXferCount == 0U)
  {
    /* Disable CSRNE interrupt */
    __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_CSRNE);

    hspdif->State = HAL_SPDIFRX_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hspdif);

    HAL_SPDIFRX_CxCpltCallback(hspdif);
  }
}

/**
  * @brief This function handles SPDIFRX Communication Timeout.
  * @param hspdif SPDIFRX handle
  * @param Flag Flag checked
  * @param Status Value of the flag expected
  * @param Timeout Duration of the timeout
  * @retval HAL status
  */
static HAL_StatusTypeDef SPDIFRX_WaitOnFlagUntilTimeout(SPDIFRX_HandleTypeDef *hspdif, uint32_t Flag, FlagStatus Status, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Wait until flag is set */
  if(Status == RESET)
  {
    while(__HAL_SPDIFRX_GET_FLAG(hspdif, Flag) == RESET)
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_RXNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_CSRNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_PERRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_OVRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SBLKIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SYNCDIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_IFEIE);

          hspdif->State= HAL_SPDIFRX_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspdif);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while(__HAL_SPDIFRX_GET_FLAG(hspdif, Flag) != RESET)
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_RXNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_CSRNE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_PERRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_OVRIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SBLKIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_SYNCDIE);
          __HAL_SPDIFRX_DISABLE_IT(hspdif, SPDIFRX_IT_IFEIE);

          hspdif->State= HAL_SPDIFRX_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspdif);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

/**
  * @}
  */
#endif /* STM32F446xx */

#endif /* HAL_SPDIFRX_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

