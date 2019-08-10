/**
  ******************************************************************************
  * @file    stm32f4xx_hal_cryp.c
  * @author  MCD Application Team
  * @brief   CRYP HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Cryptography (CRYP) peripheral:
  *           + Initialization and de-initialization functions
  *           + AES processing functions
  *           + DES processing functions
  *           + TDES processing functions
  *           + DMA callback functions
  *           + CRYP IRQ handler management
  *           + Peripheral State functions
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
      The CRYP HAL driver can be used as follows:

      (#)Initialize the CRYP low level resources by implementing the HAL_CRYP_MspInit():
         (##) Enable the CRYP interface clock using __HAL_RCC_CRYP_CLK_ENABLE()
         (##) In case of using interrupts (e.g. HAL_CRYP_AESECB_Encrypt_IT())
             (+++) Configure the CRYP interrupt priority using HAL_NVIC_SetPriority()
             (+++) Enable the CRYP IRQ handler using HAL_NVIC_EnableIRQ()
             (+++) In CRYP IRQ handler, call HAL_CRYP_IRQHandler()
         (##) In case of using DMA to control data transfer (e.g. HAL_CRYP_AESECB_Encrypt_DMA())
             (+++) Enable the DMAx interface clock using __DMAx_CLK_ENABLE()
             (+++) Configure and enable two DMA streams one for managing data transfer from
                 memory to peripheral (input stream) and another stream for managing data
                 transfer from peripheral to memory (output stream)
             (+++) Associate the initialized DMA handle to the CRYP DMA handle
                 using  __HAL_LINKDMA()
             (+++) Configure the priority and enable the NVIC for the transfer complete
                 interrupt on the two DMA Streams. The output stream should have higher
                 priority than the input stream HAL_NVIC_SetPriority() and HAL_NVIC_EnableIRQ()

      (#)Initialize the CRYP HAL using HAL_CRYP_Init(). This function configures mainly:
         (##) The data type: 1-bit, 8-bit, 16-bit and 32-bit
         (##) The key size: 128, 192 and 256. This parameter is relevant only for AES
         (##) The encryption/decryption key. It's size depends on the algorithm
              used for encryption/decryption
         (##) The initialization vector (counter). It is not used ECB mode.

      (#)Three processing (encryption/decryption) functions are available:
         (##) Polling mode: encryption and decryption APIs are blocking functions
              i.e. they process the data and wait till the processing is finished,
              e.g. HAL_CRYP_AESCBC_Encrypt()
         (##) Interrupt mode: encryption and decryption APIs are not blocking functions
              i.e. they process the data under interrupt,
              e.g. HAL_CRYP_AESCBC_Encrypt_IT()
         (##) DMA mode: encryption and decryption APIs are not blocking functions
              i.e. the data transfer is ensured by DMA,
              e.g. HAL_CRYP_AESCBC_Encrypt_DMA()

      (#)When the processing function is called at first time after HAL_CRYP_Init()
         the CRYP peripheral is initialized and processes the buffer in input.
         At second call, the processing function performs an append of the already
         processed buffer.
         When a new data block is to be processed, call HAL_CRYP_Init() then the
         processing function.

       (#)Call HAL_CRYP_DeInit() to deinitialize the CRYP peripheral.

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

#ifdef HAL_CRYP_MODULE_ENABLED

#if defined(CRYP)

/** @defgroup CRYP CRYP
  * @brief CRYP HAL module driver.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup CRYP_Private_define
  * @{
  */
#define CRYP_TIMEOUT_VALUE  1U
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup CRYP_Private_Functions_prototypes
  * @{
  */
static void CRYP_SetInitVector(CRYP_HandleTypeDef *hcryp, uint8_t *InitVector, uint32_t IVSize);
static void CRYP_SetKey(CRYP_HandleTypeDef *hcryp, uint8_t *Key, uint32_t KeySize);
static HAL_StatusTypeDef CRYP_ProcessData(CRYP_HandleTypeDef *hcryp, uint8_t* Input, uint16_t Ilength, uint8_t* Output, uint32_t Timeout);
static HAL_StatusTypeDef CRYP_ProcessData2Words(CRYP_HandleTypeDef *hcryp, uint8_t* Input, uint16_t Ilength, uint8_t* Output, uint32_t Timeout);
static void CRYP_DMAInCplt(DMA_HandleTypeDef *hdma);
static void CRYP_DMAOutCplt(DMA_HandleTypeDef *hdma);
static void CRYP_DMAError(DMA_HandleTypeDef *hdma);
static void CRYP_SetDMAConfig(CRYP_HandleTypeDef *hcryp, uint32_t inputaddr, uint16_t Size, uint32_t outputaddr);
static void CRYP_SetTDESECBMode(CRYP_HandleTypeDef *hcryp, uint32_t Direction);
static void CRYP_SetTDESCBCMode(CRYP_HandleTypeDef *hcryp, uint32_t Direction);
static void CRYP_SetDESECBMode(CRYP_HandleTypeDef *hcryp, uint32_t Direction);
static void CRYP_SetDESCBCMode(CRYP_HandleTypeDef *hcryp, uint32_t Direction);
/**
  * @}
  */


/* Private functions ---------------------------------------------------------*/

/** @addtogroup CRYP_Private_Functions
  * @{
  */


/**
  * @brief  DMA CRYP Input Data process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void CRYP_DMAInCplt(DMA_HandleTypeDef *hdma)
{
  CRYP_HandleTypeDef* hcryp = (CRYP_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  /* Disable the DMA transfer for input FIFO request by resetting the DIEN bit
     in the DMACR register */
  hcryp->Instance->DMACR &= (uint32_t)(~CRYP_DMACR_DIEN);

  /* Call input data transfer complete callback */
  HAL_CRYP_InCpltCallback(hcryp);
}

/**
  * @brief  DMA CRYP Output Data process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void CRYP_DMAOutCplt(DMA_HandleTypeDef *hdma)
{
  CRYP_HandleTypeDef* hcryp = (CRYP_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  /* Disable the DMA transfer for output FIFO request by resetting the DOEN bit
     in the DMACR register */
  hcryp->Instance->DMACR &= (uint32_t)(~CRYP_DMACR_DOEN);

  /* Disable CRYP */
  __HAL_CRYP_DISABLE(hcryp);

  /* Change the CRYP state to ready */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Call output data transfer complete callback */
  HAL_CRYP_OutCpltCallback(hcryp);
}

/**
  * @brief  DMA CRYP communication error callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void CRYP_DMAError(DMA_HandleTypeDef *hdma)
{
  CRYP_HandleTypeDef* hcryp = (CRYP_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;
  hcryp->State= HAL_CRYP_STATE_READY;
  HAL_CRYP_ErrorCallback(hcryp);
}

/**
  * @brief  Writes the Key in Key registers.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  Key Pointer to Key buffer
  * @param  KeySize Size of Key
  * @retval None
  */
static void CRYP_SetKey(CRYP_HandleTypeDef *hcryp, uint8_t *Key, uint32_t KeySize)
{
  uint32_t keyaddr = (uint32_t)Key;

  switch(KeySize)
  {
  case CRYP_KEYSIZE_256B:
    /* Key Initialisation */
    hcryp->Instance->K0LR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K0RR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K1LR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K1RR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K2LR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K2RR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K3LR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K3RR = __REV(*(uint32_t*)(keyaddr));
    break;
  case CRYP_KEYSIZE_192B:
    hcryp->Instance->K1LR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K1RR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K2LR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K2RR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K3LR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K3RR = __REV(*(uint32_t*)(keyaddr));
    break;
  case CRYP_KEYSIZE_128B:
    hcryp->Instance->K2LR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K2RR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K3LR = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->K3RR = __REV(*(uint32_t*)(keyaddr));
    break;
  default:
    break;
  }
}

/**
  * @brief  Writes the InitVector/InitCounter in IV registers.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  InitVector Pointer to InitVector/InitCounter buffer
  * @param  IVSize Size of the InitVector/InitCounter
  * @retval None
  */
static void CRYP_SetInitVector(CRYP_HandleTypeDef *hcryp, uint8_t *InitVector, uint32_t IVSize)
{
  uint32_t ivaddr = (uint32_t)InitVector;

  switch(IVSize)
  {
  case CRYP_KEYSIZE_128B:
    hcryp->Instance->IV0LR = __REV(*(uint32_t*)(ivaddr));
    ivaddr+=4U;
    hcryp->Instance->IV0RR = __REV(*(uint32_t*)(ivaddr));
    ivaddr+=4U;
    hcryp->Instance->IV1LR = __REV(*(uint32_t*)(ivaddr));
    ivaddr+=4U;
    hcryp->Instance->IV1RR = __REV(*(uint32_t*)(ivaddr));
    break;
    /* Whatever key size 192 or 256, Init vector is written in IV0LR and IV0RR */
  case CRYP_KEYSIZE_192B:
    hcryp->Instance->IV0LR = __REV(*(uint32_t*)(ivaddr));
    ivaddr+=4U;
    hcryp->Instance->IV0RR = __REV(*(uint32_t*)(ivaddr));
    break;
  case CRYP_KEYSIZE_256B:
    hcryp->Instance->IV0LR = __REV(*(uint32_t*)(ivaddr));
    ivaddr+=4U;
    hcryp->Instance->IV0RR = __REV(*(uint32_t*)(ivaddr));
    break;
  default:
    break;
  }
}

/**
  * @brief  Process Data: Writes Input data in polling mode and read the output data
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  Input Pointer to the Input buffer
  * @param  Ilength Length of the Input buffer, must be a multiple of 16.
  * @param  Output Pointer to the returned buffer
  * @param  Timeout Timeout value
  * @retval None
  */
static HAL_StatusTypeDef CRYP_ProcessData(CRYP_HandleTypeDef *hcryp, uint8_t* Input, uint16_t Ilength, uint8_t* Output, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  uint32_t i = 0U;
  uint32_t inputaddr  = (uint32_t)Input;
  uint32_t outputaddr = (uint32_t)Output;

  for(i=0U; (i < Ilength); i+=16U)
  {
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR  = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;

    /* Get tick */
    tickstart = HAL_GetTick();

    while(HAL_IS_BIT_CLR(hcryp->Instance->SR, CRYP_FLAG_OFNE))
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }
    }
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
  }
  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Process Data: Write Input data in polling mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  Input Pointer to the Input buffer
  * @param  Ilength Length of the Input buffer, must be a multiple of 8
  * @param  Output Pointer to the returned buffer
  * @param  Timeout Specify Timeout value
  * @retval None
  */
static HAL_StatusTypeDef CRYP_ProcessData2Words(CRYP_HandleTypeDef *hcryp, uint8_t* Input, uint16_t Ilength, uint8_t* Output, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  uint32_t i = 0U;
  uint32_t inputaddr  = (uint32_t)Input;
  uint32_t outputaddr = (uint32_t)Output;

  for(i=0U; (i < Ilength); i+=8U)
  {
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR  = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;

    /* Get tick */
    tickstart = HAL_GetTick();

    while(HAL_IS_BIT_CLR(hcryp->Instance->SR, CRYP_FLAG_OFNE))
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }
    }
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
  }
  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Set the DMA configuration and start the DMA transfer
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  inputaddr address of the Input buffer
  * @param  Size Size of the Input buffer, must be a multiple of 16.
  * @param  outputaddr address of the Output buffer
  * @retval None
  */
static void CRYP_SetDMAConfig(CRYP_HandleTypeDef *hcryp, uint32_t inputaddr, uint16_t Size, uint32_t outputaddr)
{
  /* Set the CRYP DMA transfer complete callback */
  hcryp->hdmain->XferCpltCallback = CRYP_DMAInCplt;
  /* Set the DMA error callback */
  hcryp->hdmain->XferErrorCallback = CRYP_DMAError;

  /* Set the CRYP DMA transfer complete callback */
  hcryp->hdmaout->XferCpltCallback = CRYP_DMAOutCplt;
  /* Set the DMA error callback */
  hcryp->hdmaout->XferErrorCallback = CRYP_DMAError;

  /* Enable CRYP */
  __HAL_CRYP_ENABLE(hcryp);

  /* Enable the DMA In DMA Stream */
  HAL_DMA_Start_IT(hcryp->hdmain, inputaddr, (uint32_t)&hcryp->Instance->DR, Size/4U);

  /* Enable In DMA request */
  hcryp->Instance->DMACR = (CRYP_DMACR_DIEN);

  /* Enable the DMA Out DMA Stream */
  HAL_DMA_Start_IT(hcryp->hdmaout, (uint32_t)&hcryp->Instance->DOUT, outputaddr, Size/4U);

  /* Enable Out DMA request */
  hcryp->Instance->DMACR |= CRYP_DMACR_DOEN;

}

/**
  * @brief  Sets the CRYP peripheral in DES ECB mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  Direction Encryption or decryption
  * @retval None
  */
static void CRYP_SetDESECBMode(CRYP_HandleTypeDef *hcryp, uint32_t Direction)
{
  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the CRYP peripheral in AES ECB mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_DES_ECB | Direction);

    /* Set the key */
    hcryp->Instance->K1LR = __REV(*(uint32_t*)(hcryp->Init.pKey));
    hcryp->Instance->K1RR = __REV(*(uint32_t*)(hcryp->Init.pKey+4U));

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }
}

/**
  * @brief  Sets the CRYP peripheral in DES CBC mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  Direction Encryption or decryption
  * @retval None
  */
static void CRYP_SetDESCBCMode(CRYP_HandleTypeDef *hcryp, uint32_t Direction)
{
  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the CRYP peripheral in AES ECB mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_DES_CBC | Direction);

    /* Set the key */
    hcryp->Instance->K1LR = __REV(*(uint32_t*)(hcryp->Init.pKey));
    hcryp->Instance->K1RR = __REV(*(uint32_t*)(hcryp->Init.pKey+4U));

    /* Set the Initialization Vector */
    CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_256B);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }
}

/**
  * @brief  Sets the CRYP peripheral in TDES ECB mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  Direction Encryption or decryption
  * @retval None
  */
static void CRYP_SetTDESECBMode(CRYP_HandleTypeDef *hcryp, uint32_t Direction)
{
  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the CRYP peripheral in AES ECB mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_TDES_ECB | Direction);

    /* Set the key */
    CRYP_SetKey(hcryp, hcryp->Init.pKey, CRYP_KEYSIZE_192B);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }
}

/**
  * @brief  Sets the CRYP peripheral in TDES CBC mode
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  Direction Encryption or decryption
  * @retval None
  */
static void CRYP_SetTDESCBCMode(CRYP_HandleTypeDef *hcryp, uint32_t Direction)
{
  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the CRYP peripheral in AES CBC mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_TDES_CBC | Direction);

    /* Set the key */
    CRYP_SetKey(hcryp, hcryp->Init.pKey, CRYP_KEYSIZE_192B);

    /* Set the Initialization Vector */
    CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_256B);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }
}

/**
  * @}
  */

 /* Exported functions --------------------------------------------------------*/
/** @addtogroup CRYP_Exported_Functions
  * @{
  */

/** @defgroup CRYP_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions.
 *
@verbatim
  ==============================================================================
              ##### Initialization and de-initialization functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the CRYP according to the specified parameters
          in the CRYP_InitTypeDef and creates the associated handle
      (+) DeInitialize the CRYP peripheral
      (+) Initialize the CRYP MSP
      (+) DeInitialize CRYP MSP

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the CRYP according to the specified
  *         parameters in the CRYP_InitTypeDef and creates the associated handle.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_Init(CRYP_HandleTypeDef *hcryp)
{
  /* Check the CRYP handle allocation */
  if(hcryp == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_CRYP_KEYSIZE(hcryp->Init.KeySize));
  assert_param(IS_CRYP_DATATYPE(hcryp->Init.DataType));

  if(hcryp->State == HAL_CRYP_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hcryp->Lock = HAL_UNLOCKED;
    /* Init the low level hardware */
    HAL_CRYP_MspInit(hcryp);
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Set the key size and data type*/
  CRYP->CR = (uint32_t) (hcryp->Init.KeySize | hcryp->Init.DataType);

  /* Reset CrypInCount and CrypOutCount */
  hcryp->CrypInCount = 0U;
  hcryp->CrypOutCount = 0U;

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Set the default CRYP phase */
  hcryp->Phase = HAL_CRYP_PHASE_READY;

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  DeInitializes the CRYP peripheral.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DeInit(CRYP_HandleTypeDef *hcryp)
{
  /* Check the CRYP handle allocation */
  if(hcryp == NULL)
  {
    return HAL_ERROR;
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Set the default CRYP phase */
  hcryp->Phase = HAL_CRYP_PHASE_READY;

  /* Reset CrypInCount and CrypOutCount */
  hcryp->CrypInCount = 0U;
  hcryp->CrypOutCount = 0U;

  /* Disable the CRYP Peripheral Clock */
  __HAL_CRYP_DISABLE(hcryp);

  /* DeInit the low level hardware: CLOCK, NVIC.*/
  HAL_CRYP_MspDeInit(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_RESET;

  /* Release Lock */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP MSP.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
__weak void HAL_CRYP_MspInit(CRYP_HandleTypeDef *hcryp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcryp);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CRYP_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes CRYP MSP.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
__weak void HAL_CRYP_MspDeInit(CRYP_HandleTypeDef *hcryp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcryp);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CRYP_MspDeInit could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup CRYP_Exported_Functions_Group2 AES processing functions
 *  @brief   processing functions.
 *
@verbatim
  ==============================================================================
                      ##### AES processing functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Encrypt plaintext using AES-128/192/256 using chaining modes
      (+) Decrypt cyphertext using AES-128/192/256 using chaining modes
    [..]  Three processing functions are available:
      (+) Polling mode
      (+) Interrupt mode
      (+) DMA mode

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the CRYP peripheral in AES ECB encryption mode
  *         then encrypt pPlainData. The cypher data are available in pCypherData
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the key */
    CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES ECB mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_ECB);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }

    /* Write Plain Data and Get Cypher Data */
    if(CRYP_ProcessData(hcryp, pPlainData, Size, pCypherData, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES CBC encryption mode
  *         then encrypt pPlainData. The cypher data are available in pCypherData
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the key */
    CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES ECB mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CBC);

    /* Set the Initialization Vector */
    CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }

    /* Write Plain Data and Get Cypher Data */
    if(CRYP_ProcessData(hcryp,pPlainData, Size, pCypherData, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES CTR encryption mode
  *         then encrypt pPlainData. The cypher data are available in pCypherData
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the key */
    CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES ECB mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CTR);

    /* Set the Initialization Vector */
    CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }

    /* Write Plain Data and Get Cypher Data */
    if(CRYP_ProcessData(hcryp, pPlainData, Size, pCypherData, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}



/**
  * @brief  Initializes the CRYP peripheral in AES ECB decryption mode
  *         then decrypted pCypherData. The cypher data are available in pPlainData
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
   uint32_t tickstart = 0U;

  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the key */
    CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES Key mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_KEY | CRYP_CR_ALGODIR);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Get tick */
    tickstart = HAL_GetTick();

    while(HAL_IS_BIT_SET(hcryp->Instance->SR, CRYP_FLAG_BUSY))
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }
    }

    /* Disable CRYP */
    __HAL_CRYP_DISABLE(hcryp);

    /* Reset the ALGOMODE bits*/
    CRYP->CR &= (uint32_t)(~CRYP_CR_ALGOMODE);

    /* Set the CRYP peripheral in AES ECB decryption mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_ECB | CRYP_CR_ALGODIR);
    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }

    /* Write Plain Data and Get Cypher Data */
    if(CRYP_ProcessData(hcryp, pCypherData, Size, pPlainData, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES ECB decryption mode
  *         then decrypted pCypherData. The cypher data are available in pPlainData
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the key */
    CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES Key mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_KEY | CRYP_CR_ALGODIR);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Get tick */
    tickstart = HAL_GetTick();

    while(HAL_IS_BIT_SET(hcryp->Instance->SR, CRYP_FLAG_BUSY))
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }
    }

    /* Reset the ALGOMODE bits*/
    CRYP->CR &= (uint32_t)(~CRYP_CR_ALGOMODE);

    /* Set the CRYP peripheral in AES CBC decryption mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CBC | CRYP_CR_ALGODIR);

    /* Set the Initialization Vector */
    CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }

    /* Write Plain Data and Get Cypher Data */
    if(CRYP_ProcessData(hcryp, pCypherData, Size, pPlainData, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES CTR decryption mode
  *         then decrypted pCypherData. The cypher data are available in pPlainData
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set the key */
    CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES CTR mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CTR | CRYP_CR_ALGODIR);

    /* Set the Initialization Vector */
    CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }

    /* Write Plain Data and Get Cypher Data */
    if(CRYP_ProcessData(hcryp, pCypherData, Size, pPlainData, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES ECB encryption mode using Interrupt.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16 bytes
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pPlainData;
    hcryp->pCrypOutBuffPtr = pCypherData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES ECB mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_ECB);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

     /* Set the phase */
     hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR  = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    hcryp->pCrypInBuffPtr += 16U;
    hcryp->CrypInCount -= 16U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    hcryp->pCrypOutBuffPtr += 16U;
    hcryp->CrypOutCount -= 16U;
    if(hcryp->CrypOutCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Process Locked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES CBC encryption mode using Interrupt.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16 bytes
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pPlainData;
    hcryp->pCrypOutBuffPtr = pCypherData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES CBC mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CBC);

      /* Set the Initialization Vector */
      CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

     /* Set the phase */
     hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }
    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR  = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    hcryp->pCrypInBuffPtr += 16U;
    hcryp->CrypInCount -= 16U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    hcryp->pCrypOutBuffPtr += 16U;
    hcryp->CrypOutCount -= 16U;
    if(hcryp->CrypOutCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Process Locked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES CTR encryption mode using Interrupt.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16 bytes
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pPlainData;
    hcryp->pCrypOutBuffPtr = pCypherData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES CTR mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CTR);

      /* Set the Initialization Vector */
      CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

     /* Set the phase */
     hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }
    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR  = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    hcryp->pCrypInBuffPtr += 16U;
    hcryp->CrypInCount -= 16U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    hcryp->pCrypOutBuffPtr += 16U;
    hcryp->CrypOutCount -= 16U;
    if(hcryp->CrypOutCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}


/**
  * @brief  Initializes the CRYP peripheral in AES ECB decryption mode using Interrupt.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t tickstart = 0U;

  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pCypherData;
    hcryp->pCrypOutBuffPtr = pPlainData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the key */
    CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES Key mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_KEY | CRYP_CR_ALGODIR);
    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Get tick */
    tickstart = HAL_GetTick();

    while(HAL_IS_BIT_SET(hcryp->Instance->SR, CRYP_FLAG_BUSY))
    {
      /* Check for the Timeout */
      if((HAL_GetTick() - tickstart ) > CRYP_TIMEOUT_VALUE)
      {
        /* Change state */
        hcryp->State = HAL_CRYP_STATE_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hcryp);

        return HAL_TIMEOUT;
      }
    }

    /* Reset the ALGOMODE bits*/
    CRYP->CR &= (uint32_t)(~CRYP_CR_ALGOMODE);

    /* Set the CRYP peripheral in AES ECB decryption mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_ECB | CRYP_CR_ALGODIR);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

     /* Set the phase */
     hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR  = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    hcryp->pCrypInBuffPtr += 16U;
    hcryp->CrypInCount -= 16U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    hcryp->pCrypOutBuffPtr += 16U;
    hcryp->CrypOutCount -= 16U;
    if(hcryp->CrypOutCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES CBC decryption mode using IT.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{

  uint32_t tickstart = 0U;
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    /* Get the buffer addresses and sizes */
    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pCypherData;
    hcryp->pCrypOutBuffPtr = pPlainData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES Key mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_KEY | CRYP_CR_ALGODIR);

      /* Enable CRYP */
      __HAL_CRYP_ENABLE(hcryp);

    /* Get tick */
    tickstart = HAL_GetTick();

    while(HAL_IS_BIT_SET(hcryp->Instance->SR, CRYP_FLAG_BUSY))
    {
      /* Check for the Timeout */
      if((HAL_GetTick() - tickstart ) > CRYP_TIMEOUT_VALUE)
      {
        /* Change state */
        hcryp->State = HAL_CRYP_STATE_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hcryp);

        return HAL_TIMEOUT;
      }
    }

      /* Reset the ALGOMODE bits*/
      CRYP->CR &= (uint32_t)(~CRYP_CR_ALGOMODE);

      /* Set the CRYP peripheral in AES CBC decryption mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CBC | CRYP_CR_ALGODIR);

      /* Set the Initialization Vector */
      CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Enable CRYP */
      __HAL_CRYP_ENABLE(hcryp);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR  = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    hcryp->pCrypInBuffPtr += 16U;
    hcryp->CrypInCount -= 16U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    hcryp->pCrypOutBuffPtr += 16U;
    hcryp->CrypOutCount -= 16U;
    if(hcryp->CrypOutCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES CTR decryption mode using Interrupt.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    /* Get the buffer addresses and sizes */
    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pCypherData;
    hcryp->pCrypOutBuffPtr = pPlainData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES CTR mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CTR | CRYP_CR_ALGODIR);

      /* Set the Initialization Vector */
      CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR  = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    hcryp->pCrypInBuffPtr += 16U;
    hcryp->CrypInCount -= 16U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    hcryp->pCrypOutBuffPtr += 16U;
    hcryp->CrypOutCount -= 16U;
    if(hcryp->CrypOutCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES ECB encryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16 bytes
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pPlainData;
    outputaddr = (uint32_t)pCypherData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES ECB mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_ECB);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

     /* Set the phase */
     hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }
    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Initializes the CRYP peripheral in AES CBC encryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pPlainData;
    outputaddr = (uint32_t)pCypherData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES ECB mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CBC);

      /* Set the Initialization Vector */
      CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

       /* Set the phase */
       hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
     }
     /* Set the input and output addresses and start DMA transfer */
     CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

     /* Process Unlocked */
     __HAL_UNLOCK(hcryp);

     /* Return function status */
     return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Initializes the CRYP peripheral in AES CTR encryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pPlainData;
    outputaddr = (uint32_t)pCypherData;

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES ECB mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CTR);

      /* Set the Initialization Vector */
      CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

       /* Set the phase */
       hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Initializes the CRYP peripheral in AES ECB decryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16 bytes
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t tickstart = 0U;
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pCypherData;
    outputaddr = (uint32_t)pPlainData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
    /* Set the key */
    CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES Key mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_KEY | CRYP_CR_ALGODIR);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Get tick */
    tickstart = HAL_GetTick();

    while(HAL_IS_BIT_SET(hcryp->Instance->SR, CRYP_FLAG_BUSY))
    {
      /* Check for the Timeout */
      if((HAL_GetTick() - tickstart ) > CRYP_TIMEOUT_VALUE)
      {
        /* Change state */
        hcryp->State = HAL_CRYP_STATE_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hcryp);

        return HAL_TIMEOUT;
      }
    }

    /* Reset the ALGOMODE bits*/
    CRYP->CR &= (uint32_t)(~CRYP_CR_ALGOMODE);

    /* Set the CRYP peripheral in AES ECB decryption mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_ECB | CRYP_CR_ALGODIR);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

     /* Set the phase */
     hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

     /* Process Unlocked */
     __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Initializes the CRYP peripheral in AES CBC encryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16 bytes
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t tickstart = 0U;
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pCypherData;
    outputaddr = (uint32_t)pPlainData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES Key mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_KEY | CRYP_CR_ALGODIR);

      /* Enable CRYP */
      __HAL_CRYP_ENABLE(hcryp);

      /* Get tick */
      tickstart = HAL_GetTick();

      while(HAL_IS_BIT_SET(hcryp->Instance->SR, CRYP_FLAG_BUSY))
      {
        /* Check for the Timeout */
        if((HAL_GetTick() - tickstart ) > CRYP_TIMEOUT_VALUE)
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }

      /* Reset the ALGOMODE bits*/
      CRYP->CR &= (uint32_t)(~CRYP_CR_ALGOMODE);

      /* Set the CRYP peripheral in AES CBC decryption mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CBC | CRYP_CR_ALGODIR);

      /* Set the Initialization Vector */
      CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Initializes the CRYP peripheral in AES CTR decryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pCypherData;
    outputaddr = (uint32_t)pPlainData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYP_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES CTR mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CTR | CRYP_CR_ALGODIR);

      /* Set the Initialization Vector */
      CRYP_SetInitVector(hcryp, hcryp->Init.pInitVect, CRYP_KEYSIZE_128B);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}


/**
  * @}
  */

/** @defgroup CRYP_Exported_Functions_Group3 DES processing functions
 *  @brief   processing functions.
 *
@verbatim
  ==============================================================================
                      ##### DES processing functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Encrypt plaintext using DES using ECB or CBC chaining modes
      (+) Decrypt cyphertext using ECB or CBC chaining modes
    [..]  Three processing functions are available:
      (+) Polling mode
      (+) Interrupt mode
      (+) DMA mode

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the CRYP peripheral in DES ECB encryption mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESECB_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Set CRYP peripheral in DES ECB encryption mode */
  CRYP_SetDESECBMode(hcryp, 0U);

  /* Enable CRYP */
  __HAL_CRYP_ENABLE(hcryp);

  /* Write Plain Data and Get Cypher Data */
  if(CRYP_ProcessData2Words(hcryp, pPlainData, Size, pCypherData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in DES ECB decryption mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESECB_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Set CRYP peripheral in DES ECB decryption mode */
  CRYP_SetDESECBMode(hcryp, CRYP_CR_ALGODIR);

  /* Enable CRYP */
  __HAL_CRYP_ENABLE(hcryp);

  /* Write Plain Data and Get Cypher Data */
  if(CRYP_ProcessData2Words(hcryp, pCypherData, Size, pPlainData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in DES CBC encryption mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESCBC_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Set CRYP peripheral in DES CBC encryption mode */
  CRYP_SetDESCBCMode(hcryp, 0U);

  /* Enable CRYP */
  __HAL_CRYP_ENABLE(hcryp);

  /* Write Plain Data and Get Cypher Data */
  if(CRYP_ProcessData2Words(hcryp, pPlainData, Size, pCypherData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in DES ECB decryption mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESCBC_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Set CRYP peripheral in DES CBC decryption mode */
  CRYP_SetDESCBCMode(hcryp, CRYP_CR_ALGODIR);

  /* Enable CRYP */
  __HAL_CRYP_ENABLE(hcryp);

  /* Write Plain Data and Get Cypher Data */
  if(CRYP_ProcessData2Words(hcryp, pCypherData, Size, pPlainData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in DES ECB encryption mode using IT.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESECB_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pPlainData;
    hcryp->pCrypOutBuffPtr = pCypherData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in DES ECB encryption mode */
    CRYP_SetDESECBMode(hcryp, 0U);

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);

    hcryp->pCrypInBuffPtr += 8U;
    hcryp->CrypInCount -= 8U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;

    hcryp->pCrypOutBuffPtr += 8U;
    hcryp->CrypOutCount -= 8U;
    if(hcryp->CrypOutCount == 0U)
    {
      /* Disable IT */
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Disable CRYP */
      __HAL_CRYP_DISABLE(hcryp);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in DES CBC encryption mode using interrupt.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESCBC_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pPlainData;
    hcryp->pCrypOutBuffPtr = pCypherData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in DES CBC encryption mode */
    CRYP_SetDESCBCMode(hcryp, 0U);

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }

  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);

    hcryp->pCrypInBuffPtr += 8U;
    hcryp->CrypInCount -= 8U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;

    hcryp->pCrypOutBuffPtr += 8U;
    hcryp->CrypOutCount -= 8U;
    if(hcryp->CrypOutCount == 0U)
    {
      /* Disable IT */
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Disable CRYP */
      __HAL_CRYP_DISABLE(hcryp);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in DES ECB decryption mode using IT.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESECB_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pCypherData;
    hcryp->pCrypOutBuffPtr = pPlainData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in DES ECB decryption mode */
    CRYP_SetDESECBMode(hcryp, CRYP_CR_ALGODIR);

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);

    hcryp->pCrypInBuffPtr += 8U;
    hcryp->CrypInCount -= 8U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;

    hcryp->pCrypOutBuffPtr += 8U;
    hcryp->CrypOutCount -= 8U;
    if(hcryp->CrypOutCount == 0U)
    {
      /* Disable IT */
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Disable CRYP */
      __HAL_CRYP_DISABLE(hcryp);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in DES ECB decryption mode using interrupt.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESCBC_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pCypherData;
    hcryp->pCrypOutBuffPtr = pPlainData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in DES CBC decryption mode */
    CRYP_SetDESCBCMode(hcryp, CRYP_CR_ALGODIR);

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);

    hcryp->pCrypInBuffPtr += 8U;
    hcryp->CrypInCount -= 8U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;

    hcryp->pCrypOutBuffPtr += 8U;
    hcryp->CrypOutCount -= 8U;
    if(hcryp->CrypOutCount == 0U)
    {
      /* Disable IT */
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Disable CRYP */
      __HAL_CRYP_DISABLE(hcryp);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in DES ECB encryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESECB_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pPlainData;
    outputaddr = (uint32_t)pCypherData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in DES ECB encryption mode */
    CRYP_SetDESECBMode(hcryp, 0U);

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Initializes the CRYP peripheral in DES CBC encryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESCBC_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pPlainData;
    outputaddr = (uint32_t)pCypherData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in DES CBC encryption mode */
    CRYP_SetDESCBCMode(hcryp, 0U);

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Initializes the CRYP peripheral in DES ECB decryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESECB_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pCypherData;
    outputaddr = (uint32_t)pPlainData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in DES ECB decryption mode */
    CRYP_SetDESECBMode(hcryp, CRYP_CR_ALGODIR);

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Initializes the CRYP peripheral in DES ECB decryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DESCBC_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pCypherData;
    outputaddr = (uint32_t)pPlainData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in DES CBC decryption mode */
    CRYP_SetDESCBCMode(hcryp, CRYP_CR_ALGODIR);

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @}
  */

/** @defgroup CRYP_Exported_Functions_Group4 TDES processing functions
 *  @brief   processing functions.
 *
@verbatim
  ==============================================================================
                      ##### TDES processing functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Encrypt plaintext using TDES based on ECB or CBC chaining modes
      (+) Decrypt cyphertext using TDES based on ECB or CBC chaining modes
    [..]  Three processing functions are available:
      (+) Polling mode
      (+) Interrupt mode
      (+) DMA mode

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the CRYP peripheral in TDES ECB encryption mode
  *         then encrypt pPlainData. The cypher data are available in pCypherData
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESECB_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Set CRYP peripheral in TDES ECB encryption mode */
  CRYP_SetTDESECBMode(hcryp, 0U);

  /* Enable CRYP */
  __HAL_CRYP_ENABLE(hcryp);

  /* Write Plain Data and Get Cypher Data */
  if(CRYP_ProcessData2Words(hcryp, pPlainData, Size, pCypherData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in TDES ECB decryption mode
  *         then decrypted pCypherData. The cypher data are available in pPlainData
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESECB_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Set CRYP peripheral in TDES ECB decryption mode */
  CRYP_SetTDESECBMode(hcryp, CRYP_CR_ALGODIR);

  /* Enable CRYP */
  __HAL_CRYP_ENABLE(hcryp);

  /* Write Cypher Data and Get Plain Data */
  if(CRYP_ProcessData2Words(hcryp, pCypherData, Size, pPlainData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in TDES CBC encryption mode
  *         then encrypt pPlainData. The cypher data are available in pCypherData
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Set CRYP peripheral in TDES CBC encryption mode */
  CRYP_SetTDESCBCMode(hcryp, 0U);

  /* Enable CRYP */
  __HAL_CRYP_ENABLE(hcryp);

  /* Write Plain Data and Get Cypher Data */
  if(CRYP_ProcessData2Words(hcryp, pPlainData, Size, pCypherData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in TDES CBC decryption mode
  *         then decrypted pCypherData. The cypher data are available in pPlainData
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Set CRYP peripheral in TDES CBC decryption mode */
  CRYP_SetTDESCBCMode(hcryp, CRYP_CR_ALGODIR);

  /* Enable CRYP */
  __HAL_CRYP_ENABLE(hcryp);

  /* Write Cypher Data and Get Plain Data */
  if(CRYP_ProcessData2Words(hcryp, pCypherData, Size, pPlainData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in TDES ECB encryption mode using interrupt.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESECB_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pPlainData;
    hcryp->pCrypOutBuffPtr = pCypherData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in TDES ECB encryption mode */
    CRYP_SetTDESECBMode(hcryp, 0U);

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);

    hcryp->pCrypInBuffPtr += 8U;
    hcryp->CrypInCount -= 8U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;

    hcryp->pCrypOutBuffPtr += 8U;
    hcryp->CrypOutCount -= 8U;
    if(hcryp->CrypOutCount == 0U)
    {
      /* Disable IT */
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Disable CRYP */
      __HAL_CRYP_DISABLE(hcryp);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call the Output data transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in TDES CBC encryption mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pPlainData;
    hcryp->pCrypOutBuffPtr = pCypherData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in TDES CBC encryption mode */
    CRYP_SetTDESCBCMode(hcryp, 0U);

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);

    hcryp->pCrypInBuffPtr += 8U;
    hcryp->CrypInCount -= 8U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;

    hcryp->pCrypOutBuffPtr += 8U;
    hcryp->CrypOutCount -= 8U;
    if(hcryp->CrypOutCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Disable CRYP */
      __HAL_CRYP_DISABLE(hcryp);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in TDES ECB decryption mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESECB_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pCypherData;
    hcryp->pCrypOutBuffPtr = pPlainData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in TDES ECB decryption mode */
    CRYP_SetTDESECBMode(hcryp, CRYP_CR_ALGODIR);

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);

    hcryp->pCrypInBuffPtr += 8U;
    hcryp->CrypInCount -= 8U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;

    hcryp->pCrypOutBuffPtr += 8U;
    hcryp->CrypOutCount -= 8U;
    if(hcryp->CrypOutCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Disable CRYP */
      __HAL_CRYP_DISABLE(hcryp);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in TDES CBC decryption mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pCypherData;
    hcryp->pCrypOutBuffPtr = pPlainData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in TDES CBC decryption mode */
    CRYP_SetTDESCBCMode(hcryp, CRYP_CR_ALGODIR);

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable CRYP */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
  {
    inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
    /* Write the Input block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(inputaddr);

    hcryp->pCrypInBuffPtr += 8U;
    hcryp->CrypInCount -= 8U;
    if(hcryp->CrypInCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_INI);
      /* Call the Input data transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if(__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
  {
    outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;
    /* Read the Output block from the Output FIFO */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUT;

    hcryp->pCrypOutBuffPtr += 8U;
    hcryp->CrypOutCount -= 8U;
    if(hcryp->CrypOutCount == 0U)
    {
      __HAL_CRYP_DISABLE_IT(hcryp, CRYP_IT_OUTI);
      /* Disable CRYP */
      __HAL_CRYP_DISABLE(hcryp);
      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in TDES ECB encryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESECB_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pPlainData;
    outputaddr = (uint32_t)pCypherData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in TDES ECB encryption mode */
    CRYP_SetTDESECBMode(hcryp, 0U);

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Initializes the CRYP peripheral in TDES CBC encryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pPlainData;
    outputaddr = (uint32_t)pCypherData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in TDES CBC encryption mode */
    CRYP_SetTDESCBCMode(hcryp, 0U);

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Initializes the CRYP peripheral in TDES ECB decryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESECB_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pCypherData;
    outputaddr = (uint32_t)pPlainData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in TDES ECB decryption mode */
    CRYP_SetTDESECBMode(hcryp, CRYP_CR_ALGODIR);

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Initializes the CRYP peripheral in TDES CBC decryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 8
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pCypherData;
    outputaddr = (uint32_t)pPlainData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set CRYP peripheral in TDES CBC decryption mode */
    CRYP_SetTDESCBCMode(hcryp, CRYP_CR_ALGODIR);

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @}
  */

/** @defgroup CRYP_Exported_Functions_Group5 DMA callback functions
 *  @brief   DMA callback functions.
 *
@verbatim
  ==============================================================================
                      ##### DMA callback functions  #####
  ==============================================================================
    [..]  This section provides DMA callback functions:
      (+) DMA Input data transfer complete
      (+) DMA Output data transfer complete
      (+) DMA error

@endverbatim
  * @{
  */

/**
  * @brief  Input FIFO transfer completed callbacks.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
__weak void HAL_CRYP_InCpltCallback(CRYP_HandleTypeDef *hcryp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcryp);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CRYP_InCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Output FIFO transfer completed callbacks.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
__weak void HAL_CRYP_OutCpltCallback(CRYP_HandleTypeDef *hcryp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcryp);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CRYP_OutCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  CRYP error callbacks.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
 __weak void HAL_CRYP_ErrorCallback(CRYP_HandleTypeDef *hcryp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcryp);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CRYP_ErrorCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup CRYP_Exported_Functions_Group6 CRYP IRQ handler management
 *  @brief   CRYP IRQ handler.
 *
@verbatim
  ==============================================================================
                ##### CRYP IRQ handler management #####
  ==============================================================================
[..]  This section provides CRYP IRQ handler function.

@endverbatim
  * @{
  */

/**
  * @brief  This function handles CRYP interrupt request.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
void HAL_CRYP_IRQHandler(CRYP_HandleTypeDef *hcryp)
{
  switch(CRYP->CR & CRYP_CR_ALGOMODE_DIRECTION)
  {
  case CRYP_CR_ALGOMODE_TDES_ECB_ENCRYPT:
    HAL_CRYP_TDESECB_Encrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_TDES_ECB_DECRYPT:
    HAL_CRYP_TDESECB_Decrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_TDES_CBC_ENCRYPT:
    HAL_CRYP_TDESCBC_Encrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_TDES_CBC_DECRYPT:
    HAL_CRYP_TDESCBC_Decrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_DES_ECB_ENCRYPT:
    HAL_CRYP_DESECB_Encrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_DES_ECB_DECRYPT:
    HAL_CRYP_DESECB_Decrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_DES_CBC_ENCRYPT:
    HAL_CRYP_DESCBC_Encrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_DES_CBC_DECRYPT:
    HAL_CRYP_DESCBC_Decrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_AES_ECB_ENCRYPT:
    HAL_CRYP_AESECB_Encrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_AES_ECB_DECRYPT:
    HAL_CRYP_AESECB_Decrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_AES_CBC_ENCRYPT:
    HAL_CRYP_AESCBC_Encrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_AES_CBC_DECRYPT:
    HAL_CRYP_AESCBC_Decrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_AES_CTR_ENCRYPT:
    HAL_CRYP_AESCTR_Encrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_AES_CTR_DECRYPT:
    HAL_CRYP_AESCTR_Decrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  default:
    break;
  }
}

/**
  * @}
  */

/** @defgroup CRYP_Exported_Functions_Group7 Peripheral State functions
 *  @brief   Peripheral State functions.
 *
@verbatim
  ==============================================================================
                      ##### Peripheral State functions #####
  ==============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the CRYP state.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval HAL state
  */
HAL_CRYP_STATETypeDef HAL_CRYP_GetState(CRYP_HandleTypeDef *hcryp)
{
  return hcryp->State;
}

/**
  * @}
  */


/**
  * @}
  */

#endif /* CRYP */

#if defined (AES)

/** @defgroup AES AES
  * @brief AES HAL module driver.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions --------------------------------------------------------*/

/** @defgroup CRYP_Private_Functions CRYP Private Functions
  * @{
  */

static HAL_StatusTypeDef CRYP_SetInitVector(CRYP_HandleTypeDef *hcryp);
static HAL_StatusTypeDef CRYP_SetKey(CRYP_HandleTypeDef *hcryp);
static HAL_StatusTypeDef CRYP_AES_IT(CRYP_HandleTypeDef *hcryp);

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup CRYP_Exported_Functions CRYP Exported Functions
  * @{
  */

/** @defgroup CRYP_Exported_Functions_Group1 Initialization and deinitialization functions
 *  @brief    Initialization and Configuration functions.
 *
@verbatim
  ==============================================================================
              ##### Initialization and deinitialization functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the CRYP according to the specified parameters
          in the CRYP_InitTypeDef and creates the associated handle
      (+) DeInitialize the CRYP peripheral
      (+) Initialize the CRYP MSP (MCU Specific Package)
      (+) De-Initialize the CRYP MSP

    [..]
    (@) Specific care must be taken to format the key and the Initialization Vector IV!

   [..] If the key is defined as a 128-bit long array key[127..0] = {b127 ... b0} where
        b127 is the MSB and b0 the LSB, the key must be stored in MCU memory
        (+) as a sequence of words where the MSB word comes first (occupies the
          lowest memory address)
        (+) where each word is byte-swapped:
         (++)   address n+0 : 0b b103 .. b96 b111 .. b104 b119 .. b112 b127 .. b120
         (++)   address n+4 : 0b b71 .. b64 b79 .. b72 b87 .. b80 b95 .. b88
         (++)   address n+8 : 0b b39 .. b32 b47 .. b40 b55 .. b48 b63 .. b56
         (++)   address n+C : 0b b7 .. b0 b15 .. b8 b23 .. b16 b31 .. b24
    [..] Hereafter, another illustration when considering a 128-bit long key made of 16 bytes {B15..B0}.
        The 4 32-bit words that make the key must be stored as follows in MCU memory:
         (+)    address n+0 : 0x B12 B13 B14 B15
         (+)    address n+4 : 0x B8 B9 B10 B11
         (+)    address n+8 : 0x B4 B5 B6 B7
         (+)    address n+C : 0x B0 B1 B2 B3
    [..]  which leads to the expected setting
      (+)       AES_KEYR3 = 0x B15 B14 B13 B12
      (+)       AES_KEYR2 = 0x B11 B10 B9 B8
      (+)       AES_KEYR1 = 0x B7 B6 B5 B4
      (+)       AES_KEYR0 = 0x B3 B2 B1 B0

   [..]  Same format must be applied for a 256-bit long key made of 32 bytes {B31..B0}.
         The 8 32-bit words that make the key must be stored as follows in MCU memory:
         (+)    address n+00 : 0x B28 B29 B30 B31
         (+)    address n+04 : 0x B24 B25 B26 B27
         (+)    address n+08 : 0x B20 B21 B22 B23
         (+)    address n+0C : 0x B16 B17 B18 B19
         (+)    address n+10 : 0x B12 B13 B14 B15
         (+)    address n+14 : 0x B8 B9 B10 B11
         (+)    address n+18 : 0x B4 B5 B6 B7
         (+)    address n+1C : 0x B0 B1 B2 B3
    [..]  which leads to the expected setting
      (+)       AES_KEYR7 = 0x B31 B30 B29 B28
      (+)       AES_KEYR6 = 0x B27 B26 B25 B24
      (+)       AES_KEYR5 = 0x B23 B22 B21 B20
      (+)       AES_KEYR4 = 0x B19 B18 B17 B16
      (+)       AES_KEYR3 = 0x B15 B14 B13 B12
      (+)       AES_KEYR2 = 0x B11 B10 B9 B8
      (+)       AES_KEYR1 = 0x B7 B6 B5 B4
      (+)       AES_KEYR0 = 0x B3 B2 B1 B0

   [..] Initialization Vector IV (4 32-bit words) format must follow the same as
        that of a 128-bit long key.

  [..]

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the CRYP according to the specified
  *         parameters in the CRYP_InitTypeDef and initialize the associated handle.
  * @note Specific care must be taken to format the key and the Initialization Vector IV
  *       stored in the MCU memory before calling HAL_CRYP_Init(). Refer to explanations
  *       hereabove.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_Init(CRYP_HandleTypeDef *hcryp)
{
  /* Check the CRYP handle allocation */
  if(hcryp == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the instance */
  assert_param(IS_AES_ALL_INSTANCE(hcryp->Instance));

  /* Check the parameters */
  assert_param(IS_CRYP_KEYSIZE(hcryp->Init.KeySize));
  assert_param(IS_CRYP_DATATYPE(hcryp->Init.DataType));
  assert_param(IS_CRYP_ALGOMODE(hcryp->Init.OperatingMode));
  /* ChainingMode parameter is irrelevant when mode is set to Key derivation */
  if (hcryp->Init.OperatingMode != CRYP_ALGOMODE_KEYDERIVATION)
  {
    assert_param(IS_CRYP_CHAINMODE(hcryp->Init.ChainingMode));
  }
  assert_param(IS_CRYP_WRITE(hcryp->Init.KeyWriteFlag));

  /*========================================================*/
  /* Check the proper operating/chaining modes combinations */
  /*========================================================*/
  /* Check the proper chaining when the operating mode is key derivation and decryption */
#if defined(AES_CR_NPBLB)
  if ((hcryp->Init.OperatingMode == CRYP_ALGOMODE_KEYDERIVATION_DECRYPT) &&\
         ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CTR)           \
       || (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)      \
       || (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC)))
#else
  if ((hcryp->Init.OperatingMode == CRYP_ALGOMODE_KEYDERIVATION_DECRYPT) &&\
         ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CTR)           \
       || (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)      \
       || (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)))
#endif
  {
    return HAL_ERROR;
  }
  /* Check that key derivation is not set in CMAC mode or CCM mode when applicable */
#if defined(AES_CR_NPBLB)
  if ((hcryp->Init.OperatingMode == CRYP_ALGOMODE_KEYDERIVATION)
   && (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC))
#else
  if ((hcryp->Init.OperatingMode == CRYP_ALGOMODE_KEYDERIVATION)
   && (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC))
#endif
  {
    return HAL_ERROR;
  }


  /*================*/
  /* Initialization */
  /*================*/
  /* Initialization start */
  if(hcryp->State == HAL_CRYP_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hcryp->Lock = HAL_UNLOCKED;

    /* Init the low level hardware */
    HAL_CRYP_MspInit(hcryp);
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Disable the Peripheral */
  __HAL_CRYP_DISABLE();

  /*=============================================================*/
  /* AES initialization common to all operating modes            */
  /*=============================================================*/
  /* Set the Key size selection */
  MODIFY_REG(hcryp->Instance->CR, AES_CR_KEYSIZE, hcryp->Init.KeySize);

  /* Set the default CRYP phase when this parameter is not used.
     Phase is updated below in case of GCM/GMAC/CMAC(/CCM) setting. */
  hcryp->Phase = HAL_CRYP_PHASE_NOT_USED;



  /*=============================================================*/
  /* Carry on the initialization based on the AES operating mode */
  /*=============================================================*/
  /* Key derivation */
  if (hcryp->Init.OperatingMode == CRYP_ALGOMODE_KEYDERIVATION)
  {
    MODIFY_REG(hcryp->Instance->CR, AES_CR_MODE, CRYP_ALGOMODE_KEYDERIVATION);

    /* Configure the Key registers */
    if (CRYP_SetKey(hcryp) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  else
  /* Encryption / Decryption (with or without key derivation) / authentication */
  {
    /* Set data type, operating and chaining modes.
       In case of GCM or GMAC, data type is forced to 0b00 */
    if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)
    {
      MODIFY_REG(hcryp->Instance->CR, AES_CR_DATATYPE|AES_CR_MODE|AES_CR_CHMOD, hcryp->Init.OperatingMode|hcryp->Init.ChainingMode);
    }
    else
    {
      MODIFY_REG(hcryp->Instance->CR, AES_CR_DATATYPE|AES_CR_MODE|AES_CR_CHMOD, hcryp->Init.DataType|hcryp->Init.OperatingMode|hcryp->Init.ChainingMode);
    }


   /* Specify the encryption/decryption phase in case of Galois counter mode (GCM),
      Galois message authentication code (GMAC), cipher message authentication code (CMAC)
      or Counter with Cipher Mode (CCM) when applicable */
#if defined(AES_CR_NPBLB)
   if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)
    || (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC))
#else
   if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)
    || (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC))
#endif
    {
      MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, hcryp->Init.GCMCMACPhase);
      hcryp->Phase = HAL_CRYP_PHASE_START;
    }


    /* Configure the Key registers if no need to bypass this step */
    if (hcryp->Init.KeyWriteFlag == CRYP_KEY_WRITE_ENABLE)
    {
      if (CRYP_SetKey(hcryp) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }

    /* If applicable, configure the Initialization Vector */
    if (hcryp->Init.ChainingMode != CRYP_CHAINMODE_AES_ECB)
    {
      if (CRYP_SetInitVector(hcryp) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }
  }

#if defined(AES_CR_NPBLB)
  /* Clear NPBLB field */
  CLEAR_BIT(hcryp->Instance->CR, AES_CR_NPBLB);
#endif

  /* Reset CrypInCount and CrypOutCount */
  hcryp->CrypInCount = 0U;
  hcryp->CrypOutCount = 0U;

  /* Reset ErrorCode field */
  hcryp->ErrorCode = HAL_CRYP_ERROR_NONE;

  /* Reset Mode suspension request */
  hcryp->SuspendRequest = HAL_CRYP_SUSPEND_NONE;

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Enable the Peripheral */
  __HAL_CRYP_ENABLE();

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  DeInitialize the CRYP peripheral.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_DeInit(CRYP_HandleTypeDef *hcryp)
{
  /* Check the CRYP handle allocation */
  if(hcryp == NULL)
  {
    return HAL_ERROR;
  }

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Set the default CRYP phase */
  hcryp->Phase = HAL_CRYP_PHASE_READY;

  /* Reset CrypInCount and CrypOutCount */
  hcryp->CrypInCount = 0U;
  hcryp->CrypOutCount = 0U;

  /* Disable the CRYP Peripheral Clock */
  __HAL_CRYP_DISABLE();

  /* DeInit the low level hardware: CLOCK, NVIC.*/
  HAL_CRYP_MspDeInit(hcryp);

  /* Change the CRYP state */
  hcryp->State = HAL_CRYP_STATE_RESET;

  /* Release Lock */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initialize the CRYP MSP.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
__weak void HAL_CRYP_MspInit(CRYP_HandleTypeDef *hcryp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcryp);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_CRYP_MspInit can be implemented in the user file
   */
}

/**
  * @brief  DeInitialize CRYP MSP.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
__weak void HAL_CRYP_MspDeInit(CRYP_HandleTypeDef *hcryp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcryp);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_CRYP_MspDeInit can be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup CRYP_Exported_Functions_Group2 AES processing functions
 *  @brief   Processing functions.
 *
@verbatim
  ==============================================================================
                      ##### AES processing functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Encrypt plaintext using AES algorithm in different chaining modes
      (+) Decrypt cyphertext using AES algorithm in different chaining modes
    [..]  Three processing functions are available:
      (+) Polling mode
      (+) Interrupt mode
      (+) DMA mode

@endverbatim
  * @{
  */


/**
  * @brief  Encrypt pPlainData in AES ECB encryption mode. The cypher data are available in pCypherData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Specify Timeout value
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_ENCRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_ECB;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES(hcryp, pPlainData, Size, pCypherData, Timeout);
}


/**
  * @brief  Encrypt pPlainData in AES CBC encryption mode with key derivation. The cypher data are available in pCypherData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Specify Timeout value
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_ENCRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CBC;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES(hcryp, pPlainData, Size, pCypherData, Timeout);
}


/**
  * @brief  Encrypt pPlainData in AES CTR encryption mode. The cypher data are available in pCypherData
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Specify Timeout value
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_ENCRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CTR;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES(hcryp, pPlainData, Size, pCypherData, Timeout);
}

/**
  * @brief  Decrypt pCypherData in AES ECB decryption mode with key derivation,
  *         the decyphered data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Timeout Specify Timeout value
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_KEYDERIVATION_DECRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_ECB;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES(hcryp, pCypherData, Size, pPlainData, Timeout);
}

/**
  * @brief  Decrypt pCypherData in AES ECB decryption mode with key derivation,
  *         the decyphered data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Timeout Specify Timeout value
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_KEYDERIVATION_DECRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CBC;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES(hcryp, pCypherData, Size, pPlainData, Timeout);
}

/**
  * @brief  Decrypt pCypherData in AES CTR decryption mode,
  *         the decyphered data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Timeout Specify Timeout value
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_DECRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CTR;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES(hcryp, pCypherData, Size, pPlainData, Timeout);
}

/**
  * @brief  Encrypt pPlainData in AES ECB encryption mode using Interrupt,
  *         the cypher data are available in pCypherData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_IT() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_ENCRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_ECB;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_IT(hcryp, pPlainData, Size, pCypherData);
}

/**
  * @brief  Encrypt pPlainData in AES CBC encryption mode using Interrupt,
  *         the cypher data are available in pCypherData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_IT() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_ENCRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CBC;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_IT(hcryp, pPlainData, Size, pCypherData);
}


/**
  * @brief  Encrypt pPlainData in AES CTR encryption mode using Interrupt,
  *         the cypher data are available in pCypherData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_IT() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_ENCRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CTR;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_IT(hcryp, pPlainData, Size, pCypherData);
}

/**
  * @brief  Decrypt pCypherData in AES ECB decryption mode using Interrupt,
  *         the decyphered data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer.
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_IT() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_KEYDERIVATION_DECRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_ECB;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_IT(hcryp, pCypherData, Size, pPlainData);
}

/**
  * @brief  Decrypt pCypherData in AES CBC decryption mode using Interrupt,
  *         the decyphered data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_IT() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_KEYDERIVATION_DECRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CBC;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_IT(hcryp, pCypherData, Size, pPlainData);
}

/**
  * @brief  Decrypt pCypherData in AES CTR decryption mode using Interrupt,
  *         the decyphered data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_IT() API instead (usage recommended).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_DECRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CTR;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_IT(hcryp, pCypherData, Size, pPlainData);
}

/**
  * @brief  Encrypt pPlainData in AES ECB encryption mode using DMA,
  *         the cypher data are available in pCypherData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_DMA() API instead (usage recommended).
  * @note   pPlainData and pCypherData buffers must be 32-bit aligned to ensure a correct DMA transfer to and from the IP.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_ENCRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_ECB;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_DMA(hcryp, pPlainData, Size, pCypherData);
}



/**
  * @brief  Encrypt pPlainData in AES CBC encryption mode using DMA,
  *         the cypher data are available in pCypherData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_DMA() API instead (usage recommended).
  * @note   pPlainData and pCypherData buffers must be 32-bit aligned to ensure a correct DMA transfer to and from the IP.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_ENCRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CBC;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_DMA(hcryp, pPlainData, Size, pCypherData);
}

/**
  * @brief  Encrypt pPlainData in AES CTR encryption mode using DMA,
  *         the cypher data are available in pCypherData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pCypherData Pointer to the cyphertext buffer.
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_DMA() API instead (usage recommended).
  * @note   pPlainData and pCypherData buffers must be 32-bit aligned to ensure a correct DMA transfer to and from the IP.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_ENCRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CTR;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_DMA(hcryp, pPlainData, Size, pCypherData);
}

/**
  * @brief  Decrypt pCypherData in AES ECB decryption mode using DMA,
  *         the decyphered data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_DMA() API instead (usage recommended).
  * @note   pPlainData and pCypherData buffers must be 32-bit aligned to ensure a correct DMA transfer to and from the IP.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESECB_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_KEYDERIVATION_DECRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_ECB;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_DMA(hcryp, pCypherData, Size, pPlainData);
}

/**
  * @brief  Decrypt pCypherData in AES CBC decryption mode using DMA,
  *         the decyphered data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_DMA() API instead (usage recommended).
  * @note   pPlainData and pCypherData buffers must be 32-bit aligned to ensure a correct DMA transfer to and from the IP.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCBC_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_KEYDERIVATION_DECRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CBC;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_DMA(hcryp, pCypherData, Size, pPlainData);
}

/**
  * @brief  Decrypt pCypherData in AES CTR decryption mode using DMA,
  *         the decyphered data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer in bytes, must be a multiple of 16.
  * @param  pPlainData Pointer to the plaintext buffer
  * @note   This API is provided only to maintain compatibility with legacy software. Users should directly
  *         resort to generic HAL_CRYPEx_AES_DMA() API instead (usage recommended).
  * @note   pPlainData and pCypherData buffers must be 32-bit aligned to ensure a correct DMA transfer to and from the IP.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYP_AESCTR_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  /* Re-initialize AES IP with proper parameters */
  if (HAL_CRYP_DeInit(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }
  hcryp->Init.OperatingMode = CRYP_ALGOMODE_DECRYPT;
  hcryp->Init.ChainingMode = CRYP_CHAINMODE_AES_CTR;
  hcryp->Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  if (HAL_CRYP_Init(hcryp) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CRYPEx_AES_DMA(hcryp, pCypherData, Size, pPlainData);
}


/**
  * @}
  */

/** @defgroup CRYP_Exported_Functions_Group3 Callback functions
 *  @brief   Callback functions.
 *
@verbatim
  ==============================================================================
                      ##### Callback functions  #####
  ==============================================================================
    [..]  This section provides Interruption and DMA callback functions:
      (+) DMA Input data transfer complete
      (+) DMA Output data transfer complete
      (+) DMA or Interrupt error

@endverbatim
  * @{
  */

/**
  * @brief  CRYP error callback.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
__weak void HAL_CRYP_ErrorCallback(CRYP_HandleTypeDef *hcryp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcryp);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_CRYP_ErrorCallback can be implemented in the user file
   */
}

/**
  * @brief  Input DMA transfer complete callback.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
__weak void HAL_CRYP_InCpltCallback(CRYP_HandleTypeDef *hcryp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcryp);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_CRYP_InCpltCallback can be implemented in the user file
   */
}

/**
  * @brief  Output DMA transfer complete callback.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
__weak void HAL_CRYP_OutCpltCallback(CRYP_HandleTypeDef *hcryp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcryp);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_CRYP_OutCpltCallback can be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup CRYP_Exported_Functions_Group4 CRYP IRQ handler
 *  @brief   AES IRQ handler.
 *
@verbatim
  ==============================================================================
                ##### AES IRQ handler management #####
  ==============================================================================
[..]  This section provides AES IRQ handler function.

@endverbatim
  * @{
  */

/**
  * @brief  Handle AES interrupt request.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
void HAL_CRYP_IRQHandler(CRYP_HandleTypeDef *hcryp)
{
  /* Check if error occurred */
  if (__HAL_CRYP_GET_IT_SOURCE(CRYP_IT_ERRIE) != RESET)
  {
    /* If Write Error occurred */
    if (__HAL_CRYP_GET_FLAG(CRYP_IT_WRERR) != RESET)
    {
      hcryp->ErrorCode |= HAL_CRYP_WRITE_ERROR;
      hcryp->State = HAL_CRYP_STATE_ERROR;
    }
    /* If Read Error occurred */
    if (__HAL_CRYP_GET_FLAG(CRYP_IT_RDERR) != RESET)
    {
      hcryp->ErrorCode |= HAL_CRYP_READ_ERROR;
      hcryp->State = HAL_CRYP_STATE_ERROR;
    }

    /* If an error has been reported */
    if (hcryp->State == HAL_CRYP_STATE_ERROR)
    {
      /* Disable Error and Computation Complete Interrupts */
      __HAL_CRYP_DISABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);
      /* Clear all Interrupt flags */
      __HAL_CRYP_CLEAR_FLAG(CRYP_ERR_CLEAR|CRYP_CCF_CLEAR);

      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);

      HAL_CRYP_ErrorCallback(hcryp);

      return;
    }
  }

  /* Check if computation complete interrupt is enabled
     and if the computation complete flag is raised */
  if((__HAL_CRYP_GET_FLAG(CRYP_IT_CCF) != RESET) && (__HAL_CRYP_GET_IT_SOURCE(CRYP_IT_CCFIE) != RESET))
  {
#if defined(AES_CR_NPBLB)
    if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)
     || (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC))
#else
    if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)
     || (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC))
#endif
    {
     /* To ensure proper suspension requests management, CCF flag
        is reset in CRYP_AES_Auth_IT() according to the current
        phase under handling */
      CRYP_AES_Auth_IT(hcryp);
    }
    else
    {
      /* Clear Computation Complete Flag */
      __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
      CRYP_AES_IT(hcryp);
    }
  }
}

/**
  * @}
  */

/** @defgroup CRYP_Exported_Functions_Group5 Peripheral State functions
 *  @brief   Peripheral State functions.
 *
@verbatim
  ==============================================================================
                      ##### Peripheral State functions #####
  ==============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Return the CRYP handle state.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval HAL state
  */
HAL_CRYP_STATETypeDef HAL_CRYP_GetState(CRYP_HandleTypeDef *hcryp)
{
  /* Return CRYP handle state */
  return hcryp->State;
}

/**
  * @brief  Return the CRYP peripheral error.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @note   The returned error is a bit-map combination of possible errors
  * @retval Error bit-map
  */
uint32_t HAL_CRYP_GetError(CRYP_HandleTypeDef *hcryp)
{
  return hcryp->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup CRYP_Private_Functions
  * @{
  */


/**
  * @brief  Write the Key in KeyRx registers.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
static HAL_StatusTypeDef  CRYP_SetKey(CRYP_HandleTypeDef *hcryp)
{
  uint32_t keyaddr = 0x0U;

  if ((uint32_t)(hcryp->Init.pKey == NULL))
  {
    return HAL_ERROR;
  }


  keyaddr = (uint32_t)(hcryp->Init.pKey);

  if (hcryp->Init.KeySize == CRYP_KEYSIZE_256B)
  {
    hcryp->Instance->KEYR7 = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->KEYR6 = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->KEYR5 = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
    hcryp->Instance->KEYR4 = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4U;
  }

  hcryp->Instance->KEYR3 = __REV(*(uint32_t*)(keyaddr));
  keyaddr+=4U;
  hcryp->Instance->KEYR2 = __REV(*(uint32_t*)(keyaddr));
  keyaddr+=4U;
  hcryp->Instance->KEYR1 = __REV(*(uint32_t*)(keyaddr));
  keyaddr+=4U;
  hcryp->Instance->KEYR0 = __REV(*(uint32_t*)(keyaddr));

  return HAL_OK;
}

/**
  * @brief  Write the InitVector/InitCounter in IVRx registers.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
static HAL_StatusTypeDef CRYP_SetInitVector(CRYP_HandleTypeDef *hcryp)
{
  uint32_t ivaddr = 0x0U;

#if !defined(AES_CR_NPBLB)
  if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)
  {
    hcryp->Instance->IVR3 = 0U;
    hcryp->Instance->IVR2 = 0U;
    hcryp->Instance->IVR1 = 0U;
    hcryp->Instance->IVR0 = 0U;
  }
  else
#endif
  {
    if (hcryp->Init.pInitVect == NULL)
    {
      return HAL_ERROR;
    }

    ivaddr = (uint32_t)(hcryp->Init.pInitVect);

    hcryp->Instance->IVR3 = __REV(*(uint32_t*)(ivaddr));
    ivaddr+=4U;
    hcryp->Instance->IVR2 = __REV(*(uint32_t*)(ivaddr));
    ivaddr+=4U;
    hcryp->Instance->IVR1 = __REV(*(uint32_t*)(ivaddr));
    ivaddr+=4U;
    hcryp->Instance->IVR0 = __REV(*(uint32_t*)(ivaddr));
  }
  return HAL_OK;
}



/**
  * @brief  Handle CRYP block input/output data handling under interruption.
  * @note   The function is called under interruption only, once
  *         interruptions have been enabled by HAL_CRYPEx_AES_IT().
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @retval HAL status
  */
static HAL_StatusTypeDef CRYP_AES_IT(CRYP_HandleTypeDef *hcryp)
{
  uint32_t inputaddr = 0U;
  uint32_t outputaddr = 0U;

  if(hcryp->State == HAL_CRYP_STATE_BUSY)
  {
    if (hcryp->Init.OperatingMode != CRYP_ALGOMODE_KEYDERIVATION)
    {
      /* Get the output data address */
      outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;

      /* Read the last available output block from the Data Output Register */
      *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
      outputaddr+=4U;
      *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
      outputaddr+=4U;
      *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
      outputaddr+=4U;
      *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
      hcryp->pCrypOutBuffPtr += 16U;
      hcryp->CrypOutCount -= 16U;

    }
    else
    {
      /* Read the derived key from the Key registers */
      if (hcryp->Init.KeySize == CRYP_KEYSIZE_256B)
      {
        *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->KEYR7);
        outputaddr+=4U;
        *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->KEYR6);
        outputaddr+=4U;
        *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->KEYR5);
        outputaddr+=4U;
        *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->KEYR4);
        outputaddr+=4U;
      }

        *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->KEYR3);
        outputaddr+=4U;
        *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->KEYR2);
        outputaddr+=4U;
        *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->KEYR1);
        outputaddr+=4U;
        *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->KEYR0);
    }

    /* In case of ciphering or deciphering, check if all output text has been retrieved;
       In case of key derivation, stop right there */
    if ((hcryp->CrypOutCount == 0U) || (hcryp->Init.OperatingMode == CRYP_ALGOMODE_KEYDERIVATION))
    {
      /* Disable Computation Complete Flag and Errors Interrupts */
      __HAL_CRYP_DISABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;

     /* Process Unlocked */
      __HAL_UNLOCK(hcryp);

      /* Call computation complete callback */
      HAL_CRYPEx_ComputationCpltCallback(hcryp);

      return HAL_OK;
    }
    /* If suspension flag has been raised, suspend processing */
    else if (hcryp->SuspendRequest == HAL_CRYP_SUSPEND)
    {
      /* reset ModeSuspend */
      hcryp->SuspendRequest = HAL_CRYP_SUSPEND_NONE;

      /* Disable Computation Complete Flag and Errors Interrupts */
      __HAL_CRYP_DISABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_SUSPENDED;

     /* Process Unlocked */
      __HAL_UNLOCK(hcryp);

      return HAL_OK;
    }
    else /* Process the rest of input data */
    {
      /* Get the Intput data address */
      inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;

      /* Increment/decrement instance pointer/counter */
      hcryp->pCrypInBuffPtr += 16U;
      hcryp->CrypInCount -= 16U;

      /* Write the next input block in the Data Input register */
      hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
      inputaddr+=4U;
      hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
      inputaddr+=4U;
      hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
      inputaddr+=4U;
      hcryp->Instance->DINR = *(uint32_t*)(inputaddr);

      return HAL_OK;
    }
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @}
  */

#endif /* AES */

#endif /* HAL_CRYP_MODULE_ENABLED */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
