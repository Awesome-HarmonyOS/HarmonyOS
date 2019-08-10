/**
  ******************************************************************************
  * @file    stm32f4xx_hal_cryp_ex.c
  * @author  MCD Application Team
  * @brief   Extended CRYP HAL module driver
  *          This file provides firmware functions to manage the following
  *          functionalities of CRYP extension peripheral:
  *           + Extended AES processing functions
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The CRYP Extension HAL driver can be used as follows:
    (#)Initialize the CRYP low level resources by implementing the HAL_CRYP_MspInit():
        (##) Enable the CRYP interface clock using __HAL_RCC_CRYP_CLK_ENABLE()
        (##) In case of using interrupts (e.g. HAL_CRYPEx_AESGCM_Encrypt_IT())
            (+++) Configure the CRYP interrupt priority using HAL_NVIC_SetPriority()
            (+++) Enable the CRYP IRQ handler using HAL_NVIC_EnableIRQ()
            (+++) In CRYP IRQ handler, call HAL_CRYP_IRQHandler()
        (##) In case of using DMA to control data transfer (e.g. HAL_AES_ECB_Encrypt_DMA())
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
        (##) The encryption/decryption key. Its size depends on the algorithm
                used for encryption/decryption
        (##) The initialization vector (counter). It is not used ECB mode.
    (#)Three processing (encryption/decryption) functions are available:
        (##) Polling mode: encryption and decryption APIs are blocking functions
             i.e. they process the data and wait till the processing is finished
             e.g. HAL_CRYPEx_AESGCM_Encrypt()
        (##) Interrupt mode: encryption and decryption APIs are not blocking functions
                i.e. they process the data under interrupt
                e.g. HAL_CRYPEx_AESGCM_Encrypt_IT()
        (##) DMA mode: encryption and decryption APIs are not blocking functions
                i.e. the data transfer is ensured by DMA
                e.g. HAL_CRYPEx_AESGCM_Encrypt_DMA()
    (#)When the processing function is called at first time after HAL_CRYP_Init()
       the CRYP peripheral is initialized and processes the buffer in input.
       At second call, the processing function performs an append of the already
       processed buffer.
       When a new data block is to be processed, call HAL_CRYP_Init() then the
       processing function.
    (#)In AES-GCM and AES-CCM modes are an authenticated encryption algorithms
       which provide authentication messages.
       HAL_AES_GCM_Finish() and HAL_AES_CCM_Finish() are used to provide those
       authentication messages.
       Call those functions after the processing ones (polling, interrupt or DMA).
       e.g. in AES-CCM mode call HAL_CRYPEx_AESCCM_Encrypt() to encrypt the plain data
            then call HAL_CRYPEx_AESCCM_Finish() to get the authentication message
     -@- For CCM Encrypt/Decrypt API's, only DataType = 8-bit is supported by this version.
     -@- The HAL_CRYPEx_AESGCM_xxxx() implementation is limited to 32bits inputs data length
           (Plain/Cyphertext, Header) compared with GCM standards specifications (800-38D).
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

/** @defgroup CRYPEx CRYPEx
  * @brief CRYP Extension HAL module driver.
  * @{
  */

#ifdef HAL_CRYP_MODULE_ENABLED

#if defined(CRYP)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup CRYPEx_Private_define
  * @{
  */
#define CRYPEx_TIMEOUT_VALUE  1U
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup CRYPEx_Private_Functions_prototypes  CRYP Private Functions Prototypes
  * @{
  */
static void CRYPEx_GCMCCM_SetInitVector(CRYP_HandleTypeDef *hcryp, uint8_t *InitVector);
static void CRYPEx_GCMCCM_SetKey(CRYP_HandleTypeDef *hcryp, uint8_t *Key, uint32_t KeySize);
static HAL_StatusTypeDef CRYPEx_GCMCCM_ProcessData(CRYP_HandleTypeDef *hcryp, uint8_t *Input, uint16_t Ilength, uint8_t *Output, uint32_t Timeout);
static HAL_StatusTypeDef CRYPEx_GCMCCM_SetHeaderPhase(CRYP_HandleTypeDef *hcryp, uint8_t* Input, uint16_t Ilength, uint32_t Timeout);
static void CRYPEx_GCMCCM_DMAInCplt(DMA_HandleTypeDef *hdma);
static void CRYPEx_GCMCCM_DMAOutCplt(DMA_HandleTypeDef *hdma);
static void CRYPEx_GCMCCM_DMAError(DMA_HandleTypeDef *hdma);
static void CRYPEx_GCMCCM_SetDMAConfig(CRYP_HandleTypeDef *hcryp, uint32_t inputaddr, uint16_t Size, uint32_t outputaddr);
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @addtogroup CRYPEx_Private_Functions
  * @{
  */

/**
  * @brief  DMA CRYP Input Data process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void CRYPEx_GCMCCM_DMAInCplt(DMA_HandleTypeDef *hdma)
{
  CRYP_HandleTypeDef* hcryp = ( CRYP_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Disable the DMA transfer for input Fifo request by resetting the DIEN bit
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
static void CRYPEx_GCMCCM_DMAOutCplt(DMA_HandleTypeDef *hdma)
{
  CRYP_HandleTypeDef* hcryp = ( CRYP_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Disable the DMA transfer for output Fifo request by resetting the DOEN bit
     in the DMACR register */
  hcryp->Instance->DMACR &= (uint32_t)(~CRYP_DMACR_DOEN);

  /* Enable the CRYP peripheral */
  __HAL_CRYP_DISABLE(hcryp);

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Call output data transfer complete callback */
  HAL_CRYP_OutCpltCallback(hcryp);
}

/**
  * @brief  DMA CRYP communication error callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void CRYPEx_GCMCCM_DMAError(DMA_HandleTypeDef *hdma)
{
  CRYP_HandleTypeDef* hcryp = ( CRYP_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
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
static void CRYPEx_GCMCCM_SetKey(CRYP_HandleTypeDef *hcryp, uint8_t *Key, uint32_t KeySize)
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
  * @retval None
  */
static void CRYPEx_GCMCCM_SetInitVector(CRYP_HandleTypeDef *hcryp, uint8_t *InitVector)
{
  uint32_t ivaddr = (uint32_t)InitVector;

  hcryp->Instance->IV0LR = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->IV0RR = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->IV1LR = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->IV1RR = __REV(*(uint32_t*)(ivaddr));
}

/**
  * @brief  Process Data: Writes Input data in polling mode and read the Output data.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  Input Pointer to the Input buffer.
  * @param  Ilength Length of the Input buffer, must be a multiple of 16
  * @param  Output Pointer to the returned buffer
  * @param  Timeout Timeout value
  * @retval None
  */
static HAL_StatusTypeDef CRYPEx_GCMCCM_ProcessData(CRYP_HandleTypeDef *hcryp, uint8_t *Input, uint16_t Ilength, uint8_t *Output, uint32_t Timeout)
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
        if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }
    }
    /* Read the Output block from the OUT FIFO */
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
  * @brief  Sets the header phase
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  Input Pointer to the Input buffer.
  * @param  Ilength Length of the Input buffer, must be a multiple of 16
  * @param  Timeout Timeout value
  * @retval None
  */
static HAL_StatusTypeDef CRYPEx_GCMCCM_SetHeaderPhase(CRYP_HandleTypeDef *hcryp, uint8_t* Input, uint16_t Ilength, uint32_t Timeout)
{
  uint32_t tickstart = 0U;
  uint32_t loopcounter = 0U;
  uint32_t headeraddr = (uint32_t)Input;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(Ilength);

  /***************************** Header phase *********************************/
  if(hcryp->Init.HeaderSize != 0U)
  {
    /* Select header phase */
    __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_HEADER);
    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    for(loopcounter = 0U; (loopcounter < hcryp->Init.HeaderSize); loopcounter+=16U)
    {
      /* Get tick */
      tickstart = HAL_GetTick();

      while(HAL_IS_BIT_CLR(hcryp->Instance->SR, CRYP_FLAG_IFEM))
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
      /* Write the Input block in the IN FIFO */
      hcryp->Instance->DR = *(uint32_t*)(headeraddr);
      headeraddr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(headeraddr);
      headeraddr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(headeraddr);
      headeraddr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(headeraddr);
      headeraddr+=4U;
    }

    /* Wait until the complete message has been processed */

    /* Get tick */
    tickstart = HAL_GetTick();

    while((hcryp->Instance->SR & CRYP_FLAG_BUSY) == CRYP_FLAG_BUSY)
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
  }
  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Sets the DMA configuration and start the DMA transfer.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  inputaddr Address of the Input buffer
  * @param  Size Size of the Input buffer, must be a multiple of 16
  * @param  outputaddr Address of the Output buffer
  * @retval None
  */
static void CRYPEx_GCMCCM_SetDMAConfig(CRYP_HandleTypeDef *hcryp, uint32_t inputaddr, uint16_t Size, uint32_t outputaddr)
{
  /* Set the CRYP DMA transfer complete callback */
  hcryp->hdmain->XferCpltCallback = CRYPEx_GCMCCM_DMAInCplt;
  /* Set the DMA error callback */
  hcryp->hdmain->XferErrorCallback = CRYPEx_GCMCCM_DMAError;

  /* Set the CRYP DMA transfer complete callback */
  hcryp->hdmaout->XferCpltCallback = CRYPEx_GCMCCM_DMAOutCplt;
  /* Set the DMA error callback */
  hcryp->hdmaout->XferErrorCallback = CRYPEx_GCMCCM_DMAError;

  /* Enable the CRYP peripheral */
  __HAL_CRYP_ENABLE(hcryp);

  /* Enable the DMA In DMA Stream */
  HAL_DMA_Start_IT(hcryp->hdmain, inputaddr, (uint32_t)&hcryp->Instance->DR, Size/4U);

  /* Enable In DMA request */
  hcryp->Instance->DMACR = CRYP_DMACR_DIEN;

  /* Enable the DMA Out DMA Stream */
  HAL_DMA_Start_IT(hcryp->hdmaout, (uint32_t)&hcryp->Instance->DOUT, outputaddr, Size/4U);

  /* Enable Out DMA request */
  hcryp->Instance->DMACR |= CRYP_DMACR_DOEN;
}

/**
  * @}
  */

/* Exported functions---------------------------------------------------------*/
/** @addtogroup CRYPEx_Exported_Functions
  * @{
  */

/** @defgroup CRYPEx_Exported_Functions_Group1 Extended AES processing functions
 *  @brief   Extended processing functions.
 *
@verbatim
  ==============================================================================
              ##### Extended AES processing functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Encrypt plaintext using AES-128/192/256 using GCM and CCM chaining modes
      (+) Decrypt cyphertext using AES-128/192/256 using GCM and CCM chaining modes
      (+) Finish the processing. This function is available only for GCM and CCM
    [..]  Three processing methods are available:
      (+) Polling mode
      (+) Interrupt mode
      (+) DMA mode

@endverbatim
  * @{
  */


/**
  * @brief  Initializes the CRYP peripheral in AES CCM encryption mode then
  *         encrypt pPlainData. The cypher data are available in pCypherData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESCCM_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  uint32_t tickstart = 0U;
  uint32_t headersize = hcryp->Init.HeaderSize;
  uint32_t headeraddr = (uint32_t)hcryp->Init.Header;
  uint32_t loopcounter = 0U;
  uint32_t bufferidx = 0U;
  uint8_t blockb0[16U] = {0};/* Block B0 */
  uint8_t ctr[16U] = {0}; /* Counter */
  uint32_t b0addr = (uint32_t)blockb0;

  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /************************ Formatting the header block *********************/
    if(headersize != 0U)
    {
      /* Check that the associated data (or header) length is lower than 2^16 - 2^8 = 65536 - 256 = 65280 */
      if(headersize < 65280U)
      {
        hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize >> 8) & 0xFF);
        hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize) & 0xFF);
        headersize += 2U;
      }
      else
      {
        /* Header is encoded as 0xff || 0xfe || [headersize]32, i.e., six octets */
        hcryp->Init.pScratch[bufferidx++] = 0xFFU;
        hcryp->Init.pScratch[bufferidx++] = 0xFEU;
        hcryp->Init.pScratch[bufferidx++] = headersize & 0xff000000U;
        hcryp->Init.pScratch[bufferidx++] = headersize & 0x00ff0000U;
        hcryp->Init.pScratch[bufferidx++] = headersize & 0x0000ff00U;
        hcryp->Init.pScratch[bufferidx++] = headersize & 0x000000ffU;
        headersize += 6U;
      }
      /* Copy the header buffer in internal buffer "hcryp->Init.pScratch" */
      for(loopcounter = 0U; loopcounter < headersize; loopcounter++)
      {
        hcryp->Init.pScratch[bufferidx++] = hcryp->Init.Header[loopcounter];
      }
      /* Check if the header size is modulo 16 */
      if ((headersize % 16U) != 0U)
      {
        /* Padd the header buffer with 0s till the hcryp->Init.pScratch length is modulo 16 */
        for(loopcounter = headersize; loopcounter <= ((headersize/16U) + 1U) * 16U; loopcounter++)
        {
          hcryp->Init.pScratch[loopcounter] = 0U;
        }
        /* Set the header size to modulo 16 */
        headersize = ((headersize/16U) + 1U) * 16U;
      }
      /* Set the pointer headeraddr to hcryp->Init.pScratch */
      headeraddr = (uint32_t)hcryp->Init.pScratch;
    }
    /*********************** Formatting the block B0 **************************/
    if(headersize != 0U)
    {
      blockb0[0U] = 0x40U;
    }
    /* Flags byte */
    /* blockb0[0] |= 0u | (((( (uint8_t) hcryp->Init.TagSize - 2) / 2) & 0x07 ) << 3 ) | ( ( (uint8_t) (15 - hcryp->Init.IVSize) - 1) & 0x07U) */
    blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)(((uint8_t)(hcryp->Init.TagSize - (uint8_t)(2))) >> 1U) & (uint8_t)0x07) << 3U);
    blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)((uint8_t)(15) - hcryp->Init.IVSize) - (uint8_t)1) & (uint8_t)0x07);

    for (loopcounter = 0U; loopcounter < hcryp->Init.IVSize; loopcounter++)
    {
      blockb0[loopcounter+1U] = hcryp->Init.pInitVect[loopcounter];
    }
    for ( ; loopcounter < 13U; loopcounter++)
    {
      blockb0[loopcounter+1U] = 0U;
    }

    blockb0[14U] = (Size >> 8U);
    blockb0[15U] = (Size & 0xFFU);

    /************************* Formatting the initial counter *****************/
    /* Byte 0:
       Bits 7 and 6 are reserved and shall be set to 0
       Bits 3, 4, and 5 shall also be set to 0, to ensure that all the counter blocks
       are distinct from B0
       Bits 0, 1, and 2 contain the same encoding of q as in B0
    */
    ctr[0U] = blockb0[0U] & 0x07U;
    /* byte 1 to NonceSize is the IV (Nonce) */
    for(loopcounter = 1U; loopcounter < hcryp->Init.IVSize + 1U; loopcounter++)
    {
      ctr[loopcounter] = blockb0[loopcounter];
    }
    /* Set the LSB to 1 */
    ctr[15U] |= 0x01U;

    /* Set the key */
    CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES CCM mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CCM_ENCRYPT);

    /* Set the Initialization Vector */
    CRYPEx_GCMCCM_SetInitVector(hcryp, ctr);

    /* Select init phase */
    __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_INIT);

    b0addr = (uint32_t)blockb0;
    /* Write the blockb0 block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(b0addr);
    b0addr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(b0addr);
    b0addr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(b0addr);
    b0addr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(b0addr);

    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    /* Get tick */
    tickstart = HAL_GetTick();

    while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
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
    /***************************** Header phase *******************************/
    if(headersize != 0U)
    {
      /* Select header phase */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_HEADER);

      /* Enable the CRYP peripheral */
      __HAL_CRYP_ENABLE(hcryp);

      for(loopcounter = 0U; (loopcounter < headersize); loopcounter+=16U)
      {
        /* Get tick */
        tickstart = HAL_GetTick();

        while(HAL_IS_BIT_CLR(hcryp->Instance->SR, CRYP_FLAG_IFEM))
        {
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
        }
        /* Write the header block in the IN FIFO */
        hcryp->Instance->DR = *(uint32_t*)(headeraddr);
        headeraddr+=4U;
        hcryp->Instance->DR = *(uint32_t*)(headeraddr);
        headeraddr+=4U;
        hcryp->Instance->DR = *(uint32_t*)(headeraddr);
        headeraddr+=4U;
        hcryp->Instance->DR = *(uint32_t*)(headeraddr);
        headeraddr+=4U;
      }

      /* Get tick */
      tickstart = HAL_GetTick();

      while((hcryp->Instance->SR & CRYP_FLAG_BUSY) == CRYP_FLAG_BUSY)
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
    }
    /* Save formatted counter into the scratch buffer pScratch */
    for(loopcounter = 0U; (loopcounter < 16U); loopcounter++)
    {
      hcryp->Init.pScratch[loopcounter] = ctr[loopcounter];
    }
    /* Reset bit 0 */
    hcryp->Init.pScratch[15U] &= 0xFEU;

    /* Select payload phase once the header phase is performed */
    __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }

  /* Write Plain Data and Get Cypher Data */
  if(CRYPEx_GCMCCM_ProcessData(hcryp,pPlainData, Size, pCypherData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES GCM encryption mode then
  *         encrypt pPlainData. The cypher data are available in pCypherData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESGCM_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the key */
    CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES GCM mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_GCM_ENCRYPT);

    /* Set the Initialization Vector */
    CRYPEx_GCMCCM_SetInitVector(hcryp, hcryp->Init.pInitVect);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    /* Get tick */
    tickstart = HAL_GetTick();

    while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
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

    /* Set the header phase */
    if(CRYPEx_GCMCCM_SetHeaderPhase(hcryp, hcryp->Init.Header, hcryp->Init.HeaderSize, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    /* Disable the CRYP peripheral */
    __HAL_CRYP_DISABLE(hcryp);

    /* Select payload phase once the header phase is performed */
    __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }

  /* Write Plain Data and Get Cypher Data */
  if(CRYPEx_GCMCCM_ProcessData(hcryp, pPlainData, Size, pCypherData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES GCM decryption mode then
  *         decrypted pCypherData. The cypher data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the cyphertext buffer, must be a multiple of 16
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESGCM_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /* Set the key */
    CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES GCM decryption mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_GCM_DECRYPT);

    /* Set the Initialization Vector */
    CRYPEx_GCMCCM_SetInitVector(hcryp, hcryp->Init.pInitVect);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    /* Get tick */
    tickstart = HAL_GetTick();

    while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
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

    /* Set the header phase */
    if(CRYPEx_GCMCCM_SetHeaderPhase(hcryp, hcryp->Init.Header, hcryp->Init.HeaderSize, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }
    /* Disable the CRYP peripheral */
    __HAL_CRYP_DISABLE(hcryp);

    /* Select payload phase once the header phase is performed */
    __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }

  /* Write Plain Data and Get Cypher Data */
  if(CRYPEx_GCMCCM_ProcessData(hcryp, pCypherData, Size, pPlainData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Computes the authentication TAG.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  Size Total length of the plain/cyphertext buffer
  * @param  AuthTag Pointer to the authentication buffer
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESGCM_Finish(CRYP_HandleTypeDef *hcryp, uint32_t Size, uint8_t *AuthTag, uint32_t Timeout)
{
  uint32_t tickstart = 0U;
  uint64_t headerlength = hcryp->Init.HeaderSize * 8U; /* Header length in bits */
  uint64_t inputlength = Size * 8U; /* input length in bits */
  uint32_t tagaddr = (uint32_t)AuthTag;

  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_PROCESS)
  {
    /* Change the CRYP phase */
    hcryp->Phase = HAL_CRYP_PHASE_FINAL;

    /* Disable CRYP to start the final phase */
    __HAL_CRYP_DISABLE(hcryp);

    /* Select final phase */
    __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_FINAL);

    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    /* Write the number of bits in header (64 bits) followed by the number of bits
       in the payload */
    if(hcryp->Init.DataType == CRYP_DATATYPE_1B)
    {
      hcryp->Instance->DR = __RBIT(headerlength >> 32U);
      hcryp->Instance->DR = __RBIT(headerlength);
      hcryp->Instance->DR = __RBIT(inputlength >> 32U);
      hcryp->Instance->DR = __RBIT(inputlength);
    }
    else if(hcryp->Init.DataType == CRYP_DATATYPE_8B)
    {
      hcryp->Instance->DR = __REV(headerlength >> 32U);
      hcryp->Instance->DR = __REV(headerlength);
      hcryp->Instance->DR = __REV(inputlength >> 32U);
      hcryp->Instance->DR = __REV(inputlength);
    }
    else if(hcryp->Init.DataType == CRYP_DATATYPE_16B)
    {
      hcryp->Instance->DR = __ROR((uint32_t)(headerlength >> 32U), 16U);
      hcryp->Instance->DR = __ROR((uint32_t)headerlength, 16U);
      hcryp->Instance->DR = __ROR((uint32_t)(inputlength >> 32U), 16U);
      hcryp->Instance->DR = __ROR((uint32_t)inputlength, 16U);
    }
    else if(hcryp->Init.DataType == CRYP_DATATYPE_32B)
    {
      hcryp->Instance->DR = (uint32_t)(headerlength >> 32U);
      hcryp->Instance->DR = (uint32_t)(headerlength);
      hcryp->Instance->DR = (uint32_t)(inputlength >> 32U);
      hcryp->Instance->DR = (uint32_t)(inputlength);
    }
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

    /* Read the Auth TAG in the IN FIFO */
    *(uint32_t*)(tagaddr) = hcryp->Instance->DOUT;
    tagaddr+=4U;
    *(uint32_t*)(tagaddr) = hcryp->Instance->DOUT;
    tagaddr+=4U;
    *(uint32_t*)(tagaddr) = hcryp->Instance->DOUT;
    tagaddr+=4U;
    *(uint32_t*)(tagaddr) = hcryp->Instance->DOUT;
  }

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Computes the authentication TAG for AES CCM mode.
  * @note   This API is called after HAL_AES_CCM_Encrypt()/HAL_AES_CCM_Decrypt()
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  AuthTag Pointer to the authentication buffer
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESCCM_Finish(CRYP_HandleTypeDef *hcryp, uint8_t *AuthTag, uint32_t Timeout)
{
  uint32_t tickstart = 0U;
  uint32_t tagaddr = (uint32_t)AuthTag;
  uint32_t ctraddr = (uint32_t)hcryp->Init.pScratch;
  uint32_t temptag[4U] = {0U}; /* Temporary TAG (MAC) */
  uint32_t loopcounter;

  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_PROCESS)
  {
    /* Change the CRYP phase */
    hcryp->Phase = HAL_CRYP_PHASE_FINAL;

    /* Disable CRYP to start the final phase */
    __HAL_CRYP_DISABLE(hcryp);

    /* Select final phase */
    __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_FINAL);

    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    /* Write the counter block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)ctraddr;
    ctraddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)ctraddr;
    ctraddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)ctraddr;
    ctraddr+=4U;
    hcryp->Instance->DR = *(uint32_t*)ctraddr;

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

    /* Read the Auth TAG in the IN FIFO */
    temptag[0U] = hcryp->Instance->DOUT;
    temptag[1U] = hcryp->Instance->DOUT;
    temptag[2U] = hcryp->Instance->DOUT;
    temptag[3U] = hcryp->Instance->DOUT;
  }

  /* Copy temporary authentication TAG in user TAG buffer */
  for(loopcounter = 0U; loopcounter < hcryp->Init.TagSize ; loopcounter++)
  {
    /* Set the authentication TAG buffer */
    *((uint8_t*)tagaddr+loopcounter) = *((uint8_t*)temptag+loopcounter);
  }

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES CCM decryption mode then
  *         decrypted pCypherData. The cypher data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESCCM_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout)
{
  uint32_t tickstart = 0U;
  uint32_t headersize = hcryp->Init.HeaderSize;
  uint32_t headeraddr = (uint32_t)hcryp->Init.Header;
  uint32_t loopcounter = 0U;
  uint32_t bufferidx = 0U;
  uint8_t blockb0[16U] = {0};/* Block B0 */
  uint8_t ctr[16U] = {0}; /* Counter */
  uint32_t b0addr = (uint32_t)blockb0;

  /* Process Locked */
  __HAL_LOCK(hcryp);

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_BUSY;

  /* Check if initialization phase has already been performed */
  if(hcryp->Phase == HAL_CRYP_PHASE_READY)
  {
    /************************ Formatting the header block *********************/
    if(headersize != 0U)
    {
      /* Check that the associated data (or header) length is lower than 2^16 - 2^8 = 65536 - 256 = 65280 */
      if(headersize < 65280U)
      {
        hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize >> 8) & 0xFF);
        hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize) & 0xFF);
        headersize += 2U;
      }
      else
      {
        /* Header is encoded as 0xff || 0xfe || [headersize]32, i.e., six octets */
        hcryp->Init.pScratch[bufferidx++] = 0xFFU;
        hcryp->Init.pScratch[bufferidx++] = 0xFEU;
        hcryp->Init.pScratch[bufferidx++] = headersize & 0xff000000U;
        hcryp->Init.pScratch[bufferidx++] = headersize & 0x00ff0000U;
        hcryp->Init.pScratch[bufferidx++] = headersize & 0x0000ff00U;
        hcryp->Init.pScratch[bufferidx++] = headersize & 0x000000ffU;
        headersize += 6U;
      }
      /* Copy the header buffer in internal buffer "hcryp->Init.pScratch" */
      for(loopcounter = 0U; loopcounter < headersize; loopcounter++)
      {
        hcryp->Init.pScratch[bufferidx++] = hcryp->Init.Header[loopcounter];
      }
      /* Check if the header size is modulo 16 */
      if ((headersize % 16U) != 0U)
      {
        /* Padd the header buffer with 0s till the hcryp->Init.pScratch length is modulo 16 */
        for(loopcounter = headersize; loopcounter <= ((headersize/16U) + 1U) * 16U; loopcounter++)
        {
          hcryp->Init.pScratch[loopcounter] = 0U;
        }
        /* Set the header size to modulo 16 */
        headersize = ((headersize/16U) + 1U) * 16U;
      }
      /* Set the pointer headeraddr to hcryp->Init.pScratch */
      headeraddr = (uint32_t)hcryp->Init.pScratch;
    }
    /*********************** Formatting the block B0 **************************/
    if(headersize != 0U)
    {
      blockb0[0U] = 0x40U;
    }
    /* Flags byte */
    /* blockb0[0] |= 0u | (((( (uint8_t) hcryp->Init.TagSize - 2) / 2) & 0x07 ) << 3 ) | ( ( (uint8_t) (15 - hcryp->Init.IVSize) - 1) & 0x07U) */
    blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)(((uint8_t)(hcryp->Init.TagSize - (uint8_t)(2U))) >> 1U) & (uint8_t)0x07U) << 3U);
    blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)((uint8_t)(15U) - hcryp->Init.IVSize) - (uint8_t)1U) & (uint8_t)0x07U);

    for (loopcounter = 0U; loopcounter < hcryp->Init.IVSize; loopcounter++)
    {
      blockb0[loopcounter+1U] = hcryp->Init.pInitVect[loopcounter];
    }
    for ( ; loopcounter < 13U; loopcounter++)
    {
      blockb0[loopcounter+1U] = 0U;
    }

    blockb0[14U] = (Size >> 8U);
    blockb0[15U] = (Size & 0xFFU);

    /************************* Formatting the initial counter *****************/
    /* Byte 0:
       Bits 7 and 6 are reserved and shall be set to 0
       Bits 3, 4, and 5 shall also be set to 0, to ensure that all the counter
       blocks are distinct from B0
       Bits 0, 1, and 2 contain the same encoding of q as in B0
    */
    ctr[0U] = blockb0[0U] & 0x07U;
    /* byte 1 to NonceSize is the IV (Nonce) */
    for(loopcounter = 1U; loopcounter < hcryp->Init.IVSize + 1U; loopcounter++)
    {
      ctr[loopcounter] = blockb0[loopcounter];
    }
    /* Set the LSB to 1 */
    ctr[15U] |= 0x01U;

    /* Set the key */
    CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

    /* Set the CRYP peripheral in AES CCM mode */
    __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CCM_DECRYPT);

    /* Set the Initialization Vector */
    CRYPEx_GCMCCM_SetInitVector(hcryp, ctr);

    /* Select init phase */
    __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_INIT);

    b0addr = (uint32_t)blockb0;
    /* Write the blockb0 block in the IN FIFO */
    hcryp->Instance->DR = *(uint32_t*)(b0addr);
    b0addr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(b0addr);
    b0addr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(b0addr);
    b0addr+=4U;
    hcryp->Instance->DR = *(uint32_t*)(b0addr);

    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    /* Get tick */
    tickstart = HAL_GetTick();

    while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
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
    /***************************** Header phase *******************************/
    if(headersize != 0U)
    {
      /* Select header phase */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_HEADER);

      /* Enable Crypto processor */
      __HAL_CRYP_ENABLE(hcryp);

      for(loopcounter = 0U; (loopcounter < headersize); loopcounter+=16U)
      {
        /* Get tick */
        tickstart = HAL_GetTick();

        while(HAL_IS_BIT_CLR(hcryp->Instance->SR, CRYP_FLAG_IFEM))
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
        /* Write the header block in the IN FIFO */
        hcryp->Instance->DR = *(uint32_t*)(headeraddr);
        headeraddr+=4U;
        hcryp->Instance->DR = *(uint32_t*)(headeraddr);
        headeraddr+=4U;
        hcryp->Instance->DR = *(uint32_t*)(headeraddr);
        headeraddr+=4U;
        hcryp->Instance->DR = *(uint32_t*)(headeraddr);
        headeraddr+=4U;
      }

      /* Get tick */
      tickstart = HAL_GetTick();

      while((hcryp->Instance->SR & CRYP_FLAG_BUSY) == CRYP_FLAG_BUSY)
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
    }
    /* Save formatted counter into the scratch buffer pScratch */
    for(loopcounter = 0U; (loopcounter < 16U); loopcounter++)
    {
      hcryp->Init.pScratch[loopcounter] = ctr[loopcounter];
    }
    /* Reset bit 0 */
    hcryp->Init.pScratch[15U] &= 0xFEU;
    /* Select payload phase once the header phase is performed */
    __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

    /* Flush FIFO */
    __HAL_CRYP_FIFO_FLUSH(hcryp);

    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    /* Set the phase */
    hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
  }

  /* Write Plain Data and Get Cypher Data */
  if(CRYPEx_GCMCCM_ProcessData(hcryp, pCypherData, Size, pPlainData, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* Change the CRYP peripheral state */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hcryp);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES GCM encryption mode using IT.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESGCM_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
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
    hcryp->pCrypInBuffPtr = pPlainData;
    hcryp->pCrypOutBuffPtr = pCypherData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP peripheral state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES GCM mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_GCM_ENCRYPT);

      /* Set the Initialization Vector */
      CRYPEx_GCMCCM_SetInitVector(hcryp, hcryp->Init.pInitVect);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Enable CRYP to start the init phase */
      __HAL_CRYP_ENABLE(hcryp);

     /* Get tick */
     tickstart = HAL_GetTick();

      while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
      {
        /* Check for the Timeout */

        if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;

        }
      }

      /* Set the header phase */
      if(CRYPEx_GCMCCM_SetHeaderPhase(hcryp, hcryp->Init.Header, hcryp->Init.HeaderSize, 1U) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }
      /* Disable the CRYP peripheral */
      __HAL_CRYP_DISABLE(hcryp);

      /* Select payload phase once the header phase is performed */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    if(Size != 0U)
    {
      /* Enable Interrupts */
      __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);
      /* Enable the CRYP peripheral */
      __HAL_CRYP_ENABLE(hcryp);
    }
    else
    {
      /* Process Locked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state and phase */
      hcryp->State = HAL_CRYP_STATE_READY;
    }
    /* Return function status */
    return HAL_OK;
  }
  else if (__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
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
  else if (__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
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
      /* Change the CRYP peripheral state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES CCM encryption mode using interrupt.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESCCM_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t tickstart = 0U;
  uint32_t inputaddr;
  uint32_t outputaddr;

  uint32_t headersize = hcryp->Init.HeaderSize;
  uint32_t headeraddr = (uint32_t)hcryp->Init.Header;
  uint32_t loopcounter = 0U;
  uint32_t bufferidx = 0U;
  uint8_t blockb0[16U] = {0};/* Block B0 */
  uint8_t ctr[16U] = {0}; /* Counter */
  uint32_t b0addr = (uint32_t)blockb0;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pPlainData;
    hcryp->pCrypOutBuffPtr = pCypherData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP peripheral state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /************************ Formatting the header block *******************/
      if(headersize != 0U)
      {
        /* Check that the associated data (or header) length is lower than 2^16 - 2^8 = 65536 - 256 = 65280 */
        if(headersize < 65280U)
        {
          hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize >> 8) & 0xFF);
          hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize) & 0xFF);
          headersize += 2U;
        }
        else
        {
          /* Header is encoded as 0xff || 0xfe || [headersize]32, i.e., six octets */
          hcryp->Init.pScratch[bufferidx++] = 0xFFU;
          hcryp->Init.pScratch[bufferidx++] = 0xFEU;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0xff000000U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x00ff0000U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x0000ff00U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x000000ffU;
          headersize += 6U;
        }
        /* Copy the header buffer in internal buffer "hcryp->Init.pScratch" */
        for(loopcounter = 0U; loopcounter < headersize; loopcounter++)
        {
          hcryp->Init.pScratch[bufferidx++] = hcryp->Init.Header[loopcounter];
        }
        /* Check if the header size is modulo 16 */
        if ((headersize % 16U) != 0U)
        {
          /* Padd the header buffer with 0s till the hcryp->Init.pScratch length is modulo 16 */
          for(loopcounter = headersize; loopcounter <= ((headersize/16U) + 1U) * 16U; loopcounter++)
          {
            hcryp->Init.pScratch[loopcounter] = 0U;
          }
          /* Set the header size to modulo 16 */
          headersize = ((headersize/16U) + 1U) * 16U;
        }
        /* Set the pointer headeraddr to hcryp->Init.pScratch */
        headeraddr = (uint32_t)hcryp->Init.pScratch;
      }
      /*********************** Formatting the block B0 ************************/
      if(headersize != 0U)
      {
        blockb0[0U] = 0x40U;
      }
      /* Flags byte */
      /* blockb0[0] |= 0u | (((( (uint8_t) hcryp->Init.TagSize - 2) / 2) & 0x07 ) << 3 ) | ( ( (uint8_t) (15 - hcryp->Init.IVSize) - 1) & 0x07U) */
      blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)(((uint8_t)(hcryp->Init.TagSize - (uint8_t)(2))) >> 1U) & (uint8_t)0x07) << 3U);
      blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)((uint8_t)(15) - hcryp->Init.IVSize) - (uint8_t)1) & (uint8_t)0x07);

      for (loopcounter = 0U; loopcounter < hcryp->Init.IVSize; loopcounter++)
      {
        blockb0[loopcounter+1U] = hcryp->Init.pInitVect[loopcounter];
      }
      for ( ; loopcounter < 13U; loopcounter++)
      {
        blockb0[loopcounter+1U] = 0U;
      }

      blockb0[14U] = (Size >> 8U);
      blockb0[15U] = (Size & 0xFFU);

      /************************* Formatting the initial counter ***************/
      /* Byte 0:
         Bits 7 and 6 are reserved and shall be set to 0
         Bits 3, 4, and 5 shall also be set to 0, to ensure that all the counter
         blocks are distinct from B0
         Bits 0, 1, and 2 contain the same encoding of q as in B0
      */
      ctr[0U] = blockb0[0U] & 0x07U;
      /* byte 1 to NonceSize is the IV (Nonce) */
      for(loopcounter = 1; loopcounter < hcryp->Init.IVSize + 1U; loopcounter++)
      {
        ctr[loopcounter] = blockb0[loopcounter];
      }
      /* Set the LSB to 1 */
      ctr[15U] |= 0x01U;

      /* Set the key */
      CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES CCM mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CCM_ENCRYPT);

      /* Set the Initialization Vector */
      CRYPEx_GCMCCM_SetInitVector(hcryp, ctr);

      /* Select init phase */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_INIT);

      b0addr = (uint32_t)blockb0;
      /* Write the blockb0 block in the IN FIFO */
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);

      /* Enable the CRYP peripheral */
      __HAL_CRYP_ENABLE(hcryp);

     /* Get tick */
     tickstart = HAL_GetTick();

      while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
      {
        /* Check for the Timeout */
        if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }
      /***************************** Header phase *****************************/
      if(headersize != 0U)
      {
        /* Select header phase */
        __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_HEADER);

        /* Enable Crypto processor */
        __HAL_CRYP_ENABLE(hcryp);

        for(loopcounter = 0U; (loopcounter < headersize); loopcounter+=16U)
        {
         /* Get tick */
         tickstart = HAL_GetTick();

          while(HAL_IS_BIT_CLR(hcryp->Instance->SR, CRYP_FLAG_IFEM))
          {
            /* Check for the Timeout */
            if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
            {
              /* Change state */
              hcryp->State = HAL_CRYP_STATE_TIMEOUT;

              /* Process Unlocked */
              __HAL_UNLOCK(hcryp);

              return HAL_TIMEOUT;
            }
          }
          /* Write the header block in the IN FIFO */
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
        }

        /* Get tick */
        tickstart = HAL_GetTick();

        while((hcryp->Instance->SR & CRYP_FLAG_BUSY) == CRYP_FLAG_BUSY)
        {
          /* Check for the Timeout */
          if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
          {
            /* Change state */
            hcryp->State = HAL_CRYP_STATE_TIMEOUT;

            /* Process Unlocked */
            __HAL_UNLOCK(hcryp);

            return HAL_TIMEOUT;
          }
        }
      }
      /* Save formatted counter into the scratch buffer pScratch */
      for(loopcounter = 0U; (loopcounter < 16U); loopcounter++)
      {
        hcryp->Init.pScratch[loopcounter] = ctr[loopcounter];
      }
      /* Reset bit 0 */
      hcryp->Init.pScratch[15U] &= 0xFEU;

      /* Select payload phase once the header phase is performed */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    if(Size != 0U)
    {
      /* Enable Interrupts */
      __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);
      /* Enable the CRYP peripheral */
      __HAL_CRYP_ENABLE(hcryp);
    }
    else
    {
      /* Change the CRYP state and phase */
      hcryp->State = HAL_CRYP_STATE_READY;
    }

    /* Return function status */
    return HAL_OK;
  }
  else if (__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
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
      /* Call Input transfer complete callback */
      HAL_CRYP_InCpltCallback(hcryp);
    }
  }
  else if (__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
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
      /* Change the CRYP peripheral state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES GCM decryption mode using IT.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the cyphertext buffer, must be a multiple of 16
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESGCM_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
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

    /* Change the CRYP peripheral state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES GCM decryption mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_GCM_DECRYPT);

      /* Set the Initialization Vector */
      CRYPEx_GCMCCM_SetInitVector(hcryp, hcryp->Init.pInitVect);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Enable CRYP to start the init phase */
      __HAL_CRYP_ENABLE(hcryp);

        /* Get tick */
        tickstart = HAL_GetTick();

      while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
      {
        /* Check for the Timeout */
        if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }

      /* Set the header phase */
      if(CRYPEx_GCMCCM_SetHeaderPhase(hcryp, hcryp->Init.Header, hcryp->Init.HeaderSize, 1U) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }
      /* Disable the CRYP peripheral */
      __HAL_CRYP_DISABLE(hcryp);

      /* Select payload phase once the header phase is performed */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    if(Size != 0U)
    {
      /* Enable Interrupts */
      __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);
      /* Enable the CRYP peripheral */
      __HAL_CRYP_ENABLE(hcryp);
    }
    else
    {
      /* Process Locked */
      __HAL_UNLOCK(hcryp);
      /* Change the CRYP state and phase */
      hcryp->State = HAL_CRYP_STATE_READY;
    }

    /* Return function status */
    return HAL_OK;
  }
  else if (__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
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
  else if (__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
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
      /* Change the CRYP peripheral state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES CCM decryption mode using interrupt
  *         then decrypted pCypherData. The cypher data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESCCM_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t inputaddr;
  uint32_t outputaddr;
  uint32_t tickstart = 0U;
  uint32_t headersize = hcryp->Init.HeaderSize;
  uint32_t headeraddr = (uint32_t)hcryp->Init.Header;
  uint32_t loopcounter = 0U;
  uint32_t bufferidx = 0U;
  uint8_t blockb0[16U] = {0};/* Block B0 */
  uint8_t ctr[16U] = {0}; /* Counter */
  uint32_t b0addr = (uint32_t)blockb0;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pCypherData;
    hcryp->pCrypOutBuffPtr = pPlainData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP peripheral state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /************************ Formatting the header block *******************/
      if(headersize != 0U)
      {
        /* Check that the associated data (or header) length is lower than 2^16 - 2^8 = 65536 - 256 = 65280 */
        if(headersize < 65280U)
        {
          hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize >> 8) & 0xFF);
          hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize) & 0xFF);
          headersize += 2U;
        }
        else
        {
          /* Header is encoded as 0xff || 0xfe || [headersize]32, i.e., six octets */
          hcryp->Init.pScratch[bufferidx++] = 0xFFU;
          hcryp->Init.pScratch[bufferidx++] = 0xFEU;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0xff000000U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x00ff0000U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x0000ff00U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x000000ffU;
          headersize += 6U;
        }
        /* Copy the header buffer in internal buffer "hcryp->Init.pScratch" */
        for(loopcounter = 0U; loopcounter < headersize; loopcounter++)
        {
          hcryp->Init.pScratch[bufferidx++] = hcryp->Init.Header[loopcounter];
        }
        /* Check if the header size is modulo 16 */
        if ((headersize % 16U) != 0U)
        {
          /* Padd the header buffer with 0s till the hcryp->Init.pScratch length is modulo 16 */
          for(loopcounter = headersize; loopcounter <= ((headersize/16U) + 1U) * 16U; loopcounter++)
          {
            hcryp->Init.pScratch[loopcounter] = 0U;
          }
          /* Set the header size to modulo 16 */
          headersize = ((headersize/16U) + 1U) * 16U;
        }
        /* Set the pointer headeraddr to hcryp->Init.pScratch */
        headeraddr = (uint32_t)hcryp->Init.pScratch;
      }
      /*********************** Formatting the block B0 ************************/
      if(headersize != 0U)
      {
        blockb0[0U] = 0x40U;
      }
      /* Flags byte */
      /* blockb0[0] |= 0u | (((( (uint8_t) hcryp->Init.TagSize - 2) / 2) & 0x07 ) << 3 ) | ( ( (uint8_t) (15 - hcryp->Init.IVSize) - 1) & 0x07U) */
      blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)(((uint8_t)(hcryp->Init.TagSize - (uint8_t)(2))) >> 1U) & (uint8_t)0x07) << 3U);
      blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)((uint8_t)(15) - hcryp->Init.IVSize) - (uint8_t)1) & (uint8_t)0x07);

      for (loopcounter = 0U; loopcounter < hcryp->Init.IVSize; loopcounter++)
      {
        blockb0[loopcounter+1U] = hcryp->Init.pInitVect[loopcounter];
      }
      for ( ; loopcounter < 13U; loopcounter++)
      {
        blockb0[loopcounter+1U] = 0U;
      }

      blockb0[14U] = (Size >> 8U);
      blockb0[15U] = (Size & 0xFFU);

      /************************* Formatting the initial counter ***************/
      /* Byte 0:
         Bits 7 and 6 are reserved and shall be set to 0
         Bits 3, 4, and 5 shall also be set to 0, to ensure that all the counter
         blocks are distinct from B0
         Bits 0, 1, and 2 contain the same encoding of q as in B0
      */
      ctr[0U] = blockb0[0U] & 0x07U;
      /* byte 1 to NonceSize is the IV (Nonce) */
      for(loopcounter = 1U; loopcounter < hcryp->Init.IVSize + 1U; loopcounter++)
      {
        ctr[loopcounter] = blockb0[loopcounter];
      }
      /* Set the LSB to 1 */
      ctr[15U] |= 0x01U;

      /* Set the key */
      CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES CCM mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CCM_DECRYPT);

      /* Set the Initialization Vector */
      CRYPEx_GCMCCM_SetInitVector(hcryp, ctr);

      /* Select init phase */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_INIT);

      b0addr = (uint32_t)blockb0;
      /* Write the blockb0 block in the IN FIFO */
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);

      /* Enable the CRYP peripheral */
      __HAL_CRYP_ENABLE(hcryp);

      /* Get tick */
      tickstart = HAL_GetTick();

      while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
      {
        /* Check for the Timeout */
        if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }
      /***************************** Header phase *****************************/
      if(headersize != 0U)
      {
        /* Select header phase */
        __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_HEADER);

        /* Enable Crypto processor */
        __HAL_CRYP_ENABLE(hcryp);

        for(loopcounter = 0U; (loopcounter < headersize); loopcounter+=16U)
        {
         /* Get tick */
         tickstart = HAL_GetTick();

          while(HAL_IS_BIT_CLR(hcryp->Instance->SR, CRYP_FLAG_IFEM))
          {
            /* Check for the Timeout */
            if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
            {
              /* Change state */
              hcryp->State = HAL_CRYP_STATE_TIMEOUT;

              /* Process Unlocked */
              __HAL_UNLOCK(hcryp);

              return HAL_TIMEOUT;
            }
          }
          /* Write the header block in the IN FIFO */
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
        }

        /* Get tick */
        tickstart = HAL_GetTick();

        while((hcryp->Instance->SR & CRYP_FLAG_BUSY) == CRYP_FLAG_BUSY)
        {
          /* Check for the Timeout */
          if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
          {
            /* Change state */
            hcryp->State = HAL_CRYP_STATE_TIMEOUT;

            /* Process Unlocked */
            __HAL_UNLOCK(hcryp);

            return HAL_TIMEOUT;
          }
        }
      }
      /* Save formatted counter into the scratch buffer pScratch */
      for(loopcounter = 0U; (loopcounter < 16U); loopcounter++)
      {
        hcryp->Init.pScratch[loopcounter] = ctr[loopcounter];
      }
      /* Reset bit 0 */
      hcryp->Init.pScratch[15U] &= 0xFEU;
      /* Select payload phase once the header phase is performed */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    /* Enable Interrupts */
    __HAL_CRYP_ENABLE_IT(hcryp, CRYP_IT_INI | CRYP_IT_OUTI);

    /* Enable the CRYP peripheral */
    __HAL_CRYP_ENABLE(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else if (__HAL_CRYP_GET_IT(hcryp, CRYP_IT_INI))
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
  else if (__HAL_CRYP_GET_IT(hcryp, CRYP_IT_OUTI))
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
      /* Change the CRYP peripheral state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Call Input transfer complete callback */
      HAL_CRYP_OutCpltCallback(hcryp);
    }
  }

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRYP peripheral in AES GCM encryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESGCM_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t tickstart = 0U;
  uint32_t inputaddr;
  uint32_t outputaddr;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pPlainData;
    outputaddr = (uint32_t)pCypherData;

    /* Change the CRYP peripheral state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES GCM mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_GCM_ENCRYPT);

      /* Set the Initialization Vector */
      CRYPEx_GCMCCM_SetInitVector(hcryp, hcryp->Init.pInitVect);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Enable CRYP to start the init phase */
      __HAL_CRYP_ENABLE(hcryp);

      /* Get tick */
      tickstart = HAL_GetTick();

      while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
      {
        /* Check for the Timeout */
        if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }
      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Set the header phase */
      if(CRYPEx_GCMCCM_SetHeaderPhase(hcryp, hcryp->Init.Header, hcryp->Init.HeaderSize, 1U) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }
      /* Disable the CRYP peripheral */
      __HAL_CRYP_DISABLE(hcryp);

      /* Select payload phase once the header phase is performed */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    /* Set the input and output addresses and start DMA transfer */
    CRYPEx_GCMCCM_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Unlock process */
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
  * @brief  Initializes the CRYP peripheral in AES CCM encryption mode using interrupt.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pPlainData Pointer to the plaintext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pCypherData Pointer to the cyphertext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESCCM_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData)
{
  uint32_t tickstart = 0U;
  uint32_t inputaddr;
  uint32_t outputaddr;
  uint32_t headersize;
  uint32_t headeraddr;
  uint32_t loopcounter = 0U;
  uint32_t bufferidx = 0U;
  uint8_t blockb0[16U] = {0};/* Block B0 */
  uint8_t ctr[16U] = {0}; /* Counter */
  uint32_t b0addr = (uint32_t)blockb0;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pPlainData;
    outputaddr = (uint32_t)pCypherData;

    headersize = hcryp->Init.HeaderSize;
    headeraddr = (uint32_t)hcryp->Init.Header;

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pPlainData;
    hcryp->pCrypOutBuffPtr = pCypherData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP peripheral state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /************************ Formatting the header block *******************/
      if(headersize != 0U)
      {
        /* Check that the associated data (or header) length is lower than 2^16 - 2^8 = 65536 - 256 = 65280 */
        if(headersize < 65280U)
        {
          hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize >> 8) & 0xFF);
          hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize) & 0xFF);
          headersize += 2U;
        }
        else
        {
          /* Header is encoded as 0xff || 0xfe || [headersize]32, i.e., six octets */
          hcryp->Init.pScratch[bufferidx++] = 0xFFU;
          hcryp->Init.pScratch[bufferidx++] = 0xFEU;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0xff000000U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x00ff0000U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x0000ff00U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x000000ffU;
          headersize += 6U;
        }
        /* Copy the header buffer in internal buffer "hcryp->Init.pScratch" */
        for(loopcounter = 0U; loopcounter < headersize; loopcounter++)
        {
          hcryp->Init.pScratch[bufferidx++] = hcryp->Init.Header[loopcounter];
        }
        /* Check if the header size is modulo 16 */
        if ((headersize % 16U) != 0U)
        {
          /* Padd the header buffer with 0s till the hcryp->Init.pScratch length is modulo 16 */
          for(loopcounter = headersize; loopcounter <= ((headersize/16U) + 1U) * 16U; loopcounter++)
          {
            hcryp->Init.pScratch[loopcounter] = 0U;
          }
          /* Set the header size to modulo 16 */
          headersize = ((headersize/16U) + 1U) * 16U;
        }
        /* Set the pointer headeraddr to hcryp->Init.pScratch */
        headeraddr = (uint32_t)hcryp->Init.pScratch;
      }
      /*********************** Formatting the block B0 ************************/
      if(headersize != 0U)
      {
        blockb0[0U] = 0x40U;
      }
      /* Flags byte */
      /* blockb0[0] |= 0u | (((( (uint8_t) hcryp->Init.TagSize - 2) / 2) & 0x07 ) << 3 ) | ( ( (uint8_t) (15 - hcryp->Init.IVSize) - 1) & 0x07U) */
      blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)(((uint8_t)(hcryp->Init.TagSize - (uint8_t)(2))) >> 1) & (uint8_t)0x07) << 3);
      blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)((uint8_t)(15) - hcryp->Init.IVSize) - (uint8_t)1) & (uint8_t)0x07);

      for (loopcounter = 0U; loopcounter < hcryp->Init.IVSize; loopcounter++)
      {
        blockb0[loopcounter+1U] = hcryp->Init.pInitVect[loopcounter];
      }
      for ( ; loopcounter < 13U; loopcounter++)
      {
        blockb0[loopcounter+1U] = 0U;
      }

      blockb0[14U] = (Size >> 8U);
      blockb0[15U] = (Size & 0xFFU);

      /************************* Formatting the initial counter ***************/
      /* Byte 0:
         Bits 7 and 6 are reserved and shall be set to 0
         Bits 3, 4, and 5 shall also be set to 0, to ensure that all the counter
         blocks are distinct from B0
         Bits 0, 1, and 2 contain the same encoding of q as in B0
      */
      ctr[0U] = blockb0[0U] & 0x07U;
      /* byte 1 to NonceSize is the IV (Nonce) */
      for(loopcounter = 1U; loopcounter < hcryp->Init.IVSize + 1U; loopcounter++)
      {
        ctr[loopcounter] = blockb0[loopcounter];
      }
      /* Set the LSB to 1 */
      ctr[15U] |= 0x01U;

      /* Set the key */
      CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES CCM mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CCM_ENCRYPT);

      /* Set the Initialization Vector */
      CRYPEx_GCMCCM_SetInitVector(hcryp, ctr);

      /* Select init phase */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_INIT);

      b0addr = (uint32_t)blockb0;
      /* Write the blockb0 block in the IN FIFO */
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);

      /* Enable the CRYP peripheral */
      __HAL_CRYP_ENABLE(hcryp);

      /* Get tick */
      tickstart = HAL_GetTick();

      while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
      {
        /* Check for the Timeout */
        if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }
      /***************************** Header phase *****************************/
      if(headersize != 0U)
      {
        /* Select header phase */
        __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_HEADER);

        /* Enable Crypto processor */
        __HAL_CRYP_ENABLE(hcryp);

        for(loopcounter = 0U; (loopcounter < headersize); loopcounter+=16U)
        {
         /* Get tick */
         tickstart = HAL_GetTick();

          while(HAL_IS_BIT_CLR(hcryp->Instance->SR, CRYP_FLAG_IFEM))
          {
            /* Check for the Timeout */
            if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
            {
              /* Change state */
              hcryp->State = HAL_CRYP_STATE_TIMEOUT;

              /* Process Unlocked */
              __HAL_UNLOCK(hcryp);

              return HAL_TIMEOUT;
            }
          }
          /* Write the header block in the IN FIFO */
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
        }

        /* Get tick */
        tickstart = HAL_GetTick();

        while((hcryp->Instance->SR & CRYP_FLAG_BUSY) == CRYP_FLAG_BUSY)
        {
          /* Check for the Timeout */
          if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
          {
            /* Change state */
            hcryp->State = HAL_CRYP_STATE_TIMEOUT;

            /* Process Unlocked */
            __HAL_UNLOCK(hcryp);

            return HAL_TIMEOUT;
          }
        }
      }
      /* Save formatted counter into the scratch buffer pScratch */
      for(loopcounter = 0U; (loopcounter < 16U); loopcounter++)
      {
        hcryp->Init.pScratch[loopcounter] = ctr[loopcounter];
      }
      /* Reset bit 0 */
      hcryp->Init.pScratch[15U] &= 0xFEU;

      /* Select payload phase once the header phase is performed */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    /* Set the input and output addresses and start DMA transfer */
    CRYPEx_GCMCCM_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Unlock process */
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
  * @brief  Initializes the CRYP peripheral in AES GCM decryption mode using DMA.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer.
  * @param  Size Length of the cyphertext buffer, must be a multiple of 16
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESGCM_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
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

    /* Change the CRYP peripheral state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /* Set the key */
      CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES GCM decryption mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_GCM_DECRYPT);

      /* Set the Initialization Vector */
      CRYPEx_GCMCCM_SetInitVector(hcryp, hcryp->Init.pInitVect);

      /* Enable CRYP to start the init phase */
      __HAL_CRYP_ENABLE(hcryp);

      /* Get tick */
      tickstart = HAL_GetTick();

      while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
      {
        /* Check for the Timeout */
        if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;
        }
      }

      /* Set the header phase */
      if(CRYPEx_GCMCCM_SetHeaderPhase(hcryp, hcryp->Init.Header, hcryp->Init.HeaderSize, 1U) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }
      /* Disable the CRYP peripheral */
      __HAL_CRYP_DISABLE(hcryp);

      /* Select payload phase once the header phase is performed */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }

    /* Set the input and output addresses and start DMA transfer */
    CRYPEx_GCMCCM_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Unlock process */
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
  * @brief  Initializes the CRYP peripheral in AES CCM decryption mode using DMA
  *         then decrypted pCypherData. The cypher data are available in pPlainData.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pCypherData Pointer to the cyphertext buffer
  * @param  Size Length of the plaintext buffer, must be a multiple of 16
  * @param  pPlainData Pointer to the plaintext buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AESCCM_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData)
{
  uint32_t tickstart = 0U;
  uint32_t inputaddr;
  uint32_t outputaddr;
  uint32_t headersize;
  uint32_t headeraddr;
  uint32_t loopcounter = 0U;
  uint32_t bufferidx = 0U;
  uint8_t blockb0[16U] = {0};/* Block B0 */
  uint8_t ctr[16U] = {0}; /* Counter */
  uint32_t b0addr = (uint32_t)blockb0;

  if((hcryp->State == HAL_CRYP_STATE_READY) || (hcryp->Phase == HAL_CRYP_PHASE_PROCESS))
  {
    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pCypherData;
    outputaddr = (uint32_t)pPlainData;

    headersize = hcryp->Init.HeaderSize;
    headeraddr = (uint32_t)hcryp->Init.Header;

    hcryp->CrypInCount = Size;
    hcryp->pCrypInBuffPtr = pCypherData;
    hcryp->pCrypOutBuffPtr = pPlainData;
    hcryp->CrypOutCount = Size;

    /* Change the CRYP peripheral state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if(hcryp->Phase == HAL_CRYP_PHASE_READY)
    {
      /************************ Formatting the header block *******************/
      if(headersize != 0U)
      {
        /* Check that the associated data (or header) length is lower than 2^16 - 2^8 = 65536 - 256 = 65280 */
        if(headersize < 65280U)
        {
          hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize >> 8) & 0xFF);
          hcryp->Init.pScratch[bufferidx++] = (uint8_t) ((headersize) & 0xFF);
          headersize += 2U;
        }
        else
        {
          /* Header is encoded as 0xff || 0xfe || [headersize]32, i.e., six octets */
          hcryp->Init.pScratch[bufferidx++] = 0xFFU;
          hcryp->Init.pScratch[bufferidx++] = 0xFEU;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0xff000000U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x00ff0000U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x0000ff00U;
          hcryp->Init.pScratch[bufferidx++] = headersize & 0x000000ffU;
          headersize += 6U;
        }
        /* Copy the header buffer in internal buffer "hcryp->Init.pScratch" */
        for(loopcounter = 0U; loopcounter < headersize; loopcounter++)
        {
          hcryp->Init.pScratch[bufferidx++] = hcryp->Init.Header[loopcounter];
        }
        /* Check if the header size is modulo 16 */
        if ((headersize % 16U) != 0U)
        {
          /* Padd the header buffer with 0s till the hcryp->Init.pScratch length is modulo 16 */
          for(loopcounter = headersize; loopcounter <= ((headersize/16U) + 1U) * 16U; loopcounter++)
          {
            hcryp->Init.pScratch[loopcounter] = 0U;
          }
          /* Set the header size to modulo 16 */
          headersize = ((headersize/16U) + 1U) * 16U;
        }
        /* Set the pointer headeraddr to hcryp->Init.pScratch */
        headeraddr = (uint32_t)hcryp->Init.pScratch;
      }
      /*********************** Formatting the block B0 ************************/
      if(headersize != 0U)
      {
        blockb0[0U] = 0x40U;
      }
      /* Flags byte */
      /* blockb0[0] |= 0u | (((( (uint8_t) hcryp->Init.TagSize - 2) / 2) & 0x07 ) << 3 ) | ( ( (uint8_t) (15 - hcryp->Init.IVSize) - 1) & 0x07U) */
      blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)(((uint8_t)(hcryp->Init.TagSize - (uint8_t)(2))) >> 1) & (uint8_t)0x07) << 3);
      blockb0[0U] |= (uint8_t)((uint8_t)((uint8_t)((uint8_t)(15) - hcryp->Init.IVSize) - (uint8_t)1) & (uint8_t)0x07);

      for (loopcounter = 0U; loopcounter < hcryp->Init.IVSize; loopcounter++)
      {
        blockb0[loopcounter+1U] = hcryp->Init.pInitVect[loopcounter];
      }
      for ( ; loopcounter < 13U; loopcounter++)
      {
        blockb0[loopcounter+1U] = 0U;
      }

      blockb0[14U] = (Size >> 8U);
      blockb0[15U] = (Size & 0xFFU);

      /************************* Formatting the initial counter ***************/
      /* Byte 0:
         Bits 7 and 6 are reserved and shall be set to 0
         Bits 3, 4, and 5 shall also be set to 0, to ensure that all the counter
         blocks are distinct from B0
         Bits 0, 1, and 2 contain the same encoding of q as in B0
      */
      ctr[0U] = blockb0[0U] & 0x07U;
      /* byte 1 to NonceSize is the IV (Nonce) */
      for(loopcounter = 1U; loopcounter < hcryp->Init.IVSize + 1U; loopcounter++)
      {
        ctr[loopcounter] = blockb0[loopcounter];
      }
      /* Set the LSB to 1 */
      ctr[15U] |= 0x01U;

      /* Set the key */
      CRYPEx_GCMCCM_SetKey(hcryp, hcryp->Init.pKey, hcryp->Init.KeySize);

      /* Set the CRYP peripheral in AES CCM mode */
      __HAL_CRYP_SET_MODE(hcryp, CRYP_CR_ALGOMODE_AES_CCM_DECRYPT);

      /* Set the Initialization Vector */
      CRYPEx_GCMCCM_SetInitVector(hcryp, ctr);

      /* Select init phase */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_INIT);

      b0addr = (uint32_t)blockb0;
      /* Write the blockb0 block in the IN FIFO */
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);
      b0addr+=4U;
      hcryp->Instance->DR = *(uint32_t*)(b0addr);

      /* Enable the CRYP peripheral */
      __HAL_CRYP_ENABLE(hcryp);

      /* Get tick */
      tickstart = HAL_GetTick();

      while((CRYP->CR & CRYP_CR_CRYPEN) == CRYP_CR_CRYPEN)
      {
        /* Check for the Timeout */

        if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
        {
          /* Change state */
          hcryp->State = HAL_CRYP_STATE_TIMEOUT;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_TIMEOUT;

        }
      }
      /***************************** Header phase *****************************/
      if(headersize != 0U)
      {
        /* Select header phase */
        __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_HEADER);

        /* Enable Crypto processor */
        __HAL_CRYP_ENABLE(hcryp);

        for(loopcounter = 0U; (loopcounter < headersize); loopcounter+=16U)
        {
         /* Get tick */
         tickstart = HAL_GetTick();

          while(HAL_IS_BIT_CLR(hcryp->Instance->SR, CRYP_FLAG_IFEM))
          {
            /* Check for the Timeout */
            if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
            {
              /* Change state */
              hcryp->State = HAL_CRYP_STATE_TIMEOUT;

              /* Process Unlocked */
              __HAL_UNLOCK(hcryp);

              return HAL_TIMEOUT;
            }
          }
          /* Write the header block in the IN FIFO */
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
          hcryp->Instance->DR = *(uint32_t*)(headeraddr);
          headeraddr+=4U;
        }

        /* Get tick */
        tickstart = HAL_GetTick();

        while((hcryp->Instance->SR & CRYP_FLAG_BUSY) == CRYP_FLAG_BUSY)
        {
          /* Check for the Timeout */
          if((HAL_GetTick() - tickstart ) > CRYPEx_TIMEOUT_VALUE)
          {
            /* Change state */
            hcryp->State = HAL_CRYP_STATE_TIMEOUT;

            /* Process Unlocked */
            __HAL_UNLOCK(hcryp);

            return HAL_TIMEOUT;
          }
        }
      }
      /* Save formatted counter into the scratch buffer pScratch */
      for(loopcounter = 0U; (loopcounter < 16U); loopcounter++)
      {
        hcryp->Init.pScratch[loopcounter] = ctr[loopcounter];
      }
      /* Reset bit 0 */
      hcryp->Init.pScratch[15U] &= 0xFEU;
      /* Select payload phase once the header phase is performed */
      __HAL_CRYP_SET_PHASE(hcryp, CRYP_PHASE_PAYLOAD);

      /* Flush FIFO */
      __HAL_CRYP_FIFO_FLUSH(hcryp);

      /* Set the phase */
      hcryp->Phase = HAL_CRYP_PHASE_PROCESS;
    }
    /* Set the input and output addresses and start DMA transfer */
    CRYPEx_GCMCCM_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Unlock process */
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

/** @defgroup CRYPEx_Exported_Functions_Group2 CRYPEx IRQ handler management
 *  @brief   CRYPEx IRQ handler.
 *
@verbatim
  ==============================================================================
                ##### CRYPEx IRQ handler management #####
  ==============================================================================
[..]  This section provides CRYPEx IRQ handler function.

@endverbatim
  * @{
  */

/**
  * @brief  This function handles CRYPEx interrupt request.
  * @param  hcryp pointer to a CRYPEx_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */

void HAL_CRYPEx_GCMCCM_IRQHandler(CRYP_HandleTypeDef *hcryp)
{
  switch(CRYP->CR & CRYP_CR_ALGOMODE_DIRECTION)
  {
  case CRYP_CR_ALGOMODE_AES_GCM_ENCRYPT:
    HAL_CRYPEx_AESGCM_Encrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_AES_GCM_DECRYPT:
    HAL_CRYPEx_AESGCM_Decrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_AES_CCM_ENCRYPT:
    HAL_CRYPEx_AESCCM_Encrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  case CRYP_CR_ALGOMODE_AES_CCM_DECRYPT:
    HAL_CRYPEx_AESCCM_Decrypt_IT(hcryp, NULL, 0U, NULL);
    break;

  default:
    break;
  }
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* CRYP */

#if defined (AES)

/** @defgroup CRYPEx_Private_Constants CRYPEx Private Constants
  * @{
  */
#define CRYP_CCF_TIMEOUTVALUE                      22000U  /*!< CCF flag raising time-out value */
#define CRYP_BUSY_TIMEOUTVALUE                     22000U  /*!< BUSY flag reset time-out value  */

#define CRYP_POLLING_OFF                           0x0U    /*!< No polling when padding */
#define CRYP_POLLING_ON                            0x1U    /*!< Polling when padding    */
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup CRYPEx_Private_Functions CRYPEx Private Functions
 * @{
 */
static HAL_StatusTypeDef CRYP_ProcessData(CRYP_HandleTypeDef *hcryp, uint8_t* Input, uint16_t Ilength, uint8_t* Output, uint32_t Timeout);
static HAL_StatusTypeDef CRYP_ReadKey(CRYP_HandleTypeDef *hcryp, uint8_t* Output, uint32_t Timeout);
static void CRYP_SetDMAConfig(CRYP_HandleTypeDef *hcryp, uint32_t inputaddr, uint16_t Size, uint32_t outputaddr);
static void CRYP_GCMCMAC_SetDMAConfig(CRYP_HandleTypeDef *hcryp, uint32_t inputaddr, uint16_t Size, uint32_t outputaddr);
static void CRYP_GCMCMAC_DMAInCplt(DMA_HandleTypeDef *hdma);
static void CRYP_GCMCMAC_DMAError(DMA_HandleTypeDef *hdma);
static void CRYP_GCMCMAC_DMAOutCplt(DMA_HandleTypeDef *hdma);
static HAL_StatusTypeDef CRYP_WaitOnCCFlag(CRYP_HandleTypeDef *hcryp, uint32_t Timeout);
static HAL_StatusTypeDef CRYP_WaitOnBusyFlagReset(CRYP_HandleTypeDef *hcryp, uint32_t Timeout);
static void CRYP_DMAInCplt(DMA_HandleTypeDef *hdma);
static void CRYP_DMAOutCplt(DMA_HandleTypeDef *hdma);
static void CRYP_DMAError(DMA_HandleTypeDef *hdma);
static void CRYP_Padding(CRYP_HandleTypeDef *hcryp, uint32_t difflength, uint32_t polling);
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup CRYPEx_Exported_Functions CRYPEx Exported Functions
  * @{
  */


/** @defgroup CRYPEx_Exported_Functions_Group1 Extended callback function
 *  @brief    Extended callback functions.
 *
@verbatim
 ===============================================================================
                 ##### Extended callback functions #####
 ===============================================================================
    [..]  This section provides callback function:
      (+) Computation completed.

@endverbatim
  * @{
  */


/**
  * @brief  Computation completed callbacks.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval None
  */
__weak void HAL_CRYPEx_ComputationCpltCallback(CRYP_HandleTypeDef *hcryp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcryp);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_CRYPEx_ComputationCpltCallback can be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup CRYPEx_Exported_Functions_Group2 AES extended processing functions
 *  @brief   Extended processing functions.
 *
@verbatim
  ==============================================================================
                      ##### AES extended processing functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Encrypt plaintext or decrypt cipher text using AES algorithm in different chaining modes.
          Functions are generic (handles ECB, CBC and CTR and all modes) and are only differentiated
          based on the processing type. Three processing types are available:
          (++) Polling mode
          (++) Interrupt mode
          (++) DMA mode
      (+) Generate and authentication tag in addition to encrypt/decrypt a plain/cipher text using AES
          algorithm in different chaining modes.
          Functions are generic (handles GCM, GMAC, CMAC and CCM when applicable) and process only one phase
          so that steps can be skipped if so required. Functions are only differentiated based on the processing type.
          Three processing types are available:
          (++) Polling mode
          (++) Interrupt mode
          (++) DMA mode

@endverbatim
  * @{
  */

/**
  * @brief  Carry out in polling mode the ciphering or deciphering operation according to
  *         hcryp->Init structure fields, all operating modes (encryption, key derivation and/or decryption) and
  *         chaining modes ECB, CBC and CTR are managed by this function in polling mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pInputData Pointer to the plain text in case of encryption or cipher text in case of decryption
  *                     or key derivation+decryption.
  *                     Parameter is meaningless in case of key derivation.
  * @param  Size Length of the input data buffer in bytes, must be a multiple of 16.
  *               Parameter is meaningless in case of key derivation.
  * @param  pOutputData Pointer to the cipher text in case of encryption or plain text in case of
  *                     decryption/key derivation+decryption, or pointer to the derivative keys in
  *                     case of key derivation only.
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AES(CRYP_HandleTypeDef *hcryp, uint8_t *pInputData, uint16_t Size, uint8_t *pOutputData, uint32_t Timeout)
{

  if (hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Check parameters setting */
    if (hcryp->Init.OperatingMode == CRYP_ALGOMODE_KEYDERIVATION)
    {
      if (pOutputData == NULL)
      {
        return  HAL_ERROR;
      }
    }
    else
    {
      if ((pInputData == NULL) || (pOutputData == NULL) || (Size == 0U))
      {
        return  HAL_ERROR;
      }
    }

    /* Process Locked */
    __HAL_LOCK(hcryp);

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Call CRYP_ReadKey() API if the operating mode is set to
       key derivation, CRYP_ProcessData() otherwise  */
    if (hcryp->Init.OperatingMode == CRYP_ALGOMODE_KEYDERIVATION)
    {
      if(CRYP_ReadKey(hcryp, pOutputData, Timeout) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }
    }
    else
    {
      if(CRYP_ProcessData(hcryp, pInputData, Size, pOutputData, Timeout) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }
    }

    /* If the state has not been set to SUSPENDED, set it to
       READY, otherwise keep it as it is */
    if (hcryp->State != HAL_CRYP_STATE_SUSPENDED)
    {
      hcryp->State = HAL_CRYP_STATE_READY;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Carry out in interrupt mode the ciphering or deciphering operation according to
  *         hcryp->Init structure fields, all operating modes (encryption, key derivation and/or decryption) and
  *         chaining modes ECB, CBC and CTR are managed by this function in interrupt mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pInputData Pointer to the plain text in case of encryption or cipher text in case of decryption
  *                     or key derivation+decryption.
  *                     Parameter is meaningless in case of key derivation.
  * @param  Size Length of the input data buffer in bytes, must be a multiple of 16.
  *               Parameter is meaningless in case of key derivation.
  * @param  pOutputData Pointer to the cipher text in case of encryption or plain text in case of
  *                     decryption/key derivation+decryption, or pointer to the derivative keys in
  *                     case of key derivation only.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AES_IT(CRYP_HandleTypeDef *hcryp,  uint8_t *pInputData, uint16_t Size, uint8_t *pOutputData)
{
  uint32_t inputaddr = 0U;

  if(hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Check parameters setting */
    if (hcryp->Init.OperatingMode == CRYP_ALGOMODE_KEYDERIVATION)
    {
      if (pOutputData == NULL)
      {
        return  HAL_ERROR;
      }
    }
    else
    {
      if ((pInputData == NULL) || (pOutputData == NULL) || (Size == 0U))
      {
        return  HAL_ERROR;
      }
    }
    /* Process Locked */
    __HAL_LOCK(hcryp);

    /* If operating mode is not limited to key derivation only,
       get the buffers addresses and sizes */
    if (hcryp->Init.OperatingMode != CRYP_ALGOMODE_KEYDERIVATION)
    {

      hcryp->CrypInCount = Size;
      hcryp->pCrypInBuffPtr = pInputData;
      hcryp->pCrypOutBuffPtr = pOutputData;
      hcryp->CrypOutCount = Size;
    }

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

      /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Enable Computation Complete Flag and Error Interrupts */
    __HAL_CRYP_ENABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);

    /* If operating mode is key derivation only, the input data have
       already been entered during the initialization process. For
       the other operating modes, they are fed to the CRYP hardware
       block at this point. */
    if (hcryp->Init.OperatingMode != CRYP_ALGOMODE_KEYDERIVATION)
    {
      /* Initiate the processing under interrupt in entering
         the first input data */
      inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;
      /* Increment/decrement instance pointer/counter */
      hcryp->pCrypInBuffPtr += 16U;
      hcryp->CrypInCount -= 16U;
      /* Write the first input block in the Data Input register */
      hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
      inputaddr+=4U;
      hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
      inputaddr+=4U;
      hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
      inputaddr+=4U;
      hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
    }

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Carry out in DMA mode the ciphering or deciphering operation according to
  *         hcryp->Init structure fields.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pInputData Pointer to the plain text in case of encryption or cipher text in case of decryption
  *                     or key derivation+decryption.
  * @param  Size Length of the input data buffer in bytes, must be a multiple of 16.
  * @param  pOutputData Pointer to the cipher text in case of encryption or plain text in case of
  *                     decryption/key derivation+decryption.
  * @note   Chaining modes ECB, CBC and CTR are managed by this function in DMA mode.
  * @note   Supported operating modes are encryption, decryption and key derivation with decryption.
  * @note   No DMA channel is provided for key derivation only and therefore, access to AES_KEYRx
  *         registers must be done by software.
  * @note   This API is not applicable to key derivation only; for such a mode, access to AES_KEYRx
  *         registers must be done by software thru HAL_CRYPEx_AES() or HAL_CRYPEx_AES_IT() APIs.
  * @note   pInputData and pOutputData buffers must be 32-bit aligned to ensure a correct DMA transfer to and from the IP.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AES_DMA(CRYP_HandleTypeDef *hcryp,  uint8_t *pInputData, uint16_t Size, uint8_t *pOutputData)
{
  uint32_t inputaddr = 0U;
  uint32_t outputaddr = 0U;

  if (hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* Check parameters setting */
    if (hcryp->Init.OperatingMode == CRYP_ALGOMODE_KEYDERIVATION)
    {
      /* no DMA channel is provided for key derivation operating mode,
         access to AES_KEYRx registers must be done by software */
      return  HAL_ERROR;
    }
    else
    {
      if ((pInputData == NULL) || (pOutputData == NULL) || (Size == 0U))
      {
        return  HAL_ERROR;
      }
    }

    /* Process Locked */
    __HAL_LOCK(hcryp);

    inputaddr  = (uint32_t)pInputData;
    outputaddr = (uint32_t)pOutputData;

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Set the input and output addresses and start DMA transfer */
    CRYP_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Carry out in polling mode the authentication tag generation as well as the ciphering or deciphering
  *         operation according to hcryp->Init structure fields.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pInputData
  *         - pointer to payload data in GCM payload phase,
  *         - pointer to B0 block in CMAC header phase,
  *         - pointer to C block in CMAC final phase.
  *         - Parameter is meaningless in case of GCM/GMAC init, header and final phases.
  * @param  Size
  *         - length of the input payload data buffer in bytes,
  *         - length of B0 block (in bytes) in CMAC header phase,
  *         - length of C block (in bytes) in CMAC final phase.
  *         - Parameter is meaningless in case of GCM/GMAC init and header phases.
  * @param  pOutputData
  *         - pointer to plain or cipher text in GCM payload phase,
  *         - pointer to authentication tag in GCM/GMAC and CMAC final phases.
  *         - Parameter is meaningless in case of GCM/GMAC init and header phases
  *           and in case of CMAC header phase.
  * @param  Timeout Specify Timeout value
  * @note   Supported operating modes are encryption and decryption, supported chaining modes are GCM, GMAC, CMAC and CCM when the latter is applicable.
  * @note   Phases are singly processed according to hcryp->Init.GCMCMACPhase so that steps in these specific chaining modes
  *         can be skipped by the user if so required.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AES_Auth(CRYP_HandleTypeDef *hcryp, uint8_t *pInputData, uint64_t Size, uint8_t *pOutputData, uint32_t Timeout)
{
  uint32_t index          = 0U;
  uint32_t inputaddr      = 0U;
  uint32_t outputaddr     = 0U;
  uint32_t tagaddr        = 0U;
  uint64_t headerlength   = 0U;
  uint64_t inputlength    = 0U;
  uint64_t payloadlength  = 0U;
  uint32_t difflength     = 0U;
  uint32_t addhoc_process = 0U;

  if (hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* input/output parameters check */
    if (hcryp->Init.GCMCMACPhase == CRYP_HEADER_PHASE)
    {
      if ((hcryp->Init.Header != NULL) && (hcryp->Init.HeaderSize == 0U))
      {
        return  HAL_ERROR;
      }
#if defined(AES_CR_NPBLB)
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC)
#else
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)
#endif
      {
        /* In case of CMAC (or CCM) header phase resumption, we can have pInputData = NULL and  Size = 0 */
        if (((pInputData != NULL) && (Size == 0U)) || ((pInputData == NULL) && (Size != 0U)))
        {
          return  HAL_ERROR;
        }
      }
    }
    else if (hcryp->Init.GCMCMACPhase == CRYP_PAYLOAD_PHASE)
    {
      if ((pInputData == NULL) || (pOutputData == NULL) || (Size == 0U))
      {
        return  HAL_ERROR;
      }
    }
    else if (hcryp->Init.GCMCMACPhase == CRYP_FINAL_PHASE)
    {
      if (pOutputData == NULL)
      {
        return  HAL_ERROR;
      }
#if defined(AES_CR_NPBLB)
      if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC) && (pInputData == NULL))
#else
      if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC) && (pInputData == NULL))
#endif
      {
        return  HAL_ERROR;
      }
    }

    /* Process Locked */
    __HAL_LOCK(hcryp);

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /*==============================================*/
    /* GCM/GMAC (or CCM when applicable) init phase */
    /*==============================================*/
    /* In case of init phase, the input data (Key and Initialization Vector) have
       already been entered during the initialization process. Therefore, the
       API just waits for the CCF flag to be set. */
    if (hcryp->Init.GCMCMACPhase == CRYP_INIT_PHASE)
    {
      /* just wait for hash computation */
      if(CRYP_WaitOnCCFlag(hcryp, Timeout) != HAL_OK)
      {
        hcryp->State = HAL_CRYP_STATE_READY;
        __HAL_UNLOCK(hcryp);
        return HAL_TIMEOUT;
      }

      /* Clear CCF Flag */
      __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
      /* Mark that the initialization phase is over */
      hcryp->Phase = HAL_CRYP_PHASE_INIT_OVER;
    }
    /*=====================================*/
    /* GCM/GMAC or (CCM/)CMAC header phase */
    /*=====================================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_HEADER_PHASE)
    {
      /* Set header phase; for GCM or GMAC, set data-byte at this point */
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)
      {
        MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH|AES_CR_DATATYPE, CRYP_HEADER_PHASE|hcryp->Init.DataType);
      }
      else
      {
        MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, CRYP_HEADER_PHASE);
      }

      /* Enable the Peripheral */
      __HAL_CRYP_ENABLE();

#if !defined(AES_CR_NPBLB)
      /* in case of CMAC, enter B0 block in header phase, before the header itself. */
      /* If Size = 0 (possible case of resumption after CMAC header phase suspension),
         skip these steps and go directly to header buffer feeding to the HW */
      if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC) && (Size != 0U))
      {
        inputaddr = (uint32_t)pInputData;

        for(index=0U; (index < Size); index += 16U)
        {
          /* Write the Input block in the Data Input register */
          hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
          inputaddr+=4U;
          hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
          inputaddr+=4U;
          hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
          inputaddr+=4U;
          hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
          inputaddr+=4U;

          if(CRYP_WaitOnCCFlag(hcryp, Timeout) != HAL_OK)
          {
            hcryp->State = HAL_CRYP_STATE_READY;
            __HAL_UNLOCK(hcryp);
            return HAL_TIMEOUT;
          }
          /* Clear CCF Flag */
          __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);

          /* If the suspension flag has been raised and if the processing is not about
           to end, suspend processing */
          if ((hcryp->SuspendRequest == HAL_CRYP_SUSPEND) && ((index+16U) < Size))
          {
            /* reset SuspendRequest */
            hcryp->SuspendRequest = HAL_CRYP_SUSPEND_NONE;
            /* Change the CRYP state */
            hcryp->State = HAL_CRYP_STATE_SUSPENDED;
            /* Mark that the header phase is over */
            hcryp->Phase = HAL_CRYP_PHASE_HEADER_SUSPENDED;

           /* Save current reading and writing locations of Input and Output buffers */
           hcryp->pCrypInBuffPtr  =  (uint8_t *)inputaddr;
           /* Save the total number of bytes (B blocks + header) that remain to be
              processed at this point */
           hcryp->CrypInCount     =  hcryp->Init.HeaderSize + Size - (index+16U);

           /* Process Unlocked */
            __HAL_UNLOCK(hcryp);

            return HAL_OK;
          }
        } /* for(index=0; (index < Size); index += 16) */
      }
#endif /* !defined(AES_CR_NPBLB) */

      /* Enter header */
      inputaddr = (uint32_t)hcryp->Init.Header;
      /* Local variable headerlength is a number of bytes multiple of 128 bits,
         remaining header data (if any) are handled after this loop */
      headerlength =  (((hcryp->Init.HeaderSize)/16U)*16U) ;
      if ((hcryp->Init.HeaderSize % 16U) != 0U)
      {
        difflength = (uint32_t) (hcryp->Init.HeaderSize - headerlength);
      }
      for(index=0U; index < headerlength; index += 16U)
      {
        /* Write the Input block in the Data Input register */
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;

        if(CRYP_WaitOnCCFlag(hcryp, Timeout) != HAL_OK)
        {
          hcryp->State = HAL_CRYP_STATE_READY;
          __HAL_UNLOCK(hcryp);
          return HAL_TIMEOUT;
        }
        /* Clear CCF Flag */
        __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);

        /* If the suspension flag has been raised and if the processing is not about
         to end, suspend processing */
        if ((hcryp->SuspendRequest == HAL_CRYP_SUSPEND) && ((index+16U) < headerlength))
        {
          /* reset SuspendRequest */
          hcryp->SuspendRequest = HAL_CRYP_SUSPEND_NONE;
          /* Change the CRYP state */
          hcryp->State = HAL_CRYP_STATE_SUSPENDED;
          /* Mark that the header phase is over */
          hcryp->Phase = HAL_CRYP_PHASE_HEADER_SUSPENDED;

         /* Save current reading and writing locations of Input and Output buffers */
         hcryp->pCrypInBuffPtr  =  (uint8_t *)inputaddr;
         /* Save the total number of bytes that remain to be processed at this point */
          hcryp->CrypInCount =  hcryp->Init.HeaderSize - (index+16U);

         /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_OK;
        }
      }

      /* Case header length is not a multiple of 16 bytes */
      if (difflength != 0U)
      {
        hcryp->pCrypInBuffPtr  =  (uint8_t *)inputaddr;
        CRYP_Padding(hcryp, difflength, CRYP_POLLING_ON);
      }

      /* Mark that the header phase is over */
      hcryp->Phase = HAL_CRYP_PHASE_HEADER_OVER;
    }
    /*============================================*/
    /* GCM (or CCM when applicable) payload phase */
    /*============================================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_PAYLOAD_PHASE)
    {

      MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, CRYP_PAYLOAD_PHASE);

      /* if the header phase has been bypassed, AES must be enabled again */
      if (hcryp->Phase == HAL_CRYP_PHASE_INIT_OVER)
      {
        __HAL_CRYP_ENABLE();
      }

      inputaddr  = (uint32_t)pInputData;
      outputaddr = (uint32_t)pOutputData;

      /* Enter payload */
      /* Specific handling to manage payload last block size less than 128 bits */
      if ((Size % 16U) != 0U)
      {
        payloadlength = (Size/16U) * 16U;
        difflength = (uint32_t) (Size - payloadlength);
        addhoc_process = 1U;
      }
      else
      {
        payloadlength = Size;
        addhoc_process = 0U;
      }

      /* Feed payload */
      for(index=0U; index < payloadlength; index += 16U)
      {
        /* Write the Input block in the Data Input register */
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;

        if(CRYP_WaitOnCCFlag(hcryp, Timeout) != HAL_OK)
        {
          hcryp->State = HAL_CRYP_STATE_READY;
          __HAL_UNLOCK(hcryp);
          return HAL_TIMEOUT;
        }

        /* Clear CCF Flag */
        __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);

        /* Retrieve output data: read the output block
           from the Data Output Register */
        *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
        outputaddr+=4U;
        *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
        outputaddr+=4U;
        *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
        outputaddr+=4U;
        *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
        outputaddr+=4U;

        /* If the suspension flag has been raised and if the processing is not about
         to end, suspend processing */
        if ((hcryp->SuspendRequest == HAL_CRYP_SUSPEND) && ((index+16U) < payloadlength))
        {
          /* no flag waiting under IRQ handling */
          if (hcryp->Init.OperatingMode == CRYP_ALGOMODE_ENCRYPT)
          {
            /* Ensure that Busy flag is reset */
            if(CRYP_WaitOnBusyFlagReset(hcryp, CRYP_BUSY_TIMEOUTVALUE) != HAL_OK)
            {
              hcryp->State = HAL_CRYP_STATE_READY;
              __HAL_UNLOCK(hcryp);
              return HAL_TIMEOUT;
            }
          }
          /* reset SuspendRequest */
          hcryp->SuspendRequest = HAL_CRYP_SUSPEND_NONE;
          /* Change the CRYP state */
          hcryp->State = HAL_CRYP_STATE_SUSPENDED;
          /* Mark that the header phase is over */
          hcryp->Phase = HAL_CRYP_PHASE_HEADER_SUSPENDED;

          /* Save current reading and writing locations of Input and Output buffers */
          hcryp->pCrypOutBuffPtr =  (uint8_t *)outputaddr;
          hcryp->pCrypInBuffPtr  =  (uint8_t *)inputaddr;
          /* Save the number of bytes that remain to be processed at this point */
          hcryp->CrypInCount     =  Size - (index+16U);

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          return HAL_OK;
        }

      }

      /* Additional processing to manage GCM(/CCM) encryption and decryption cases when
         payload last block size less than 128 bits */
      if (addhoc_process == 1U)
      {
        hcryp->pCrypInBuffPtr  =  (uint8_t *)inputaddr;
        hcryp->pCrypOutBuffPtr =  (uint8_t *)outputaddr;
        CRYP_Padding(hcryp, difflength, CRYP_POLLING_ON);
      } /* (addhoc_process == 1) */

      /* Mark that the payload phase is over */
      hcryp->Phase = HAL_CRYP_PHASE_PAYLOAD_OVER;
    }
    /*====================================*/
    /* GCM/GMAC or (CCM/)CMAC final phase */
    /*====================================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_FINAL_PHASE)
    {
      tagaddr = (uint32_t)pOutputData;

#if defined(AES_CR_NPBLB)
     /* By default, clear NPBLB field */
      CLEAR_BIT(hcryp->Instance->CR, AES_CR_NPBLB);
#endif

      MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, CRYP_FINAL_PHASE);

      /* if the header and payload phases have been bypassed, AES must be enabled again */
      if (hcryp->Phase == HAL_CRYP_PHASE_INIT_OVER)
      {
        __HAL_CRYP_ENABLE();
      }

      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)
      {
        headerlength = hcryp->Init.HeaderSize * 8U; /* Header length in bits */
        inputlength = Size * 8U;                    /* input length in bits */


        if(hcryp->Init.DataType == CRYP_DATATYPE_1B)
        {
          hcryp->Instance->DINR = __RBIT((headerlength)>>32U);
          hcryp->Instance->DINR = __RBIT(headerlength);
          hcryp->Instance->DINR = __RBIT((inputlength)>>32U);
          hcryp->Instance->DINR = __RBIT(inputlength);
        }
        else if(hcryp->Init.DataType == CRYP_DATATYPE_8B)
        {
          hcryp->Instance->DINR = __REV((headerlength)>>32U);
          hcryp->Instance->DINR = __REV(headerlength);
          hcryp->Instance->DINR = __REV((inputlength)>>32U);
          hcryp->Instance->DINR = __REV(inputlength);
        }
        else if(hcryp->Init.DataType == CRYP_DATATYPE_16B)
        {
          hcryp->Instance->DINR = __ROR((headerlength)>>32U, 16U);
          hcryp->Instance->DINR = __ROR(headerlength, 16U);
          hcryp->Instance->DINR = __ROR((inputlength)>>32U, 16U);
          hcryp->Instance->DINR = __ROR(inputlength, 16U);
        }
        else if(hcryp->Init.DataType == CRYP_DATATYPE_32B)
        {
          hcryp->Instance->DINR = (uint32_t)(headerlength>>32U);
          hcryp->Instance->DINR = (uint32_t)(headerlength);
          hcryp->Instance->DINR = (uint32_t)(inputlength>>32U);
          hcryp->Instance->DINR = (uint32_t)(inputlength);
        }
      }
#if !defined(AES_CR_NPBLB)
      else if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)
      {
        inputaddr  = (uint32_t)pInputData;
        /* Enter the last block made of a 128-bit value formatted
           from the original B0 packet. */
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
      }
#endif


      if(CRYP_WaitOnCCFlag(hcryp, Timeout) != HAL_OK)
      {
          hcryp->State = HAL_CRYP_STATE_READY;
          __HAL_UNLOCK(hcryp);
          return HAL_TIMEOUT;
      }

      /* Read the Auth TAG in the Data Out register */
      *(uint32_t*)(tagaddr) = hcryp->Instance->DOUTR;
      tagaddr+=4U;
      *(uint32_t*)(tagaddr) = hcryp->Instance->DOUTR;
      tagaddr+=4U;
      *(uint32_t*)(tagaddr) = hcryp->Instance->DOUTR;
      tagaddr+=4U;
      *(uint32_t*)(tagaddr) = hcryp->Instance->DOUTR;


      /* Clear CCF Flag */
      __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
      /* Mark that the final phase is over */
      hcryp->Phase = HAL_CRYP_PHASE_FINAL_OVER;
      /* Disable the Peripheral */
      __HAL_CRYP_DISABLE();
    }
    /*=================================================*/
    /* case incorrect hcryp->Init.GCMCMACPhase setting */
    /*=================================================*/
    else
    {
      hcryp->State = HAL_CRYP_STATE_ERROR;
      __HAL_UNLOCK(hcryp);
      return HAL_ERROR;
    }

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}




/**
  * @brief  Carry out in interrupt mode the authentication tag generation as well as the ciphering or deciphering
  *         operation according to hcryp->Init structure fields.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pInputData
  *         - pointer to payload data in GCM payload phase,
  *         - pointer to B0 block in CMAC header phase,
  *         - pointer to C block in CMAC final phase.
  *         Parameter is meaningless in case of GCM/GMAC init, header and final phases.
  * @param  Size
  *         - length of the input payload data buffer in bytes,
  *         - length of B0 block (in bytes) in CMAC header phase,
  *         - length of C block (in bytes) in CMAC final phase.
  *         - Parameter is meaningless in case of GCM/GMAC init and header phases.
  * @param  pOutputData
  *         - pointer to plain or cipher text in GCM payload phase,
  *         - pointer to authentication tag in GCM/GMAC and CMAC final phases.
  *         - Parameter is meaningless in case of GCM/GMAC init and header phases
  *           and in case of CMAC header phase.
  * @note   Supported operating modes are encryption and decryption, supported chaining modes are GCM, GMAC and CMAC.
  * @note   Phases are singly processed according to hcryp->Init.GCMCMACPhase so that steps in these specific chaining modes
  *         can be skipped by the user if so required.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AES_Auth_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pInputData, uint64_t Size, uint8_t *pOutputData)
{

  uint32_t inputaddr      = 0U;
  uint64_t headerlength   = 0U;
  uint64_t inputlength    = 0U;
  uint32_t index          = 0U;
  uint32_t addhoc_process = 0U;
  uint32_t difflength     = 0U;
  uint32_t difflengthmod4 = 0U;
  uint32_t mask[3U]       = {0x0FFU, 0x0FFFFU, 0x0FFFFFFU};


  if (hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* input/output parameters check */
    if (hcryp->Init.GCMCMACPhase == CRYP_HEADER_PHASE)
    {
      if ((hcryp->Init.Header != NULL) && (hcryp->Init.HeaderSize == 0U))
      {
        return  HAL_ERROR;
      }
#if defined(AES_CR_NPBLB)
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC)
#else
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)
#endif
      {
        /* In case of CMAC header phase resumption, we can have pInputData = NULL and  Size = 0 */
        if (((pInputData != NULL) && (Size == 0U)) || ((pInputData == NULL) && (Size != 0U)))
        {
          return  HAL_ERROR;
        }
      }
    }
    else if (hcryp->Init.GCMCMACPhase == CRYP_PAYLOAD_PHASE)
    {
      if ((pInputData == NULL) || (pOutputData == NULL) || (Size == 0U))
      {
        return  HAL_ERROR;
      }
    }
    else if (hcryp->Init.GCMCMACPhase == CRYP_FINAL_PHASE)
    {
      if (pOutputData == NULL)
      {
        return  HAL_ERROR;
      }
#if defined(AES_CR_NPBLB)
      if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC) && (pInputData == NULL))
#else
      if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC) && (pInputData == NULL))
#endif
      {
        return  HAL_ERROR;
      }
    }

    /* Process Locked */
    __HAL_LOCK(hcryp);

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    /* Enable Computation Complete Flag and Error Interrupts */
    __HAL_CRYP_ENABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);

    /*==============================================*/
    /* GCM/GMAC (or CCM when applicable) init phase */
    /*==============================================*/
    if (hcryp->Init.GCMCMACPhase == CRYP_INIT_PHASE)
    {
    /* In case of init phase, the input data (Key and Initialization Vector) have
       already been entered during the initialization process. Therefore, the
       software just waits for the CCF interrupt to be raised and which will
       be handled by CRYP_AES_Auth_IT() API. */
    }
    /*=====================================*/
    /* GCM/GMAC or (CCM/)CMAC header phase */
    /*=====================================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_HEADER_PHASE)
    {

#if defined(AES_CR_NPBLB)
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC)
#else
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)
#endif
      {
        /* In case of CMAC, B blocks are first entered, before the header.
           Therefore, B blocks and the header are entered back-to-back
           as if it was only one single block.
           However, in case of resumption after suspension, if all the
           B blocks have been entered (in that case, Size = 0), only the
           remainder of the non-processed header bytes are entered. */
          if (Size != 0U)
          {
            hcryp->CrypInCount = Size + hcryp->Init.HeaderSize;
            hcryp->pCrypInBuffPtr = pInputData;
          }
          else
          {
            hcryp->CrypInCount = hcryp->Init.HeaderSize;
            hcryp->pCrypInBuffPtr = hcryp->Init.Header;
          }
      }
      else
      {
        /* Get the header addresses and sizes */
        hcryp->CrypInCount = hcryp->Init.HeaderSize;
        hcryp->pCrypInBuffPtr = hcryp->Init.Header;
      }

      inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;

      /* Set header phase; for GCM or GMAC, set data-byte at this point */
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)
      {
        MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH|AES_CR_DATATYPE, CRYP_HEADER_PHASE|hcryp->Init.DataType);
      }
      else
      {
        MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, CRYP_HEADER_PHASE);
      }

      /* Enable the Peripheral */
      __HAL_CRYP_ENABLE();

      /* Increment/decrement instance pointer/counter */
      if (hcryp->CrypInCount == 0U)
      {
        /* Case of no header */
        hcryp->State = HAL_CRYP_STATE_READY;
        return HAL_OK;
      }
      else if (hcryp->CrypInCount < 16U)
      {
        hcryp->CrypInCount = 0U;
        addhoc_process = 1U;
        difflength = (uint32_t) (hcryp->Init.HeaderSize);
        difflengthmod4 = difflength%4U;
      }
      else
      {
        hcryp->pCrypInBuffPtr += 16U;
        hcryp->CrypInCount -= 16U;
      }

#if defined(AES_CR_NPBLB)
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC)
#else
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)
#endif
      {
        if (hcryp->CrypInCount == hcryp->Init.HeaderSize)
        {
          /* All B blocks will have been entered after the next
             four DINR writing, so point at header buffer for
             the next iteration */
          hcryp->pCrypInBuffPtr = hcryp->Init.Header;
        }
      }

      /* Enter header first block to initiate the process
         in the Data Input register */
      if (addhoc_process == 0U)
      {
        /* Header has size equal or larger than 128 bits */
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
      }
      else
      {
        /* Header has size less than 128 bits */
        /* Enter complete words when possible */
        for(index=0U; index < (difflength/4U); index ++)
        {
          /* Write the Input block in the Data Input register */
          hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
          inputaddr+=4U;
        }
        /* Enter incomplete word padded with zeroes if applicable
          (case of header length not a multiple of 32-bits) */
        if (difflengthmod4 != 0U)
        {
          hcryp->Instance->DINR = ((*(uint32_t*)(inputaddr)) & mask[difflengthmod4-1U]);
        }
        /* Pad with zero-words to reach 128-bit long block and wrap-up header feeding to the IP */
        for(index=0U; index < (4U - ((difflength+3U)/4U)); index ++)
        {
          hcryp->Instance->DINR = 0U;
        }

      }
    }
    /*============================================*/
    /* GCM (or CCM when applicable) payload phase */
    /*============================================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_PAYLOAD_PHASE)
    {
      /* Get the buffer addresses and sizes */
      hcryp->CrypInCount = Size;
      hcryp->pCrypInBuffPtr = pInputData;
      hcryp->pCrypOutBuffPtr = pOutputData;
      hcryp->CrypOutCount = Size;

      inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;

      MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, CRYP_GCM_PAYLOAD_PHASE);

      /* if the header phase has been bypassed, AES must be enabled again */
      if (hcryp->Phase == HAL_CRYP_PHASE_INIT_OVER)
      {
        __HAL_CRYP_ENABLE();
      }

     /* Specific handling to manage payload size less than 128 bits */
      if (Size < 16U)
      {
#if defined(AES_CR_NPBLB)
        /* In case of GCM encryption or CCM decryption, specify the number of padding
           bytes in last block of payload */
        if (READ_BIT(hcryp->Instance->CR, AES_CR_GCMPH) == CRYP_PAYLOAD_PHASE)
        {
          if (((READ_BIT(hcryp->Instance->CR, AES_CR_CHMOD) == CRYP_CHAINMODE_AES_GCM_GMAC)
               &&  (READ_BIT(hcryp->Instance->CR, AES_CR_MODE) == CRYP_ALGOMODE_ENCRYPT))
           ||  ((READ_BIT(hcryp->Instance->CR, AES_CR_CHMOD) == CRYP_CHAINMODE_AES_CCM_CMAC)
               &&  (READ_BIT(hcryp->Instance->CR, AES_CR_MODE) == CRYP_ALGOMODE_DECRYPT)))
          {
            /* Set NPBLB field in writing the number of padding bytes
               for the last block of payload */
            MODIFY_REG(hcryp->Instance->CR, AES_CR_NPBLB, 16U - difflength);
          }
        }
#else
        /* Software workaround applied to GCM encryption only */
        if (hcryp->Init.OperatingMode == CRYP_ALGOMODE_ENCRYPT)
        {
          /* Change the mode configured in CHMOD bits of CR register to select CTR mode */
          __HAL_CRYP_SET_CHAININGMODE(CRYP_CHAINMODE_AES_CTR);
        }
#endif

        /* Set hcryp->CrypInCount to 0 (no more data to enter) */
        hcryp->CrypInCount = 0U;

        /*  Insert the last block (which size is inferior to 128 bits) padded with zeroes,
            to have a complete block of 128 bits */
        difflength = (uint32_t) (Size);
        difflengthmod4 = difflength%4U;
        /*  Insert the last block (which size is inferior to 128 bits) padded with zeroes
            to have a complete block of 128 bits */
        for(index=0U; index < (difflength/4U); index ++)
        {
          /* Write the Input block in the Data Input register */
          hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
          inputaddr+=4U;
        }
        /* If required, manage input data size not multiple of 32 bits */
        if (difflengthmod4 != 0U)
        {
          hcryp->Instance->DINR = ((*(uint32_t*)(inputaddr)) & mask[difflengthmod4-1U]);
        }
        /* Wrap-up in padding with zero-words if applicable */
        for(index=0U; index < (4U - ((difflength+3U)/4U)); index ++)
        {
          hcryp->Instance->DINR = 0U;
        }
      }
      else
      {
        /* Increment/decrement instance pointer/counter */
        hcryp->pCrypInBuffPtr += 16U;
        hcryp->CrypInCount -= 16U;

        /* Enter payload first block to initiate the process
           in the Data Input register */
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
      }
    }
    /*====================================*/
    /* GCM/GMAC or (CCM/)CMAC final phase */
    /*====================================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_FINAL_PHASE)
    {
       hcryp->pCrypOutBuffPtr = pOutputData;

#if defined(AES_CR_NPBLB)
     /* By default, clear NPBLB field */
      CLEAR_BIT(hcryp->Instance->CR, AES_CR_NPBLB);
#endif

       MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, CRYP_FINAL_PHASE);

      /* if the header and payload phases have been bypassed, AES must be enabled again */
      if (hcryp->Phase == HAL_CRYP_PHASE_INIT_OVER)
      {
        __HAL_CRYP_ENABLE();
      }

      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)
      {
        headerlength = hcryp->Init.HeaderSize * 8U; /* Header length in bits */
        inputlength = Size * 8U;                    /* Input length in bits */
        /* Write the number of bits in the header on 64 bits followed by the number
           of bits in the payload on 64 bits as well */
        if(hcryp->Init.DataType == CRYP_DATATYPE_1B)
        {
          hcryp->Instance->DINR = __RBIT((headerlength)>>32U);
          hcryp->Instance->DINR = __RBIT(headerlength);
          hcryp->Instance->DINR = __RBIT((inputlength)>>32U);
          hcryp->Instance->DINR = __RBIT(inputlength);
        }
        else if(hcryp->Init.DataType == CRYP_DATATYPE_8B)
        {
          hcryp->Instance->DINR = __REV((headerlength)>>32U);
          hcryp->Instance->DINR = __REV(headerlength);
          hcryp->Instance->DINR = __REV((inputlength)>>32U);
          hcryp->Instance->DINR = __REV(inputlength);
        }
        else if(hcryp->Init.DataType == CRYP_DATATYPE_16B)
        {
          hcryp->Instance->DINR = __ROR((headerlength)>>32U, 16U);
          hcryp->Instance->DINR = __ROR(headerlength, 16U);
          hcryp->Instance->DINR = __ROR((inputlength)>>32U, 16U);
          hcryp->Instance->DINR = __ROR(inputlength, 16U);
        }
        else if(hcryp->Init.DataType == CRYP_DATATYPE_32B)
        {
          hcryp->Instance->DINR = (uint32_t)(headerlength>>32U);
          hcryp->Instance->DINR = (uint32_t)(headerlength);
          hcryp->Instance->DINR = (uint32_t)(inputlength>>32U);
          hcryp->Instance->DINR = (uint32_t)(inputlength);
        }
      }
#if !defined(AES_CR_NPBLB)
      else if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)
      {
        inputaddr  = (uint32_t)pInputData;
        /* Enter the last block made of a 128-bit value formatted
           from the original B0 packet. */
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
      }
#endif
    }
    /*=================================================*/
    /* case incorrect hcryp->Init.GCMCMACPhase setting */
    /*=================================================*/
    else
    {
      hcryp->State = HAL_CRYP_STATE_ERROR;
      return HAL_ERROR;
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}




/**
  * @brief  Carry out in DMA mode the authentication tag generation as well as the ciphering or deciphering
  *         operation according to hcryp->Init structure fields.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  pInputData
  *         - pointer to payload data in GCM payload phase,
  *         - pointer to B0 block in CMAC header phase,
  *         - pointer to C block in CMAC final phase.
  *         - Parameter is meaningless in case of GCM/GMAC init, header and final phases.
  * @param  Size
  *         - length of the input payload data buffer in bytes,
  *         - length of B block (in bytes) in CMAC header phase,
  *         - length of C block (in bytes) in CMAC final phase.
  *         - Parameter is meaningless in case of GCM/GMAC init and header phases.
  * @param  pOutputData
  *         - pointer to plain or cipher text in GCM payload phase,
  *         - pointer to authentication tag in GCM/GMAC and CMAC final phases.
  *         - Parameter is meaningless in case of GCM/GMAC init and header phases
  *           and in case of CMAC header phase.
  * @note   Supported operating modes are encryption and decryption, supported chaining modes are GCM, GMAC and CMAC.
  * @note   Phases are singly processed according to hcryp->Init.GCMCMACPhase so that steps in these specific chaining modes
  *         can be skipped by the user if so required.
  * @note   pInputData and pOutputData buffers must be 32-bit aligned to ensure a correct DMA transfer to and from the IP.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRYPEx_AES_Auth_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pInputData, uint64_t Size, uint8_t *pOutputData)
{
  uint32_t inputaddr      = 0U;
  uint32_t outputaddr     = 0U;
  uint32_t tagaddr        = 0U;
  uint64_t headerlength   = 0U;
  uint64_t inputlength    = 0U;
  uint64_t payloadlength  = 0U;


  if (hcryp->State == HAL_CRYP_STATE_READY)
  {
    /* input/output parameters check */
    if (hcryp->Init.GCMCMACPhase == CRYP_HEADER_PHASE)
    {
      if ((hcryp->Init.Header != NULL) && (hcryp->Init.HeaderSize == 0U))
      {
        return  HAL_ERROR;
      }
#if defined(AES_CR_NPBLB)
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC)
#else
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)
#endif
      {
        if ((pInputData == NULL) || (Size == 0U))
        {
          return  HAL_ERROR;
        }
      }
    }
    else if (hcryp->Init.GCMCMACPhase == CRYP_PAYLOAD_PHASE)
    {
      if ((pInputData == NULL) || (pOutputData == NULL) || (Size == 0U))
      {
        return  HAL_ERROR;
      }
    }
    else if (hcryp->Init.GCMCMACPhase == CRYP_FINAL_PHASE)
    {
      if (pOutputData == NULL)
      {
        return  HAL_ERROR;
      }
#if defined(AES_CR_NPBLB)
      if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC) && (pInputData == NULL))
#else
      if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC) && (pInputData == NULL))
#endif
      {
        return  HAL_ERROR;
      }
    }

    /* Process Locked */
    __HAL_LOCK(hcryp);

    /* Change the CRYP state */
    hcryp->State = HAL_CRYP_STATE_BUSY;

    /*==============================================*/
    /* GCM/GMAC (or CCM when applicable) init phase */
    /*==============================================*/
    /* In case of init phase, the input data (Key and Initialization Vector) have
       already been entered during the initialization process. No DMA transfer is
       required at that point therefore, the software just waits for the CCF flag
       to be raised. */
    if (hcryp->Init.GCMCMACPhase == CRYP_INIT_PHASE)
    {
      /* just wait for hash computation */
      if(CRYP_WaitOnCCFlag(hcryp, CRYP_CCF_TIMEOUTVALUE) != HAL_OK)
      {
        hcryp->State = HAL_CRYP_STATE_READY;
        __HAL_UNLOCK(hcryp);
        return HAL_TIMEOUT;
      }

      /* Clear CCF Flag */
      __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
      /* Mark that the initialization phase is over */
      hcryp->Phase = HAL_CRYP_PHASE_INIT_OVER;
      hcryp->State = HAL_CRYP_STATE_READY;
    }
    /*===============================*/
    /* GCM/GMAC or CMAC header phase */
    /*===============================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_GCMCMAC_HEADER_PHASE)
    {
      /* Set header phase; for GCM or GMAC, set data-byte at this point */
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)
      {
        MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH|AES_CR_DATATYPE, CRYP_GCMCMAC_HEADER_PHASE|hcryp->Init.DataType);
      }
      else
      {
        MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, CRYP_GCMCMAC_HEADER_PHASE);
      }

#if !defined(AES_CR_NPBLB)
      /* enter first B0 block in polling mode (no DMA transfer for B0) */
      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)
      {
         /* Enable the CRYP peripheral */
        __HAL_CRYP_ENABLE();

        inputaddr  = (uint32_t)pInputData;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);

        if(CRYP_WaitOnCCFlag(hcryp, CRYP_CCF_TIMEOUTVALUE) != HAL_OK)
        {
          hcryp->State = HAL_CRYP_STATE_READY;
          __HAL_UNLOCK(hcryp);
          return HAL_TIMEOUT;
        }
        /* Clear CCF Flag */
        __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
      }
#endif

      /* No header case */
      if (hcryp->Init.Header == NULL)
      {
        hcryp->State = HAL_CRYP_STATE_READY;
        /* Mark that the header phase is over */
        hcryp->Phase = HAL_CRYP_PHASE_HEADER_OVER;
        /* Process Unlocked */
        __HAL_UNLOCK(hcryp);

        return HAL_OK;
      }

      inputaddr = (uint32_t)hcryp->Init.Header;
      if ((hcryp->Init.HeaderSize % 16U) != 0U)
      {

        if (hcryp->Init.HeaderSize < 16U)
        {
          CRYP_Padding(hcryp, (uint32_t) (hcryp->Init.HeaderSize), CRYP_POLLING_OFF);

          hcryp->State = HAL_CRYP_STATE_READY;
          /* Mark that the header phase is over */
          hcryp->Phase = HAL_CRYP_PHASE_HEADER_OVER;

          /* CCF flag indicating header phase AES processing completion
             will be checked at the start of the next phase:
            - payload phase (GCM / CCM when applicable)
            - final phase (GMAC or CMAC).  */
        }
        else
        {
          /* Local variable headerlength is a number of bytes multiple of 128 bits,
            remaining header data (if any) are handled after this loop */
          headerlength =  (((hcryp->Init.HeaderSize)/16U)*16U) ;
          /* Store the ending transfer point */
          hcryp->pCrypInBuffPtr = hcryp->Init.Header + headerlength;
          hcryp->CrypInCount = (uint32_t)(hcryp->Init.HeaderSize - headerlength); /* remainder */

          /* Set the input and output addresses and start DMA transfer */
          /* (incomplete DMA transfer, will be wrapped up after completion of
             the first one (initiated here) with data padding */
          CRYP_GCMCMAC_SetDMAConfig(hcryp, inputaddr, headerlength, 0U);
        }
      }
      else
      {
        hcryp->CrypInCount = 0U;
        /* Set the input address and start DMA transfer */
        CRYP_GCMCMAC_SetDMAConfig(hcryp, inputaddr, hcryp->Init.HeaderSize, 0U);
      }

    }
    /*============================================*/
    /* GCM (or CCM when applicable) payload phase */
    /*============================================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_PAYLOAD_PHASE)
    {
      /* Coming from header phase, wait for CCF flag to be raised
          if header present and fed to the IP in the previous phase */
      if (hcryp->Init.Header != NULL)
      {
        if(CRYP_WaitOnCCFlag(hcryp, CRYP_CCF_TIMEOUTVALUE) != HAL_OK)
        {
          hcryp->State = HAL_CRYP_STATE_READY;
          __HAL_UNLOCK(hcryp);
          return HAL_TIMEOUT;
        }
      }
      else
      {
        /* Enable the Peripheral since wasn't in header phase (no header case) */
        __HAL_CRYP_ENABLE();
      }
      /* Clear CCF Flag */
      __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);

      MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, CRYP_PAYLOAD_PHASE);

      /* Specific handling to manage payload size less than 128 bits */
      if ((Size % 16U) != 0U)
      {
        inputaddr  = (uint32_t)pInputData;
        outputaddr = (uint32_t)pOutputData;
        if (Size < 16U)
        {
          /* Block is now entered in polling mode, no actual gain in resorting to DMA */
          hcryp->pCrypInBuffPtr  =  (uint8_t *)inputaddr;
          hcryp->pCrypOutBuffPtr =  (uint8_t *)outputaddr;

          CRYP_Padding(hcryp, (uint32_t)Size, CRYP_POLLING_ON);

          /* Change the CRYP state to ready */
          hcryp->State = HAL_CRYP_STATE_READY;
          /* Mark that the payload phase is over */
          hcryp->Phase = HAL_CRYP_PHASE_PAYLOAD_OVER;

          /* Call output data transfer complete callback */
          HAL_CRYP_OutCpltCallback(hcryp);
        }
        else
        {
          payloadlength = (Size/16U) * 16U;

          /* Store the ending transfer points */
          hcryp->pCrypInBuffPtr = pInputData + payloadlength;
          hcryp->pCrypOutBuffPtr = pOutputData + payloadlength;
          hcryp->CrypInCount = (uint32_t)(Size - payloadlength); /* remainder */

          /* Set the input and output addresses and start DMA transfer */
          /* (incomplete DMA transfer, will be wrapped up with data padding
             after completion of the one initiated here) */
          CRYP_GCMCMAC_SetDMAConfig(hcryp, inputaddr, payloadlength, outputaddr);
        }
      }
      else
      {
        hcryp->CrypInCount = 0U;
        inputaddr  = (uint32_t)pInputData;
        outputaddr = (uint32_t)pOutputData;

        /* Set the input and output addresses and start DMA transfer */
        CRYP_GCMCMAC_SetDMAConfig(hcryp, inputaddr, Size, outputaddr);
      }
    }
    /*====================================*/
    /* GCM/GMAC or (CCM/)CMAC final phase */
    /*====================================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_FINAL_PHASE)
    {
      /* If coming from header phase (GMAC or CMAC case),
         wait for CCF flag to be raised */
      if (READ_BIT(hcryp->Instance->CR, AES_CR_GCMPH) == CRYP_HEADER_PHASE)
      {
        if(CRYP_WaitOnCCFlag(hcryp, CRYP_CCF_TIMEOUTVALUE) != HAL_OK)
        {
          hcryp->State = HAL_CRYP_STATE_READY;
          __HAL_UNLOCK(hcryp);
          return HAL_TIMEOUT;
        }
        /* Clear CCF Flag */
        __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
      }

      tagaddr = (uint32_t)pOutputData;

      MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, CRYP_FINAL_PHASE);

      /* if the header and payload phases have been bypassed, AES must be enabled again */
      if (hcryp->Phase == HAL_CRYP_PHASE_INIT_OVER)
      {
        __HAL_CRYP_ENABLE();
      }

      if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC)
      {
        headerlength = hcryp->Init.HeaderSize * 8U; /* Header length in bits */
        inputlength = Size * 8U;  /* input length in bits */
        /* Write the number of bits in the header on 64 bits followed by the number
           of bits in the payload on 64 bits as well */
        if(hcryp->Init.DataType == CRYP_DATATYPE_1B)
        {
          hcryp->Instance->DINR = __RBIT((headerlength)>>32U);
          hcryp->Instance->DINR = __RBIT(headerlength);
          hcryp->Instance->DINR = __RBIT((inputlength)>>32U);
          hcryp->Instance->DINR = __RBIT(inputlength);
        }
        else if(hcryp->Init.DataType == CRYP_DATATYPE_8B)
        {
          hcryp->Instance->DINR = __REV((headerlength)>>32U);
          hcryp->Instance->DINR = __REV(headerlength);
          hcryp->Instance->DINR = __REV((inputlength)>>32U);
          hcryp->Instance->DINR = __REV(inputlength);
        }
        else if(hcryp->Init.DataType == CRYP_DATATYPE_16B)
        {
          hcryp->Instance->DINR = __ROR((headerlength)>>32U, 16U);
          hcryp->Instance->DINR = __ROR(headerlength, 16U);
          hcryp->Instance->DINR = __ROR((inputlength)>>32U, 16U);
          hcryp->Instance->DINR = __ROR(inputlength, 16U);
        }
        else if(hcryp->Init.DataType == CRYP_DATATYPE_32B)
        {
          hcryp->Instance->DINR = (uint32_t)(headerlength>>32U);
          hcryp->Instance->DINR = (uint32_t)(headerlength);
          hcryp->Instance->DINR = (uint32_t)(inputlength>>32U);
          hcryp->Instance->DINR = (uint32_t)(inputlength);
        }
      }
#if !defined(AES_CR_NPBLB)
      else if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)
      {
        __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);

        inputaddr  = (uint32_t)pInputData;
        /* Enter the last block made of a 128-bit value formatted
           from the original B0 packet. */
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
        hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        inputaddr+=4U;
      }
#endif

      /* No DMA transfer is required at that point therefore, the software
         just waits for the CCF flag to be raised. */
      if(CRYP_WaitOnCCFlag(hcryp, CRYP_CCF_TIMEOUTVALUE) != HAL_OK)
      {
          hcryp->State = HAL_CRYP_STATE_READY;
          __HAL_UNLOCK(hcryp);
          return HAL_TIMEOUT;
      }
      /* Clear CCF Flag */
      __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
      /* Read the Auth TAG in the IN FIFO */
      *(uint32_t*)(tagaddr) = hcryp->Instance->DOUTR;
      tagaddr+=4U;
      *(uint32_t*)(tagaddr) = hcryp->Instance->DOUTR;
      tagaddr+=4U;
      *(uint32_t*)(tagaddr) = hcryp->Instance->DOUTR;
      tagaddr+=4U;
      *(uint32_t*)(tagaddr) = hcryp->Instance->DOUTR;

      /* Mark that the final phase is over */
      hcryp->Phase = HAL_CRYP_PHASE_FINAL_OVER;
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Disable the Peripheral */
      __HAL_CRYP_DISABLE();
    }
    /*=================================================*/
    /* case incorrect hcryp->Init.GCMCMACPhase setting */
    /*=================================================*/
    else
    {
      hcryp->State = HAL_CRYP_STATE_ERROR;
      __HAL_UNLOCK(hcryp);
      return HAL_ERROR;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hcryp);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @}
  */

/** @defgroup CRYPEx_Exported_Functions_Group3 AES suspension/resumption functions
 *  @brief   Extended processing functions.
 *
@verbatim
  ==============================================================================
                    ##### AES extended suspension and resumption functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) save in memory the Initialization Vector, the Key registers, the Control register or
          the Suspend registers when a process is suspended by a higher priority message
      (+) write back in CRYP hardware block the saved values listed above when the suspended
          lower priority message processing is resumed.

@endverbatim
  * @{
  */


/**
  * @brief  In case of message processing suspension, read the Initialization Vector.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Output Pointer to the buffer containing the saved Initialization Vector.
  * @note   This value has to be stored for reuse by writing the AES_IVRx registers
  *         as soon as the interrupted processing has to be resumed.
  *         Applicable to all chaining modes.
  * @note   AES must be disabled when reading or resetting the IV values.
  * @retval None
  */
void HAL_CRYPEx_Read_IVRegisters(CRYP_HandleTypeDef *hcryp, uint8_t* Output)
{
  uint32_t outputaddr = (uint32_t)Output;

  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->IVR3);
  outputaddr+=4U;
  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->IVR2);
  outputaddr+=4U;
  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->IVR1);
  outputaddr+=4U;
  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->IVR0);
}

/**
  * @brief  In case of message processing resumption, rewrite the Initialization
  *         Vector in the AES_IVRx registers.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Input Pointer to the buffer containing the saved Initialization Vector to
  *         write back in the CRYP hardware block.
  * @note   Applicable to all chaining modes.
  * @note   AES must be disabled when reading or resetting the IV values.
  * @retval None
  */
void HAL_CRYPEx_Write_IVRegisters(CRYP_HandleTypeDef *hcryp, uint8_t* Input)
{
  uint32_t ivaddr = (uint32_t)Input;

  hcryp->Instance->IVR3 = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->IVR2 = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->IVR1 = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->IVR0 = __REV(*(uint32_t*)(ivaddr));
}


/**
  * @brief  In case of message GCM/GMAC or CMAC processing suspension, read the Suspend Registers.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Output Pointer to the buffer containing the saved Suspend Registers.
  * @note   These values have to be stored for reuse by writing back the AES_SUSPxR registers
  *         as soon as the interrupted processing has to be resumed.
  * @retval None
  */
void HAL_CRYPEx_Read_SuspendRegisters(CRYP_HandleTypeDef *hcryp, uint8_t* Output)
{
  uint32_t outputaddr = (uint32_t)Output;

  /* In case of GCM payload phase encryption, check that suspension can be carried out */
  if (READ_BIT(hcryp->Instance->CR, (AES_CR_GCMPH|AES_CR_MODE)) == (CRYP_GCM_PAYLOAD_PHASE|CRYP_ALGOMODE_ENCRYPT))
  {
    /* Ensure that Busy flag is reset */
    if(CRYP_WaitOnBusyFlagReset(hcryp, CRYP_BUSY_TIMEOUTVALUE) != HAL_OK)
    {
      hcryp->ErrorCode |= HAL_CRYP_BUSY_ERROR;
      hcryp->State = HAL_CRYP_STATE_ERROR;

      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);

      HAL_CRYP_ErrorCallback(hcryp);
      return ;
    }
  }

  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->SUSP7R);
  outputaddr+=4U;
  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->SUSP6R);
  outputaddr+=4U;
  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->SUSP5R);
  outputaddr+=4U;
  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->SUSP4R);
  outputaddr+=4U;
  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->SUSP3R);
  outputaddr+=4U;
  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->SUSP2R);
  outputaddr+=4U;
  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->SUSP1R);
  outputaddr+=4U;
  *(uint32_t*)(outputaddr) = __REV(hcryp->Instance->SUSP0R);
}

/**
  * @brief  In case of message GCM/GMAC or CMAC processing resumption, rewrite the Suspend
  *         Registers in the AES_SUSPxR registers.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Input Pointer to the buffer containing the saved suspend registers to
  *         write back in the CRYP hardware block.
  * @retval None
  */
void HAL_CRYPEx_Write_SuspendRegisters(CRYP_HandleTypeDef *hcryp, uint8_t* Input)
{
  uint32_t ivaddr = (uint32_t)Input;

  hcryp->Instance->SUSP7R = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->SUSP6R = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->SUSP5R = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->SUSP4R = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->SUSP3R = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->SUSP2R = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->SUSP1R = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4U;
  hcryp->Instance->SUSP0R = __REV(*(uint32_t*)(ivaddr));
}


/**
  * @brief  In case of message GCM/GMAC or CMAC processing suspension, read the Key Registers.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Output Pointer to the buffer containing the saved Key Registers.
  * @param  KeySize Indicates the key size (128 or 256 bits).
  * @note   These values have to be stored for reuse by writing back the AES_KEYRx registers
  *         as soon as the interrupted processing has to be resumed.
  * @retval None
  */
void HAL_CRYPEx_Read_KeyRegisters(CRYP_HandleTypeDef *hcryp, uint8_t* Output, uint32_t KeySize)
{
  uint32_t keyaddr = (uint32_t)Output;

  if (KeySize == CRYP_KEYSIZE_256B)
  {
    *(uint32_t*)(keyaddr) = __REV(hcryp->Instance->KEYR7);
    keyaddr+=4U;
    *(uint32_t*)(keyaddr) = __REV(hcryp->Instance->KEYR6);
    keyaddr+=4U;
    *(uint32_t*)(keyaddr) = __REV(hcryp->Instance->KEYR5);
    keyaddr+=4U;
    *(uint32_t*)(keyaddr) = __REV(hcryp->Instance->KEYR4);
    keyaddr+=4U;
  }

  *(uint32_t*)(keyaddr) = __REV(hcryp->Instance->KEYR3);
  keyaddr+=4U;
  *(uint32_t*)(keyaddr) = __REV(hcryp->Instance->KEYR2);
  keyaddr+=4U;
  *(uint32_t*)(keyaddr) = __REV(hcryp->Instance->KEYR1);
  keyaddr+=4U;
  *(uint32_t*)(keyaddr) = __REV(hcryp->Instance->KEYR0);
}

/**
  * @brief  In case of message GCM/GMAC or CMAC processing resumption, rewrite the Key
  *         Registers in the AES_KEYRx registers.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Input Pointer to the buffer containing the saved key registers to
  *         write back in the CRYP hardware block.
  * @param  KeySize Indicates the key size (128 or 256 bits)
  * @retval None
  */
void HAL_CRYPEx_Write_KeyRegisters(CRYP_HandleTypeDef *hcryp, uint8_t* Input, uint32_t KeySize)
{
  uint32_t keyaddr = (uint32_t)Input;

  if (KeySize == CRYP_KEYSIZE_256B)
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
}


/**
  * @brief  In case of message GCM/GMAC or CMAC processing suspension, read the Control Register.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Output Pointer to the buffer containing the saved Control Register.
  * @note   This values has to be stored for reuse by writing back the AES_CR register
  *         as soon as the interrupted processing has to be resumed.
  * @retval None
  */
void HAL_CRYPEx_Read_ControlRegister(CRYP_HandleTypeDef *hcryp, uint8_t* Output)
{
  *(uint32_t*)(Output) = hcryp->Instance->CR;
}

/**
  * @brief  In case of message GCM/GMAC or CMAC processing resumption, rewrite the Control
  *         Registers in the AES_CR register.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Input Pointer to the buffer containing the saved Control Register to
  *         write back in the CRYP hardware block.
  * @retval None
  */
void HAL_CRYPEx_Write_ControlRegister(CRYP_HandleTypeDef *hcryp, uint8_t* Input)
{
  hcryp->Instance->CR = *(uint32_t*)(Input);
  /* At the same time, set handle state back to READY to be able to resume the AES calculations
     without the processing APIs returning HAL_BUSY when called. */
  hcryp->State        = HAL_CRYP_STATE_READY;
}

/**
  * @brief  Request CRYP processing suspension when in polling or interruption mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @note   Set the handle field SuspendRequest to the appropriate value so that
  *         the on-going CRYP processing is suspended as soon as the required
  *         conditions are met.
  * @note   It is advised not to suspend the CRYP processing when the DMA controller
  *         is managing the data transfer
  * @retval None
  */
void HAL_CRYPEx_ProcessSuspend(CRYP_HandleTypeDef *hcryp)
{
  /* Set Handle Suspend Request field */
  hcryp->SuspendRequest = HAL_CRYP_SUSPEND;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup CRYPEx_Private_Functions
  * @{
  */

/**
  * @brief  DMA CRYP Input Data process complete callback
  *         for GCM, GMAC or CMAC chainging modes.
  * @note   Specific setting of hcryp fields are required only
  *         in the case of header phase where no output data DMA
  *         transfer is on-going (only input data transfer is enabled
  *         in such a case).
  * @param  hdma DMA handle.
  * @retval None
  */
static void CRYP_GCMCMAC_DMAInCplt(DMA_HandleTypeDef *hdma)
{
  uint32_t difflength = 0U;

  CRYP_HandleTypeDef* hcryp = (CRYP_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  /* Disable the DMA transfer for input request  */
  CLEAR_BIT(hcryp->Instance->CR, AES_CR_DMAINEN);

  if (hcryp->Init.GCMCMACPhase == CRYP_HEADER_PHASE)
  {

    if (hcryp->CrypInCount != 0U)
    {
      /* Last block is now entered in polling mode, no actual gain in resorting to DMA */
      difflength = hcryp->CrypInCount;
      hcryp->CrypInCount = 0U;

      CRYP_Padding(hcryp, difflength, CRYP_POLLING_OFF);
    }
    hcryp->State = HAL_CRYP_STATE_READY;
    /* Mark that the header phase is over */
    hcryp->Phase = HAL_CRYP_PHASE_HEADER_OVER;
  }
  /* CCF flag indicating header phase AES processing completion
     will be checked at the start of the next phase:
     - payload phase (GCM or CCM when applicable)
     - final phase (GMAC or CMAC).
    This allows to avoid the Wait on Flag within the IRQ handling.  */

  /* Call input data transfer complete callback */
  HAL_CRYP_InCpltCallback(hcryp);
}

/**
  * @brief  DMA CRYP Output Data process complete callback
  *         for GCM, GMAC or CMAC chainging modes.
  * @note   This callback is called only in the payload phase.
  * @param  hdma DMA handle.
  * @retval None
  */
static void CRYP_GCMCMAC_DMAOutCplt(DMA_HandleTypeDef *hdma)
{
  uint32_t difflength = 0U;
  CRYP_HandleTypeDef* hcryp = (CRYP_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  /* Disable the DMA transfer for output request */
  CLEAR_BIT(hcryp->Instance->CR, AES_CR_DMAOUTEN);

  /* Clear CCF Flag */
  __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);

  /* Initiate additional transfer to wrap-up data feeding to the IP */
  if (hcryp->CrypInCount != 0U)
  {
    /* Last block is now entered in polling mode, no actual gain in resorting to DMA */
    difflength = hcryp->CrypInCount;
    hcryp->CrypInCount = 0U;

    CRYP_Padding(hcryp, difflength, CRYP_POLLING_ON);
  }

  /* Change the CRYP state to ready */
  hcryp->State = HAL_CRYP_STATE_READY;
  /* Mark that the payload phase is over */
  hcryp->Phase = HAL_CRYP_PHASE_PAYLOAD_OVER;

  /* Call output data transfer complete callback */
  HAL_CRYP_OutCpltCallback(hcryp);
}

/**
  * @brief  DMA CRYP communication error callback
  *         for GCM, GMAC or CMAC chainging modes.
  * @param  hdma DMA handle
  * @retval None
  */
static void CRYP_GCMCMAC_DMAError(DMA_HandleTypeDef *hdma)
{
  CRYP_HandleTypeDef* hcryp = (CRYP_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  hcryp->State= HAL_CRYP_STATE_ERROR;
  hcryp->ErrorCode |= HAL_CRYP_DMA_ERROR;
  HAL_CRYP_ErrorCallback(hcryp);
  /* Clear Error Flag */
  __HAL_CRYP_CLEAR_FLAG(CRYP_ERR_CLEAR);
}

/**
  * @brief  Handle CRYP block input/output data handling under interruption
  *         for GCM, GMAC or CMAC chaining modes.
  * @note   The function is called under interruption only, once
  *         interruptions have been enabled by HAL_CRYPEx_AES_Auth_IT().
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @retval HAL status
  */
HAL_StatusTypeDef CRYP_AES_Auth_IT(CRYP_HandleTypeDef *hcryp)
{
  uint32_t inputaddr   = 0x0U;
  uint32_t outputaddr  = 0x0U;
  uint32_t index       = 0x0U;
  uint32_t addhoc_process = 0U;
  uint32_t difflength     = 0U;
  uint32_t difflengthmod4 = 0U;
  uint32_t mask[3]        = {0x0FFU, 0x0FFFFU, 0x0FFFFFFU};
  uint32_t intermediate_data[4U] = {0U};

  if(hcryp->State == HAL_CRYP_STATE_BUSY)
  {
    /*===========================*/
    /* GCM/GMAC(/CCM) init phase */
    /*===========================*/
    if (hcryp->Init.GCMCMACPhase == CRYP_INIT_PHASE)
    {
      /* Clear Computation Complete Flag */
      __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
      /* Disable Computation Complete Flag and Errors Interrupts */
      __HAL_CRYP_DISABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;

      /* Mark that the initialization phase is over */
      hcryp->Phase = HAL_CRYP_PHASE_INIT_OVER;

      /* Process Unlocked */
      __HAL_UNLOCK(hcryp);
      /* Call computation complete callback */
      HAL_CRYPEx_ComputationCpltCallback(hcryp);
      return HAL_OK;
    }
    /*=====================================*/
    /* GCM/GMAC or (CCM/)CMAC header phase */
    /*=====================================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_HEADER_PHASE)
    {
      /* Check if all input header data have been entered */
      if (hcryp->CrypInCount == 0U)
      {
        /* Clear Computation Complete Flag */
        __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
        /* Disable Computation Complete Flag and Errors Interrupts */
        __HAL_CRYP_DISABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);
        /* Change the CRYP state */
        hcryp->State = HAL_CRYP_STATE_READY;
       /* Mark that the header phase is over */
        hcryp->Phase = HAL_CRYP_PHASE_HEADER_OVER;

       /* Process Unlocked */
        __HAL_UNLOCK(hcryp);

        /* Call computation complete callback */
        HAL_CRYPEx_ComputationCpltCallback(hcryp);

        return HAL_OK;
      }
      /* If suspension flag has been raised, suspend processing */
      else if (hcryp->SuspendRequest == HAL_CRYP_SUSPEND)
      {
        /* Clear CCF Flag */
        __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);

        /* reset SuspendRequest */
        hcryp->SuspendRequest = HAL_CRYP_SUSPEND_NONE;
        /* Disable Computation Complete Flag and Errors Interrupts */
        __HAL_CRYP_DISABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);
        /* Change the CRYP state */
        hcryp->State = HAL_CRYP_STATE_SUSPENDED;
        /* Mark that the header phase is over */
        hcryp->Phase = HAL_CRYP_PHASE_HEADER_SUSPENDED;

       /* Process Unlocked */
        __HAL_UNLOCK(hcryp);

        return HAL_OK;
      }
      else /* Carry on feeding input data to the CRYP hardware block */
      {
        /* Clear Computation Complete Flag */
        __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
        /* Get the last Input data address */
        inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;

        /* Increment/decrement instance pointer/counter */
        if (hcryp->CrypInCount < 16U)
        {
          difflength = hcryp->CrypInCount;
          hcryp->CrypInCount = 0U;
          addhoc_process = 1U;
          difflengthmod4 = difflength%4U;
        }
        else
        {
          hcryp->pCrypInBuffPtr += 16U;
          hcryp->CrypInCount -= 16U;
        }

#if defined(AES_CR_NPBLB)
        if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CCM_CMAC)
#else
        if (hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_CMAC)
#endif
        {
          if (hcryp->CrypInCount == hcryp->Init.HeaderSize)
          {
            /* All B blocks will have been entered after the next
              four DINR writing, so point at header buffer for
              the next iteration */
            hcryp->pCrypInBuffPtr = hcryp->Init.Header;
          }
        }

        /* Write the Input block in the Data Input register */
        if (addhoc_process == 0U)
        {
          hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
          inputaddr+=4U;
          hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
          inputaddr+=4U;
          hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
          inputaddr+=4U;
          hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        }
        else
        {
          /* Header remainder has size less than 128 bits */
          /* Enter complete words when possible */
          for(index=0U; index < (difflength/4U); index ++)
          {
            /* Write the Input block in the Data Input register */
            hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
            inputaddr+=4U;
          }
          /* Enter incomplete word padded with zeroes if applicable
            (case of header length not a multiple of 32-bits) */
          if (difflengthmod4 != 0U)
          {
            hcryp->Instance->DINR = ((*(uint32_t*)(inputaddr)) & mask[difflengthmod4-1]);
          }
          /* Pad with zero-words to reach 128-bit long block and wrap-up header feeding to the IP */
          for(index=0U; index < (4U - ((difflength+3U)/4U)); index ++)
          {
            hcryp->Instance->DINR = 0U;
          }
        }

        return HAL_OK;
      }
    }
    /*=======================*/
    /* GCM/CCM payload phase */
    /*=======================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_PAYLOAD_PHASE)
    {
      /* Get the last output data address */
      outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;

     /* Specific handling to manage payload size less than 128 bits
        when GCM (or CCM when applicable) encryption or decryption is selected.
        Check here if the last block output data are read */
#if defined(AES_CR_NPBLB)
      if ((hcryp->CrypOutCount < 16U)                                && \
          (hcryp->CrypOutCount > 0U))
#else
      if ((hcryp->Init.ChainingMode == CRYP_CHAINMODE_AES_GCM_GMAC) && \
          (hcryp->CrypOutCount < 16U)                                && \
          (hcryp->CrypOutCount > 0U))
#endif
      {
        addhoc_process = 1U;
        difflength = hcryp->CrypOutCount;
        difflengthmod4 = difflength%4U;
        hcryp->CrypOutCount = 0U;   /* mark that no more output data will be needed */
        /* Retrieve intermediate data */
        for(index=0U; index < 4U; index ++)
        {
          intermediate_data[index] = hcryp->Instance->DOUTR;
        }
        /* Retrieve last words of cyphered data */
        /* First, retrieve complete output words */
        for(index=0U; index < (difflength/4U); index ++)
        {
          *(uint32_t*)(outputaddr) = intermediate_data[index];
          outputaddr+=4U;
        }
        /* Next, retrieve partial output word if applicable;
           at the same time, start masking intermediate data
           with a mask of zeros of same size than the padding
           applied to the last block of payload */
        if (difflengthmod4 != 0U)
        {
          intermediate_data[difflength/4U] &= mask[difflengthmod4-1U];
          *(uint32_t*)(outputaddr) = intermediate_data[difflength/4U];
        }

#if !defined(AES_CR_NPBLB)
        if (hcryp->Init.OperatingMode == CRYP_ALGOMODE_ENCRYPT)
        {
          /* Change again CHMOD configuration to GCM mode */
          __HAL_CRYP_SET_CHAININGMODE(CRYP_CHAINMODE_AES_GCM_GMAC);

          /* Select FINAL phase */
          MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, CRYP_GCMCMAC_FINAL_PHASE);

          /* Before inserting the intermediate data, carry on masking operation
             with a mask of zeros of same size than the padding applied to the last block of payload */
          for(index=0U; index < (4U - ((difflength+3U)/4U)); index ++)
          {
            intermediate_data[(difflength+3U)/4U+index] = 0U;
          }

          /* Insert intermediate data to trigger an additional DOUTR reading round */
          /* Clear Computation Complete Flag before entering new block */
          __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
          for(index=0U; index < 4U; index ++)
          {
            hcryp->Instance->DINR = intermediate_data[index];
          }
        }
        else
#endif
        {
          /* Payload phase is now over */
          /* Clear Computation Complete Flag */
          __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
          /* Disable Computation Complete Flag and Errors Interrupts */
          __HAL_CRYP_DISABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);
          /* Change the CRYP state */
          hcryp->State = HAL_CRYP_STATE_READY;
          /* Mark that the payload phase is over */
          hcryp->Phase = HAL_CRYP_PHASE_PAYLOAD_OVER;

          /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          /* Call computation complete callback */
          HAL_CRYPEx_ComputationCpltCallback(hcryp);
        }
        return HAL_OK;
      }
      else
      {
        if (hcryp->CrypOutCount != 0U)
        {
          /* Usual case (different than GCM/CCM last block < 128 bits ciphering) */
          /* Retrieve the last block available from the CRYP hardware block:
            read the output block from the Data Output Register */
          *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
          outputaddr+=4U;
          *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
          outputaddr+=4U;
          *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
          outputaddr+=4U;
          *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;

          /* Increment/decrement instance pointer/counter */
          hcryp->pCrypOutBuffPtr += 16U;
          hcryp->CrypOutCount -= 16U;
        }
#if !defined(AES_CR_NPBLB)
        else
        {
          /* Software work-around: additional DOUTR reading round to discard the data */
          for(index=0U; index < 4U; index ++)
          {
            intermediate_data[index] = hcryp->Instance->DOUTR;
          }
        }
#endif
      }

      /* Check if all output text has been retrieved */
      if (hcryp->CrypOutCount == 0U)
      {
#if !defined(AES_CR_NPBLB)
        /* Make sure that software-work around is not running before disabling
          the interruptions (indeed, if software work-around is running, the
          interruptions must not be disabled to allow the additional DOUTR
          reading round */
        if (addhoc_process == 0U)
#endif
        {
          /* Clear Computation Complete Flag */
          __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
          /* Disable Computation Complete Flag and Errors Interrupts */
          __HAL_CRYP_DISABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);
          /* Change the CRYP state */
          hcryp->State = HAL_CRYP_STATE_READY;
         /* Mark that the payload phase is over */
          hcryp->Phase = HAL_CRYP_PHASE_PAYLOAD_OVER;

         /* Process Unlocked */
          __HAL_UNLOCK(hcryp);

          /* Call computation complete callback */
          HAL_CRYPEx_ComputationCpltCallback(hcryp);
        }

        return HAL_OK;
      }
      /* If suspension flag has been raised, suspend processing */
      else if (hcryp->SuspendRequest == HAL_CRYP_SUSPEND)
      {
        /* Clear CCF Flag */
        __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);

        /* reset SuspendRequest */
        hcryp->SuspendRequest = HAL_CRYP_SUSPEND_NONE;
        /* Disable Computation Complete Flag and Errors Interrupts */
        __HAL_CRYP_DISABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);
        /* Change the CRYP state */
        hcryp->State = HAL_CRYP_STATE_SUSPENDED;
        /* Mark that the header phase is over */
        hcryp->Phase = HAL_CRYP_PHASE_HEADER_SUSPENDED;

       /* Process Unlocked */
        __HAL_UNLOCK(hcryp);

        return HAL_OK;
      }
      else /* Output data are still expected, carry on feeding the CRYP
               hardware block with input data */
      {
        /* Clear Computation Complete Flag */
        __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
        /* Get the last Input data address */
        inputaddr = (uint32_t)hcryp->pCrypInBuffPtr;

        /* Usual input data feeding case */
        if (hcryp->CrypInCount < 16U)
        {
          difflength = (uint32_t) (hcryp->CrypInCount);
          difflengthmod4 = difflength%4U;
          hcryp->CrypInCount = 0U;

#if defined(AES_CR_NPBLB)
          /* In case of GCM encryption or CCM decryption, specify the number of padding
             bytes in last block of payload */
               if (((READ_BIT(hcryp->Instance->CR, AES_CR_CHMOD) == CRYP_CHAINMODE_AES_GCM_GMAC)
                    &&  (READ_BIT(hcryp->Instance->CR, AES_CR_MODE) == CRYP_ALGOMODE_ENCRYPT))
                ||  ((READ_BIT(hcryp->Instance->CR, AES_CR_CHMOD) == CRYP_CHAINMODE_AES_CCM_CMAC)
                    &&  (READ_BIT(hcryp->Instance->CR, AES_CR_MODE) == CRYP_ALGOMODE_DECRYPT)))
               {
                 /* Set NPBLB field in writing the number of padding bytes
                    for the last block of payload */
                 MODIFY_REG(hcryp->Instance->CR, AES_CR_NPBLB, 16U - difflength);
               }
#else
          /* Software workaround applied to GCM encryption only */
          if (hcryp->Init.OperatingMode == CRYP_ALGOMODE_ENCRYPT)
          {
            /* Change the mode configured in CHMOD bits of CR register to select CTR mode */
            __HAL_CRYP_SET_CHAININGMODE(CRYP_CHAINMODE_AES_CTR);
          }
#endif

          /*  Insert the last block (which size is inferior to 128 bits) padded with zeroes
              to have a complete block of 128 bits */
          for(index=0U; index < (difflength/4U); index ++)
          {
            /* Write the Input block in the Data Input register */
            hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
            inputaddr+=4U;
          }
          /* If required, manage input data size not multiple of 32 bits */
          if (difflengthmod4 != 0U)
          {
            hcryp->Instance->DINR = ((*(uint32_t*)(inputaddr)) & mask[difflengthmod4-1U]);
          }
          /* Wrap-up in padding with zero-words if applicable */
          for(index=0U; index < (4U - ((difflength+3U)/4U)); index ++)
          {
            hcryp->Instance->DINR = 0U;
          }
        }
        else
        {
          hcryp->pCrypInBuffPtr += 16U;
          hcryp->CrypInCount -= 16U;

          /* Write the Input block in the Data Input register */
          hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
          inputaddr+=4U;
          hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
          inputaddr+=4U;
          hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
          inputaddr+=4U;
          hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
        }

        return HAL_OK;
      }
    }
    /*====================================*/
    /* GCM/GMAC or (CCM/)CMAC final phase */
    /*====================================*/
    else if (hcryp->Init.GCMCMACPhase == CRYP_FINAL_PHASE)
    {
      /* Clear Computation Complete Flag */
      __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);

      /* Get the last output data address */
      outputaddr = (uint32_t)hcryp->pCrypOutBuffPtr;

      /* Retrieve the last expected data from the CRYP hardware block:
         read the output block from the Data Output Register */
      *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
      outputaddr+=4U;
      *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
      outputaddr+=4U;
      *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
      outputaddr+=4U;
      *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;

      /* Disable Computation Complete Flag and Errors Interrupts */
      __HAL_CRYP_DISABLE_IT(CRYP_IT_CCFIE|CRYP_IT_ERRIE);
      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_READY;
      /* Mark that the header phase is over */
      hcryp->Phase = HAL_CRYP_PHASE_FINAL_OVER;

      /* Disable the Peripheral */
      __HAL_CRYP_DISABLE();
      /* Process Unlocked */
       __HAL_UNLOCK(hcryp);

      /* Call computation complete callback */
      HAL_CRYPEx_ComputationCpltCallback(hcryp);

      return HAL_OK;
    }
    else
    {
      /* Clear Computation Complete Flag */
      __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
      hcryp->State = HAL_CRYP_STATE_ERROR;
      __HAL_UNLOCK(hcryp);
      return HAL_ERROR;
    }
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Set the DMA configuration and start the DMA transfer
  *         for GCM, GMAC or CMAC chainging modes.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  inputaddr Address of the Input buffer.
  * @param  Size Size of the Input buffer un bytes, must be a multiple of 16.
  * @param  outputaddr Address of the Output buffer, null pointer when no output DMA stream
  *         has to be configured.
  * @retval None
  */
static void CRYP_GCMCMAC_SetDMAConfig(CRYP_HandleTypeDef *hcryp, uint32_t inputaddr, uint16_t Size, uint32_t outputaddr)
{

  /* Set the input CRYP DMA transfer complete callback */
  hcryp->hdmain->XferCpltCallback = CRYP_GCMCMAC_DMAInCplt;
  /* Set the DMA error callback */
  hcryp->hdmain->XferErrorCallback = CRYP_GCMCMAC_DMAError;

  if (outputaddr != 0U)
  {
    /* Set the output CRYP DMA transfer complete callback */
    hcryp->hdmaout->XferCpltCallback = CRYP_GCMCMAC_DMAOutCplt;
    /* Set the DMA error callback */
    hcryp->hdmaout->XferErrorCallback = CRYP_GCMCMAC_DMAError;
  }

  /* Enable the CRYP peripheral */
  __HAL_CRYP_ENABLE();

  /* Enable the DMA input stream */
  HAL_DMA_Start_IT(hcryp->hdmain, inputaddr, (uint32_t)&hcryp->Instance->DINR, Size/4U);

  /* Enable the DMA input request */
  SET_BIT(hcryp->Instance->CR, AES_CR_DMAINEN);


  if (outputaddr != 0U)
  {
    /* Enable the DMA output stream */
    HAL_DMA_Start_IT(hcryp->hdmaout, (uint32_t)&hcryp->Instance->DOUTR, outputaddr, Size/4U);

    /* Enable the DMA output request */
    SET_BIT(hcryp->Instance->CR, AES_CR_DMAOUTEN);
  }
}

/**
  * @brief  Write/read input/output data in polling mode.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Input Pointer to the Input buffer.
  * @param  Ilength Length of the Input buffer in bytes, must be a multiple of 16.
  * @param  Output Pointer to the returned buffer.
  * @param  Timeout Specify Timeout value.
  * @retval HAL status
  */
static HAL_StatusTypeDef CRYP_ProcessData(CRYP_HandleTypeDef *hcryp, uint8_t* Input, uint16_t Ilength, uint8_t* Output, uint32_t Timeout)
{
  uint32_t index = 0U;
  uint32_t inputaddr  = (uint32_t)Input;
  uint32_t outputaddr = (uint32_t)Output;


  for(index=0U; (index < Ilength); index += 16U)
  {
    /* Write the Input block in the Data Input register */
    hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DINR  = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
    hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;

    /* Wait for CCF flag to be raised */
    if(CRYP_WaitOnCCFlag(hcryp, Timeout) != HAL_OK)
    {
      hcryp->State = HAL_CRYP_STATE_READY;
      __HAL_UNLOCK(hcryp);
      return HAL_TIMEOUT;
    }

    /* Clear CCF Flag */
    __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);

    /* Read the Output block from the Data Output Register */
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
    outputaddr+=4U;
    *(uint32_t*)(outputaddr) = hcryp->Instance->DOUTR;
    outputaddr+=4U;

    /* If the suspension flag has been raised and if the processing is not about
       to end, suspend processing */
    if ((hcryp->SuspendRequest == HAL_CRYP_SUSPEND) && ((index+16U) < Ilength))
    {
      /* Reset SuspendRequest */
      hcryp->SuspendRequest = HAL_CRYP_SUSPEND_NONE;

      /* Save current reading and writing locations of Input and Output buffers */
      hcryp->pCrypOutBuffPtr =  (uint8_t *)outputaddr;
      hcryp->pCrypInBuffPtr  =  (uint8_t *)inputaddr;
      /* Save the number of bytes that remain to be processed at this point */
      hcryp->CrypInCount     =  Ilength - (index+16U);

      /* Change the CRYP state */
      hcryp->State = HAL_CRYP_STATE_SUSPENDED;

      return HAL_OK;
    }

  }
  /* Return function status */
  return HAL_OK;

}

/**
  * @brief  Read derivative key in polling mode when CRYP hardware block is set
  *         in key derivation operating mode (mode 2).
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Output Pointer to the returned buffer.
  * @param  Timeout Specify Timeout value.
  * @retval HAL status
  */
static HAL_StatusTypeDef CRYP_ReadKey(CRYP_HandleTypeDef *hcryp, uint8_t* Output, uint32_t Timeout)
{
  uint32_t outputaddr = (uint32_t)Output;

  /* Wait for CCF flag to be raised */
  if(CRYP_WaitOnCCFlag(hcryp, Timeout) != HAL_OK)
  {
    hcryp->State = HAL_CRYP_STATE_READY;
    __HAL_UNLOCK(hcryp);
    return HAL_TIMEOUT;
  }
  /* Clear CCF Flag */
  __HAL_CRYP_CLEAR_FLAG( CRYP_CCF_CLEAR);

    /* Read the derivative key from the AES_KEYRx registers */
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

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Set the DMA configuration and start the DMA transfer.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  inputaddr Address of the Input buffer.
  * @param  Size Size of the Input buffer in bytes, must be a multiple of 16.
  * @param  outputaddr Address of the Output buffer.
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

  /* Enable the DMA input stream */
  HAL_DMA_Start_IT(hcryp->hdmain, inputaddr, (uint32_t)&hcryp->Instance->DINR, Size/4U);

  /* Enable the DMA output stream */
  HAL_DMA_Start_IT(hcryp->hdmaout, (uint32_t)&hcryp->Instance->DOUTR, outputaddr, Size/4U);

  /* Enable In and Out DMA requests */
  SET_BIT(hcryp->Instance->CR, (AES_CR_DMAINEN | AES_CR_DMAOUTEN));

  /* Enable the CRYP peripheral */
  __HAL_CRYP_ENABLE();
}

/**
  * @brief  Handle CRYP hardware block Timeout when waiting for CCF flag to be raised.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Timeout Timeout duration.
  * @retval HAL status
  */
static HAL_StatusTypeDef CRYP_WaitOnCCFlag(CRYP_HandleTypeDef *hcryp, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Get timeout */
  tickstart = HAL_GetTick();

  while(HAL_IS_BIT_CLR(hcryp->Instance->SR, AES_SR_CCF))
  {
    /* Check for the Timeout */
    if(Timeout != HAL_MAX_DELAY)
    {
      if((HAL_GetTick() - tickstart ) > Timeout)
      {
        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  Wait for Busy Flag to be reset during a GCM payload encryption process suspension.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  Timeout Timeout duration.
  * @retval HAL status
  */
static HAL_StatusTypeDef CRYP_WaitOnBusyFlagReset(CRYP_HandleTypeDef *hcryp, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Get timeout */
  tickstart = HAL_GetTick();

  while(HAL_IS_BIT_SET(hcryp->Instance->SR, AES_SR_BUSY))
  {
    /* Check for the Timeout */
    if(Timeout != HAL_MAX_DELAY)
    {
      if((HAL_GetTick() - tickstart ) > Timeout)
      {
        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  DMA CRYP Input Data process complete callback.
  * @param  hdma DMA handle.
  * @retval None
  */
static void CRYP_DMAInCplt(DMA_HandleTypeDef *hdma)
{
  CRYP_HandleTypeDef* hcryp = (CRYP_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  /* Disable the DMA transfer for input request  */
  CLEAR_BIT(hcryp->Instance->CR, AES_CR_DMAINEN);

  /* Call input data transfer complete callback */
  HAL_CRYP_InCpltCallback(hcryp);
}

/**
  * @brief  DMA CRYP Output Data process complete callback.
  * @param  hdma DMA handle.
  * @retval None
  */
static void CRYP_DMAOutCplt(DMA_HandleTypeDef *hdma)
{
  CRYP_HandleTypeDef* hcryp = (CRYP_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  /* Disable the DMA transfer for output request */
  CLEAR_BIT(hcryp->Instance->CR, AES_CR_DMAOUTEN);

  /* Clear CCF Flag */
  __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);

  /* Disable CRYP */
  __HAL_CRYP_DISABLE();

  /* Change the CRYP state to ready */
  hcryp->State = HAL_CRYP_STATE_READY;

  /* Call output data transfer complete callback */
  HAL_CRYP_OutCpltCallback(hcryp);
}

/**
  * @brief  DMA CRYP communication error callback.
  * @param  hdma DMA handle.
  * @retval None
  */
static void CRYP_DMAError(DMA_HandleTypeDef *hdma)
{
  CRYP_HandleTypeDef* hcryp = (CRYP_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  hcryp->State= HAL_CRYP_STATE_ERROR;
  hcryp->ErrorCode |= HAL_CRYP_DMA_ERROR;
  HAL_CRYP_ErrorCallback(hcryp);
  /* Clear Error Flag */
  __HAL_CRYP_CLEAR_FLAG(CRYP_ERR_CLEAR);
}

/**
  * @brief  Last header or payload block padding when size is not a multiple of 128 bits.
  * @param  hcryp pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module.
  * @param  difflength size remainder after having fed all complete 128-bit blocks.
  * @param  polling specifies whether or not polling on CCF must be done after having
  *                  entered a complete block.
  * @retval None
  */
static void CRYP_Padding(CRYP_HandleTypeDef *hcryp, uint32_t difflength, uint32_t polling)
{
  uint32_t index          = 0U;
  uint32_t difflengthmod4 = difflength%4U;
  uint32_t inputaddr      = (uint32_t)hcryp->pCrypInBuffPtr;
  uint32_t outputaddr     = (uint32_t)hcryp->pCrypOutBuffPtr;
  uint32_t mask[3U]       = {0x0FFU, 0x0FFFFU, 0x0FFFFFFU};
  uint32_t intermediate_data[4U] = {0U};

#if defined(AES_CR_NPBLB)
  /* In case of GCM encryption or CCM decryption, specify the number of padding
     bytes in last block of payload */
     if (READ_BIT(hcryp->Instance->CR,AES_CR_GCMPH) == CRYP_PAYLOAD_PHASE)
     {
       if (((READ_BIT(hcryp->Instance->CR, AES_CR_CHMOD) == CRYP_CHAINMODE_AES_GCM_GMAC)
            &&  (READ_BIT(hcryp->Instance->CR, AES_CR_MODE) == CRYP_ALGOMODE_ENCRYPT))
        ||  ((READ_BIT(hcryp->Instance->CR, AES_CR_CHMOD) == CRYP_CHAINMODE_AES_CCM_CMAC)
            &&  (READ_BIT(hcryp->Instance->CR, AES_CR_MODE) == CRYP_ALGOMODE_DECRYPT)))
       {
         /* Set NPBLB field in writing the number of padding bytes
            for the last block of payload */
         MODIFY_REG(hcryp->Instance->CR, AES_CR_NPBLB, 16U - difflength);
       }
     }
#else
  /* Software workaround applied to GCM encryption only */
  if ((hcryp->Init.GCMCMACPhase == CRYP_GCM_PAYLOAD_PHASE) &&
      (hcryp->Init.OperatingMode == CRYP_ALGOMODE_ENCRYPT))
  {
    /* Change the mode configured in CHMOD bits of CR register to select CTR mode */
    __HAL_CRYP_SET_CHAININGMODE(CRYP_CHAINMODE_AES_CTR);
  }
#endif

  /* Wrap-up entering header or payload data */
  /* Enter complete words when possible */
  for(index=0U; index < (difflength/4U); index ++)
  {
    /* Write the Input block in the Data Input register */
    hcryp->Instance->DINR = *(uint32_t*)(inputaddr);
    inputaddr+=4U;
  }
  /* Enter incomplete word padded with zeroes if applicable
    (case of header length not a multiple of 32-bits) */
  if (difflengthmod4 != 0U)
  {
    hcryp->Instance->DINR = ((*(uint32_t*)(inputaddr)) & mask[difflengthmod4-1]);
  }
  /* Pad with zero-words to reach 128-bit long block and wrap-up header feeding to the IP */
  for(index=0U; index < (4U - ((difflength+3U)/4U)); index ++)
  {
    hcryp->Instance->DINR = 0U;
  }

  if (polling == CRYP_POLLING_ON)
  {
        if(CRYP_WaitOnCCFlag(hcryp, CRYP_CCF_TIMEOUTVALUE) != HAL_OK)
    {
        hcryp->State = HAL_CRYP_STATE_READY;
        __HAL_UNLOCK(hcryp);
       HAL_CRYP_ErrorCallback(hcryp);
      }

    /* Clear CCF Flag */
    __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
  }

    /* if payload */
  if (hcryp->Init.GCMCMACPhase == CRYP_GCM_PAYLOAD_PHASE)
    {

    /* Retrieve intermediate data */
    for(index=0U; index < 4U; index ++)
    {
      intermediate_data[index] = hcryp->Instance->DOUTR;
    }
    /* Retrieve last words of cyphered data */
    /* First, retrieve complete output words */
    for(index=0U; index < (difflength/4U); index ++)
    {
      *(uint32_t*)(outputaddr) = intermediate_data[index];
      outputaddr+=4U;
    }
    /* Next, retrieve partial output word if applicable;
       at the same time, start masking intermediate data
       with a mask of zeros of same size than the padding
       applied to the last block of payload */
    if (difflengthmod4 != 0U)
    {
      intermediate_data[difflength/4U] &= mask[difflengthmod4-1U];
      *(uint32_t*)(outputaddr) = intermediate_data[difflength/4U];
    }

#if !defined(AES_CR_NPBLB)
    /* Software workaround applied to GCM encryption only,
       applicable for AES IP v2 version (where NPBLB is not defined) */
    if (hcryp->Init.OperatingMode == CRYP_ALGOMODE_ENCRYPT)
    {
      /* Change again CHMOD configuration to GCM mode */
      __HAL_CRYP_SET_CHAININGMODE(CRYP_CHAINMODE_AES_GCM_GMAC);

      /* Select FINAL phase */
      MODIFY_REG(hcryp->Instance->CR, AES_CR_GCMPH, CRYP_GCMCMAC_FINAL_PHASE);

      /* Before inserting the intermediate data, carry on masking operation
         with a mask of zeros of same size than the padding applied to the last block of payload */
      for(index=0U; index < (4U - ((difflength+3U)/4U)); index ++)
      {
        intermediate_data[(difflength+3U)/4U+index] = 0U;
      }
      /* Insert intermediate data */
      for(index=0U; index < 4U; index ++)
      {
        hcryp->Instance->DINR = intermediate_data[index];
      }

      /*  Wait for completion, and read data on DOUT. This data is to discard. */
      if(CRYP_WaitOnCCFlag(hcryp, CRYP_CCF_TIMEOUTVALUE) != HAL_OK)
      {
        hcryp->State = HAL_CRYP_STATE_READY;
        __HAL_UNLOCK(hcryp);
        HAL_CRYP_ErrorCallback(hcryp);
      }

      /* Read data to discard */
      /* Clear CCF Flag */
      __HAL_CRYP_CLEAR_FLAG(CRYP_CCF_CLEAR);
      for(index=0U; index < 4U; index ++)
      {
        intermediate_data[index] = hcryp->Instance->DOUTR;
      }

      } /* if (hcryp->Init.OperatingMode == CRYP_ALGOMODE_ENCRYPT) */
#endif  /* !defined(AES_CR_NPBLB) */
    }   /* if (hcryp->Init.GCMCMACPhase == CRYP_GCM_PAYLOAD_PHASE) */

}

/**
  * @}
  */

#endif /* AES */

#endif /* HAL_CRYP_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
