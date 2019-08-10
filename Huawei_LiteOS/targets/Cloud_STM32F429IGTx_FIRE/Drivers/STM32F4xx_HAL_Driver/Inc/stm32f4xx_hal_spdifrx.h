/**
  ******************************************************************************
  * @file    stm32f4xx_hal_spdifrx.h
  * @author  MCD Application Team
  * @brief   Header file of SPDIFRX HAL module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_SPDIFRX_H
#define __STM32F4xx_HAL_SPDIFRX_H

#ifdef __cplusplus
 extern "C" {
#endif

#if defined(STM32F446xx)

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_def.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @addtogroup SPDIFRX
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SPDIFRX_Exported_Types SPDIFRX Exported Types
  * @{
  */

/**
  * @brief SPDIFRX Init structure definition
  */
typedef struct
{
  uint32_t InputSelection;           /*!< Specifies the SPDIF input selection.
                                          This parameter can be a value of @ref SPDIFRX_Input_Selection */

  uint32_t Retries;                  /*!< Specifies the Maximum allowed re-tries during synchronization phase.
                                          This parameter can be a value of @ref SPDIFRX_Max_Retries */

  uint32_t WaitForActivity;          /*!< Specifies the wait for activity on SPDIF selected input.
                                          This parameter can be a value of @ref SPDIFRX_Wait_For_Activity. */

  uint32_t ChannelSelection;         /*!< Specifies whether the control flow will take the channel status from channel A or B.
                                          This parameter can be a value of @ref SPDIFRX_Channel_Selection */

  uint32_t DataFormat;               /*!< Specifies the Data samples format (LSB, MSB, ...).
                                          This parameter can be a value of @ref SPDIFRX_Data_Format */

  uint32_t StereoMode;               /*!< Specifies whether the peripheral is in stereo or mono mode.
                                          This parameter can be a value of @ref SPDIFRX_Stereo_Mode */

    uint32_t PreambleTypeMask;          /*!< Specifies whether The preamble type bits are copied or not into the received frame.
                                                                                   This parameter can be a value of @ref SPDIFRX_PT_Mask */

    uint32_t ChannelStatusMask;        /*!< Specifies whether the channel status and user bits are copied or not into the received frame.
                                                                                  This parameter can be a value of @ref SPDIFRX_ChannelStatus_Mask */

    uint32_t ValidityBitMask;          /*!< Specifies whether the validity bit is copied or not into the received frame.
                                                                                  This parameter can be a value of @ref SPDIFRX_V_Mask */

    uint32_t ParityErrorMask;          /*!< Specifies whether the parity error bit is copied or not into the received frame.
                                                                                  This parameter can be a value of @ref SPDIFRX_PE_Mask */
}SPDIFRX_InitTypeDef;

/**
  * @brief SPDIFRX SetDataFormat structure definition
  */
typedef struct
{
  uint32_t DataFormat;               /*!< Specifies the Data samples format (LSB, MSB, ...).
                                          This parameter can be a value of @ref SPDIFRX_Data_Format */

  uint32_t StereoMode;               /*!< Specifies whether the peripheral is in stereo or mono mode.
                                          This parameter can be a value of @ref SPDIFRX_Stereo_Mode */

  uint32_t PreambleTypeMask;          /*!< Specifies whether The preamble type bits are copied or not into the received frame.
                                                                                   This parameter can be a value of @ref SPDIFRX_PT_Mask */

  uint32_t ChannelStatusMask;        /*!< Specifies whether the channel status and user bits are copied or not into the received frame.
                                                                                  This parameter can be a value of @ref SPDIFRX_ChannelStatus_Mask */

  uint32_t ValidityBitMask;          /*!< Specifies whether the validity bit is copied or not into the received frame.
                                                                                  This parameter can be a value of @ref SPDIFRX_V_Mask */

  uint32_t ParityErrorMask;          /*!< Specifies whether the parity error bit is copied or not into the received frame.
                                                                                  This parameter can be a value of @ref SPDIFRX_PE_Mask */
}SPDIFRX_SetDataFormatTypeDef;

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
  HAL_SPDIFRX_STATE_RESET      = 0x00U,  /*!< SPDIFRX not yet initialized or disabled                */
  HAL_SPDIFRX_STATE_READY      = 0x01U,  /*!< SPDIFRX initialized and ready for use                  */
  HAL_SPDIFRX_STATE_BUSY       = 0x02U,  /*!< SPDIFRX internal process is ongoing                    */
  HAL_SPDIFRX_STATE_BUSY_RX    = 0x03U,  /*!< SPDIFRX internal Data Flow RX process is ongoing       */
  HAL_SPDIFRX_STATE_BUSY_CX    = 0x04U,  /*!< SPDIFRX internal Control Flow RX process is ongoing    */
  HAL_SPDIFRX_STATE_ERROR      = 0x07U   /*!< SPDIFRX error state                                    */
}HAL_SPDIFRX_StateTypeDef;

/**
  * @brief SPDIFRX handle Structure definition
  */
typedef struct
{
  SPDIFRX_TypeDef            *Instance;    /* SPDIFRX registers base address */

  SPDIFRX_InitTypeDef        Init;         /* SPDIFRX communication parameters */

  uint32_t                   *pRxBuffPtr;  /* Pointer to SPDIFRX Rx transfer buffer */

    uint32_t                   *pCsBuffPtr;  /* Pointer to SPDIFRX Cx transfer buffer */

  __IO uint16_t              RxXferSize;   /* SPDIFRX Rx transfer size */

  __IO uint16_t              RxXferCount;  /* SPDIFRX Rx transfer counter
                                              (This field is initialized at the
                                               same value as transfer size at the
                                               beginning of the transfer and
                                               decremented when a sample is received.
                                               NbSamplesReceived = RxBufferSize-RxBufferCount) */

  __IO uint16_t              CsXferSize;   /* SPDIFRX Rx transfer size */

  __IO uint16_t              CsXferCount;  /* SPDIFRX Rx transfer counter
                                              (This field is initialized at the
                                               same value as transfer size at the
                                               beginning of the transfer and
                                               decremented when a sample is received.
                                               NbSamplesReceived = RxBufferSize-RxBufferCount) */

  DMA_HandleTypeDef          *hdmaCsRx;    /* SPDIFRX EC60958_channel_status and user_information DMA handle parameters */

  DMA_HandleTypeDef          *hdmaDrRx;    /* SPDIFRX Rx DMA handle parameters */

  __IO HAL_LockTypeDef       Lock;         /* SPDIFRX locking object */

  __IO HAL_SPDIFRX_StateTypeDef  State;    /* SPDIFRX communication state */

  __IO uint32_t  ErrorCode;                /* SPDIFRX Error code                 */
}SPDIFRX_HandleTypeDef;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SPDIFRX_Exported_Constants SPDIFRX Exported Constants
  * @{
  */
/** @defgroup SPDIFRX_ErrorCode SPDIFRX Error Code
  * @{
  */
#define HAL_SPDIFRX_ERROR_NONE      0x00000000U  /*!< No error           */
#define HAL_SPDIFRX_ERROR_TIMEOUT   0x00000001U  /*!< Timeout error      */
#define HAL_SPDIFRX_ERROR_OVR       0x00000002U  /*!< OVR error          */
#define HAL_SPDIFRX_ERROR_PE        0x00000004U  /*!< Parity error       */
#define HAL_SPDIFRX_ERROR_DMA       0x00000008U  /*!< DMA transfer error */
#define HAL_SPDIFRX_ERROR_UNKNOWN   0x00000010U  /*!< Unknown Error error */
/**
  * @}
  */

/** @defgroup SPDIFRX_Input_Selection SPDIFRX Input Selection
  * @{
  */
#define SPDIFRX_INPUT_IN0               0x00000000U
#define SPDIFRX_INPUT_IN1               0x00010000U
#define SPDIFRX_INPUT_IN2               0x00020000U
#define SPDIFRX_INPUT_IN3               0x00030000U
/**
  * @}
  */

/** @defgroup SPDIFRX_Max_Retries SPDIFRX Maximum Retries
  * @{
  */
#define SPDIFRX_MAXRETRIES_NONE            0x00000000U
#define SPDIFRX_MAXRETRIES_3               0x00001000U
#define SPDIFRX_MAXRETRIES_15              0x00002000U
#define SPDIFRX_MAXRETRIES_63              0x00003000U
/**
  * @}
  */

/** @defgroup SPDIFRX_Wait_For_Activity SPDIFRX Wait For Activity
  * @{
  */
#define SPDIFRX_WAITFORACTIVITY_OFF                   0x00000000U
#define SPDIFRX_WAITFORACTIVITY_ON                    ((uint32_t)SPDIFRX_CR_WFA)
/**
  * @}
  */

/** @defgroup SPDIFRX_PT_Mask SPDIFRX Preamble Type Mask
* @{
*/
#define SPDIFRX_PREAMBLETYPEMASK_OFF                   0x00000000U
#define SPDIFRX_PREAMBLETYPEMASK_ON                    ((uint32_t)SPDIFRX_CR_PTMSK)
/**
  * @}
  */

/** @defgroup SPDIFRX_ChannelStatus_Mask  SPDIFRX Channel Status Mask
* @{
*/
#define SPDIFRX_CHANNELSTATUS_OFF                 0x00000000U        /* The channel status and user bits are copied into the SPDIF_DR */
#define SPDIFRX_CHANNELSTATUS_ON                  ((uint32_t)SPDIFRX_CR_CUMSK)  /* The channel status and user bits are not copied into the SPDIF_DR, zeros are written instead*/
/**
  * @}
  */

/** @defgroup SPDIFRX_V_Mask SPDIFRX Validity Mask
* @{
*/
#define SPDIFRX_VALIDITYMASK_OFF                   0x00000000U
#define SPDIFRX_VALIDITYMASK_ON                    ((uint32_t)SPDIFRX_CR_VMSK)
/**
  * @}
  */

/** @defgroup SPDIFRX_PE_Mask  SPDIFRX Parity Error Mask
* @{
*/
#define SPDIFRX_PARITYERRORMASK_OFF                   0x00000000U
#define SPDIFRX_PARITYERRORMASK_ON                    ((uint32_t)SPDIFRX_CR_PMSK)
/**
  * @}
  */

/** @defgroup SPDIFRX_Channel_Selection  SPDIFRX Channel Selection
  * @{
  */
#define SPDIFRX_CHANNEL_A      0x00000000U
#define SPDIFRX_CHANNEL_B      ((uint32_t)SPDIFRX_CR_CHSEL)
/**
  * @}
  */

/** @defgroup SPDIFRX_Data_Format SPDIFRX Data Format
  * @{
  */
#define SPDIFRX_DATAFORMAT_LSB                   0x00000000U
#define SPDIFRX_DATAFORMAT_MSB                   0x00000010U
#define SPDIFRX_DATAFORMAT_32BITS                0x00000020U
/**
  * @}
  */

/** @defgroup SPDIFRX_Stereo_Mode SPDIFRX Stereo Mode
  * @{
  */
#define SPDIFRX_STEREOMODE_DISABLE           0x00000000U
#define SPDIFRX_STEREOMODE_ENABLE           ((uint32_t)SPDIFRX_CR_RXSTEO)
/**
  * @}
  */

/** @defgroup SPDIFRX_State SPDIFRX State
  * @{
  */

#define SPDIFRX_STATE_IDLE    0xFFFFFFFCU
#define SPDIFRX_STATE_SYNC    0x00000001U
#define SPDIFRX_STATE_RCV     ((uint32_t)SPDIFRX_CR_SPDIFEN)
/**
  * @}
  */

/** @defgroup SPDIFRX_Interrupts_Definition SPDIFRX Interrupts Definition
  * @{
  */
#define SPDIFRX_IT_RXNE                       ((uint32_t)SPDIFRX_IMR_RXNEIE)
#define SPDIFRX_IT_CSRNE                      ((uint32_t)SPDIFRX_IMR_CSRNEIE)
#define SPDIFRX_IT_PERRIE                     ((uint32_t)SPDIFRX_IMR_PERRIE)
#define SPDIFRX_IT_OVRIE                      ((uint32_t)SPDIFRX_IMR_OVRIE)
#define SPDIFRX_IT_SBLKIE                     ((uint32_t)SPDIFRX_IMR_SBLKIE)
#define SPDIFRX_IT_SYNCDIE                    ((uint32_t)SPDIFRX_IMR_SYNCDIE)
#define SPDIFRX_IT_IFEIE                      ((uint32_t)SPDIFRX_IMR_IFEIE )
/**
  * @}
  */

/** @defgroup SPDIFRX_Flags_Definition SPDIFRX Flags Definition
  * @{
  */
#define SPDIFRX_FLAG_RXNE                   ((uint32_t)SPDIFRX_SR_RXNE)
#define SPDIFRX_FLAG_CSRNE                  ((uint32_t)SPDIFRX_SR_CSRNE)
#define SPDIFRX_FLAG_PERR                   ((uint32_t)SPDIFRX_SR_PERR)
#define SPDIFRX_FLAG_OVR                    ((uint32_t)SPDIFRX_SR_OVR)
#define SPDIFRX_FLAG_SBD                    ((uint32_t)SPDIFRX_SR_SBD)
#define SPDIFRX_FLAG_SYNCD                  ((uint32_t)SPDIFRX_SR_SYNCD)
#define SPDIFRX_FLAG_FERR                   ((uint32_t)SPDIFRX_SR_FERR)
#define SPDIFRX_FLAG_SERR                   ((uint32_t)SPDIFRX_SR_SERR)
#define SPDIFRX_FLAG_TERR                   ((uint32_t)SPDIFRX_SR_TERR)
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup SPDIFRX_Exported_macros SPDIFRX Exported Macros
  * @{
  */

/** @brief  Reset SPDIFRX handle state
  * @param  __HANDLE__ SPDIFRX handle.
  * @retval None
  */
#define __HAL_SPDIFRX_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = (uint16_t)SPDIFRX_CR_SPDIFEN)

/** @brief  Disable the specified SPDIFRX peripheral (IDLE State).
  * @param  __HANDLE__ specifies the SPDIFRX Handle.
  * @retval None
  */
#define __HAL_SPDIFRX_IDLE(__HANDLE__) ((__HANDLE__)->Instance->CR &= SPDIFRX_STATE_IDLE)

/** @brief  Enable the specified SPDIFRX peripheral (SYNC State).
  * @param  __HANDLE__ specifies the SPDIFRX Handle.
  * @retval None
  */
#define __HAL_SPDIFRX_SYNC(__HANDLE__) ((__HANDLE__)->Instance->CR |= SPDIFRX_STATE_SYNC)


/** @brief  Enable the specified SPDIFRX peripheral (RCV State).
  * @param  __HANDLE__ specifies the SPDIFRX Handle.
  * @retval None
  */
#define __HAL_SPDIFRX_RCV(__HANDLE__) ((__HANDLE__)->Instance->CR |= SPDIFRX_STATE_RCV)

/** @brief  Enable or disable the specified SPDIFRX interrupts.
  * @param  __HANDLE__ specifies the SPDIFRX Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to enable or disable.
  *        This parameter can be one of the following values:
  *            @arg SPDIFRX_IT_RXNE
  *            @arg SPDIFRX_IT_CSRNE
  *            @arg SPDIFRX_IT_PERRIE
  *            @arg SPDIFRX_IT_OVRIE
  *            @arg SPDIFRX_IT_SBLKIE
  *            @arg SPDIFRX_IT_SYNCDIE
  *            @arg SPDIFRX_IT_IFEIE
  * @retval None
  */
#define __HAL_SPDIFRX_ENABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->IMR |= (__INTERRUPT__))
#define __HAL_SPDIFRX_DISABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->IMR &= (uint16_t)(~(__INTERRUPT__)))

/** @brief  Checks if the specified SPDIFRX interrupt source is enabled or disabled.
  * @param  __HANDLE__ specifies the SPDIFRX Handle.
  * @param  __INTERRUPT__ specifies the SPDIFRX interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg SPDIFRX_IT_RXNE
  *            @arg SPDIFRX_IT_CSRNE
  *            @arg SPDIFRX_IT_PERRIE
  *            @arg SPDIFRX_IT_OVRIE
  *            @arg SPDIFRX_IT_SBLKIE
  *            @arg SPDIFRX_IT_SYNCDIE
  *            @arg SPDIFRX_IT_IFEIE
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __HAL_SPDIFRX_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->IMR & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Checks whether the specified SPDIFRX flag is set or not.
  * @param  __HANDLE__ specifies the SPDIFRX Handle.
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg SPDIFRX_FLAG_RXNE
  *            @arg SPDIFRX_FLAG_CSRNE
  *            @arg SPDIFRX_FLAG_PERR
  *            @arg SPDIFRX_FLAG_OVR
  *            @arg SPDIFRX_FLAG_SBD
  *            @arg SPDIFRX_FLAG_SYNCD
  *            @arg SPDIFRX_FLAG_FERR
  *            @arg SPDIFRX_FLAG_SERR
  *            @arg SPDIFRX_FLAG_TERR
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_SPDIFRX_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->SR) & (__FLAG__)) == (__FLAG__))

/** @brief  Clears the specified SPDIFRX SR flag, in setting the proper IFCR register bit.
  * @param  __HANDLE__ specifies the USART Handle.
  * @param  __IT_CLEAR__ specifies the interrupt clear register flag that needs to be set
  *                       to clear the corresponding interrupt
  *          This parameter can be one of the following values:
  *            @arg SPDIFRX_FLAG_PERR
  *            @arg SPDIFRX_FLAG_OVR
  *            @arg SPDIFRX_SR_SBD
  *            @arg SPDIFRX_SR_SYNCD
  * @retval None
  */
#define __HAL_SPDIFRX_CLEAR_IT(__HANDLE__, __IT_CLEAR__) ((__HANDLE__)->Instance->IFCR = (uint32_t)(__IT_CLEAR__))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SPDIFRX_Exported_Functions
  * @{
  */

/** @addtogroup SPDIFRX_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  **********************************/
HAL_StatusTypeDef HAL_SPDIFRX_Init(SPDIFRX_HandleTypeDef *hspdif);
HAL_StatusTypeDef HAL_SPDIFRX_DeInit (SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_MspInit(SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_MspDeInit(SPDIFRX_HandleTypeDef *hspdif);
HAL_StatusTypeDef HAL_SPDIFRX_SetDataFormat(SPDIFRX_HandleTypeDef *hspdif, SPDIFRX_SetDataFormatTypeDef  sDataFormat);
/**
  * @}
  */

/** @addtogroup SPDIFRX_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions  ***************************************************/
 /* Blocking mode: Polling */
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveDataFlow(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveControlFlow(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size, uint32_t Timeout);

 /* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveControlFlow_IT(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveDataFlow_IT(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size);
void HAL_SPDIFRX_IRQHandler(SPDIFRX_HandleTypeDef *hspdif);

/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveControlFlow_DMA(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveDataFlow_DMA(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_SPDIFRX_DMAStop(SPDIFRX_HandleTypeDef *hspdif);

/* Callbacks used in non blocking modes (Interrupt and DMA) *******************/
void HAL_SPDIFRX_RxHalfCpltCallback(SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_RxCpltCallback(SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_ErrorCallback(SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_CxHalfCpltCallback(SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_CxCpltCallback(SPDIFRX_HandleTypeDef *hspdif);
/**
  * @}
  */

/** @addtogroup SPDIFRX_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control and State functions  ************************************/
HAL_SPDIFRX_StateTypeDef HAL_SPDIFRX_GetState(SPDIFRX_HandleTypeDef *hspdif);
uint32_t HAL_SPDIFRX_GetError(SPDIFRX_HandleTypeDef *hspdif);
/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup SPDIFRX_Private_Macros SPDIFRX Private Macros
  * @{
  */
#define IS_SPDIFRX_INPUT_SELECT(INPUT)  (((INPUT) == SPDIFRX_INPUT_IN1) || \
                                         ((INPUT) == SPDIFRX_INPUT_IN2) || \
                                         ((INPUT) == SPDIFRX_INPUT_IN3)  || \
                                         ((INPUT) == SPDIFRX_INPUT_IN0))
#define IS_SPDIFRX_MAX_RETRIES(RET)   (((RET) == SPDIFRX_MAXRETRIES_NONE) || \
                                      ((RET) == SPDIFRX_MAXRETRIES_3)  || \
                                      ((RET) == SPDIFRX_MAXRETRIES_15) || \
                                      ((RET) == SPDIFRX_MAXRETRIES_63))
#define IS_SPDIFRX_WAIT_FOR_ACTIVITY(VAL)    (((VAL) == SPDIFRX_WAITFORACTIVITY_ON) || \
                                               ((VAL) == SPDIFRX_WAITFORACTIVITY_OFF))
#define IS_PREAMBLE_TYPE_MASK(VAL)           (((VAL) == SPDIFRX_PREAMBLETYPEMASK_ON) || \
                                             ((VAL) == SPDIFRX_PREAMBLETYPEMASK_OFF))
#define IS_VALIDITY_MASK(VAL)               (((VAL) == SPDIFRX_VALIDITYMASK_OFF) || \
                                             ((VAL) == SPDIFRX_VALIDITYMASK_ON))
#define IS_PARITY_ERROR_MASK(VAL)            (((VAL) == SPDIFRX_PARITYERRORMASK_OFF) || \
                                             ((VAL) == SPDIFRX_PARITYERRORMASK_ON))
#define IS_SPDIFRX_CHANNEL(CHANNEL)   (((CHANNEL) == SPDIFRX_CHANNEL_A) || \
                                       ((CHANNEL) == SPDIFRX_CHANNEL_B))
#define IS_SPDIFRX_DATA_FORMAT(FORMAT)           (((FORMAT) == SPDIFRX_DATAFORMAT_LSB) || \
                                                 ((FORMAT) == SPDIFRX_DATAFORMAT_MSB) || \
                                                 ((FORMAT) == SPDIFRX_DATAFORMAT_32BITS))
#define IS_STEREO_MODE(MODE)                 (((MODE) == SPDIFRX_STEREOMODE_DISABLE) || \
                                             ((MODE) == SPDIFRX_STEREOMODE_ENABLE))

#define IS_CHANNEL_STATUS_MASK(VAL)          (((VAL) == SPDIFRX_CHANNELSTATUS_ON) || \
                                              ((VAL) == SPDIFRX_CHANNELSTATUS_OFF))
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup SPDIFRX_Private_Functions SPDIFRX Private Functions
  * @{
  */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* STM32F446xx */

#ifdef __cplusplus
}
#endif


#endif /* __STM32F4xx_HAL_SPDIFRX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
