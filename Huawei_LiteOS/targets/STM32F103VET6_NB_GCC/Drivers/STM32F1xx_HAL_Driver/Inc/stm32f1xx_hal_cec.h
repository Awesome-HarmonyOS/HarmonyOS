/**
  ******************************************************************************
  * @file    stm32f1xx_hal_cec.h
  * @author  MCD Application Team
  * @brief   Header file of CEC HAL module.
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
#ifndef __STM32F1xx_HAL_CEC_H
#define __STM32F1xx_HAL_CEC_H

#ifdef __cplusplus
 extern "C" {
#endif

#if defined(STM32F100xB) || defined(STM32F100xE)
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal_def.h"

/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */

/** @addtogroup CEC
  * @{
  */
 
/* Exported types ------------------------------------------------------------*/ 
/** @defgroup CEC_Exported_Types CEC Exported Types
  * @{
  */
/** 
  * @brief CEC Init Structure definition  
  */ 
typedef struct
{
  uint32_t TimingErrorFree; /*!< Configures the CEC Bit Timing Error Mode. 
                                 This parameter can be a value of @ref CEC_BitTimingErrorMode */
  uint32_t PeriodErrorFree; /*!< Configures the CEC Bit Period Error Mode. 
                                 This parameter can be a value of @ref CEC_BitPeriodErrorMode */
  uint16_t  OwnAddress;     /*!< Own addresses configuration
                                 This parameter can be a value of @ref CEC_OWN_ADDRESS */
  uint8_t  *RxBuffer;       /*!< CEC Rx buffer pointeur */
}CEC_InitTypeDef;

/** 
  * @brief HAL CEC State structures definition 
  * @note  HAL CEC State value is a combination of 2 different substates: gState and RxState.
  *        - gState contains CEC state information related to global Handle management 
  *          and also information related to Tx operations.
  *          gState value coding follow below described bitmap :
  *          b7 (not used)
  *             x  : Should be set to 0
  *          b6  Error information 
  *             0  : No Error
  *             1  : Error
  *          b5     IP initilisation status
  *             0  : Reset (IP not initialized)
  *             1  : Init done (IP initialized. HAL CEC Init function already called)
  *          b4-b3  (not used)
  *             xx : Should be set to 00
  *          b2     Intrinsic process state
  *             0  : Ready
  *             1  : Busy (IP busy with some configuration or internal operations)
  *          b1     (not used)
  *             x  : Should be set to 0
  *          b0     Tx state
  *             0  : Ready (no Tx operation ongoing)
  *             1  : Busy (Tx operation ongoing)
  *        - RxState contains information related to Rx operations.
  *          RxState value coding follow below described bitmap :
  *          b7-b6  (not used)
  *             xx : Should be set to 00
  *          b5     IP initilisation status
  *             0  : Reset (IP not initialized)
  *             1  : Init done (IP initialized)
  *          b4-b2  (not used)
  *            xxx : Should be set to 000
  *          b1     Rx state
  *             0  : Ready (no Rx operation ongoing)
  *             1  : Busy (Rx operation ongoing)
  *          b0     (not used)
  *             x  : Should be set to 0.  
  */ 
typedef enum
{
  HAL_CEC_STATE_RESET             = 0x00U,    /*!< Peripheral is not yet Initialized 
                                                   Value is allowed for gState and RxState             */
  HAL_CEC_STATE_READY             = 0x20U,    /*!< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState             */
  HAL_CEC_STATE_BUSY              = 0x24U,    /*!< an internal process is ongoing
                                                   Value is allowed for gState only                    */
  HAL_CEC_STATE_BUSY_RX           = 0x22U,    /*!< Data Reception process is ongoing
                                                   Value is allowed for RxState only                   */
  HAL_CEC_STATE_BUSY_TX           = 0x21U,    /*!< Data Transmission process is ongoing 
                                                   Value is allowed for gState only                    */
  HAL_CEC_STATE_BUSY_RX_TX        = 0x23U,    /*!< an internal process is ongoing
                                                   Value is allowed for gState only                    */
  HAL_CEC_STATE_ERROR             = 0x60U     /*!< Error Value is allowed for gState only              */
}HAL_CEC_StateTypeDef;

/** 
  * @brief  CEC handle Structure definition  
  */  
typedef struct
{
  CEC_TypeDef             *Instance;      /*!< CEC registers base address */
  
  CEC_InitTypeDef         Init;           /*!< CEC communication parameters */
  
  uint8_t                 *pTxBuffPtr;    /*!< Pointer to CEC Tx transfer Buffer */
  
  uint16_t                TxXferCount;    /*!< CEC Tx Transfer Counter */
  
  uint16_t                RxXferSize;     /*!< CEC Rx Transfer size, 0: header received only */
  
  HAL_LockTypeDef         Lock;           /*!< Locking object */

  HAL_CEC_StateTypeDef    gState;         /*!< CEC state information related to global Handle management 
                                               and also related to Tx operations.
                                               This parameter can be a value of @ref HAL_CEC_StateTypeDef */
  
  HAL_CEC_StateTypeDef    RxState;        /*!< CEC state information related to Rx operations.
                                               This parameter can be a value of @ref HAL_CEC_StateTypeDef */
  
  uint32_t                ErrorCode;      /*!< For errors handling purposes, copy of ISR register 
                                               in case error is reported */    
}CEC_HandleTypeDef;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup CEC_Exported_Constants CEC Exported Constants
  * @{
  */

/** @defgroup CEC_Error_Code CEC Error Code
  * @{
  */
#define HAL_CEC_ERROR_NONE   0x00000000U    /*!< no error */
#define HAL_CEC_ERROR_BTE    CEC_ESR_BTE    /*!< Bit Timing Error */
#define HAL_CEC_ERROR_BPE    CEC_ESR_BPE    /*!< Bit Period Error */
#define HAL_CEC_ERROR_RBTFE  CEC_ESR_RBTFE  /*!< Rx Block Transfer Finished Error */
#define HAL_CEC_ERROR_SBE    CEC_ESR_SBE    /*!< Start Bit Error */
#define HAL_CEC_ERROR_ACKE   CEC_ESR_ACKE   /*!< Block Acknowledge Error */
#define HAL_CEC_ERROR_LINE   CEC_ESR_LINE   /*!< Line Error */
#define HAL_CEC_ERROR_TBTFE  CEC_ESR_TBTFE  /*!< Tx Block Transfer Finished Error */
/**
  * @}
  */

/** @defgroup CEC_BitTimingErrorMode Bit Timing Error Mode
  * @{
  */ 
#define CEC_BIT_TIMING_ERROR_MODE_STANDARD  0x00000000U      /*!< Bit timing error Standard Mode */
#define CEC_BIT_TIMING_ERROR_MODE_ERRORFREE CEC_CFGR_BTEM    /*!< Bit timing error Free Mode */
/**
  * @}
  */

/** @defgroup CEC_BitPeriodErrorMode Bit Period Error Mode
  * @{
  */ 
#define CEC_BIT_PERIOD_ERROR_MODE_STANDARD 0x00000000U      /*!< Bit period error Standard Mode */
#define CEC_BIT_PERIOD_ERROR_MODE_FLEXIBLE CEC_CFGR_BPEM    /*!< Bit period error Flexible Mode */
/**
  * @}
  */ 
  
/** @defgroup CEC_Initiator_Position   CEC Initiator logical address position in message header     
  * @{
  */
#define CEC_INITIATOR_LSB_POS                  4U
/**
  * @}
  */

/** @defgroup CEC_OWN_ADDRESS   CEC Own Address    
  * @{
  */
#define CEC_OWN_ADDRESS_NONE            CEC_OWN_ADDRESS_0    /* Reset value */
#define CEC_OWN_ADDRESS_0              ((uint16_t)0x0000U)   /* Logical Address 0 */
#define CEC_OWN_ADDRESS_1              ((uint16_t)0x0001U)   /* Logical Address 1 */
#define CEC_OWN_ADDRESS_2              ((uint16_t)0x0002U)   /* Logical Address 2 */
#define CEC_OWN_ADDRESS_3              ((uint16_t)0x0003U)   /* Logical Address 3 */
#define CEC_OWN_ADDRESS_4              ((uint16_t)0x0004U)   /* Logical Address 4 */
#define CEC_OWN_ADDRESS_5              ((uint16_t)0x0005U)   /* Logical Address 5 */
#define CEC_OWN_ADDRESS_6              ((uint16_t)0x0006U)   /* Logical Address 6 */
#define CEC_OWN_ADDRESS_7              ((uint16_t)0x0007U)   /* Logical Address 7 */
#define CEC_OWN_ADDRESS_8              ((uint16_t)0x0008U)   /* Logical Address 8 */
#define CEC_OWN_ADDRESS_9              ((uint16_t)0x0009U)   /* Logical Address 9 */
#define CEC_OWN_ADDRESS_10             ((uint16_t)0x000AU)   /* Logical Address 10 */
#define CEC_OWN_ADDRESS_11             ((uint16_t)0x000BU)   /* Logical Address 11 */
#define CEC_OWN_ADDRESS_12             ((uint16_t)0x000CU)   /* Logical Address 12 */
#define CEC_OWN_ADDRESS_13             ((uint16_t)0x000DU)   /* Logical Address 13 */
#define CEC_OWN_ADDRESS_14             ((uint16_t)0x000EU)   /* Logical Address 14 */
#define CEC_OWN_ADDRESS_15             ((uint16_t)0x000FU)   /* Logical Address 15 */
/**
  * @}
  */

/** @defgroup CEC_Interrupts_Definitions  Interrupts definition
  * @{
  */
#define CEC_IT_IE CEC_CFGR_IE
/**
  * @}
  */

/** @defgroup CEC_Flags_Definitions  Flags definition
  * @{
  */
#define CEC_FLAG_TSOM  CEC_CSR_TSOM
#define CEC_FLAG_TEOM  CEC_CSR_TEOM
#define CEC_FLAG_TERR  CEC_CSR_TERR
#define CEC_FLAG_TBTRF CEC_CSR_TBTRF
#define CEC_FLAG_RSOM  CEC_CSR_RSOM
#define CEC_FLAG_REOM  CEC_CSR_REOM
#define CEC_FLAG_RERR  CEC_CSR_RERR
#define CEC_FLAG_RBTF  CEC_CSR_RBTF
/**
  * @}
  */
  
/**
  * @}
  */  
  
/* Exported macros -----------------------------------------------------------*/
/** @defgroup CEC_Exported_Macros CEC Exported Macros
  * @{
  */

/** @brief  Reset CEC handle gstate & RxState
  * @param  __HANDLE__: CEC handle.
  * @retval None
  */
#define __HAL_CEC_RESET_HANDLE_STATE(__HANDLE__) do{                                                   \
                                                       (__HANDLE__)->gState = HAL_CEC_STATE_RESET;     \
                                                       (__HANDLE__)->RxState = HAL_CEC_STATE_RESET;    \
                                                     } while(0U)

/** @brief  Checks whether or not the specified CEC interrupt flag is set.
  * @param  __HANDLE__: specifies the CEC Handle.
  * @param  __FLAG__: specifies the flag to check.
  *     @arg CEC_FLAG_TERR: Tx Error
  *     @arg CEC_FLAG_TBTRF:Tx Block Transfer Finished
  *     @arg CEC_FLAG_RERR: Rx Error
  *     @arg CEC_FLAG_RBTF: Rx Block Transfer Finished
  * @retval ITStatus
  */
#define __HAL_CEC_GET_FLAG(__HANDLE__, __FLAG__) READ_BIT((__HANDLE__)->Instance->CSR,(__FLAG__)) 

/** @brief  Clears the CEC's pending flags.
  * @param  __HANDLE__: specifies the CEC Handle.
  * @param  __FLAG__: specifies the flag to clear.
  *   This parameter can be any combination of the following values:
  *     @arg CEC_CSR_TERR: Tx Error
  *     @arg CEC_FLAG_TBTRF: Tx Block Transfer Finished
  *     @arg CEC_CSR_RERR: Rx Error
  *     @arg CEC_CSR_RBTF: Rx Block Transfer Finished
  * @retval none  
  */
#define __HAL_CEC_CLEAR_FLAG(__HANDLE__, __FLAG__)                                                                   \
                          do {                                                                                       \
                            uint32_t tmp = 0x0U;                                                                     \
                            tmp = (__HANDLE__)->Instance->CSR & 0x00000002U;                                         \
                            (__HANDLE__)->Instance->CSR &= (uint32_t)(((~(uint32_t)(__FLAG__)) & 0xFFFFFFFCU) | tmp);\
                          } while(0U)

/** @brief  Enables the specified CEC interrupt.
  * @param  __HANDLE__: specifies the CEC Handle.
  * @param  __INTERRUPT__: specifies the CEC interrupt to enable.
  *          This parameter can be:
  *            @arg CEC_IT_IE         : Interrupt Enable.
  * @retval none
  */
#define __HAL_CEC_ENABLE_IT(__HANDLE__, __INTERRUPT__) SET_BIT((__HANDLE__)->Instance->CFGR, (__INTERRUPT__))

/** @brief  Disables the specified CEC interrupt.
  * @param  __HANDLE__: specifies the CEC Handle.
  * @param  __INTERRUPT__: specifies the CEC interrupt to disable.
  *          This parameter can be:
  *            @arg CEC_IT_IE         : Interrupt Enable
  * @retval none
  */   
#define __HAL_CEC_DISABLE_IT(__HANDLE__, __INTERRUPT__) CLEAR_BIT((__HANDLE__)->Instance->CFGR, (__INTERRUPT__))

/** @brief  Checks whether or not the specified CEC interrupt is enabled.
  * @param  __HANDLE__: specifies the CEC Handle.
  * @param  __INTERRUPT__: specifies the CEC interrupt to check.
  *          This parameter can be:
  *            @arg CEC_IT_IE         : Interrupt Enable
  * @retval FlagStatus  
  */
#define __HAL_CEC_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) READ_BIT((__HANDLE__)->Instance->CFGR, (__INTERRUPT__))

/** @brief  Enables the CEC device
  * @param  __HANDLE__: specifies the CEC Handle.               
  * @retval none 
  */
#define __HAL_CEC_ENABLE(__HANDLE__) SET_BIT((__HANDLE__)->Instance->CFGR, CEC_CFGR_PE)

/** @brief  Disables the CEC device
  * @param  __HANDLE__: specifies the CEC Handle.               
  * @retval none 
  */
#define __HAL_CEC_DISABLE(__HANDLE__) CLEAR_BIT((__HANDLE__)->Instance->CFGR, CEC_CFGR_PE)

/** @brief  Set Transmission Start flag
  * @param  __HANDLE__: specifies the CEC Handle.               
  * @retval none 
  */
#define __HAL_CEC_FIRST_BYTE_TX_SET(__HANDLE__) SET_BIT((__HANDLE__)->Instance->CSR, CEC_CSR_TSOM)

/** @brief  Set Transmission End flag
  * @param  __HANDLE__: specifies the CEC Handle.               
  * @retval none
  */
#define __HAL_CEC_LAST_BYTE_TX_SET(__HANDLE__) SET_BIT((__HANDLE__)->Instance->CSR, CEC_CSR_TEOM)

/** @brief  Get Transmission Start flag
  * @param  __HANDLE__: specifies the CEC Handle.               
  * @retval FlagStatus 
  */
#define __HAL_CEC_GET_TRANSMISSION_START_FLAG(__HANDLE__) READ_BIT((__HANDLE__)->Instance->CSR, CEC_CSR_TSOM)

/** @brief  Get Transmission End flag
  * @param  __HANDLE__: specifies the CEC Handle.               
  * @retval FlagStatus 
  */
#define __HAL_CEC_GET_TRANSMISSION_END_FLAG(__HANDLE__) READ_BIT((__HANDLE__)->Instance->CSR, CEC_CSR_TEOM)

/** @brief  Clear OAR register
  * @param  __HANDLE__: specifies the CEC Handle.               
  * @retval none 
  */
#define __HAL_CEC_CLEAR_OAR(__HANDLE__)   CLEAR_BIT((__HANDLE__)->Instance->OAR, CEC_OAR_OA)

/** @brief  Set OAR register
  * @param  __HANDLE__: specifies the CEC Handle. 
  * @param  __ADDRESS__: Own Address value.
  * @retval none 
  */
#define __HAL_CEC_SET_OAR(__HANDLE__,__ADDRESS__) MODIFY_REG((__HANDLE__)->Instance->OAR, CEC_OAR_OA, (__ADDRESS__));

/**
  * @}
  */                       

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CEC_Exported_Functions CEC Exported Functions
  * @{
  */

/** @addtogroup CEC_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions 
  * @{
  */
/* Initialization and de-initialization functions  ****************************/
HAL_StatusTypeDef HAL_CEC_Init(CEC_HandleTypeDef *hcec);
HAL_StatusTypeDef HAL_CEC_DeInit(CEC_HandleTypeDef *hcec);
HAL_StatusTypeDef HAL_CEC_SetDeviceAddress(CEC_HandleTypeDef *hcec, uint16_t CEC_OwnAddress);
void HAL_CEC_MspInit(CEC_HandleTypeDef *hcec);
void HAL_CEC_MspDeInit(CEC_HandleTypeDef *hcec);
/**
  * @}
  */

/** @addtogroup CEC_Exported_Functions_Group2 Input and Output operation functions 
  *  @brief CEC Transmit/Receive functions 
  * @{
  */
/* I/O operation functions  ***************************************************/
HAL_StatusTypeDef HAL_CEC_Transmit_IT(CEC_HandleTypeDef *hcec, uint8_t InitiatorAddress,uint8_t DestinationAddress, uint8_t *pData, uint32_t Size);
uint32_t HAL_CEC_GetLastReceivedFrameSize(CEC_HandleTypeDef *hcec);
void HAL_CEC_ChangeRxBuffer(CEC_HandleTypeDef *hcec, uint8_t* Rxbuffer);
void HAL_CEC_IRQHandler(CEC_HandleTypeDef *hcec);
void HAL_CEC_TxCpltCallback(CEC_HandleTypeDef *hcec);
void HAL_CEC_RxCpltCallback(CEC_HandleTypeDef *hcec, uint32_t RxFrameSize);
void HAL_CEC_ErrorCallback(CEC_HandleTypeDef *hcec);
/**
  * @}
  */

/** @defgroup CEC_Exported_Functions_Group3 Peripheral Control functions 
  *  @brief   CEC control functions 
  * @{
  */
/* Peripheral State and Error functions ***************************************/
HAL_CEC_StateTypeDef HAL_CEC_GetState(CEC_HandleTypeDef *hcec);
uint32_t HAL_CEC_GetError(CEC_HandleTypeDef *hcec);
/**
  * @}
  */

/**
  * @}
  */
  
/* Private types -------------------------------------------------------------*/
/** @defgroup CEC_Private_Types CEC Private Types
  * @{
  */

/**
  * @}
  */ 

/* Private variables ---------------------------------------------------------*/
/** @defgroup CEC_Private_Variables CEC Private Variables
  * @{
  */
  
/**
  * @}
  */ 

/* Private constants ---------------------------------------------------------*/
/** @defgroup CEC_Private_Constants CEC Private Constants
  * @{
  */

/**
  * @}
  */ 

/* Private macros ------------------------------------------------------------*/
/** @defgroup CEC_Private_Macros CEC Private Macros
  * @{
  */
#define IS_CEC_BIT_TIMING_ERROR_MODE(MODE) (((MODE) == CEC_BIT_TIMING_ERROR_MODE_STANDARD) || \
                                            ((MODE) == CEC_BIT_TIMING_ERROR_MODE_ERRORFREE))

#define IS_CEC_BIT_PERIOD_ERROR_MODE(MODE) (((MODE) == CEC_BIT_PERIOD_ERROR_MODE_STANDARD) || \
                                            ((MODE) == CEC_BIT_PERIOD_ERROR_MODE_FLEXIBLE))

/** @brief Check CEC message size.
  *       The message size is the payload size: without counting the header, 
  *       it varies from 0 byte (ping operation, one header only, no payload) to 
  *       15 bytes (1 opcode and up to 14 operands following the header). 
  * @param  __SIZE__: CEC message size.               
  * @retval Test result (TRUE or FALSE).
  */
#define IS_CEC_MSGSIZE(__SIZE__) ((__SIZE__) <= 0x10U)
/** @brief Check CEC device Own Address Register (OAR) setting.
  * @param  __ADDRESS__: CEC own address.               
  * @retval Test result (TRUE or FALSE).
  */
#define IS_CEC_OWN_ADDRESS(__ADDRESS__) ((__ADDRESS__) <= 0x0000000FU)

/** @brief Check CEC initiator or destination logical address setting.
  *        Initiator and destination addresses are coded over 4 bits. 
  * @param  __ADDRESS__: CEC initiator or logical address.               
  * @retval Test result (TRUE or FALSE).
  */
#define IS_CEC_ADDRESS(__ADDRESS__) ((__ADDRESS__) <= 0x0000000FU)



/**
  * @}
  */
/* Private functions ---------------------------------------------------------*/
/** @defgroup CEC_Private_Functions CEC Private Functions
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
#endif /* defined(STM32F100xB) || defined(STM32F100xE) */
#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_CEC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
