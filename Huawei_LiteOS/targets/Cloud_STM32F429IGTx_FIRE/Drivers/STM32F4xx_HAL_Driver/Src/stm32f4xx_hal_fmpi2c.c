/**
  ******************************************************************************
  * @file    stm32f4xx_hal_fmpi2c.c
  * @author  MCD Application Team
  * @brief   FMPI2C HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Inter Integrated Circuit (FMPI2C) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State and Errors functions
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
    The FMPI2C HAL driver can be used as follows:

    (#) Declare a FMPI2C_HandleTypeDef handle structure, for example:
        FMPI2C_HandleTypeDef  hfmpi2c;

    (#)Initialize the FMPI2C low level resources by implementing the HAL_FMPI2C_MspInit() API:
        (##) Enable the FMPI2Cx interface clock
        (##) FMPI2C pins configuration
            (+++) Enable the clock for the FMPI2C GPIOs
            (+++) Configure FMPI2C pins as alternate function open-drain
        (##) NVIC configuration if you need to use interrupt process
            (+++) Configure the FMPI2Cx interrupt priority
            (+++) Enable the NVIC FMPI2C IRQ Channel
        (##) DMA Configuration if you need to use DMA process
            (+++) Declare a DMA_HandleTypeDef handle structure for the transmit or receive stream
            (+++) Enable the DMAx interface clock using
            (+++) Configure the DMA handle parameters
            (+++) Configure the DMA Tx or Rx stream
            (+++) Associate the initialized DMA handle to the hfmpi2c DMA Tx or Rx handle
            (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on
                  the DMA Tx or Rx stream

    (#) Configure the Communication Clock Timing, Own Address1, Master Addressing mode, Dual Addressing mode,
        Own Address2, Own Address2 Mask, General call and Nostretch mode in the hfmpi2c Init structure.

    (#) Initialize the FMPI2C registers by calling the HAL_FMPI2C_Init(), configures also the low level Hardware
        (GPIO, CLOCK, NVIC...etc) by calling the customized HAL_FMPI2C_MspInit(&hfmpi2c) API.

    (#) To check if target device is ready for communication, use the function HAL_FMPI2C_IsDeviceReady()

    (#) For FMPI2C IO and IO MEM operations, three operation modes are available within this driver :

    *** Polling mode IO operation ***
    =================================
    [..]
      (+) Transmit in master mode an amount of data in blocking mode using HAL_FMPI2C_Master_Transmit()
      (+) Receive in master mode an amount of data in blocking mode using HAL_FMPI2C_Master_Receive()
      (+) Transmit in slave mode an amount of data in blocking mode using HAL_FMPI2C_Slave_Transmit()
      (+) Receive in slave mode an amount of data in blocking mode using HAL_FMPI2C_Slave_Receive()

    *** Polling mode IO MEM operation ***
    =====================================
    [..]
      (+) Write an amount of data in blocking mode to a specific memory address using HAL_FMPI2C_Mem_Write()
      (+) Read an amount of data in blocking mode from a specific memory address using HAL_FMPI2C_Mem_Read()


    *** Interrupt mode IO operation ***
    ===================================
    [..]
      (+) Transmit in master mode an amount of data in non-blocking mode using HAL_FMPI2C_Master_Transmit_IT()
      (+) At transmission end of transfer, HAL_FMPI2C_MasterTxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_MasterTxCpltCallback()
      (+) Receive in master mode an amount of data in non-blocking mode using HAL_FMPI2C_Master_Receive_IT()
      (+) At reception end of transfer, HAL_FMPI2C_MasterRxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_MasterRxCpltCallback()
      (+) Transmit in slave mode an amount of data in non-blocking mode using HAL_FMPI2C_Slave_Transmit_IT()
      (+) At transmission end of transfer, HAL_FMPI2C_SlaveTxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_SlaveTxCpltCallback()
      (+) Receive in slave mode an amount of data in non-blocking mode using HAL_FMPI2C_Slave_Receive_IT()
      (+) At reception end of transfer, HAL_FMPI2C_SlaveRxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_SlaveRxCpltCallback()
      (+) In case of transfer Error, HAL_FMPI2C_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_ErrorCallback()
      (+) Abort a master FMPI2C process communication with Interrupt using HAL_FMPI2C_Master_Abort_IT()
      (+) End of abort process, HAL_FMPI2C_AbortCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_AbortCpltCallback()
      (+) Discard a slave FMPI2C process communication using __HAL_FMPI2C_GENERATE_NACK() macro.
           This action will inform Master to generate a Stop condition to discard the communication.


    *** Interrupt mode IO sequential operation ***
    ==============================================
    [..]
      (@) These interfaces allow to manage a sequential transfer with a repeated start condition
          when a direction change during transfer
    [..]
      (+) A specific option field manage the different steps of a sequential transfer
      (+) Option field values are defined through @ref FMPI2C_XFEROPTIONS and are listed below:
      (++) FMPI2C_FIRST_AND_LAST_FRAME: No sequential usage, functionnal is same as associated interfaces in no sequential mode
      (++) FMPI2C_FIRST_FRAME: Sequential usage, this option allow to manage a sequence with start condition, address
                            and data to transfer without a final stop condition
      (++) FMPI2C_FIRST_AND_NEXT_FRAME: Sequential usage (Master only), this option allow to manage a sequence with start condition, address
                            and data to transfer without a final stop condition, an then permit a call the same master sequential interface
                            several times (like HAL_FMPI2C_Master_Sequential_Transmit_IT() then HAL_FMPI2C_Master_Sequential_Transmit_IT())
      (++) FMPI2C_NEXT_FRAME: Sequential usage, this option allow to manage a sequence with a restart condition, address
                            and with new data to transfer if the direction change or manage only the new data to transfer
                            if no direction change and without a final stop condition in both cases
      (++) FMPI2C_LAST_FRAME: Sequential usage, this option allow to manage a sequance with a restart condition, address
                            and with new data to transfer if the direction change or manage only the new data to transfer
                            if no direction change and with a final stop condition in both cases
      (++) FMPI2C_LAST_FRAME_NO_STOP: Sequential usage (Master only), this option allow to manage a restart condition after several call of the same master sequential
                            interface several times (link with option FMPI2C_FIRST_AND_NEXT_FRAME).
                            Usage can, transfer several bytes one by one using HAL_FMPI2C_Master_Sequential_Transmit_IT(option FMPI2C_FIRST_AND_NEXT_FRAME then FMPI2C_NEXT_FRAME)
                            or HAL_FMPI2C_Master_Sequential_Receive_IT(option FMPI2C_FIRST_AND_NEXT_FRAME then FMPI2C_NEXT_FRAME).
                            Then usage of this option FMPI2C_LAST_FRAME_NO_STOP at the last Transmit or Receive sequence permit to call the oposite interface Receive or Transmit
                            without stopping the communication and so generate a restart condition.

      (+) Differents sequential FMPI2C interfaces are listed below:
      (++) Sequential transmit in master FMPI2C mode an amount of data in non-blocking mode using HAL_FMPI2C_Master_Sequential_Transmit_IT()
      (+++) At transmission end of current frame transfer, HAL_FMPI2C_MasterTxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_MasterTxCpltCallback()
      (++) Sequential receive in master FMPI2C mode an amount of data in non-blocking mode using HAL_FMPI2C_Master_Sequential_Receive_IT()
      (+++) At reception end of current frame transfer, HAL_FMPI2C_MasterRxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_MasterRxCpltCallback()
      (++) Abort a master FMPI2C process communication with Interrupt using HAL_FMPI2C_Master_Abort_IT()
      (+++) End of abort process, HAL_FMPI2C_AbortCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_AbortCpltCallback()
      (++) Enable/disable the Address listen mode in slave FMPI2C mode using HAL_FMPI2C_EnableListen_IT() HAL_FMPI2C_DisableListen_IT()
      (+++) When address slave FMPI2C match, HAL_FMPI2C_AddrCallback() is executed and user can
           add his own code to check the Address Match Code and the transmission direction request by master (Write/Read).
      (+++) At Listen mode end HAL_FMPI2C_ListenCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_ListenCpltCallback()
      (++) Sequential transmit in slave FMPI2C mode an amount of data in non-blocking mode using HAL_FMPI2C_Slave_Sequential_Transmit_IT()
      (+++) At transmission end of current frame transfer, HAL_FMPI2C_SlaveTxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_SlaveTxCpltCallback()
      (++) Sequential receive in slave FMPI2C mode an amount of data in non-blocking mode using HAL_FMPI2C_Slave_Sequential_Receive_IT()
      (+++) At reception end of current frame transfer, HAL_FMPI2C_SlaveRxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_SlaveRxCpltCallback()
      (++) In case of transfer Error, HAL_FMPI2C_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_ErrorCallback()
      (++) Abort a master FMPI2C process communication with Interrupt using HAL_FMPI2C_Master_Abort_IT()
      (++) End of abort process, HAL_FMPI2C_AbortCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_AbortCpltCallback()
      (++) Discard a slave FMPI2C process communication using __HAL_FMPI2C_GENERATE_NACK() macro.
           This action will inform Master to generate a Stop condition to discard the communication.

    *** Interrupt mode IO MEM operation ***
    =======================================
    [..]
      (+) Write an amount of data in non-blocking mode with Interrupt to a specific memory address using
          HAL_FMPI2C_Mem_Write_IT()
      (+) At Memory end of write transfer, HAL_FMPI2C_MemTxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_MemTxCpltCallback()
      (+) Read an amount of data in non-blocking mode with Interrupt from a specific memory address using
          HAL_FMPI2C_Mem_Read_IT()
      (+) At Memory end of read transfer, HAL_FMPI2C_MemRxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_MemRxCpltCallback()
      (+) In case of transfer Error, HAL_FMPI2C_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_ErrorCallback()

    *** DMA mode IO operation ***
    ==============================
    [..]
      (+) Transmit in master mode an amount of data in non-blocking mode (DMA) using
          HAL_FMPI2C_Master_Transmit_DMA()
      (+) At transmission end of transfer, HAL_FMPI2C_MasterTxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_MasterTxCpltCallback()
      (+) Receive in master mode an amount of data in non-blocking mode (DMA) using
          HAL_FMPI2C_Master_Receive_DMA()
      (+) At reception end of transfer, HAL_FMPI2C_MasterRxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_MasterRxCpltCallback()
      (+) Transmit in slave mode an amount of data in non-blocking mode (DMA) using
          HAL_FMPI2C_Slave_Transmit_DMA()
      (+) At transmission end of transfer, HAL_FMPI2C_SlaveTxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_SlaveTxCpltCallback()
      (+) Receive in slave mode an amount of data in non-blocking mode (DMA) using
          HAL_FMPI2C_Slave_Receive_DMA()
      (+) At reception end of transfer, HAL_FMPI2C_SlaveRxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_SlaveRxCpltCallback()
      (+) In case of transfer Error, HAL_FMPI2C_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_ErrorCallback()
      (+) Abort a master FMPI2C process communication with Interrupt using HAL_FMPI2C_Master_Abort_IT()
      (+) End of abort process, HAL_FMPI2C_AbortCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_AbortCpltCallback()
      (+) Discard a slave FMPI2C process communication using __HAL_FMPI2C_GENERATE_NACK() macro.
           This action will inform Master to generate a Stop condition to discard the communication.

    *** DMA mode IO MEM operation ***
    =================================
    [..]
      (+) Write an amount of data in non-blocking mode with DMA to a specific memory address using
          HAL_FMPI2C_Mem_Write_DMA()
      (+) At Memory end of write transfer, HAL_FMPI2C_MemTxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_MemTxCpltCallback()
      (+) Read an amount of data in non-blocking mode with DMA from a specific memory address using
          HAL_FMPI2C_Mem_Read_DMA()
      (+) At Memory end of read transfer, HAL_FMPI2C_MemRxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_MemRxCpltCallback()
      (+) In case of transfer Error, HAL_FMPI2C_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer HAL_FMPI2C_ErrorCallback()


     *** FMPI2C HAL driver macros list ***
     ==================================
     [..]
       Below the list of most used macros in FMPI2C HAL driver.

      (+) __HAL_FMPI2C_ENABLE: Enable the FMPI2C peripheral
      (+) __HAL_FMPI2C_DISABLE: Disable the FMPI2C peripheral
      (+) __HAL_FMPI2C_GENERATE_NACK: Generate a Non-Acknowledge FMPI2C peripheral in Slave mode
      (+) __HAL_FMPI2C_GET_FLAG: Check whether the specified FMPI2C flag is set or not
      (+) __HAL_FMPI2C_CLEAR_FLAG: Clear the specified FMPI2C pending flag
      (+) __HAL_FMPI2C_ENABLE_IT: Enable the specified FMPI2C interrupt
      (+) __HAL_FMPI2C_DISABLE_IT: Disable the specified FMPI2C interrupt

     [..]
       (@) You can refer to the FMPI2C HAL driver header file for more useful macros

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

/** @defgroup FMPI2C FMPI2C
  * @brief FMPI2C HAL module driver
  * @{
  */

#ifdef HAL_FMPI2C_MODULE_ENABLED

#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F446xx) || defined(STM32F412Zx) ||\
    defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup FMPI2C_Private_Define FMPI2C Private Define
  * @{
  */
#define TIMING_CLEAR_MASK      (0xF0FFFFFFU)  /*!< FMPI2C TIMING clear register Mask */
#define FMPI2C_TIMEOUT_ADDR    (10000U)       /*!< 10 s  */
#define FMPI2C_TIMEOUT_BUSY    (25U)          /*!< 25 ms */
#define FMPI2C_TIMEOUT_DIR     (25U)          /*!< 25 ms */
#define FMPI2C_TIMEOUT_RXNE    (25U)          /*!< 25 ms */
#define FMPI2C_TIMEOUT_STOPF   (25U)          /*!< 25 ms */
#define FMPI2C_TIMEOUT_TC      (25U)          /*!< 25 ms */
#define FMPI2C_TIMEOUT_TCR     (25U)          /*!< 25 ms */
#define FMPI2C_TIMEOUT_TXIS    (25U)          /*!< 25 ms */
#define FMPI2C_TIMEOUT_FLAG    (25U)          /*!< 25 ms */

#define MAX_NBYTE_SIZE      255U
#define SlaveAddr_SHIFT     7U
#define SlaveAddr_MSK       0x06U

/* Private define for @ref PreviousState usage */
#define FMPI2C_STATE_MSK             ((uint32_t)((HAL_FMPI2C_STATE_BUSY_TX | HAL_FMPI2C_STATE_BUSY_RX) & (~((uint32_t)HAL_FMPI2C_STATE_READY)))) /*!< Mask State define, keep only RX and TX bits            */
#define FMPI2C_STATE_NONE            ((uint32_t)(HAL_FMPI2C_MODE_NONE))                                                        /*!< Default Value                                          */
#define FMPI2C_STATE_MASTER_BUSY_TX  ((uint32_t)((HAL_FMPI2C_STATE_BUSY_TX & FMPI2C_STATE_MSK) | HAL_FMPI2C_MODE_MASTER))            /*!< Master Busy TX, combinaison of State LSB and Mode enum */
#define FMPI2C_STATE_MASTER_BUSY_RX  ((uint32_t)((HAL_FMPI2C_STATE_BUSY_RX & FMPI2C_STATE_MSK) | HAL_FMPI2C_MODE_MASTER))            /*!< Master Busy RX, combinaison of State LSB and Mode enum */
#define FMPI2C_STATE_SLAVE_BUSY_TX   ((uint32_t)((HAL_FMPI2C_STATE_BUSY_TX & FMPI2C_STATE_MSK) | HAL_FMPI2C_MODE_SLAVE))             /*!< Slave Busy TX, combinaison of State LSB and Mode enum  */
#define FMPI2C_STATE_SLAVE_BUSY_RX   ((uint32_t)((HAL_FMPI2C_STATE_BUSY_RX & FMPI2C_STATE_MSK) | HAL_FMPI2C_MODE_SLAVE))             /*!< Slave Busy RX, combinaison of State LSB and Mode enum  */
#define FMPI2C_STATE_MEM_BUSY_TX     ((uint32_t)((HAL_FMPI2C_STATE_BUSY_TX & FMPI2C_STATE_MSK) | HAL_FMPI2C_MODE_MEM))               /*!< Memory Busy TX, combinaison of State LSB and Mode enum */
#define FMPI2C_STATE_MEM_BUSY_RX     ((uint32_t)((HAL_FMPI2C_STATE_BUSY_RX & FMPI2C_STATE_MSK) | HAL_FMPI2C_MODE_MEM))               /*!< Memory Busy RX, combinaison of State LSB and Mode enum */


/* Private define to centralize the enable/disable of Interrupts */
#define FMPI2C_XFER_TX_IT          (0x00000001U)
#define FMPI2C_XFER_RX_IT          (0x00000002U)
#define FMPI2C_XFER_LISTEN_IT      (0x00000004U)

#define FMPI2C_XFER_ERROR_IT       (0x00000011U)
#define FMPI2C_XFER_CPLT_IT        (0x00000012U)
#define FMPI2C_XFER_RELOAD_IT      (0x00000012U)

/* Private define Sequential Transfer Options default/reset value */
#define FMPI2C_NO_OPTION_FRAME     (0xFFFF0000U)
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
#define FMPI2C_GET_DMA_REMAIN_DATA(__HANDLE__) ((((__HANDLE__)->State) == HAL_FMPI2C_STATE_BUSY_TX)   ? \
                                              ((uint32_t)(((DMA_Stream_TypeDef *)(__HANDLE__)->hdmatx->Instance)->NDTR)) :  \
                                              ((uint32_t)(((DMA_Stream_TypeDef *)(__HANDLE__)->hdmarx->Instance)->NDTR)))

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @defgroup FMPI2C_Private_Functions FMPI2C Private Functions
  * @{
  */
/* Private functions to handle DMA transfer */
static void FMPI2C_DMAMasterTransmitCplt(DMA_HandleTypeDef *hdma);
static void FMPI2C_DMAMasterReceiveCplt(DMA_HandleTypeDef *hdma);
static void FMPI2C_DMASlaveTransmitCplt(DMA_HandleTypeDef *hdma);
static void FMPI2C_DMASlaveReceiveCplt(DMA_HandleTypeDef *hdma);
static void FMPI2C_DMAError(DMA_HandleTypeDef *hdma);
static void FMPI2C_DMAAbort(DMA_HandleTypeDef *hdma);

/* Private functions to handle IT transfer */
static void FMPI2C_ITAddrCplt(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags);
static void FMPI2C_ITMasterSequentialCplt(FMPI2C_HandleTypeDef *hfmpi2c);
static void FMPI2C_ITSlaveSequentialCplt(FMPI2C_HandleTypeDef *hfmpi2c);
static void FMPI2C_ITMasterCplt(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags);
static void FMPI2C_ITSlaveCplt(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags);
static void FMPI2C_ITListenCplt(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags);
static void FMPI2C_ITError(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ErrorCode);

/* Private functions to handle IT transfer */
static HAL_StatusTypeDef FMPI2C_RequestMemoryWrite(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef FMPI2C_RequestMemoryRead(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);

/* Private functions for FMPI2C transfer IRQ handler */
static HAL_StatusTypeDef FMPI2C_Master_ISR_IT(struct __FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef FMPI2C_Slave_ISR_IT(struct __FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef FMPI2C_Master_ISR_DMA(struct __FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef FMPI2C_Slave_ISR_DMA(struct __FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags, uint32_t ITSources);

/* Private functions to handle flags during polling transfer */
static HAL_StatusTypeDef FMPI2C_WaitOnFlagUntilTimeout(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef FMPI2C_WaitOnTXISFlagUntilTimeout(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef FMPI2C_WaitOnRXNEFlagUntilTimeout(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef FMPI2C_WaitOnSTOPFlagUntilTimeout(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef FMPI2C_IsAcknowledgeFailed(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t Timeout, uint32_t Tickstart);

/* Private functions to centralize the enable/disable of Interrupts */
static HAL_StatusTypeDef FMPI2C_Enable_IRQ(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t InterruptRequest);
static HAL_StatusTypeDef FMPI2C_Disable_IRQ(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t InterruptRequest);

/* Private functions to flush TXDR register */
static void FMPI2C_Flush_TXDR(FMPI2C_HandleTypeDef *hfmpi2c);

/* Private functions to handle  start, restart or stop a transfer */
static void FMPI2C_TransferConfig(FMPI2C_HandleTypeDef *hfmpi2c,  uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup FMPI2C_Exported_Functions FMPI2C Exported Functions
  * @{
  */

/** @defgroup FMPI2C_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          deinitialize the FMPI2Cx peripheral:

      (+) User must Implement HAL_FMPI2C_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function HAL_FMPI2C_Init() to configure the selected device with
          the selected configuration:
        (++) Clock Timing
        (++) Own Address 1
        (++) Addressing mode (Master, Slave)
        (++) Dual Addressing mode
        (++) Own Address 2
        (++) Own Address 2 Mask
        (++) General call mode
        (++) Nostretch mode

      (+) Call the function HAL_FMPI2C_DeInit() to restore the default configuration
          of the selected FMPI2Cx peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the FMPI2C according to the specified parameters
  *         in the FMPI2C_InitTypeDef and initialize the associated handle.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Init(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Check the FMPI2C handle allocation */
  if (hfmpi2c == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_INSTANCE(hfmpi2c->Instance));
  assert_param(IS_FMPI2C_OWN_ADDRESS1(hfmpi2c->Init.OwnAddress1));
  assert_param(IS_FMPI2C_ADDRESSING_MODE(hfmpi2c->Init.AddressingMode));
  assert_param(IS_FMPI2C_DUAL_ADDRESS(hfmpi2c->Init.DualAddressMode));
  assert_param(IS_FMPI2C_OWN_ADDRESS2(hfmpi2c->Init.OwnAddress2));
  assert_param(IS_FMPI2C_OWN_ADDRESS2_MASK(hfmpi2c->Init.OwnAddress2Masks));
  assert_param(IS_FMPI2C_GENERAL_CALL(hfmpi2c->Init.GeneralCallMode));
  assert_param(IS_FMPI2C_NO_STRETCH(hfmpi2c->Init.NoStretchMode));

  if (hfmpi2c->State == HAL_FMPI2C_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hfmpi2c->Lock = HAL_UNLOCKED;

    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    HAL_FMPI2C_MspInit(hfmpi2c);
  }

  hfmpi2c->State = HAL_FMPI2C_STATE_BUSY;

  /* Disable the selected FMPI2C peripheral */
  __HAL_FMPI2C_DISABLE(hfmpi2c);

  /*---------------------------- FMPI2Cx TIMINGR Configuration ------------------*/
  /* Configure FMPI2Cx: Frequency range */
  hfmpi2c->Instance->TIMINGR = hfmpi2c->Init.Timing & TIMING_CLEAR_MASK;

  /*---------------------------- FMPI2Cx OAR1 Configuration ---------------------*/
  /* Disable Own Address1 before set the Own Address1 configuration */
  hfmpi2c->Instance->OAR1 &= ~FMPI2C_OAR1_OA1EN;

  /* Configure FMPI2Cx: Own Address1 and ack own address1 mode */
  if (hfmpi2c->Init.AddressingMode == FMPI2C_ADDRESSINGMODE_7BIT)
  {
    hfmpi2c->Instance->OAR1 = (FMPI2C_OAR1_OA1EN | hfmpi2c->Init.OwnAddress1);
  }
  else /* FMPI2C_ADDRESSINGMODE_10BIT */
  {
    hfmpi2c->Instance->OAR1 = (FMPI2C_OAR1_OA1EN | FMPI2C_OAR1_OA1MODE | hfmpi2c->Init.OwnAddress1);
  }

  /*---------------------------- FMPI2Cx CR2 Configuration ----------------------*/
  /* Configure FMPI2Cx: Addressing Master mode */
  if (hfmpi2c->Init.AddressingMode == FMPI2C_ADDRESSINGMODE_10BIT)
  {
    hfmpi2c->Instance->CR2 = (FMPI2C_CR2_ADD10);
  }
  /* Enable the AUTOEND by default, and enable NACK (should be disable only during Slave process */
  hfmpi2c->Instance->CR2 |= (FMPI2C_CR2_AUTOEND | FMPI2C_CR2_NACK);

  /*---------------------------- FMPI2Cx OAR2 Configuration ---------------------*/
  /* Disable Own Address2 before set the Own Address2 configuration */
  hfmpi2c->Instance->OAR2 &= ~FMPI2C_DUALADDRESS_ENABLE;

  /* Configure FMPI2Cx: Dual mode and Own Address2 */
  hfmpi2c->Instance->OAR2 = (hfmpi2c->Init.DualAddressMode | hfmpi2c->Init.OwnAddress2 | (hfmpi2c->Init.OwnAddress2Masks << 8));

  /*---------------------------- FMPI2Cx CR1 Configuration ----------------------*/
  /* Configure FMPI2Cx: Generalcall and NoStretch mode */
  hfmpi2c->Instance->CR1 = (hfmpi2c->Init.GeneralCallMode | hfmpi2c->Init.NoStretchMode);

  /* Enable the selected FMPI2C peripheral */
  __HAL_FMPI2C_ENABLE(hfmpi2c);

  hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;
  hfmpi2c->State = HAL_FMPI2C_STATE_READY;
  hfmpi2c->PreviousState = FMPI2C_STATE_NONE;
  hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

  return HAL_OK;
}

/**
  * @brief  DeInitialize the FMPI2C peripheral.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_DeInit(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Check the FMPI2C handle allocation */
  if (hfmpi2c == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_INSTANCE(hfmpi2c->Instance));

  hfmpi2c->State = HAL_FMPI2C_STATE_BUSY;

  /* Disable the FMPI2C Peripheral Clock */
  __HAL_FMPI2C_DISABLE(hfmpi2c);

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  HAL_FMPI2C_MspDeInit(hfmpi2c);

  hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;
  hfmpi2c->State = HAL_FMPI2C_STATE_RESET;
  hfmpi2c->PreviousState = FMPI2C_STATE_NONE;
  hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

  /* Release Lock */
  __HAL_UNLOCK(hfmpi2c);

  return HAL_OK;
}

/**
  * @brief Initialize the FMPI2C MSP.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
__weak void HAL_FMPI2C_MspInit(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_MspInit could be implemented in the user file
   */
}

/**
  * @brief DeInitialize the FMPI2C MSP.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
__weak void HAL_FMPI2C_MspDeInit(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_MspDeInit could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup FMPI2C_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the FMPI2C data
    transfers.

    (#) There are two modes of transfer:
       (++) Blocking mode : The communication is performed in the polling mode.
            The status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode : The communication is performed using Interrupts
            or DMA. These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated FMPI2C IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.

    (#) Blocking mode functions are :
        (++) HAL_FMPI2C_Master_Transmit()
        (++) HAL_FMPI2C_Master_Receive()
        (++) HAL_FMPI2C_Slave_Transmit()
        (++) HAL_FMPI2C_Slave_Receive()
        (++) HAL_FMPI2C_Mem_Write()
        (++) HAL_FMPI2C_Mem_Read()
        (++) HAL_FMPI2C_IsDeviceReady()

    (#) No-Blocking mode functions with Interrupt are :
        (++) HAL_FMPI2C_Master_Transmit_IT()
        (++) HAL_FMPI2C_Master_Receive_IT()
        (++) HAL_FMPI2C_Slave_Transmit_IT()
        (++) HAL_FMPI2C_Slave_Receive_IT()
        (++) HAL_FMPI2C_Master_Sequential_Transmit_IT()
        (++) HAL_FMPI2C_Master_Sequential_Receive_IT()
        (++) HAL_FMPI2C_Slave_Sequential_Transmit_IT()
        (++) HAL_FMPI2C_Slave_Sequential_Receive_IT()
        (++) HAL_FMPI2C_Mem_Write_IT()
        (++) HAL_FMPI2C_Mem_Read_IT()

    (#) No-Blocking mode functions with DMA are :
        (++) HAL_FMPI2C_Master_Transmit_DMA()
        (++) HAL_FMPI2C_Master_Receive_DMA()
        (++) HAL_FMPI2C_Slave_Transmit_DMA()
        (++) HAL_FMPI2C_Slave_Receive_DMA()
        (++) HAL_FMPI2C_Mem_Write_DMA()
        (++) HAL_FMPI2C_Mem_Read_DMA()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) HAL_FMPI2C_MemTxCpltCallback()
        (++) HAL_FMPI2C_MemRxCpltCallback()
        (++) HAL_FMPI2C_MasterTxCpltCallback()
        (++) HAL_FMPI2C_MasterRxCpltCallback()
        (++) HAL_FMPI2C_SlaveTxCpltCallback()
        (++) HAL_FMPI2C_SlaveRxCpltCallback()
        (++) HAL_FMPI2C_ErrorCallback()

@endverbatim
  * @{
  */

/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Master_Transmit(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_BUSY, SET, FMPI2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    hfmpi2c->State     = HAL_FMPI2C_STATE_BUSY_TX;
    hfmpi2c->Mode      = HAL_FMPI2C_MODE_MASTER;
    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr  = pData;
    hfmpi2c->XferCount = Size;
    hfmpi2c->XferISR   = NULL;

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_RELOAD_MODE, FMPI2C_GENERATE_START_WRITE);
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_AUTOEND_MODE, FMPI2C_GENERATE_START_WRITE);
    }

    while (hfmpi2c->XferCount > 0U)
    {
      /* Wait until TXIS flag is set */
      if (FMPI2C_WaitOnTXISFlagUntilTimeout(hfmpi2c, Timeout, tickstart) != HAL_OK)
      {
        if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
        {
          return HAL_ERROR;
        }
        else
        {
          return HAL_TIMEOUT;
        }
      }
      /* Write data to TXDR */
      hfmpi2c->Instance->TXDR = (*hfmpi2c->pBuffPtr++);
      hfmpi2c->XferCount--;
      hfmpi2c->XferSize--;

      if ((hfmpi2c->XferSize == 0U) && (hfmpi2c->XferCount != 0U))
      {
        /* Wait until TCR flag is set */
        if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hfmpi2c->XferSize = MAX_NBYTE_SIZE;
          FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_RELOAD_MODE, FMPI2C_NO_STARTSTOP);
        }
        else
        {
          hfmpi2c->XferSize = hfmpi2c->XferCount;
          FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_AUTOEND_MODE, FMPI2C_NO_STARTSTOP);
        }
      }
    }

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set */
    if (FMPI2C_WaitOnSTOPFlagUntilTimeout(hfmpi2c, Timeout, tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        return HAL_ERROR;
      }
      else
      {
        return HAL_TIMEOUT;
      }
    }

    /* Clear STOP Flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    FMPI2C_RESET_CR2(hfmpi2c);

    hfmpi2c->State = HAL_FMPI2C_STATE_READY;
    hfmpi2c->Mode  = HAL_FMPI2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receives in master mode an amount of data in blocking mode.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Master_Receive(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_BUSY, SET, FMPI2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    hfmpi2c->State     = HAL_FMPI2C_STATE_BUSY_RX;
    hfmpi2c->Mode      = HAL_FMPI2C_MODE_MASTER;
    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr  = pData;
    hfmpi2c->XferCount = Size;
    hfmpi2c->XferISR   = NULL;

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_RELOAD_MODE, FMPI2C_GENERATE_START_READ);
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_AUTOEND_MODE, FMPI2C_GENERATE_START_READ);
    }

    while (hfmpi2c->XferCount > 0U)
    {
      /* Wait until RXNE flag is set */
      if (FMPI2C_WaitOnRXNEFlagUntilTimeout(hfmpi2c, Timeout, tickstart) != HAL_OK)
      {
        if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
        {
          return HAL_ERROR;
        }
        else
        {
          return HAL_TIMEOUT;
        }
      }

      /* Read data from RXDR */
      (*hfmpi2c->pBuffPtr++) = hfmpi2c->Instance->RXDR;
      hfmpi2c->XferSize--;
      hfmpi2c->XferCount--;

      if ((hfmpi2c->XferSize == 0U) && (hfmpi2c->XferCount != 0U))
      {
        /* Wait until TCR flag is set */
        if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hfmpi2c->XferSize = MAX_NBYTE_SIZE;
          FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_RELOAD_MODE, FMPI2C_NO_STARTSTOP);
        }
        else
        {
          hfmpi2c->XferSize = hfmpi2c->XferCount;
          FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_AUTOEND_MODE, FMPI2C_NO_STARTSTOP);
        }
      }
    }

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set */
    if (FMPI2C_WaitOnSTOPFlagUntilTimeout(hfmpi2c, Timeout, tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        return HAL_ERROR;
      }
      else
      {
        return HAL_TIMEOUT;
      }
    }

    /* Clear STOP Flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    FMPI2C_RESET_CR2(hfmpi2c);

    hfmpi2c->State = HAL_FMPI2C_STATE_READY;
    hfmpi2c->Mode  = HAL_FMPI2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmits in slave mode an amount of data in blocking mode.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Slave_Transmit(FMPI2C_HandleTypeDef *hfmpi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hfmpi2c->State     = HAL_FMPI2C_STATE_BUSY_TX;
    hfmpi2c->Mode      = HAL_FMPI2C_MODE_SLAVE;
    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr  = pData;
    hfmpi2c->XferCount = Size;
    hfmpi2c->XferISR   = NULL;

    /* Enable Address Acknowledge */
    hfmpi2c->Instance->CR2 &= ~FMPI2C_CR2_NACK;

    /* Wait until ADDR flag is set */
    if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_ADDR, RESET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;
      return HAL_TIMEOUT;
    }

    /* Clear ADDR flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_ADDR);

    /* If 10bit addressing mode is selected */
    if (hfmpi2c->Init.AddressingMode == FMPI2C_ADDRESSINGMODE_10BIT)
    {
      /* Wait until ADDR flag is set */
      if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_ADDR, RESET, Timeout, tickstart) != HAL_OK)
      {
        /* Disable Address Acknowledge */
        hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;
        return HAL_TIMEOUT;
      }

      /* Clear ADDR flag */
      __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_ADDR);
    }

    /* Wait until DIR flag is set Transmitter mode */
    if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_DIR, RESET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;
      return HAL_TIMEOUT;
    }

    while (hfmpi2c->XferCount > 0U)
    {
      /* Wait until TXIS flag is set */
      if (FMPI2C_WaitOnTXISFlagUntilTimeout(hfmpi2c, Timeout, tickstart) != HAL_OK)
      {
        /* Disable Address Acknowledge */
        hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;

        if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
        {
          return HAL_ERROR;
        }
        else
        {
          return HAL_TIMEOUT;
        }
      }

      /* Write data to TXDR */
      hfmpi2c->Instance->TXDR = (*hfmpi2c->pBuffPtr++);
      hfmpi2c->XferCount--;
    }

    /* Wait until STOP flag is set */
    if (FMPI2C_WaitOnSTOPFlagUntilTimeout(hfmpi2c, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;

      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        /* Normal use case for Transmitter mode */
        /* A NACK is generated to confirm the end of transfer */
        hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;
      }
      else
      {
        return HAL_TIMEOUT;
      }
    }

    /* Clear STOP flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);

    /* Wait until BUSY flag is reset */
    if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_BUSY, SET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;
      return HAL_TIMEOUT;
    }

    /* Disable Address Acknowledge */
    hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;

    hfmpi2c->State = HAL_FMPI2C_STATE_READY;
    hfmpi2c->Mode  = HAL_FMPI2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in slave mode an amount of data in blocking mode
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Slave_Receive(FMPI2C_HandleTypeDef *hfmpi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hfmpi2c->State     = HAL_FMPI2C_STATE_BUSY_RX;
    hfmpi2c->Mode      = HAL_FMPI2C_MODE_SLAVE;
    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr  = pData;
    hfmpi2c->XferCount = Size;
    hfmpi2c->XferISR   = NULL;

    /* Enable Address Acknowledge */
    hfmpi2c->Instance->CR2 &= ~FMPI2C_CR2_NACK;

    /* Wait until ADDR flag is set */
    if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_ADDR, RESET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;
      return HAL_TIMEOUT;
    }

    /* Clear ADDR flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_ADDR);

    /* Wait until DIR flag is reset Receiver mode */
    if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_DIR, SET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;
      return HAL_TIMEOUT;
    }

    while (hfmpi2c->XferCount > 0U)
    {
      /* Wait until RXNE flag is set */
      if (FMPI2C_WaitOnRXNEFlagUntilTimeout(hfmpi2c, Timeout, tickstart) != HAL_OK)
      {
        /* Disable Address Acknowledge */
        hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;

        /* Store Last receive data if any */
        if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_RXNE) == SET)
        {
          /* Read data from RXDR */
          (*hfmpi2c->pBuffPtr++) = hfmpi2c->Instance->RXDR;
          hfmpi2c->XferCount--;
        }

        if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_TIMEOUT)
        {
          return HAL_TIMEOUT;
        }
        else
        {
          return HAL_ERROR;
        }
      }

      /* Read data from RXDR */
      (*hfmpi2c->pBuffPtr++) = hfmpi2c->Instance->RXDR;
      hfmpi2c->XferCount--;
    }

    /* Wait until STOP flag is set */
    if (FMPI2C_WaitOnSTOPFlagUntilTimeout(hfmpi2c, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;

      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        return HAL_ERROR;
      }
      else
      {
        return HAL_TIMEOUT;
      }
    }

    /* Clear STOP flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);

    /* Wait until BUSY flag is reset */
    if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_BUSY, SET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;
      return HAL_TIMEOUT;
    }

    /* Disable Address Acknowledge */
    hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;

    hfmpi2c->State = HAL_FMPI2C_STATE_READY;
    hfmpi2c->Mode  = HAL_FMPI2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmit in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Master_Transmit_IT(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  uint32_t xfermode = 0U;

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_TX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_MASTER;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Master_ISR_IT;

    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = FMPI2C_RELOAD_MODE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      xfermode = FMPI2C_AUTOEND_MODE;
    }

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE */
    FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, xfermode, FMPI2C_GENERATE_START_WRITE);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* FMPI2C_IT_ERRI | FMPI2C_IT_TCI| FMPI2C_IT_STOPI| FMPI2C_IT_NACKI | FMPI2C_IT_ADDRI | FMPI2C_IT_RXI | FMPI2C_IT_TXI */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Master_Receive_IT(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  uint32_t xfermode = 0U;

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_RX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_MASTER;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Master_ISR_IT;

    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = FMPI2C_RELOAD_MODE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      xfermode = FMPI2C_AUTOEND_MODE;
    }

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE */
    FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, xfermode, FMPI2C_GENERATE_START_READ);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, RXI interrupt */
    /* possible to enable all of these */
    /* FMPI2C_IT_ERRI | FMPI2C_IT_TCI| FMPI2C_IT_STOPI| FMPI2C_IT_NACKI | FMPI2C_IT_ADDRI | FMPI2C_IT_RXI | FMPI2C_IT_TXI */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_RX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Slave_Transmit_IT(FMPI2C_HandleTypeDef *hfmpi2c, uint8_t *pData, uint16_t Size)
{
  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_TX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_SLAVE;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Enable Address Acknowledge */
    hfmpi2c->Instance->CR2 &= ~FMPI2C_CR2_NACK;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferSize    = hfmpi2c->XferCount;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Slave_ISR_IT;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* FMPI2C_IT_ERRI | FMPI2C_IT_TCI| FMPI2C_IT_STOPI| FMPI2C_IT_NACKI | FMPI2C_IT_ADDRI | FMPI2C_IT_RXI | FMPI2C_IT_TXI */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT | FMPI2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Slave_Receive_IT(FMPI2C_HandleTypeDef *hfmpi2c, uint8_t *pData, uint16_t Size)
{
  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_RX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_SLAVE;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Enable Address Acknowledge */
    hfmpi2c->Instance->CR2 &= ~FMPI2C_CR2_NACK;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferSize    = hfmpi2c->XferCount;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Slave_ISR_IT;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, RXI interrupt */
    /* possible to enable all of these */
    /* FMPI2C_IT_ERRI | FMPI2C_IT_TCI| FMPI2C_IT_STOPI| FMPI2C_IT_NACKI | FMPI2C_IT_ADDRI | FMPI2C_IT_RXI | FMPI2C_IT_TXI */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_RX_IT | FMPI2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmit in master mode an amount of data in non-blocking mode with DMA
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Master_Transmit_DMA(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  uint32_t xfermode = 0U;

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_TX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_MASTER;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Master_ISR_DMA;

    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = FMPI2C_RELOAD_MODE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      xfermode = FMPI2C_AUTOEND_MODE;
    }

    if (hfmpi2c->XferSize > 0U)
    {
      /* Set the FMPI2C DMA transfer complete callback */
      hfmpi2c->hdmatx->XferCpltCallback = FMPI2C_DMAMasterTransmitCplt;

      /* Set the DMA error callback */
      hfmpi2c->hdmatx->XferErrorCallback = FMPI2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hfmpi2c->hdmatx->XferHalfCpltCallback = NULL;
      hfmpi2c->hdmatx->XferAbortCallback = NULL;

      /* Enable the DMA stream */
      HAL_DMA_Start_IT(hfmpi2c->hdmatx, (uint32_t)pData, (uint32_t)&hfmpi2c->Instance->TXDR, hfmpi2c->XferSize);

      /* Send Slave Address */
      /* Set NBYTES to write and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, xfermode, FMPI2C_GENERATE_START_WRITE);

      /* Update XferCount value */
      hfmpi2c->XferCount -= hfmpi2c->XferSize;

      /* Process Unlocked */
      __HAL_UNLOCK(hfmpi2c);

      /* Note : The FMPI2C interrupts must be enabled after unlocking current process
                to avoid the risk of FMPI2C interrupt handle execution before current
                process unlock */
      /* Enable ERR and NACK interrupts */
      FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_ERROR_IT);

      /* Enable DMA Request */
      hfmpi2c->Instance->CR1 |= FMPI2C_CR1_TXDMAEN;
    }
    else
    {
      /* Update Transfer ISR function pointer */
      hfmpi2c->XferISR = FMPI2C_Master_ISR_IT;

      /* Send Slave Address */
      /* Set NBYTES to write and generate START condition */
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_AUTOEND_MODE, FMPI2C_GENERATE_START_WRITE);

      /* Process Unlocked */
      __HAL_UNLOCK(hfmpi2c);

      /* Note : The FMPI2C interrupts must be enabled after unlocking current process
                to avoid the risk of FMPI2C interrupt handle execution before current
                process unlock */
      /* Enable ERR, TC, STOP, NACK, TXI interrupt */
      /* possible to enable all of these */
      /* FMPI2C_IT_ERRI | FMPI2C_IT_TCI| FMPI2C_IT_STOPI| FMPI2C_IT_NACKI | FMPI2C_IT_ADDRI | FMPI2C_IT_RXI | FMPI2C_IT_TXI */
      FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT);
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with DMA
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Master_Receive_DMA(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  uint32_t xfermode = 0U;

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_RX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_MASTER;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Master_ISR_DMA;

    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = FMPI2C_RELOAD_MODE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      xfermode = FMPI2C_AUTOEND_MODE;
    }

    if (hfmpi2c->XferSize > 0U)
    {
      /* Set the FMPI2C DMA transfer complete callback */
      hfmpi2c->hdmarx->XferCpltCallback = FMPI2C_DMAMasterReceiveCplt;

      /* Set the DMA error callback */
      hfmpi2c->hdmarx->XferErrorCallback = FMPI2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hfmpi2c->hdmarx->XferHalfCpltCallback = NULL;
      hfmpi2c->hdmarx->XferAbortCallback = NULL;

      /* Enable the DMA stream */
      HAL_DMA_Start_IT(hfmpi2c->hdmarx, (uint32_t)&hfmpi2c->Instance->RXDR, (uint32_t)pData, hfmpi2c->XferSize);

      /* Send Slave Address */
      /* Set NBYTES to read and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, xfermode, FMPI2C_GENERATE_START_READ);

      /* Update XferCount value */
      hfmpi2c->XferCount -= hfmpi2c->XferSize;

      /* Process Unlocked */
      __HAL_UNLOCK(hfmpi2c);

      /* Note : The FMPI2C interrupts must be enabled after unlocking current process
                to avoid the risk of FMPI2C interrupt handle execution before current
                process unlock */
      /* Enable ERR and NACK interrupts */
      FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_ERROR_IT);

      /* Enable DMA Request */
      hfmpi2c->Instance->CR1 |= FMPI2C_CR1_RXDMAEN;
    }
    else
    {
      /* Update Transfer ISR function pointer */
      hfmpi2c->XferISR = FMPI2C_Master_ISR_IT;

      /* Send Slave Address */
      /* Set NBYTES to read and generate START condition */
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_AUTOEND_MODE, FMPI2C_GENERATE_START_READ);

      /* Process Unlocked */
      __HAL_UNLOCK(hfmpi2c);

      /* Note : The FMPI2C interrupts must be enabled after unlocking current process
                to avoid the risk of FMPI2C interrupt handle execution before current
                process unlock */
      /* Enable ERR, TC, STOP, NACK, TXI interrupt */
      /* possible to enable all of these */
      /* FMPI2C_IT_ERRI | FMPI2C_IT_TCI| FMPI2C_IT_STOPI| FMPI2C_IT_NACKI | FMPI2C_IT_ADDRI | FMPI2C_IT_RXI | FMPI2C_IT_TXI */
      FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT);
    }
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with DMA
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Slave_Transmit_DMA(FMPI2C_HandleTypeDef *hfmpi2c, uint8_t *pData, uint16_t Size)
{
  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_TX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_SLAVE;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferSize    = hfmpi2c->XferCount;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Slave_ISR_DMA;

    /* Set the FMPI2C DMA transfer complete callback */
    hfmpi2c->hdmatx->XferCpltCallback = FMPI2C_DMASlaveTransmitCplt;

    /* Set the DMA error callback */
    hfmpi2c->hdmatx->XferErrorCallback = FMPI2C_DMAError;

    /* Set the unused DMA callbacks to NULL */
    hfmpi2c->hdmatx->XferHalfCpltCallback = NULL;
    hfmpi2c->hdmatx->XferAbortCallback = NULL;

    /* Enable the DMA stream */
    HAL_DMA_Start_IT(hfmpi2c->hdmatx, (uint32_t)pData, (uint32_t)&hfmpi2c->Instance->TXDR, hfmpi2c->XferSize);

    /* Enable Address Acknowledge */
    hfmpi2c->Instance->CR2 &= ~FMPI2C_CR2_NACK;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */
    /* Enable ERR, STOP, NACK, ADDR interrupts */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_LISTEN_IT);

    /* Enable DMA Request */
    hfmpi2c->Instance->CR1 |= FMPI2C_CR1_TXDMAEN;

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with DMA
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Slave_Receive_DMA(FMPI2C_HandleTypeDef *hfmpi2c, uint8_t *pData, uint16_t Size)
{
  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_RX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_SLAVE;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferSize    = hfmpi2c->XferCount;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Slave_ISR_DMA;

    /* Set the FMPI2C DMA transfer complete callback */
    hfmpi2c->hdmarx->XferCpltCallback = FMPI2C_DMASlaveReceiveCplt;

    /* Set the DMA error callback */
    hfmpi2c->hdmarx->XferErrorCallback = FMPI2C_DMAError;

    /* Set the unused DMA callbacks to NULL */
    hfmpi2c->hdmarx->XferHalfCpltCallback = NULL;
    hfmpi2c->hdmarx->XferAbortCallback = NULL;

    /* Enable the DMA stream */
    HAL_DMA_Start_IT(hfmpi2c->hdmarx, (uint32_t)&hfmpi2c->Instance->RXDR, (uint32_t)pData, hfmpi2c->XferSize);

    /* Enable Address Acknowledge */
    hfmpi2c->Instance->CR2 &= ~FMPI2C_CR2_NACK;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */
    /* Enable ERR, STOP, NACK, ADDR interrupts */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_LISTEN_IT);

    /* Enable DMA Request */
    hfmpi2c->Instance->CR1 |= FMPI2C_CR1_RXDMAEN;

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Mem_Write(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Check the parameters */
  assert_param(IS_FMPI2C_MEMADD_SIZE(MemAddSize));

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_BUSY, SET, FMPI2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    hfmpi2c->State     = HAL_FMPI2C_STATE_BUSY_TX;
    hfmpi2c->Mode      = HAL_FMPI2C_MODE_MEM;
    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr  = pData;
    hfmpi2c->XferCount = Size;
    hfmpi2c->XferISR   = NULL;

    /* Send Slave Address and Memory Address */
    if (FMPI2C_RequestMemoryWrite(hfmpi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_ERROR;
      }
      else
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_TIMEOUT;
      }
    }

    /* Set NBYTES to write and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE */
    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_RELOAD_MODE, FMPI2C_NO_STARTSTOP);
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_AUTOEND_MODE, FMPI2C_NO_STARTSTOP);
    }

    do
    {
      /* Wait until TXIS flag is set */
      if (FMPI2C_WaitOnTXISFlagUntilTimeout(hfmpi2c, Timeout, tickstart) != HAL_OK)
      {
        if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
        {
          return HAL_ERROR;
        }
        else
        {
          return HAL_TIMEOUT;
        }
      }

      /* Write data to TXDR */
      hfmpi2c->Instance->TXDR = (*hfmpi2c->pBuffPtr++);
      hfmpi2c->XferCount--;
      hfmpi2c->XferSize--;

      if ((hfmpi2c->XferSize == 0U) && (hfmpi2c->XferCount != 0U))
      {
        /* Wait until TCR flag is set */
        if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hfmpi2c->XferSize = MAX_NBYTE_SIZE;
          FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_RELOAD_MODE, FMPI2C_NO_STARTSTOP);
        }
        else
        {
          hfmpi2c->XferSize = hfmpi2c->XferCount;
          FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_AUTOEND_MODE, FMPI2C_NO_STARTSTOP);
        }
      }

    }
    while (hfmpi2c->XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
    if (FMPI2C_WaitOnSTOPFlagUntilTimeout(hfmpi2c, Timeout, tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        return HAL_ERROR;
      }
      else
      {
        return HAL_TIMEOUT;
      }
    }

    /* Clear STOP Flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    FMPI2C_RESET_CR2(hfmpi2c);

    hfmpi2c->State = HAL_FMPI2C_STATE_READY;
    hfmpi2c->Mode  = HAL_FMPI2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Mem_Read(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Check the parameters */
  assert_param(IS_FMPI2C_MEMADD_SIZE(MemAddSize));

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_BUSY, SET, FMPI2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    hfmpi2c->State     = HAL_FMPI2C_STATE_BUSY_RX;
    hfmpi2c->Mode      = HAL_FMPI2C_MODE_MEM;
    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr  = pData;
    hfmpi2c->XferCount = Size;
    hfmpi2c->XferISR   = NULL;

    /* Send Slave Address and Memory Address */
    if (FMPI2C_RequestMemoryRead(hfmpi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_ERROR;
      }
      else
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_TIMEOUT;
      }
    }

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_RELOAD_MODE, FMPI2C_GENERATE_START_READ);
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_AUTOEND_MODE, FMPI2C_GENERATE_START_READ);
    }

    do
    {
      /* Wait until RXNE flag is set */
      if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_RXNE, RESET, Timeout, tickstart) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }

      /* Read data from RXDR */
      (*hfmpi2c->pBuffPtr++) = hfmpi2c->Instance->RXDR;
      hfmpi2c->XferSize--;
      hfmpi2c->XferCount--;

      if ((hfmpi2c->XferSize == 0U) && (hfmpi2c->XferCount != 0U))
      {
        /* Wait until TCR flag is set */
        if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hfmpi2c->XferSize = MAX_NBYTE_SIZE;
          FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_RELOAD_MODE, FMPI2C_NO_STARTSTOP);
        }
        else
        {
          hfmpi2c->XferSize = hfmpi2c->XferCount;
          FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, FMPI2C_AUTOEND_MODE, FMPI2C_NO_STARTSTOP);
        }
      }
    }
    while (hfmpi2c->XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
    if (FMPI2C_WaitOnSTOPFlagUntilTimeout(hfmpi2c, Timeout, tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        return HAL_ERROR;
      }
      else
      {
        return HAL_TIMEOUT;
      }
    }

    /* Clear STOP Flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    FMPI2C_RESET_CR2(hfmpi2c);

    hfmpi2c->State = HAL_FMPI2C_STATE_READY;
    hfmpi2c->Mode  = HAL_FMPI2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/**
  * @brief  Write an amount of data in non-blocking mode with Interrupt to a specific memory address
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Mem_Write_IT(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  uint32_t tickstart = 0U;
  uint32_t xfermode = 0U;

  /* Check the parameters */
  assert_param(IS_FMPI2C_MEMADD_SIZE(MemAddSize));

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_TX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_MEM;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Master_ISR_IT;

    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = FMPI2C_RELOAD_MODE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      xfermode = FMPI2C_AUTOEND_MODE;
    }

    /* Send Slave Address and Memory Address */
    if (FMPI2C_RequestMemoryWrite(hfmpi2c, DevAddress, MemAddress, MemAddSize, FMPI2C_TIMEOUT_FLAG, tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_ERROR;
      }
      else
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_TIMEOUT;
      }
    }

    /* Set NBYTES to write and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, xfermode, FMPI2C_NO_STARTSTOP);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* FMPI2C_IT_ERRI | FMPI2C_IT_TCI| FMPI2C_IT_STOPI| FMPI2C_IT_NACKI | FMPI2C_IT_ADDRI | FMPI2C_IT_RXI | FMPI2C_IT_TXI */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Read an amount of data in non-blocking mode with Interrupt from a specific memory address
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Mem_Read_IT(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  uint32_t tickstart = 0U;
  uint32_t xfermode = 0U;

  /* Check the parameters */
  assert_param(IS_FMPI2C_MEMADD_SIZE(MemAddSize));

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_RX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_MEM;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Master_ISR_IT;

    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = FMPI2C_RELOAD_MODE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      xfermode = FMPI2C_AUTOEND_MODE;
    }

    /* Send Slave Address and Memory Address */
    if (FMPI2C_RequestMemoryRead(hfmpi2c, DevAddress, MemAddress, MemAddSize, FMPI2C_TIMEOUT_FLAG, tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_ERROR;
      }
      else
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_TIMEOUT;
      }
    }

    /* Set NBYTES to write and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, xfermode, FMPI2C_GENERATE_START_READ);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, RXI interrupt */
    /* possible to enable all of these */
    /* FMPI2C_IT_ERRI | FMPI2C_IT_TCI| FMPI2C_IT_STOPI| FMPI2C_IT_NACKI | FMPI2C_IT_ADDRI | FMPI2C_IT_RXI | FMPI2C_IT_TXI */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_RX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/**
  * @brief  Write an amount of data in non-blocking mode with DMA to a specific memory address
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Mem_Write_DMA(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  uint32_t tickstart = 0U;
  uint32_t xfermode = 0U;

  /* Check the parameters */
  assert_param(IS_FMPI2C_MEMADD_SIZE(MemAddSize));

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_TX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_MEM;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Master_ISR_DMA;

    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = FMPI2C_RELOAD_MODE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      xfermode = FMPI2C_AUTOEND_MODE;
    }

    /* Send Slave Address and Memory Address */
    if (FMPI2C_RequestMemoryWrite(hfmpi2c, DevAddress, MemAddress, MemAddSize, FMPI2C_TIMEOUT_FLAG, tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_ERROR;
      }
      else
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_TIMEOUT;
      }
    }

    /* Set the FMPI2C DMA transfer complete callback */
    hfmpi2c->hdmatx->XferCpltCallback = FMPI2C_DMAMasterTransmitCplt;

    /* Set the DMA error callback */
    hfmpi2c->hdmatx->XferErrorCallback = FMPI2C_DMAError;

    /* Set the unused DMA callbacks to NULL */
    hfmpi2c->hdmatx->XferHalfCpltCallback = NULL;
    hfmpi2c->hdmatx->XferAbortCallback = NULL;

    /* Enable the DMA stream */
    HAL_DMA_Start_IT(hfmpi2c->hdmatx, (uint32_t)pData, (uint32_t)&hfmpi2c->Instance->TXDR, hfmpi2c->XferSize);

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, xfermode, FMPI2C_NO_STARTSTOP);

    /* Update XferCount value */
    hfmpi2c->XferCount -= hfmpi2c->XferSize;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */
    /* Enable ERR and NACK interrupts */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_ERROR_IT);

    /* Enable DMA Request */
    hfmpi2c->Instance->CR1 |= FMPI2C_CR1_TXDMAEN;

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Reads an amount of data in non-blocking mode with DMA from a specific memory address.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be read
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Mem_Read_DMA(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  uint32_t tickstart = 0U;
  uint32_t xfermode = 0U;

  /* Check the parameters */
  assert_param(IS_FMPI2C_MEMADD_SIZE(MemAddSize));

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hfmpi2c->State       = HAL_FMPI2C_STATE_BUSY_RX;
    hfmpi2c->Mode        = HAL_FMPI2C_MODE_MEM;
    hfmpi2c->ErrorCode   = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->XferISR     = FMPI2C_Master_ISR_DMA;

    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = FMPI2C_RELOAD_MODE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      xfermode = FMPI2C_AUTOEND_MODE;
    }

    /* Send Slave Address and Memory Address */
    if (FMPI2C_RequestMemoryRead(hfmpi2c, DevAddress, MemAddress, MemAddSize, FMPI2C_TIMEOUT_FLAG, tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_ERROR;
      }
      else
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_TIMEOUT;
      }
    }

    /* Set the FMPI2C DMA transfer complete callback */
    hfmpi2c->hdmarx->XferCpltCallback = FMPI2C_DMAMasterReceiveCplt;

    /* Set the DMA error callback */
    hfmpi2c->hdmarx->XferErrorCallback = FMPI2C_DMAError;

    /* Set the unused DMA callbacks to NULL */
    hfmpi2c->hdmarx->XferHalfCpltCallback = NULL;
    hfmpi2c->hdmarx->XferAbortCallback = NULL;

    /* Enable the DMA stream */
    HAL_DMA_Start_IT(hfmpi2c->hdmarx, (uint32_t)&hfmpi2c->Instance->RXDR, (uint32_t)pData, hfmpi2c->XferSize);

    /* Set NBYTES to write and reload if hfmpi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, xfermode, FMPI2C_GENERATE_START_READ);

    /* Update XferCount value */
    hfmpi2c->XferCount -= hfmpi2c->XferSize;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Enable DMA Request */
    hfmpi2c->Instance->CR1 |= FMPI2C_CR1_RXDMAEN;

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */
    /* Enable ERR and NACK interrupts */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_ERROR_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  Trials Number of trials
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_IsDeviceReady(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  __IO uint32_t FMPI2C_Trials = 0U;

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State = HAL_FMPI2C_STATE_BUSY;
    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;

    do
    {
      /* Generate Start */
      hfmpi2c->Instance->CR2 = FMPI2C_GENERATE_START(hfmpi2c->Init.AddressingMode, DevAddress);

      /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
      /* Wait until STOPF flag is set or a NACK flag is set*/
      tickstart = HAL_GetTick();
      while ((__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF) == RESET) && (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_AF) == RESET) && (hfmpi2c->State != HAL_FMPI2C_STATE_TIMEOUT))
      {
        if (Timeout != HAL_MAX_DELAY)
        {
          if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
          {
            /* Device is ready */
            hfmpi2c->State = HAL_FMPI2C_STATE_READY;
            /* Process Unlocked */
            __HAL_UNLOCK(hfmpi2c);
            return HAL_TIMEOUT;
          }
        }
      }

      /* Check if the NACKF flag has not been set */
      if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_AF) == RESET)
      {
        /* Wait until STOPF flag is reset */
        if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        /* Clear STOP Flag */
        __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);

        /* Device is ready */
        hfmpi2c->State = HAL_FMPI2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);

        return HAL_OK;
      }
      else
      {
        /* Wait until STOPF flag is reset */
        if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        /* Clear NACK Flag */
        __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_AF);

        /* Clear STOP Flag, auto generated with autoend*/
        __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);
      }

      /* Check if the maximum allowed number of trials has been reached */
      if (FMPI2C_Trials++ == Trials)
      {
        /* Generate Stop */
        hfmpi2c->Instance->CR2 |= FMPI2C_CR2_STOP;

        /* Wait until STOPF flag is reset */
        if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        /* Clear STOP Flag */
        __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);
      }
    }
    while (FMPI2C_Trials < Trials);

    hfmpi2c->State = HAL_FMPI2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    return HAL_TIMEOUT;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Sequential transmit in master FMPI2C mode an amount of data in non-blocking mode with Interrupt.
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref FMPI2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Master_Sequential_Transmit_IT(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  uint32_t xfermode = 0U;
  uint32_t xferrequest = FMPI2C_GENERATE_START_WRITE;

  /* Check the parameters */
  assert_param(IS_FMPI2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State     = HAL_FMPI2C_STATE_BUSY_TX;
    hfmpi2c->Mode      = HAL_FMPI2C_MODE_MASTER;
    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferOptions = XferOptions;
    hfmpi2c->XferISR     = FMPI2C_Master_ISR_IT;

    /* If size > MAX_NBYTE_SIZE, use reload mode */
    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = FMPI2C_RELOAD_MODE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      xfermode = hfmpi2c->XferOptions;
    }

    /* If transfer direction not change, do not generate Restart Condition */
    /* Mean Previous state is same as current state */
    if (hfmpi2c->PreviousState == FMPI2C_STATE_MASTER_BUSY_TX)
    {
      xferrequest = FMPI2C_NO_STARTSTOP;
    }

    /* Send Slave Address and set NBYTES to write */
    FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, xfermode, xferrequest);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Sequential receive in master FMPI2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref FMPI2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Master_Sequential_Receive_IT(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  uint32_t xfermode = 0U;
  uint32_t xferrequest = FMPI2C_GENERATE_START_READ;

  /* Check the parameters */
  assert_param(IS_FMPI2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State     = HAL_FMPI2C_STATE_BUSY_RX;
    hfmpi2c->Mode      = HAL_FMPI2C_MODE_MASTER;
    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferOptions = XferOptions;
    hfmpi2c->XferISR     = FMPI2C_Master_ISR_IT;

    /* If hfmpi2c->XferCount > MAX_NBYTE_SIZE, use reload mode */
    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = FMPI2C_RELOAD_MODE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
      xfermode = hfmpi2c->XferOptions;
    }

    /* If transfer direction not change, do not generate Restart Condition */
    /* Mean Previous state is same as current state */
    if (hfmpi2c->PreviousState == FMPI2C_STATE_MASTER_BUSY_RX)
    {
      xferrequest = FMPI2C_NO_STARTSTOP;
    }

    /* Send Slave Address and set NBYTES to read */
    FMPI2C_TransferConfig(hfmpi2c, DevAddress, hfmpi2c->XferSize, xfermode, xferrequest);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_RX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Sequential transmit in slave/device FMPI2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref FMPI2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Slave_Sequential_Transmit_IT(FMPI2C_HandleTypeDef *hfmpi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if ((hfmpi2c->State & HAL_FMPI2C_STATE_LISTEN) == HAL_FMPI2C_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
    FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_LISTEN_IT | FMPI2C_XFER_TX_IT);

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* FMPI2C cannot manage full duplex exchange so disable previous IT enabled if any */
    /* and then toggle the HAL slave RX state to TX state */
    if (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_RX_LISTEN)
    {
      /* Disable associated Interrupts */
      FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_RX_IT);
    }

    hfmpi2c->State     = HAL_FMPI2C_STATE_BUSY_TX_LISTEN;
    hfmpi2c->Mode      = HAL_FMPI2C_MODE_SLAVE;
    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;

    /* Enable Address Acknowledge */
    hfmpi2c->Instance->CR2 &= ~FMPI2C_CR2_NACK;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferSize    = hfmpi2c->XferCount;
    hfmpi2c->XferOptions = XferOptions;
    hfmpi2c->XferISR     = FMPI2C_Slave_ISR_IT;

    if (FMPI2C_GET_DIR(hfmpi2c) == FMPI2C_DIRECTION_RECEIVE)
    {
      /* Clear ADDR flag after prepare the transfer parameters */
      /* This action will generate an acknowledge to the Master */
      __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_ADDR);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
    to avoid the risk of FMPI2C interrupt handle execution before current
    process unlock */
    /* REnable ADDR interrupt */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT | FMPI2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Sequential receive in slave/device FMPI2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref FMPI2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Slave_Sequential_Receive_IT(FMPI2C_HandleTypeDef *hfmpi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if ((hfmpi2c->State & HAL_FMPI2C_STATE_LISTEN) == HAL_FMPI2C_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
    FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_LISTEN_IT | FMPI2C_XFER_RX_IT);

    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* FMPI2C cannot manage full duplex exchange so disable previous IT enabled if any */
    /* and then toggle the HAL slave TX state to RX state */
    if (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_TX_LISTEN)
    {
      /* Disable associated Interrupts */
      FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT);
    }

    hfmpi2c->State     = HAL_FMPI2C_STATE_BUSY_RX_LISTEN;
    hfmpi2c->Mode      = HAL_FMPI2C_MODE_SLAVE;
    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;

    /* Enable Address Acknowledge */
    hfmpi2c->Instance->CR2 &= ~FMPI2C_CR2_NACK;

    /* Prepare transfer parameters */
    hfmpi2c->pBuffPtr    = pData;
    hfmpi2c->XferCount   = Size;
    hfmpi2c->XferSize    = hfmpi2c->XferCount;
    hfmpi2c->XferOptions = XferOptions;
    hfmpi2c->XferISR     = FMPI2C_Slave_ISR_IT;

    if (FMPI2C_GET_DIR(hfmpi2c) == FMPI2C_DIRECTION_TRANSMIT)
    {
      /* Clear ADDR flag after prepare the transfer parameters */
      /* This action will generate an acknowledge to the Master */
      __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_ADDR);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
    to avoid the risk of FMPI2C interrupt handle execution before current
    process unlock */
    /* REnable ADDR interrupt */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_RX_IT | FMPI2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Enable the Address listen mode with Interrupt.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_EnableListen_IT(FMPI2C_HandleTypeDef *hfmpi2c)
{
  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    hfmpi2c->State = HAL_FMPI2C_STATE_LISTEN;
    hfmpi2c->XferISR = FMPI2C_Slave_ISR_IT;

    /* Enable the Address Match interrupt */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Disable the Address listen mode with Interrupt.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_DisableListen_IT(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Declaration of tmp to prevent undefined behavior of volatile usage */
  uint32_t tmp;

  /* Disable Address listen mode only if a transfer is not ongoing */
  if (hfmpi2c->State == HAL_FMPI2C_STATE_LISTEN)
  {
    tmp = (uint32_t)(hfmpi2c->State) & FMPI2C_STATE_MSK;
    hfmpi2c->PreviousState = tmp | (uint32_t)(hfmpi2c->Mode);
    hfmpi2c->State = HAL_FMPI2C_STATE_READY;
    hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;
    hfmpi2c->XferISR = NULL;

    /* Disable the Address Match interrupt */
    FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Abort a master FMPI2C IT or DMA process communication with Interrupt.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2C_Master_Abort_IT(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress)
{
  if (hfmpi2c->Mode == HAL_FMPI2C_MODE_MASTER)
  {
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    /* Disable Interrupts */
    FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_RX_IT);
    FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT);

    /* Set State at HAL_FMPI2C_STATE_ABORT */
    hfmpi2c->State = HAL_FMPI2C_STATE_ABORT;

    /* Set NBYTES to 1 to generate a dummy read on FMPI2C peripheral */
    /* Set AUTOEND mode, this will generate a NACK then STOP condition to abort the current transfer */
    FMPI2C_TransferConfig(hfmpi2c, DevAddress, 1, FMPI2C_AUTOEND_MODE, FMPI2C_GENERATE_STOP);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Note : The FMPI2C interrupts must be enabled after unlocking current process
              to avoid the risk of FMPI2C interrupt handle execution before current
              process unlock */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_CPLT_IT);

    return HAL_OK;
  }
  else
  {
    /* Wrong usage of abort function */
    /* This function should be used only in case of abort monitored by master device */
    return HAL_ERROR;
  }
}

/**
  * @}
  */

/** @defgroup FMPI2C_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */

/**
  * @brief  This function handles FMPI2C event interrupt request.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
void HAL_FMPI2C_EV_IRQHandler(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Get current IT Flags and IT sources value */
  uint32_t itflags   = READ_REG(hfmpi2c->Instance->ISR);
  uint32_t itsources = READ_REG(hfmpi2c->Instance->CR1);

  /* FMPI2C events treatment -------------------------------------*/
  if (hfmpi2c->XferISR != NULL)
  {
    hfmpi2c->XferISR(hfmpi2c, itflags, itsources);
  }
}

/**
  * @brief  This function handles FMPI2C error interrupt request.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
void HAL_FMPI2C_ER_IRQHandler(FMPI2C_HandleTypeDef *hfmpi2c)
{
  uint32_t itflags   = READ_REG(hfmpi2c->Instance->ISR);
  uint32_t itsources = READ_REG(hfmpi2c->Instance->CR1);

  /* FMPI2C Bus error interrupt occurred ------------------------------------*/
  if (((itflags & FMPI2C_FLAG_BERR) != RESET) && ((itsources & FMPI2C_IT_ERRI) != RESET))
  {
    hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_BERR;

    /* Clear BERR flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_BERR);
  }

  /* FMPI2C Over-Run/Under-Run interrupt occurred ----------------------------------------*/
  if (((itflags & FMPI2C_FLAG_OVR) != RESET) && ((itsources & FMPI2C_IT_ERRI) != RESET))
  {
    hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_OVR;

    /* Clear OVR flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_OVR);
  }

  /* FMPI2C Arbitration Loss error interrupt occurred -------------------------------------*/
  if (((itflags & FMPI2C_FLAG_ARLO) != RESET) && ((itsources & FMPI2C_IT_ERRI) != RESET))
  {
    hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_ARLO;

    /* Clear ARLO flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_ARLO);
  }

  /* Call the Error Callback in case of Error detected */
  if ((hfmpi2c->ErrorCode & (HAL_FMPI2C_ERROR_BERR | HAL_FMPI2C_ERROR_OVR | HAL_FMPI2C_ERROR_ARLO)) !=  HAL_FMPI2C_ERROR_NONE)
  {
    FMPI2C_ITError(hfmpi2c, hfmpi2c->ErrorCode);
  }
}

/**
  * @brief  Master Tx Transfer completed callback.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
__weak void HAL_FMPI2C_MasterTxCpltCallback(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_MasterTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Master Rx Transfer completed callback.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
__weak void HAL_FMPI2C_MasterRxCpltCallback(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_MasterRxCpltCallback could be implemented in the user file
   */
}

/** @brief  Slave Tx Transfer completed callback.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
__weak void HAL_FMPI2C_SlaveTxCpltCallback(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_SlaveTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
__weak void HAL_FMPI2C_SlaveRxCpltCallback(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_SlaveRxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Slave Address Match callback.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  TransferDirection Master request Transfer Direction (Write/Read), value of @ref FMPI2C_XFERDIRECTION
  * @param  AddrMatchCode Address Match Code
  * @retval None
  */
__weak void HAL_FMPI2C_AddrCallback(FMPI2C_HandleTypeDef *hfmpi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);
  UNUSED(TransferDirection);
  UNUSED(AddrMatchCode);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_AddrCallback() could be implemented in the user file
   */
}

/**
  * @brief  Listen Complete callback.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
__weak void HAL_FMPI2C_ListenCpltCallback(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_ListenCpltCallback() could be implemented in the user file
   */
}

/**
  * @brief  Memory Tx Transfer completed callback.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
__weak void HAL_FMPI2C_MemTxCpltCallback(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_MemTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Memory Rx Transfer completed callback.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
__weak void HAL_FMPI2C_MemRxCpltCallback(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_MemRxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  FMPI2C error callback.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
__weak void HAL_FMPI2C_ErrorCallback(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_ErrorCallback could be implemented in the user file
   */
}

/**
  * @brief  FMPI2C abort callback.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval None
  */
__weak void HAL_FMPI2C_AbortCpltCallback(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfmpi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_FMPI2C_AbortCpltCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup FMPI2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
 *  @brief   Peripheral State, Mode and Error functions
 *
@verbatim
 ===============================================================================
            ##### Peripheral State, Mode and Error functions #####
 ===============================================================================
    [..]
    This subsection permit to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the FMPI2C handle state.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @retval HAL state
  */
HAL_FMPI2C_StateTypeDef HAL_FMPI2C_GetState(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Return FMPI2C handle state */
  return hfmpi2c->State;
}

/**
  * @brief  Returns the FMPI2C Master, Slave, Memory or no mode.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *         the configuration information for FMPI2C module
  * @retval HAL mode
  */
HAL_FMPI2C_ModeTypeDef HAL_FMPI2C_GetMode(FMPI2C_HandleTypeDef *hfmpi2c)
{
  return hfmpi2c->Mode;
}

/**
* @brief  Return the FMPI2C error code.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *              the configuration information for the specified FMPI2C.
* @retval FMPI2C Error Code
*/
uint32_t HAL_FMPI2C_GetError(FMPI2C_HandleTypeDef *hfmpi2c)
{
  return hfmpi2c->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup FMPI2C_Private_Functions
  * @{
  */

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with Interrupt.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_Master_ISR_IT(struct __FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags, uint32_t ITSources)
{
  uint16_t devaddress = 0U;

  /* Process Locked */
  __HAL_LOCK(hfmpi2c);

  if (((ITFlags & FMPI2C_FLAG_AF) != RESET) && ((ITSources & FMPI2C_IT_NACKI) != RESET))
  {
    /* Clear NACK Flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_AF);

    /* Set corresponding Error Code */
    /* No need to generate STOP, it is automatically done */
    /* Error callback will be send during stop flag treatment */
    hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_AF;

    /* Flush TX register */
    FMPI2C_Flush_TXDR(hfmpi2c);
  }
  else if (((ITFlags & FMPI2C_FLAG_RXNE) != RESET) && ((ITSources & FMPI2C_IT_RXI) != RESET))
  {
    /* Read data from RXDR */
    (*hfmpi2c->pBuffPtr++) = hfmpi2c->Instance->RXDR;
    hfmpi2c->XferSize--;
    hfmpi2c->XferCount--;
  }
  else if (((ITFlags & FMPI2C_FLAG_TXIS) != RESET) && ((ITSources & FMPI2C_IT_TXI) != RESET))
  {
    /* Write data to TXDR */
    hfmpi2c->Instance->TXDR = (*hfmpi2c->pBuffPtr++);
    hfmpi2c->XferSize--;
    hfmpi2c->XferCount--;
  }
  else if (((ITFlags & FMPI2C_FLAG_TCR) != RESET) && ((ITSources & FMPI2C_IT_TCI) != RESET))
  {
    if ((hfmpi2c->XferSize == 0U) && (hfmpi2c->XferCount != 0U))
    {
      devaddress = (hfmpi2c->Instance->CR2 & FMPI2C_CR2_SADD);

      if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
      {
        hfmpi2c->XferSize = MAX_NBYTE_SIZE;
        FMPI2C_TransferConfig(hfmpi2c, devaddress, hfmpi2c->XferSize, FMPI2C_RELOAD_MODE, FMPI2C_NO_STARTSTOP);
      }
      else
      {
        hfmpi2c->XferSize = hfmpi2c->XferCount;
        if (hfmpi2c->XferOptions != FMPI2C_NO_OPTION_FRAME)
        {
          FMPI2C_TransferConfig(hfmpi2c, devaddress, hfmpi2c->XferSize, hfmpi2c->XferOptions, FMPI2C_NO_STARTSTOP);
        }
        else
        {
          FMPI2C_TransferConfig(hfmpi2c, devaddress, hfmpi2c->XferSize, FMPI2C_AUTOEND_MODE, FMPI2C_NO_STARTSTOP);
        }
      }
    }
    else
    {
      /* Call TxCpltCallback() if no stop mode is set */
      if (FMPI2C_GET_STOP_MODE(hfmpi2c) != FMPI2C_AUTOEND_MODE)
      {
        /* Call FMPI2C Master Sequential complete process */
        FMPI2C_ITMasterSequentialCplt(hfmpi2c);
      }
      else
      {
        /* Wrong size Status regarding TCR flag event */
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        FMPI2C_ITError(hfmpi2c, HAL_FMPI2C_ERROR_SIZE);
      }
    }
  }
  else if (((ITFlags & FMPI2C_FLAG_TC) != RESET) && ((ITSources & FMPI2C_IT_TCI) != RESET))
  {
    if (hfmpi2c->XferCount == 0U)
    {
      if (FMPI2C_GET_STOP_MODE(hfmpi2c) != FMPI2C_AUTOEND_MODE)
      {
        /* Generate a stop condition in case of no transfer option */
        if (hfmpi2c->XferOptions == FMPI2C_NO_OPTION_FRAME)
        {
          /* Generate Stop */
          hfmpi2c->Instance->CR2 |= FMPI2C_CR2_STOP;
        }
        else
        {
          /* Call FMPI2C Master Sequential complete process */
          FMPI2C_ITMasterSequentialCplt(hfmpi2c);
        }
      }
    }
    else
    {
      /* Wrong size Status regarding TC flag event */
      /* Call the corresponding callback to inform upper layer of End of Transfer */
      FMPI2C_ITError(hfmpi2c, HAL_FMPI2C_ERROR_SIZE);
    }
  }

  if (((ITFlags & FMPI2C_FLAG_STOPF) != RESET) && ((ITSources & FMPI2C_IT_STOPI) != RESET))
  {
    /* Call FMPI2C Master complete process */
    FMPI2C_ITMasterCplt(hfmpi2c, ITFlags);
  }

  /* Process Unlocked */
  __HAL_UNLOCK(hfmpi2c);

  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with Interrupt.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_Slave_ISR_IT(struct __FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags, uint32_t ITSources)
{
  /* Process locked */
  __HAL_LOCK(hfmpi2c);

  if (((ITFlags & FMPI2C_FLAG_AF) != RESET) && ((ITSources & FMPI2C_IT_NACKI) != RESET))
  {
    /* Check that FMPI2C transfer finished */
    /* if yes, normal use case, a NACK is sent by the MASTER when Transfer is finished */
    /* Mean XferCount == 0*/
    /* So clear Flag NACKF only */
    if (hfmpi2c->XferCount == 0U)
    {
      if (((hfmpi2c->XferOptions == FMPI2C_FIRST_AND_LAST_FRAME) || (hfmpi2c->XferOptions == FMPI2C_LAST_FRAME)) && \
          (hfmpi2c->State == HAL_FMPI2C_STATE_LISTEN))
      {
        /* Call FMPI2C Listen complete process */
        FMPI2C_ITListenCplt(hfmpi2c, ITFlags);
      }
      else if ((hfmpi2c->XferOptions != FMPI2C_NO_OPTION_FRAME) && (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_TX_LISTEN))
      {
        /* Clear NACK Flag */
        __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_AF);

        /* Flush TX register */
        FMPI2C_Flush_TXDR(hfmpi2c);

        /* Last Byte is Transmitted */
        /* Call FMPI2C Slave Sequential complete process */
        FMPI2C_ITSlaveSequentialCplt(hfmpi2c);
      }
      else
      {
        /* Clear NACK Flag */
        __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_AF);
      }
    }
    else
    {
      /* if no, error use case, a Non-Acknowledge of last Data is generated by the MASTER*/
      /* Clear NACK Flag */
      __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_AF);

      /* Set ErrorCode corresponding to a Non-Acknowledge */
      hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_AF;
    }
  }
  else if (((ITFlags & FMPI2C_FLAG_RXNE) != RESET) && ((ITSources & FMPI2C_IT_RXI) != RESET))
  {
    if (hfmpi2c->XferCount > 0U)
    {
      /* Read data from RXDR */
      (*hfmpi2c->pBuffPtr++) = hfmpi2c->Instance->RXDR;
      hfmpi2c->XferSize--;
      hfmpi2c->XferCount--;
    }

    if ((hfmpi2c->XferCount == 0U) && \
        (hfmpi2c->XferOptions != FMPI2C_NO_OPTION_FRAME))
    {
      /* Call FMPI2C Slave Sequential complete process */
      FMPI2C_ITSlaveSequentialCplt(hfmpi2c);
    }
  }
  else if (((ITFlags & FMPI2C_FLAG_ADDR) != RESET) && ((ITSources & FMPI2C_IT_ADDRI) != RESET))
  {
    FMPI2C_ITAddrCplt(hfmpi2c, ITFlags);
  }
  else if (((ITFlags & FMPI2C_FLAG_TXIS) != RESET) && ((ITSources & FMPI2C_IT_TXI) != RESET))
  {
    /* Write data to TXDR only if XferCount not reach "0" */
    /* A TXIS flag can be set, during STOP treatment      */
    /* Check if all Datas have already been sent */
    /* If it is the case, this last write in TXDR is not sent, correspond to a dummy TXIS event */
    if (hfmpi2c->XferCount > 0U)
    {
      /* Write data to TXDR */
      hfmpi2c->Instance->TXDR = (*hfmpi2c->pBuffPtr++);
      hfmpi2c->XferCount--;
      hfmpi2c->XferSize--;
    }
    else
    {
      if ((hfmpi2c->XferOptions == FMPI2C_NEXT_FRAME) || (hfmpi2c->XferOptions == FMPI2C_FIRST_FRAME))
      {
        /* Last Byte is Transmitted */
        /* Call FMPI2C Slave Sequential complete process */
        FMPI2C_ITSlaveSequentialCplt(hfmpi2c);
      }
    }
  }

  /* Check if STOPF is set */
  if (((ITFlags & FMPI2C_FLAG_STOPF) != RESET) && ((ITSources & FMPI2C_IT_STOPI) != RESET))
  {
    /* Call FMPI2C Slave complete process */
    FMPI2C_ITSlaveCplt(hfmpi2c, ITFlags);
  }

  /* Process Unlocked */
  __HAL_UNLOCK(hfmpi2c);

  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with DMA.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_Master_ISR_DMA(struct __FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags, uint32_t ITSources)
{
  uint16_t devaddress = 0U;
  uint32_t xfermode = 0U;

  /* Process Locked */
  __HAL_LOCK(hfmpi2c);

  if (((ITFlags & FMPI2C_FLAG_AF) != RESET) && ((ITSources & FMPI2C_IT_NACKI) != RESET))
  {
    /* Clear NACK Flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_AF);

    /* Set corresponding Error Code */
    hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_AF;

    /* No need to generate STOP, it is automatically done */
    /* But enable STOP interrupt, to treat it */
    /* Error callback will be send during stop flag treatment */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_CPLT_IT);

    /* Flush TX register */
    FMPI2C_Flush_TXDR(hfmpi2c);
  }
  else if (((ITFlags & FMPI2C_FLAG_TCR) != RESET) && ((ITSources & FMPI2C_IT_TCI) != RESET))
  {
    /* Disable TC interrupt */
    __HAL_FMPI2C_DISABLE_IT(hfmpi2c, FMPI2C_IT_TCI);

    if (hfmpi2c->XferCount != 0U)
    {
      /* Recover Slave address */
      devaddress = (hfmpi2c->Instance->CR2 & FMPI2C_CR2_SADD);

      /* Prepare the new XferSize to transfer */
      if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
      {
        hfmpi2c->XferSize = MAX_NBYTE_SIZE;
        xfermode = FMPI2C_RELOAD_MODE;
      }
      else
      {
        hfmpi2c->XferSize = hfmpi2c->XferCount;
        xfermode = FMPI2C_AUTOEND_MODE;
      }

      /* Set the new XferSize in Nbytes register */
      FMPI2C_TransferConfig(hfmpi2c, devaddress, hfmpi2c->XferSize, xfermode, FMPI2C_NO_STARTSTOP);

      /* Update XferCount value */
      hfmpi2c->XferCount -= hfmpi2c->XferSize;

      /* Enable DMA Request */
      if (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_RX)
      {
        hfmpi2c->Instance->CR1 |= FMPI2C_CR1_RXDMAEN;
      }
      else
      {
        hfmpi2c->Instance->CR1 |= FMPI2C_CR1_TXDMAEN;
      }
    }
    else
    {
      /* Wrong size Status regarding TCR flag event */
      /* Call the corresponding callback to inform upper layer of End of Transfer */
      FMPI2C_ITError(hfmpi2c, HAL_FMPI2C_ERROR_SIZE);
    }
  }
  else if (((ITFlags & FMPI2C_FLAG_STOPF) != RESET) && ((ITSources & FMPI2C_IT_STOPI) != RESET))
  {
    /* Call FMPI2C Master complete process */
    FMPI2C_ITMasterCplt(hfmpi2c, ITFlags);
  }

  /* Process Unlocked */
  __HAL_UNLOCK(hfmpi2c);

  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with DMA.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_Slave_ISR_DMA(struct __FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags, uint32_t ITSources)
{
  /* Process locked */
  __HAL_LOCK(hfmpi2c);

  if (((ITFlags & FMPI2C_FLAG_AF) != RESET) && ((ITSources & FMPI2C_IT_NACKI) != RESET))
  {
    /* Check that FMPI2C transfer finished */
    /* if yes, normal use case, a NACK is sent by the MASTER when Transfer is finished */
    /* Mean XferCount == 0 */
    /* So clear Flag NACKF only */
    if (FMPI2C_GET_DMA_REMAIN_DATA(hfmpi2c) == 0U)
    {
      /* Clear NACK Flag */
      __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_AF);
    }
    else
    {
      /* if no, error use case, a Non-Acknowledge of last Data is generated by the MASTER*/
      /* Clear NACK Flag */
      __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_AF);

      /* Set ErrorCode corresponding to a Non-Acknowledge */
      hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_AF;
    }
  }
  else if (((ITFlags & FMPI2C_FLAG_ADDR) != RESET) && ((ITSources & FMPI2C_IT_ADDRI) != RESET))
  {
    /* Clear ADDR flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_ADDR);
  }
  else if (((ITFlags & FMPI2C_FLAG_STOPF) != RESET) && ((ITSources & FMPI2C_IT_STOPI) != RESET))
  {
    /* Call FMPI2C Slave complete process */
    FMPI2C_ITSlaveCplt(hfmpi2c, ITFlags);
  }

  /* Process Unlocked */
  __HAL_UNLOCK(hfmpi2c);

  return HAL_OK;
}

/**
  * @brief  Master sends target device address followed by internal memory address for write request.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_RequestMemoryWrite(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
  FMPI2C_TransferConfig(hfmpi2c, DevAddress, MemAddSize, FMPI2C_RELOAD_MODE, FMPI2C_GENERATE_START_WRITE);

  /* Wait until TXIS flag is set */
  if (FMPI2C_WaitOnTXISFlagUntilTimeout(hfmpi2c, Timeout, Tickstart) != HAL_OK)
  {
    if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
    {
      return HAL_ERROR;
    }
    else
    {
      return HAL_TIMEOUT;
    }
  }

  /* If Memory address size is 8Bit */
  if (MemAddSize == FMPI2C_MEMADD_SIZE_8BIT)
  {
    /* Send Memory Address */
    hfmpi2c->Instance->TXDR = FMPI2C_MEM_ADD_LSB(MemAddress);
  }
  /* If Memory address size is 16Bit */
  else
  {
    /* Send MSB of Memory Address */
    hfmpi2c->Instance->TXDR = FMPI2C_MEM_ADD_MSB(MemAddress);

    /* Wait until TXIS flag is set */
    if (FMPI2C_WaitOnTXISFlagUntilTimeout(hfmpi2c, Timeout, Tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        return HAL_ERROR;
      }
      else
      {
        return HAL_TIMEOUT;
      }
    }

    /* Send LSB of Memory Address */
    hfmpi2c->Instance->TXDR = FMPI2C_MEM_ADD_LSB(MemAddress);
  }

  /* Wait until TCR flag is set */
  if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_TCR, RESET, Timeout, Tickstart) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  return HAL_OK;
}

/**
  * @brief  Master sends target device address followed by internal memory address for read request.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_RequestMemoryRead(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
  FMPI2C_TransferConfig(hfmpi2c, DevAddress, MemAddSize, FMPI2C_SOFTEND_MODE, FMPI2C_GENERATE_START_WRITE);

  /* Wait until TXIS flag is set */
  if (FMPI2C_WaitOnTXISFlagUntilTimeout(hfmpi2c, Timeout, Tickstart) != HAL_OK)
  {
    if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
    {
      return HAL_ERROR;
    }
    else
    {
      return HAL_TIMEOUT;
    }
  }

  /* If Memory address size is 8Bit */
  if (MemAddSize == FMPI2C_MEMADD_SIZE_8BIT)
  {
    /* Send Memory Address */
    hfmpi2c->Instance->TXDR = FMPI2C_MEM_ADD_LSB(MemAddress);
  }
  /* If Memory address size is 16Bit */
  else
  {
    /* Send MSB of Memory Address */
    hfmpi2c->Instance->TXDR = FMPI2C_MEM_ADD_MSB(MemAddress);

    /* Wait until TXIS flag is set */
    if (FMPI2C_WaitOnTXISFlagUntilTimeout(hfmpi2c, Timeout, Tickstart) != HAL_OK)
    {
      if (hfmpi2c->ErrorCode == HAL_FMPI2C_ERROR_AF)
      {
        return HAL_ERROR;
      }
      else
      {
        return HAL_TIMEOUT;
      }
    }

    /* Send LSB of Memory Address */
    hfmpi2c->Instance->TXDR = FMPI2C_MEM_ADD_LSB(MemAddress);
  }

  /* Wait until TC flag is set */
  if (FMPI2C_WaitOnFlagUntilTimeout(hfmpi2c, FMPI2C_FLAG_TC, RESET, Timeout, Tickstart) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  return HAL_OK;
}

/**
  * @brief  FMPI2C Address complete process callback.
  * @param  hfmpi2c FMPI2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
static void FMPI2C_ITAddrCplt(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags)
{
  uint8_t transferdirection = 0;
  uint16_t slaveaddrcode = 0;
  uint16_t ownadd1code = 0;
  uint16_t ownadd2code = 0;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(ITFlags);

  /* In case of Listen state, need to inform upper layer of address match code event */
  if ((hfmpi2c->State & HAL_FMPI2C_STATE_LISTEN) == HAL_FMPI2C_STATE_LISTEN)
  {
    transferdirection = FMPI2C_GET_DIR(hfmpi2c);
    slaveaddrcode     = FMPI2C_GET_ADDR_MATCH(hfmpi2c);
    ownadd1code       = FMPI2C_GET_OWN_ADDRESS1(hfmpi2c);
    ownadd2code       = FMPI2C_GET_OWN_ADDRESS2(hfmpi2c);

    /* If 10bits addressing mode is selected */
    if (hfmpi2c->Init.AddressingMode == FMPI2C_ADDRESSINGMODE_10BIT)
    {
      if ((slaveaddrcode & SlaveAddr_MSK) == ((ownadd1code >> SlaveAddr_SHIFT) & SlaveAddr_MSK))
      {
        slaveaddrcode = ownadd1code;
        hfmpi2c->AddrEventCount++;
        if (hfmpi2c->AddrEventCount == 2U)
        {
          /* Reset Address Event counter */
          hfmpi2c->AddrEventCount = 0U;

          /* Clear ADDR flag */
          __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_ADDR);

          /* Process Unlocked */
          __HAL_UNLOCK(hfmpi2c);

          /* Call Slave Addr callback */
          HAL_FMPI2C_AddrCallback(hfmpi2c, transferdirection, slaveaddrcode);
        }
      }
      else
      {
        slaveaddrcode = ownadd2code;

        /* Disable ADDR Interrupts */
        FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_LISTEN_IT);

        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);

        /* Call Slave Addr callback */
        HAL_FMPI2C_AddrCallback(hfmpi2c, transferdirection, slaveaddrcode);
      }
    }
    /* else 7 bits addressing mode is selected */
    else
    {
      /* Disable ADDR Interrupts */
      FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_LISTEN_IT);

      /* Process Unlocked */
      __HAL_UNLOCK(hfmpi2c);

      /* Call Slave Addr callback */
      HAL_FMPI2C_AddrCallback(hfmpi2c, transferdirection, slaveaddrcode);
    }
  }
  /* Else clear address flag only */
  else
  {
    /* Clear ADDR flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_ADDR);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);
  }
}

/**
  * @brief  FMPI2C Master sequential complete process.
  * @param  hfmpi2c FMPI2C handle.
  * @retval None
  */
static void FMPI2C_ITMasterSequentialCplt(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Reset FMPI2C handle mode */
  hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

  /* No Generate Stop, to permit restart mode */
  /* The stop will be done at the end of transfer, when FMPI2C_AUTOEND_MODE enable */
  if (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_TX)
  {
    hfmpi2c->State         = HAL_FMPI2C_STATE_READY;
    hfmpi2c->PreviousState = FMPI2C_STATE_MASTER_BUSY_TX;
    hfmpi2c->XferISR       = NULL;

    /* Disable Interrupts */
    FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    HAL_FMPI2C_MasterTxCpltCallback(hfmpi2c);
  }
  /* hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_RX */
  else
  {
    hfmpi2c->State         = HAL_FMPI2C_STATE_READY;
    hfmpi2c->PreviousState = FMPI2C_STATE_MASTER_BUSY_RX;
    hfmpi2c->XferISR       = NULL;

    /* Disable Interrupts */
    FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_RX_IT);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    HAL_FMPI2C_MasterRxCpltCallback(hfmpi2c);
  }
}

/**
  * @brief  FMPI2C Slave sequential complete process.
  * @param  hfmpi2c FMPI2C handle.
  * @retval None
  */
static void FMPI2C_ITSlaveSequentialCplt(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* Reset FMPI2C handle mode */
  hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

  if (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_TX_LISTEN)
  {
    /* Remove HAL_FMPI2C_STATE_SLAVE_BUSY_TX, keep only HAL_FMPI2C_STATE_LISTEN */
    hfmpi2c->State         = HAL_FMPI2C_STATE_LISTEN;
    hfmpi2c->PreviousState = FMPI2C_STATE_SLAVE_BUSY_TX;

    /* Disable Interrupts */
    FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Call the Tx complete callback to inform upper layer of the end of transmit process */
    HAL_FMPI2C_SlaveTxCpltCallback(hfmpi2c);
  }

  else if (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_RX_LISTEN)
  {
    /* Remove HAL_FMPI2C_STATE_SLAVE_BUSY_RX, keep only HAL_FMPI2C_STATE_LISTEN */
    hfmpi2c->State         = HAL_FMPI2C_STATE_LISTEN;
    hfmpi2c->PreviousState = FMPI2C_STATE_SLAVE_BUSY_RX;

    /* Disable Interrupts */
    FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_RX_IT);

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Call the Rx complete callback to inform upper layer of the end of receive process */
    HAL_FMPI2C_SlaveRxCpltCallback(hfmpi2c);
  }
}

/**
  * @brief  FMPI2C Master complete process.
  * @param  hfmpi2c FMPI2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
static void FMPI2C_ITMasterCplt(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags)
{
  /* Clear STOP Flag */
  __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);

  /* Clear Configuration Register 2 */
  FMPI2C_RESET_CR2(hfmpi2c);

  /* Reset handle parameters */
  hfmpi2c->PreviousState = FMPI2C_STATE_NONE;
  hfmpi2c->XferISR       = NULL;
  hfmpi2c->XferOptions   = FMPI2C_NO_OPTION_FRAME;

  if ((ITFlags & FMPI2C_FLAG_AF) != RESET)
  {
    /* Clear NACK Flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_AF);

    /* Set acknowledge error code */
    hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_AF;
  }

  /* Flush TX register */
  FMPI2C_Flush_TXDR(hfmpi2c);

  /* Disable Interrupts */
  FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_TX_IT | FMPI2C_XFER_RX_IT);

  /* Call the corresponding callback to inform upper layer of End of Transfer */
  if ((hfmpi2c->ErrorCode != HAL_FMPI2C_ERROR_NONE) || (hfmpi2c->State == HAL_FMPI2C_STATE_ABORT))
  {
    /* Call the corresponding callback to inform upper layer of End of Transfer */
    FMPI2C_ITError(hfmpi2c, hfmpi2c->ErrorCode);
  }
  /* hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_TX */
  else if (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_TX)
  {
    hfmpi2c->State = HAL_FMPI2C_STATE_READY;

    if (hfmpi2c->Mode == HAL_FMPI2C_MODE_MEM)
    {
      hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

      /* Process Unlocked */
      __HAL_UNLOCK(hfmpi2c);

      /* Call the corresponding callback to inform upper layer of End of Transfer */
      HAL_FMPI2C_MemTxCpltCallback(hfmpi2c);
    }
    else
    {
      hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

      /* Process Unlocked */
      __HAL_UNLOCK(hfmpi2c);

      /* Call the corresponding callback to inform upper layer of End of Transfer */
      HAL_FMPI2C_MasterTxCpltCallback(hfmpi2c);
    }
  }
  /* hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_RX */
  else if (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_RX)
  {
    hfmpi2c->State = HAL_FMPI2C_STATE_READY;

    if (hfmpi2c->Mode == HAL_FMPI2C_MODE_MEM)
    {
      hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

      /* Process Unlocked */
      __HAL_UNLOCK(hfmpi2c);

      HAL_FMPI2C_MemRxCpltCallback(hfmpi2c);
    }
    else
    {
      hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

      /* Process Unlocked */
      __HAL_UNLOCK(hfmpi2c);

      HAL_FMPI2C_MasterRxCpltCallback(hfmpi2c);
    }
  }
}

/**
  * @brief  FMPI2C Slave complete process.
  * @param  hfmpi2c FMPI2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
static void FMPI2C_ITSlaveCplt(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags)
{
  /* Clear STOP Flag */
  __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);

  /* Clear ADDR flag */
  __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_ADDR);

  /* Disable all interrupts */
  FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_LISTEN_IT | FMPI2C_XFER_TX_IT | FMPI2C_XFER_RX_IT);

  /* Disable Address Acknowledge */
  hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;

  /* Clear Configuration Register 2 */
  FMPI2C_RESET_CR2(hfmpi2c);

  /* Flush TX register */
  FMPI2C_Flush_TXDR(hfmpi2c);

  /* If a DMA is ongoing, Update handle size context */
  if (((hfmpi2c->Instance->CR1 & FMPI2C_CR1_TXDMAEN) == FMPI2C_CR1_TXDMAEN) ||
      ((hfmpi2c->Instance->CR1 & FMPI2C_CR1_RXDMAEN) == FMPI2C_CR1_RXDMAEN))
  {
    hfmpi2c->XferCount = FMPI2C_GET_DMA_REMAIN_DATA(hfmpi2c);
  }

  /* All data are not transferred, so set error code accordingly */
  if (hfmpi2c->XferCount != 0U)
  {
    /* Set ErrorCode corresponding to a Non-Acknowledge */
    hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_AF;
  }

  /* Store Last receive data if any */
  if (((ITFlags & FMPI2C_FLAG_RXNE) != RESET))
  {
    /* Read data from RXDR */
    (*hfmpi2c->pBuffPtr++) = hfmpi2c->Instance->RXDR;

    if ((hfmpi2c->XferSize > 0U))
    {
      hfmpi2c->XferSize--;
      hfmpi2c->XferCount--;

      /* Set ErrorCode corresponding to a Non-Acknowledge */
      hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_AF;
    }
  }

  hfmpi2c->PreviousState = FMPI2C_STATE_NONE;
  hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;
  hfmpi2c->XferISR = NULL;

  if (hfmpi2c->ErrorCode != HAL_FMPI2C_ERROR_NONE)
  {
    /* Call the corresponding callback to inform upper layer of End of Transfer */
    FMPI2C_ITError(hfmpi2c, hfmpi2c->ErrorCode);

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
    if (hfmpi2c->State == HAL_FMPI2C_STATE_LISTEN)
    {
      /* Call FMPI2C Listen complete process */
      FMPI2C_ITListenCplt(hfmpi2c, ITFlags);
    }
  }
  else if (hfmpi2c->XferOptions != FMPI2C_NO_OPTION_FRAME)
  {
    hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
    hfmpi2c->State = HAL_FMPI2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
    HAL_FMPI2C_ListenCpltCallback(hfmpi2c);
  }
  /* Call the corresponding callback to inform upper layer of End of Transfer */
  else if (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_RX)
  {
    hfmpi2c->State = HAL_FMPI2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Call the Slave Rx Complete callback */
    HAL_FMPI2C_SlaveRxCpltCallback(hfmpi2c);
  }
  else
  {
    hfmpi2c->State = HAL_FMPI2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Call the Slave Tx Complete callback */
    HAL_FMPI2C_SlaveTxCpltCallback(hfmpi2c);
  }
}

/**
  * @brief  FMPI2C Listen complete process.
  * @param  hfmpi2c FMPI2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
static void FMPI2C_ITListenCplt(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ITFlags)
{
  /* Reset handle parameters */
  hfmpi2c->XferOptions = FMPI2C_NO_OPTION_FRAME;
  hfmpi2c->PreviousState = FMPI2C_STATE_NONE;
  hfmpi2c->State = HAL_FMPI2C_STATE_READY;
  hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;
  hfmpi2c->XferISR = NULL;

  /* Store Last receive data if any */
  if (((ITFlags & FMPI2C_FLAG_RXNE) != RESET))
  {
    /* Read data from RXDR */
    (*hfmpi2c->pBuffPtr++) = hfmpi2c->Instance->RXDR;

    if ((hfmpi2c->XferSize > 0U))
    {
      hfmpi2c->XferSize--;
      hfmpi2c->XferCount--;

      /* Set ErrorCode corresponding to a Non-Acknowledge */
      hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_AF;
    }
  }

  /* Disable all Interrupts*/
  FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_LISTEN_IT | FMPI2C_XFER_RX_IT | FMPI2C_XFER_TX_IT);

  /* Clear NACK Flag */
  __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_AF);

  /* Process Unlocked */
  __HAL_UNLOCK(hfmpi2c);

  /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
  HAL_FMPI2C_ListenCpltCallback(hfmpi2c);
}

/**
  * @brief  FMPI2C interrupts error process.
  * @param  hfmpi2c FMPI2C handle.
  * @param  ErrorCode Error code to handle.
  * @retval None
  */
static void FMPI2C_ITError(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t ErrorCode)
{
  /* Reset handle parameters */
  hfmpi2c->Mode          = HAL_FMPI2C_MODE_NONE;
  hfmpi2c->XferOptions   = FMPI2C_NO_OPTION_FRAME;
  hfmpi2c->XferCount     = 0U;

  /* Set new error code */
  hfmpi2c->ErrorCode |= ErrorCode;

  /* Disable Interrupts */
  if ((hfmpi2c->State == HAL_FMPI2C_STATE_LISTEN)         ||
      (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_TX_LISTEN) ||
      (hfmpi2c->State == HAL_FMPI2C_STATE_BUSY_RX_LISTEN))
  {
    /* Disable all interrupts, except interrupts related to LISTEN state */
    FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_RX_IT | FMPI2C_XFER_TX_IT);

    /* keep HAL_FMPI2C_STATE_LISTEN if set */
    hfmpi2c->State         = HAL_FMPI2C_STATE_LISTEN;
    hfmpi2c->PreviousState = FMPI2C_STATE_NONE;
    hfmpi2c->XferISR       = FMPI2C_Slave_ISR_IT;
  }
  else
  {
    /* Disable all interrupts */
    FMPI2C_Disable_IRQ(hfmpi2c, FMPI2C_XFER_LISTEN_IT | FMPI2C_XFER_RX_IT | FMPI2C_XFER_TX_IT);

    /* If state is an abort treatment on goind, don't change state */
    /* This change will be do later */
    if (hfmpi2c->State != HAL_FMPI2C_STATE_ABORT)
    {
      /* Set HAL_FMPI2C_STATE_READY */
      hfmpi2c->State         = HAL_FMPI2C_STATE_READY;
    }
    hfmpi2c->PreviousState = FMPI2C_STATE_NONE;
    hfmpi2c->XferISR       = NULL;
  }

  /* Abort DMA TX transfer if any */
  if ((hfmpi2c->Instance->CR1 & FMPI2C_CR1_TXDMAEN) == FMPI2C_CR1_TXDMAEN)
  {
    hfmpi2c->Instance->CR1 &= ~FMPI2C_CR1_TXDMAEN;

    /* Set the FMPI2C DMA Abort callback :
       will lead to call HAL_FMPI2C_ErrorCallback() at end of DMA abort procedure */
    hfmpi2c->hdmatx->XferAbortCallback = FMPI2C_DMAAbort;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Abort DMA TX */
    if (HAL_DMA_Abort_IT(hfmpi2c->hdmatx) != HAL_OK)
    {
      /* Call Directly XferAbortCallback function in case of error */
      hfmpi2c->hdmatx->XferAbortCallback(hfmpi2c->hdmatx);
    }
  }
  /* Abort DMA RX transfer if any */
  else if ((hfmpi2c->Instance->CR1 & FMPI2C_CR1_RXDMAEN) == FMPI2C_CR1_RXDMAEN)
  {
    hfmpi2c->Instance->CR1 &= ~FMPI2C_CR1_RXDMAEN;

    /* Set the FMPI2C DMA Abort callback :
       will lead to call HAL_FMPI2C_ErrorCallback() at end of DMA abort procedure */
    hfmpi2c->hdmarx->XferAbortCallback = FMPI2C_DMAAbort;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Abort DMA RX */
    if (HAL_DMA_Abort_IT(hfmpi2c->hdmarx) != HAL_OK)
    {
      /* Call Directly hfmpi2c->hdmarx->XferAbortCallback function in case of error */
      hfmpi2c->hdmarx->XferAbortCallback(hfmpi2c->hdmarx);
    }
  }
  else if (hfmpi2c->State == HAL_FMPI2C_STATE_ABORT)
  {
    hfmpi2c->State = HAL_FMPI2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    HAL_FMPI2C_AbortCpltCallback(hfmpi2c);
  }
  else
  {
    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    HAL_FMPI2C_ErrorCallback(hfmpi2c);
  }
}

/**
  * @brief  FMPI2C Tx data register flush process.
  * @param  hfmpi2c FMPI2C handle.
  * @retval None
  */
static void FMPI2C_Flush_TXDR(FMPI2C_HandleTypeDef *hfmpi2c)
{
  /* If a pending TXIS flag is set */
  /* Write a dummy data in TXDR to clear it */
  if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_TXIS) != RESET)
  {
    hfmpi2c->Instance->TXDR = 0x00U;
  }

  /* Flush TX register if not empty */
  if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_TXE) == RESET)
  {
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_TXE);
  }
}

/**
  * @brief  DMA FMPI2C master transmit process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void FMPI2C_DMAMasterTransmitCplt(DMA_HandleTypeDef *hdma)
{
  FMPI2C_HandleTypeDef *hfmpi2c = (FMPI2C_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Disable DMA Request */
  hfmpi2c->Instance->CR1 &= ~FMPI2C_CR1_TXDMAEN;

  /* If last transfer, enable STOP interrupt */
  if (hfmpi2c->XferCount == 0U)
  {
    /* Enable STOP interrupt */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_CPLT_IT);
  }
  /* else prepare a new DMA transfer and enable TCReload interrupt */
  else
  {
    /* Update Buffer pointer */
    hfmpi2c->pBuffPtr += hfmpi2c->XferSize;

    /* Set the XferSize to transfer */
    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
    }

    /* Enable the DMA stream */
    HAL_DMA_Start_IT(hfmpi2c->hdmatx, (uint32_t)hfmpi2c->pBuffPtr, (uint32_t)&hfmpi2c->Instance->TXDR, hfmpi2c->XferSize);

    /* Enable TC interrupts */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_RELOAD_IT);
  }
}

/**
  * @brief  DMA FMPI2C slave transmit process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void FMPI2C_DMASlaveTransmitCplt(DMA_HandleTypeDef *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdma);

  /* No specific action, Master fully manage the generation of STOP condition */
  /* Mean that this generation can arrive at any time, at the end or during DMA process */
  /* So STOP condition should be manage through Interrupt treatment */
}

/**
  * @brief DMA FMPI2C master receive process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void FMPI2C_DMAMasterReceiveCplt(DMA_HandleTypeDef *hdma)
{
  FMPI2C_HandleTypeDef *hfmpi2c = (FMPI2C_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Disable DMA Request */
  hfmpi2c->Instance->CR1 &= ~FMPI2C_CR1_RXDMAEN;

  /* If last transfer, enable STOP interrupt */
  if (hfmpi2c->XferCount == 0U)
  {
    /* Enable STOP interrupt */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_CPLT_IT);
  }
  /* else prepare a new DMA transfer and enable TCReload interrupt */
  else
  {
    /* Update Buffer pointer */
    hfmpi2c->pBuffPtr += hfmpi2c->XferSize;

    /* Set the XferSize to transfer */
    if (hfmpi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hfmpi2c->XferSize = MAX_NBYTE_SIZE;
    }
    else
    {
      hfmpi2c->XferSize = hfmpi2c->XferCount;
    }

    /* Enable the DMA stream */
    HAL_DMA_Start_IT(hfmpi2c->hdmarx, (uint32_t)&hfmpi2c->Instance->RXDR, (uint32_t)hfmpi2c->pBuffPtr, hfmpi2c->XferSize);

    /* Enable TC interrupts */
    FMPI2C_Enable_IRQ(hfmpi2c, FMPI2C_XFER_RELOAD_IT);
  }
}

/**
  * @brief  DMA FMPI2C slave receive process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void FMPI2C_DMASlaveReceiveCplt(DMA_HandleTypeDef *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdma);

  /* No specific action, Master fully manage the generation of STOP condition */
  /* Mean that this generation can arrive at any time, at the end or during DMA process */
  /* So STOP condition should be manage through Interrupt treatment */
}

/**
  * @brief  DMA FMPI2C communication error callback.
  * @param hdma DMA handle
  * @retval None
  */
static void FMPI2C_DMAError(DMA_HandleTypeDef *hdma)
{
  FMPI2C_HandleTypeDef *hfmpi2c = (FMPI2C_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Disable Acknowledge */
  hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;

  /* Call the corresponding callback to inform upper layer of End of Transfer */
  FMPI2C_ITError(hfmpi2c, HAL_FMPI2C_ERROR_DMA);
}

/**
  * @brief DMA FMPI2C communication abort callback
  *        (To be called at end of DMA Abort procedure).
  * @param hdma DMA handle.
  * @retval None
  */
static void FMPI2C_DMAAbort(DMA_HandleTypeDef *hdma)
{
  FMPI2C_HandleTypeDef *hfmpi2c = (FMPI2C_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Disable Acknowledge */
  hfmpi2c->Instance->CR2 |= FMPI2C_CR2_NACK;

  /* Reset AbortCpltCallback */
  hfmpi2c->hdmatx->XferAbortCallback = NULL;
  hfmpi2c->hdmarx->XferAbortCallback = NULL;

  /* Check if come from abort from user */
  if (hfmpi2c->State == HAL_FMPI2C_STATE_ABORT)
  {
    hfmpi2c->State = HAL_FMPI2C_STATE_READY;

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    HAL_FMPI2C_AbortCpltCallback(hfmpi2c);
  }
  else
  {
    /* Call the corresponding callback to inform upper layer of End of Transfer */
    HAL_FMPI2C_ErrorCallback(hfmpi2c);
  }
}

/**
  * @brief  This function handles FMPI2C Communication Timeout.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  Flag Specifies the FMPI2C flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_WaitOnFlagUntilTimeout(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{
  while (__HAL_FMPI2C_GET_FLAG(hfmpi2c, Flag) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
      {
        hfmpi2c->State = HAL_FMPI2C_STATE_READY;
        hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);
        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  This function handles FMPI2C Communication Timeout for specific usage of TXIS flag.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_WaitOnTXISFlagUntilTimeout(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t Timeout, uint32_t Tickstart)
{
  while (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_TXIS) == RESET)
  {
    /* Check if a NACK is detected */
    if (FMPI2C_IsAcknowledgeFailed(hfmpi2c, Timeout, Tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
      {
        hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_TIMEOUT;
        hfmpi2c->State = HAL_FMPI2C_STATE_READY;
        hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);

        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  This function handles FMPI2C Communication Timeout for specific usage of STOP flag.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_WaitOnSTOPFlagUntilTimeout(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t Timeout, uint32_t Tickstart)
{
  while (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF) == RESET)
  {
    /* Check if a NACK is detected */
    if (FMPI2C_IsAcknowledgeFailed(hfmpi2c, Timeout, Tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Check for the Timeout */
    if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
    {
      hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_TIMEOUT;
      hfmpi2c->State = HAL_FMPI2C_STATE_READY;
      hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

      /* Process Unlocked */
      __HAL_UNLOCK(hfmpi2c);

      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

/**
  * @brief  This function handles FMPI2C Communication Timeout for specific usage of RXNE flag.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_WaitOnRXNEFlagUntilTimeout(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t Timeout, uint32_t Tickstart)
{
  while (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_RXNE) == RESET)
  {
    /* Check if a NACK is detected */
    if (FMPI2C_IsAcknowledgeFailed(hfmpi2c, Timeout, Tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Check if a STOPF is detected */
    if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF) == SET)
    {
      /* Check if an RXNE is pending */
      /* Store Last receive data if any */
      if ((__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_RXNE) == SET) && (hfmpi2c->XferSize > 0U))
      {
        /* Return HAL_OK */
        /* The Reading of data from RXDR will be done in caller function */
        return HAL_OK;
      }
      else
      {
        /* Clear STOP Flag */
        __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);

        /* Clear Configuration Register 2 */
        FMPI2C_RESET_CR2(hfmpi2c);

        hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_NONE;
        hfmpi2c->State = HAL_FMPI2C_STATE_READY;
        hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hfmpi2c);

        return HAL_ERROR;
      }
    }

    /* Check for the Timeout */
    if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
    {
      hfmpi2c->ErrorCode |= HAL_FMPI2C_ERROR_TIMEOUT;
      hfmpi2c->State = HAL_FMPI2C_STATE_READY;

      /* Process Unlocked */
      __HAL_UNLOCK(hfmpi2c);

      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

/**
  * @brief  This function handles Acknowledge failed detection during an FMPI2C Communication.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_IsAcknowledgeFailed(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t Timeout, uint32_t Tickstart)
{
  if (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_AF) == SET)
  {
    /* Wait until STOP Flag is reset */
    /* AutoEnd should be initiate after AF */
    while (__HAL_FMPI2C_GET_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF) == RESET)
    {
      /* Check for the Timeout */
      if (Timeout != HAL_MAX_DELAY)
      {
        if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
        {
          hfmpi2c->State = HAL_FMPI2C_STATE_READY;
          hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

          /* Process Unlocked */
          __HAL_UNLOCK(hfmpi2c);
          return HAL_TIMEOUT;
        }
      }
    }

    /* Clear NACKF Flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_AF);

    /* Clear STOP Flag */
    __HAL_FMPI2C_CLEAR_FLAG(hfmpi2c, FMPI2C_FLAG_STOPF);

    /* Flush TX register */
    FMPI2C_Flush_TXDR(hfmpi2c);

    /* Clear Configuration Register 2 */
    FMPI2C_RESET_CR2(hfmpi2c);

    hfmpi2c->ErrorCode = HAL_FMPI2C_ERROR_AF;
    hfmpi2c->State = HAL_FMPI2C_STATE_READY;
    hfmpi2c->Mode = HAL_FMPI2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    return HAL_ERROR;
  }
  return HAL_OK;
}

/**
  * @brief  Handles FMPI2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @param  hfmpi2c FMPI2C handle.
  * @param  DevAddress Specifies the slave address to be programmed.
  * @param  Size Specifies the number of bytes to be programmed.
  *   This parameter must be a value between 0 and 255.
  * @param  Mode New state of the FMPI2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg @ref FMPI2C_RELOAD_MODE Enable Reload mode .
  *     @arg @ref FMPI2C_AUTOEND_MODE Enable Automatic end mode.
  *     @arg @ref FMPI2C_SOFTEND_MODE Enable Software end mode.
  * @param  Request New state of the FMPI2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg @ref FMPI2C_NO_STARTSTOP Don't Generate stop and start condition.
  *     @arg @ref FMPI2C_GENERATE_STOP Generate stop condition (Size should be set to 0).
  *     @arg @ref FMPI2C_GENERATE_START_READ Generate Restart for read request.
  *     @arg @ref FMPI2C_GENERATE_START_WRITE Generate Restart for write request.
  * @retval None
  */
static void FMPI2C_TransferConfig(FMPI2C_HandleTypeDef *hfmpi2c,  uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_INSTANCE(hfmpi2c->Instance));
  assert_param(IS_TRANSFER_MODE(Mode));
  assert_param(IS_TRANSFER_REQUEST(Request));

  /* update CR2 register */
  MODIFY_REG(hfmpi2c->Instance->CR2, ((FMPI2C_CR2_SADD | FMPI2C_CR2_NBYTES | FMPI2C_CR2_RELOAD | FMPI2C_CR2_AUTOEND | (FMPI2C_CR2_RD_WRN & (uint32_t)(Request >> (31U - FMPI2C_CR2_RD_WRN_Pos))) | FMPI2C_CR2_START | FMPI2C_CR2_STOP)), \
             (uint32_t)(((uint32_t)DevAddress & FMPI2C_CR2_SADD) | (((uint32_t)Size << FMPI2C_CR2_NBYTES_Pos) & FMPI2C_CR2_NBYTES) | (uint32_t)Mode | (uint32_t)Request));
}

/**
  * @brief  Manage the enabling of Interrupts.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  InterruptRequest Value of @ref FMPI2C_Interrupt_configuration_definition.
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_Enable_IRQ(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t InterruptRequest)
{
  uint32_t tmpisr = 0U;

  if ((hfmpi2c->XferISR == FMPI2C_Master_ISR_DMA) || \
      (hfmpi2c->XferISR == FMPI2C_Slave_ISR_DMA))
  {
    if ((InterruptRequest & FMPI2C_XFER_LISTEN_IT) == FMPI2C_XFER_LISTEN_IT)
    {
      /* Enable ERR, STOP, NACK and ADDR interrupts */
      tmpisr |= FMPI2C_IT_ADDRI | FMPI2C_IT_STOPI | FMPI2C_IT_NACKI | FMPI2C_IT_ERRI;
    }

    if ((InterruptRequest & FMPI2C_XFER_ERROR_IT) == FMPI2C_XFER_ERROR_IT)
    {
      /* Enable ERR and NACK interrupts */
      tmpisr |= FMPI2C_IT_ERRI | FMPI2C_IT_NACKI;
    }

    if ((InterruptRequest & FMPI2C_XFER_CPLT_IT) == FMPI2C_XFER_CPLT_IT)
    {
      /* Enable STOP interrupts */
      tmpisr |= FMPI2C_IT_STOPI;
    }

    if ((InterruptRequest & FMPI2C_XFER_RELOAD_IT) == FMPI2C_XFER_RELOAD_IT)
    {
      /* Enable TC interrupts */
      tmpisr |= FMPI2C_IT_TCI;
    }
  }
  else
  {
    if ((InterruptRequest & FMPI2C_XFER_LISTEN_IT) == FMPI2C_XFER_LISTEN_IT)
    {
      /* Enable ERR, STOP, NACK, and ADDR interrupts */
      tmpisr |= FMPI2C_IT_ADDRI | FMPI2C_IT_STOPI | FMPI2C_IT_NACKI | FMPI2C_IT_ERRI;
    }

    if ((InterruptRequest & FMPI2C_XFER_TX_IT) == FMPI2C_XFER_TX_IT)
    {
      /* Enable ERR, TC, STOP, NACK and RXI interrupts */
      tmpisr |= FMPI2C_IT_ERRI | FMPI2C_IT_TCI | FMPI2C_IT_STOPI | FMPI2C_IT_NACKI | FMPI2C_IT_TXI;
    }

    if ((InterruptRequest & FMPI2C_XFER_RX_IT) == FMPI2C_XFER_RX_IT)
    {
      /* Enable ERR, TC, STOP, NACK and TXI interrupts */
      tmpisr |= FMPI2C_IT_ERRI | FMPI2C_IT_TCI | FMPI2C_IT_STOPI | FMPI2C_IT_NACKI | FMPI2C_IT_RXI;
    }

    if ((InterruptRequest & FMPI2C_XFER_CPLT_IT) == FMPI2C_XFER_CPLT_IT)
    {
      /* Enable STOP interrupts */
      tmpisr |= FMPI2C_IT_STOPI;
    }
  }

  /* Enable interrupts only at the end */
  /* to avoid the risk of FMPI2C interrupt handle execution before */
  /* all interrupts requested done */
  __HAL_FMPI2C_ENABLE_IT(hfmpi2c, tmpisr);

  return HAL_OK;
}

/**
  * @brief  Manage the disabling of Interrupts.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2C.
  * @param  InterruptRequest Value of @ref FMPI2C_Interrupt_configuration_definition.
  * @retval HAL status
  */
static HAL_StatusTypeDef FMPI2C_Disable_IRQ(FMPI2C_HandleTypeDef *hfmpi2c, uint16_t InterruptRequest)
{
  uint32_t tmpisr = 0U;

  if ((InterruptRequest & FMPI2C_XFER_TX_IT) == FMPI2C_XFER_TX_IT)
  {
    /* Disable TC and TXI interrupts */
    tmpisr |= FMPI2C_IT_TCI | FMPI2C_IT_TXI;

    if ((hfmpi2c->State & HAL_FMPI2C_STATE_LISTEN) != HAL_FMPI2C_STATE_LISTEN)
    {
      /* Disable NACK and STOP interrupts */
      tmpisr |= FMPI2C_IT_STOPI | FMPI2C_IT_NACKI | FMPI2C_IT_ERRI;
    }
  }

  if ((InterruptRequest & FMPI2C_XFER_RX_IT) == FMPI2C_XFER_RX_IT)
  {
    /* Disable TC and RXI interrupts */
    tmpisr |= FMPI2C_IT_TCI | FMPI2C_IT_RXI;

    if ((hfmpi2c->State & HAL_FMPI2C_STATE_LISTEN) != HAL_FMPI2C_STATE_LISTEN)
    {
      /* Disable NACK and STOP interrupts */
      tmpisr |= FMPI2C_IT_STOPI | FMPI2C_IT_NACKI | FMPI2C_IT_ERRI;
    }
  }

  if ((InterruptRequest & FMPI2C_XFER_LISTEN_IT) == FMPI2C_XFER_LISTEN_IT)
  {
    /* Disable ADDR, NACK and STOP interrupts */
    tmpisr |= FMPI2C_IT_ADDRI | FMPI2C_IT_STOPI | FMPI2C_IT_NACKI | FMPI2C_IT_ERRI;
  }

  if ((InterruptRequest & FMPI2C_XFER_ERROR_IT) == FMPI2C_XFER_ERROR_IT)
  {
    /* Enable ERR and NACK interrupts */
    tmpisr |= FMPI2C_IT_ERRI | FMPI2C_IT_NACKI;
  }

  if ((InterruptRequest & FMPI2C_XFER_CPLT_IT) == FMPI2C_XFER_CPLT_IT)
  {
    /* Enable STOP interrupts */
    tmpisr |= FMPI2C_IT_STOPI;
  }

  if ((InterruptRequest & FMPI2C_XFER_RELOAD_IT) == FMPI2C_XFER_RELOAD_IT)
  {
    /* Enable TC interrupts */
    tmpisr |= FMPI2C_IT_TCI;
  }

  /* Disable interrupts only at the end */
  /* to avoid a breaking situation like at "t" time */
  /* all disable interrupts request are not done */
  __HAL_FMPI2C_DISABLE_IT(hfmpi2c, tmpisr);

  return HAL_OK;
}

/**
  * @}
  */
#endif /* STM32F410xx || STM32F446xx || STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F412Cx || STM32F413xx || STM32F423xx */
#endif /* HAL_FMPI2C_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
