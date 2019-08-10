/**
  ******************************************************************************
  * @file    stm32f1xx_ll_i2c.c
  * @author  MCD Application Team
  * @brief   I2C LL module driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#if defined(USE_FULL_LL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#ifdef  USE_FULL_ASSERT
#include "stm32_assert.h"
#else
#define assert_param(expr) ((void)0U)
#endif

/** @addtogroup STM32F1xx_LL_Driver
  * @{
  */

#if defined (I2C1) || defined (I2C2)

/** @defgroup I2C_LL I2C
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup I2C_LL_Private_Macros
  * @{
  */

#define IS_LL_I2C_PERIPHERAL_MODE(__VALUE__)    (((__VALUE__) == LL_I2C_MODE_I2C)          || \
                                                 ((__VALUE__) == LL_I2C_MODE_SMBUS_HOST)   || \
                                                 ((__VALUE__) == LL_I2C_MODE_SMBUS_DEVICE) || \
                                                 ((__VALUE__) == LL_I2C_MODE_SMBUS_DEVICE_ARP))

#define IS_LL_I2C_CLOCK_SPEED(__VALUE__)           (((__VALUE__) > 0U) && ((__VALUE__) <= LL_I2C_MAX_SPEED_FAST))

#define IS_LL_I2C_DUTY_CYCLE(__VALUE__)            (((__VALUE__) == LL_I2C_DUTYCYCLE_2) || \
                                                 ((__VALUE__) == LL_I2C_DUTYCYCLE_16_9))

#define IS_LL_I2C_OWN_ADDRESS1(__VALUE__)       ((__VALUE__) <= 0x000003FFU)

#define IS_LL_I2C_TYPE_ACKNOWLEDGE(__VALUE__)   (((__VALUE__) == LL_I2C_ACK) || \
                                                 ((__VALUE__) == LL_I2C_NACK))

#define IS_LL_I2C_OWN_ADDRSIZE(__VALUE__)       (((__VALUE__) == LL_I2C_OWNADDRESS1_7BIT) || \
                                                 ((__VALUE__) == LL_I2C_OWNADDRESS1_10BIT))
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2C_LL_Exported_Functions
  * @{
  */

/** @addtogroup I2C_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the I2C registers to their default reset values.
  * @param  I2Cx I2C Instance.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS  I2C registers are de-initialized
  *          - ERROR  I2C registers are not de-initialized
  */
uint32_t LL_I2C_DeInit(I2C_TypeDef *I2Cx)
{
  ErrorStatus status = SUCCESS;

  /* Check the I2C Instance I2Cx */
  assert_param(IS_I2C_ALL_INSTANCE(I2Cx));

  if (I2Cx == I2C1)
  {
    /* Force reset of I2C clock */
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);

    /* Release reset of I2C clock */
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);
  }
#if defined(I2C2)
  else if (I2Cx == I2C2)
  {
    /* Force reset of I2C clock */
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C2);

    /* Release reset of I2C clock */
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C2);

  }
#endif /* I2C2 */
  else
  {
    status = ERROR;
  }

  return status;
}

/**
  * @brief  Initialize the I2C registers according to the specified parameters in I2C_InitStruct.
  * @param  I2Cx I2C Instance.
  * @param  I2C_InitStruct pointer to a @ref LL_I2C_InitTypeDef structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS  I2C registers are initialized
  *          - ERROR  Not applicable
  */
uint32_t LL_I2C_Init(I2C_TypeDef *I2Cx, LL_I2C_InitTypeDef *I2C_InitStruct)
{
  LL_RCC_ClocksTypeDef rcc_clocks;

  /* Check the I2C Instance I2Cx */
  assert_param(IS_I2C_ALL_INSTANCE(I2Cx));

  /* Check the I2C parameters from I2C_InitStruct */
  assert_param(IS_LL_I2C_PERIPHERAL_MODE(I2C_InitStruct->PeripheralMode));
  assert_param(IS_LL_I2C_CLOCK_SPEED(I2C_InitStruct->ClockSpeed));
  assert_param(IS_LL_I2C_DUTY_CYCLE(I2C_InitStruct->DutyCycle));
  assert_param(IS_LL_I2C_OWN_ADDRESS1(I2C_InitStruct->OwnAddress1));
  assert_param(IS_LL_I2C_TYPE_ACKNOWLEDGE(I2C_InitStruct->TypeAcknowledge));
  assert_param(IS_LL_I2C_OWN_ADDRSIZE(I2C_InitStruct->OwnAddrSize));

  /* Disable the selected I2Cx Peripheral */
  LL_I2C_Disable(I2Cx);

  /* Retrieve Clock frequencies */
  LL_RCC_GetSystemClocksFreq(&rcc_clocks);

  /*---------------------------- I2Cx SCL Clock Speed Configuration ------------
   * Configure the SCL speed :
   * - ClockSpeed: I2C_CR2_FREQ[5:0], I2C_TRISE_TRISE[5:0], I2C_CCR_FS,
   *           and I2C_CCR_CCR[11:0] bits
   * - DutyCycle: I2C_CCR_DUTY[7:0] bits
   */
  LL_I2C_ConfigSpeed(I2Cx, rcc_clocks.PCLK1_Frequency, I2C_InitStruct->ClockSpeed, I2C_InitStruct->DutyCycle);

  /*---------------------------- I2Cx OAR1 Configuration -----------------------
   * Disable, Configure and Enable I2Cx device own address 1 with parameters :
   * - OwnAddress1:  I2C_OAR1_ADD[9:8], I2C_OAR1_ADD[7:1] and I2C_OAR1_ADD0 bits
   * - OwnAddrSize:  I2C_OAR1_ADDMODE bit
   */
  LL_I2C_SetOwnAddress1(I2Cx, I2C_InitStruct->OwnAddress1, I2C_InitStruct->OwnAddrSize);

  /*---------------------------- I2Cx MODE Configuration -----------------------
  * Configure I2Cx peripheral mode with parameter :
   * - PeripheralMode: I2C_CR1_SMBUS, I2C_CR1_SMBTYPE and I2C_CR1_ENARP bits
   */
  LL_I2C_SetMode(I2Cx, I2C_InitStruct->PeripheralMode);

  /* Enable the selected I2Cx Peripheral */
  LL_I2C_Enable(I2Cx);

  /*---------------------------- I2Cx CR2 Configuration ------------------------
   * Configure the ACKnowledge or Non ACKnowledge condition
   * after the address receive match code or next received byte with parameter :
   * - TypeAcknowledge: I2C_CR2_NACK bit
   */
  LL_I2C_AcknowledgeNextData(I2Cx, I2C_InitStruct->TypeAcknowledge);

  return SUCCESS;
}

/**
  * @brief  Set each @ref LL_I2C_InitTypeDef field to default value.
  * @param  I2C_InitStruct Pointer to a @ref LL_I2C_InitTypeDef structure.
  * @retval None
  */
void LL_I2C_StructInit(LL_I2C_InitTypeDef *I2C_InitStruct)
{
  /* Set I2C_InitStruct fields to default values */
  I2C_InitStruct->PeripheralMode  = LL_I2C_MODE_I2C;
  I2C_InitStruct->ClockSpeed      = 5000U;
  I2C_InitStruct->DutyCycle       = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct->OwnAddress1     = 0U;
  I2C_InitStruct->TypeAcknowledge = LL_I2C_NACK;
  I2C_InitStruct->OwnAddrSize     = LL_I2C_OWNADDRESS1_7BIT;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* I2C1 || I2C2 */

/**
  * @}
  */

#endif /* USE_FULL_LL_DRIVER */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
