/**
  ******************************************************************************
  * @file    stm32f1xx_ll_gpio.c
  * @author  MCD Application Team
  * @brief   GPIO LL module driver.
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
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"
#ifdef  USE_FULL_ASSERT
#include "stm32_assert.h"
#else
#define assert_param(expr) ((void)0U)
#endif

/** @addtogroup STM32F1xx_LL_Driver
  * @{
  */

#if defined (GPIOA) || defined (GPIOB) || defined (GPIOC) || defined (GPIOD) || defined (GPIOE) || defined (GPIOF) || defined (GPIOG)

/** @addtogroup GPIO_LL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup GPIO_LL_Private_Macros
  * @{
  */

#define IS_LL_GPIO_PIN(__VALUE__)          ((((__VALUE__) & LL_GPIO_PIN_ALL)!= 0U) &&\
                                            (((__VALUE__) & (~LL_GPIO_PIN_ALL))== 0U))

#define IS_LL_GPIO_MODE(__VALUE__)         (((__VALUE__) == LL_GPIO_MODE_ANALOG)       ||\
                                            ((__VALUE__) == LL_GPIO_MODE_FLOATING)     ||\
                                            ((__VALUE__) == LL_GPIO_MODE_INPUT)        ||\
                                            ((__VALUE__) == LL_GPIO_MODE_OUTPUT)       ||\
                                            ((__VALUE__) == LL_GPIO_MODE_ALTERNATE))

#define IS_LL_GPIO_SPEED(__VALUE__)        (((__VALUE__) == LL_GPIO_SPEED_FREQ_LOW)       ||\
                                            ((__VALUE__) == LL_GPIO_SPEED_FREQ_MEDIUM)    ||\
                                            ((__VALUE__) == LL_GPIO_SPEED_FREQ_HIGH))

#define IS_LL_GPIO_OUTPUT_TYPE(__VALUE__)  (((__VALUE__) == LL_GPIO_OUTPUT_PUSHPULL)  ||\
                                            ((__VALUE__) == LL_GPIO_OUTPUT_OPENDRAIN))

#define IS_LL_GPIO_PULL(__VALUE__)         (((__VALUE__) == LL_GPIO_PULL_DOWN)   ||\
                                            ((__VALUE__) == LL_GPIO_PULL_UP))

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup GPIO_LL_Exported_Functions
  * @{
  */

/** @addtogroup GPIO_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize GPIO registers (Registers restored to their default values).
  * @param  GPIOx GPIO Port
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: GPIO registers are de-initialized
  *          - ERROR:   Wrong GPIO Port
  */
ErrorStatus LL_GPIO_DeInit(GPIO_TypeDef *GPIOx)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));

  /* Force and Release reset on clock of GPIOx Port */
  if (GPIOx == GPIOA)
  {
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_GPIOA);
  }
  else if (GPIOx == GPIOB)
  {
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_GPIOB);
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_GPIOB);
  }
  else if (GPIOx == GPIOC)
  {
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_GPIOC);
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_GPIOC);
  }
  else if (GPIOx == GPIOD)
  {
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_GPIOD);
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_GPIOD);
  }
#if defined(GPIOE)
  else if (GPIOx == GPIOE)
  {
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_GPIOE);
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_GPIOE);
  }
#endif
#if defined(GPIOF)
  else if (GPIOx == GPIOF)
  {
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_GPIOF);
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_GPIOF);
  }
#endif
#if defined(GPIOG)
  else if (GPIOx == GPIOG)
  {
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_GPIOG);
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_GPIOG);
  }
#endif
  else
  {
    status = ERROR;
  }

  return (status);
}

/**
  * @brief  Initialize GPIO registers according to the specified parameters in GPIO_InitStruct.
  * @param  GPIOx GPIO Port
  * @param  GPIO_InitStruct: pointer to a @ref LL_GPIO_InitTypeDef structure
  *         that contains the configuration information for the specified GPIO peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: GPIO registers are initialized according to GPIO_InitStruct content
  *          - ERROR:   Not applicable
  */
ErrorStatus LL_GPIO_Init(GPIO_TypeDef *GPIOx, LL_GPIO_InitTypeDef *GPIO_InitStruct)
{
  uint32_t pinmask;
  uint32_t pinpos;
  uint32_t currentpin;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
  assert_param(IS_LL_GPIO_PIN(GPIO_InitStruct->Pin));

  /* ------------------------- Configure the port pins ---------------- */
  /* Initialize  pinpos on first pin set */

  pinmask = ((GPIO_InitStruct->Pin) << GPIO_PIN_MASK_POS) >> GPIO_PIN_NB;
  pinpos = POSITION_VAL(pinmask);

  /* Configure the port pins */
  while ((pinmask  >> pinpos) != 0U)
  {
    /* skip if bit is not set */
    if ((pinmask & (1U << pinpos)) != 0U)
    {
      /* Get current io position */
      if (pinpos < GPIO_PIN_MASK_POS)
      {
        currentpin = (0x00000101U << pinpos);
      }
      else
      {
        currentpin = ((0x00010001U << (pinpos - GPIO_PIN_MASK_POS)) | 0x04000000U);
      }

      /* Check Pin Mode and Pin Pull parameters */
      assert_param(IS_LL_GPIO_MODE(GPIO_InitStruct->Mode));
      assert_param(IS_LL_GPIO_PULL(GPIO_InitStruct->Pull));

      /* Pin Mode configuration */
      LL_GPIO_SetPinMode(GPIOx, currentpin, GPIO_InitStruct->Mode);

      /* Pull-up Pull-down resistor configuration*/
      LL_GPIO_SetPinPull(GPIOx, currentpin, GPIO_InitStruct->Pull);

      if ((GPIO_InitStruct->Mode == LL_GPIO_MODE_OUTPUT) || (GPIO_InitStruct->Mode == LL_GPIO_MODE_ALTERNATE))
      {
        /* Check speed and Output mode parameters */
        assert_param(IS_LL_GPIO_SPEED(GPIO_InitStruct->Speed));
        assert_param(IS_LL_GPIO_OUTPUT_TYPE(GPIO_InitStruct->OutputType));

        /* Speed mode configuration */
        LL_GPIO_SetPinSpeed(GPIOx, currentpin, GPIO_InitStruct->Speed);

        /* Output mode configuration*/
        LL_GPIO_SetPinOutputType(GPIOx, currentpin, GPIO_InitStruct->OutputType);
      }
    }
    pinpos++;
  }
  return (SUCCESS);
}

/**
  * @brief Set each @ref LL_GPIO_InitTypeDef field to default value.
  * @param GPIO_InitStruct: pointer to a @ref LL_GPIO_InitTypeDef structure
  *                          whose fields will be set to default values.
  * @retval None
  */

void LL_GPIO_StructInit(LL_GPIO_InitTypeDef *GPIO_InitStruct)
{
  /* Reset GPIO init structure parameters values */
  GPIO_InitStruct->Pin        = LL_GPIO_PIN_ALL;
  GPIO_InitStruct->Mode       = LL_GPIO_MODE_FLOATING;
  GPIO_InitStruct->Speed      = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct->OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct->Pull       = LL_GPIO_PULL_DOWN;
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

#endif /* defined (GPIOA) || defined (GPIOB) || defined (GPIOC) || defined (GPIOD) || defined (GPIOE) || defined (GPIOF) || defined (GPIOG) */

/**
  * @}
  */

#endif /* USE_FULL_LL_DRIVER */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
