/*----------------------------------------------------------------------------
 * Copyright (c) <2016-2018>, <Huawei Technologies Co., Ltd>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific prior written
 * permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
 * Notice of Export Control Law
 * ===============================================
 * Huawei LiteOS may be subject to applicable export control laws and regulations, which might
 * include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
 * applicable export control laws and regulations.
 *---------------------------------------------------------------------------*/

#include "hal_spi_flash.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_spi.h"

#ifdef HAL_SPI_MODULE_ENABLED

#define SPI_FLASH_PAGESIZE                         256
#define SPI_FLASH_SECTOR                           4096
#define SPI_FLASH_ID                               0xEF4018
#define SPI_FLASH_TOTAL_SIZE                       0x10000000

#define SPI_FLASH_WriteEnable                      0x06
#define SPI_FLASH_WriteDisable                     0x04
#define SPI_FLASH_ReadStatusReg                    0x05
#define SPI_FLASH_WriteStatusReg                   0x01
#define SPI_FLASH_ReadData                         0x03
#define SPI_FLASH_FastReadData                     0x0B
#define SPI_FLASH_FastReadDual                     0x3B
#define SPI_FLASH_PageProgram                      0x02
#define SPI_FLASH_BlockErase                       0xD8
#define SPI_FLASH_SectorErase                      0x20
#define SPI_FLASH_ChipErase                        0xC7
#define SPI_FLASH_PowerDown                        0xB9
#define SPI_FLASH_ReleasePowerDown                 0xAB
#define SPI_FLASH_DeviceID                         0xAB
#define SPI_FLASH_ManufactDeviceID                 0x90
#define SPI_FLASH_JedecDeviceID                    0x9F
#define SPI_FLASH_WIP_FLAG                         0x01
#define SPI_FLASH_DUMMY_BYTE                       0xFF

#define SPI_FLASH_PERIPHERAL                       SPI5
#define SPI_FLASH_ALTERNATE                        GPIO_AF5_SPI5
#define SPI_FLASH_GPIO_PORT                        GPIOF
#define SPI_FLASH_SCK_PIN                          GPIO_PIN_7
#define SPI_FLASH_MISO_PIN                         GPIO_PIN_8
#define SPI_FLASH_MOSI_PIN                         GPIO_PIN_9
#define SPI_FLASH_CS_PORT                          GPIOF
#define SPI_FLASH_CS_PIN                           GPIO_PIN_6

#define CHOOSE_BIT_16                              16
#define CHOOSE_BIT_8                               8

#define SPI_FLASH_ENABLE(__HANDLE__)               __HAL_SPI_ENABLE(__HANDLE__)
#define SPI_FLASH_DISABLE(__HANDLE__)              __HAL_SPI_DISABLE(__HANDLE__)
#define SPI_FLASH_RCC_CLK_ENABLE()                 __HAL_RCC_SPI5_CLK_ENABLE()
#define SPI_FLASH_RCC_CLK_DISABLE()                __HAL_RCC_SPI5_CLK_DISABLE()

#define SPI_FLASH_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOF_CLK_ENABLE()
#define SPI_FLASH_CS_CLK_ENABLE()                  __HAL_RCC_GPIOF_CLK_ENABLE()

#define SPI_FLASH_CS_ENABLE()                      HAL_GPIO_WritePin(SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN, GPIO_PIN_RESET)
#define SPI_FLASH_CS_DISABLE()                     HAL_GPIO_WritePin(SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN, GPIO_PIN_SET)

#define CHECK_RET_RETURN(ret) \
    do \
    { \
        if ((ret) < 0) \
        { \
            return ret; \
        } \
    } while (0)

SPI_HandleTypeDef g_spi_flash;

/* This function is called by inner-HAL lib */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if (hspi->Instance == SPI_FLASH_PERIPHERAL)
    {
        SPI_FLASH_RCC_CLK_ENABLE();
        SPI_FLASH_GPIO_CLK_ENABLE();
        SPI_FLASH_CS_CLK_ENABLE();

        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Pin = SPI_FLASH_CS_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(SPI_FLASH_CS_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = SPI_FLASH_SCK_PIN | SPI_FLASH_MOSI_PIN | SPI_FLASH_MISO_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Alternate = SPI_FLASH_ALTERNATE;
        HAL_GPIO_Init(SPI_FLASH_GPIO_PORT, &GPIO_InitStruct);
    }
}

/* This function is called by inner-HAL lib */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
    if (hspi->Instance == SPI_FLASH_PERIPHERAL)
    {
        SPI_FLASH_RCC_CLK_DISABLE();
        HAL_GPIO_DeInit(SPI_FLASH_GPIO_PORT, SPI_FLASH_SCK_PIN | SPI_FLASH_MOSI_PIN | SPI_FLASH_MISO_PIN);
        HAL_GPIO_DeInit(SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN);
        SPI_FLASH_DISABLE(hspi);
    }
}

static int prv_spi_flash_send_byte(uint8_t send, uint8_t* recv)
{
    uint8_t tmp;
    uint8_t t_send = send;

    if (HAL_SPI_TransmitReceive(&g_spi_flash, &t_send, &tmp, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return -1;
    }
    if (NULL != recv)
    {
        *recv = tmp;
    }

    return 0;
}

static int prv_spi_flash_send_cmd(uint8_t cmd, uint32_t addr)
{
    int ret = 0;

    ret = prv_spi_flash_send_byte(cmd, NULL);
    CHECK_RET_RETURN(ret);
    ret = prv_spi_flash_send_byte((addr & 0xFF0000) >> CHOOSE_BIT_16, NULL);
    CHECK_RET_RETURN(ret);
    ret = prv_spi_flash_send_byte((addr & 0xFF00) >> CHOOSE_BIT_8, NULL);
    CHECK_RET_RETURN(ret);
    ret = prv_spi_flash_send_byte(addr & 0xFF, NULL);
    CHECK_RET_RETURN(ret);

    return ret;
}

static void prv_spi_flash_write_enable(void)
{
    SPI_FLASH_CS_ENABLE();
    (void)prv_spi_flash_send_byte(SPI_FLASH_WriteEnable, NULL);
    SPI_FLASH_CS_DISABLE();
}

static void prv_spi_flash_wait_write_end(void)
{
    uint8_t status = 0;

    SPI_FLASH_CS_ENABLE();

    (void)prv_spi_flash_send_byte(SPI_FLASH_ReadStatusReg, NULL);

    /* Loop as long as the memory is busy with a write cycle */
    do
    {
        /* Send a dummy byte to generate the clock needed by the FLASH
        and put the value of the status register in status variable */
        if (prv_spi_flash_send_byte(SPI_FLASH_DUMMY_BYTE, &status) == -1)
        {
            break;
        }
    } while ((status & SPI_FLASH_WIP_FLAG) == SET); /* Write in progress */

    SPI_FLASH_CS_DISABLE();
}

static int prv_spi_flash_write_page(const uint8_t* buf, uint32_t addr, int32_t len)
{
    int ret = 0;
    int i;

    if(0 == len)
    {
        return 0;
    }

    prv_spi_flash_write_enable();
    SPI_FLASH_CS_ENABLE();

    if ((ret = prv_spi_flash_send_cmd(SPI_FLASH_PageProgram, addr)) != -1)
    {
        for (i = 0; i < len; ++i)
        {
            if (prv_spi_flash_send_byte(buf[i], NULL) == -1)
            {
                ret = -1;
                break;
            }
        }
    }

    SPI_FLASH_CS_DISABLE();
    prv_spi_flash_wait_write_end();

    return ret;
}

static int prv_spi_flash_erase_sector(uint32_t addr)
{
    int ret = 0;

    prv_spi_flash_write_enable();
    prv_spi_flash_wait_write_end();
    SPI_FLASH_CS_ENABLE();

    ret = prv_spi_flash_send_cmd(SPI_FLASH_SectorErase, addr);

    SPI_FLASH_CS_DISABLE();
    prv_spi_flash_wait_write_end();

    return ret;
}

void hal_spi_flash_config(void)
{
    g_spi_flash.Instance = SPI_FLASH_PERIPHERAL;
    g_spi_flash.State = HAL_SPI_STATE_RESET;
    g_spi_flash.Init.Mode = SPI_MODE_MASTER;
    g_spi_flash.Init.Direction = SPI_DIRECTION_2LINES;
    g_spi_flash.Init.DataSize = SPI_DATASIZE_8BIT;
    g_spi_flash.Init.CLKPolarity = SPI_POLARITY_HIGH;
    g_spi_flash.Init.CLKPhase = SPI_PHASE_2EDGE;
    g_spi_flash.Init.NSS = SPI_NSS_SOFT;
    g_spi_flash.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    g_spi_flash.Init.FirstBit = SPI_FIRSTBIT_MSB;
    g_spi_flash.Init.TIMode = SPI_TIMODE_DISABLE;
    g_spi_flash.Init.CRCPolynomial = 7;
    HAL_SPI_Init(&g_spi_flash);
    SPI_FLASH_ENABLE(&g_spi_flash);
    SPI_FLASH_CS_DISABLE();
}

int hal_spi_flash_erase(uint32_t addr, int32_t len)
{
    uint32_t begin;
    uint32_t end;
    int i;

    if (len < 0
        || addr > SPI_FLASH_TOTAL_SIZE
        || addr + len > SPI_FLASH_TOTAL_SIZE)
    {
        return -1;
    }

    begin = addr / SPI_FLASH_SECTOR * SPI_FLASH_SECTOR;
    end = (addr + len - 1) / SPI_FLASH_SECTOR * SPI_FLASH_SECTOR;

    for (i = begin; i <= end; i += SPI_FLASH_SECTOR)
    {
        if (prv_spi_flash_erase_sector(i) == -1)
        {
            return -1;
        }
    }

    return 0;
}

int hal_spi_flash_write(const void* buf, int32_t len, uint32_t* location)
{
    const uint8_t* pbuf = (const uint8_t*)buf;
    int page_cnt = 0;
    int remain_cnt = 0;
    int temp = 0;
    uint32_t loc_addr;
    uint8_t addr = 0;
    uint8_t count = 0;
    int i;
    int ret = 0;

    if (NULL == pbuf
        || NULL == location
        || len < 0
        || *location > SPI_FLASH_TOTAL_SIZE
        || len + *location > SPI_FLASH_TOTAL_SIZE)
    {
        return -1;
    }

    loc_addr = *location;
    addr = loc_addr % SPI_FLASH_PAGESIZE;
    count = SPI_FLASH_PAGESIZE - addr;
    page_cnt = len / SPI_FLASH_PAGESIZE;
    remain_cnt = len % SPI_FLASH_PAGESIZE;

    if (addr == 0) /* addr is aligned to SPI_FLASH_PAGESIZE */
    {
        if (page_cnt == 0) /* len < SPI_FLASH_PAGESIZE */
        {
            ret = prv_spi_flash_write_page(pbuf, loc_addr, len);
            CHECK_RET_RETURN(ret);
        }
        else /* len > SPI_FLASH_PAGESIZE */
        {
            for (i = 0; i < page_cnt; ++i)
            {
                ret = prv_spi_flash_write_page(pbuf + i * SPI_FLASH_PAGESIZE, loc_addr, SPI_FLASH_PAGESIZE);
                CHECK_RET_RETURN(ret);
                loc_addr += SPI_FLASH_PAGESIZE;
            }

            ret = prv_spi_flash_write_page(pbuf + page_cnt * SPI_FLASH_PAGESIZE, loc_addr, remain_cnt);
            CHECK_RET_RETURN(ret);
        }
    }
    else /* addr is not aligned to SPI_FLASH_PAGESIZE */
    {
        if (page_cnt == 0) /* len < SPI_FLASH_PAGESIZE */
        {
            if (remain_cnt > count) /* (len + loc_addr) > SPI_FLASH_PAGESIZE */
            {
                temp = remain_cnt - count;

                ret = prv_spi_flash_write_page(pbuf, loc_addr, count);
                CHECK_RET_RETURN(ret);

                ret = prv_spi_flash_write_page(pbuf + count, loc_addr + count, temp);
                CHECK_RET_RETURN(ret);
            }
            else
            {
                ret = prv_spi_flash_write_page(pbuf, loc_addr, len);
                CHECK_RET_RETURN(ret);
            }
        }
        else /* len > SPI_FLASH_PAGESIZE */
        {
            len -= count;
            page_cnt = len / SPI_FLASH_PAGESIZE;
            remain_cnt = len % SPI_FLASH_PAGESIZE;

            ret = prv_spi_flash_write_page(pbuf, loc_addr, count);
            CHECK_RET_RETURN(ret);
            loc_addr += count;

            for (i = 0; i < page_cnt; ++i)
            {
                ret = prv_spi_flash_write_page(pbuf + count + i * SPI_FLASH_PAGESIZE, loc_addr, SPI_FLASH_PAGESIZE);
                CHECK_RET_RETURN(ret);
                loc_addr += SPI_FLASH_PAGESIZE;
            }

            if (remain_cnt != 0)
            {
                ret = prv_spi_flash_write_page(pbuf + count + page_cnt * SPI_FLASH_PAGESIZE, loc_addr, remain_cnt);
                CHECK_RET_RETURN(ret);
            }
        }
    }

    *location += len;
    return ret;
}

int hal_spi_flash_erase_write(const void* buf, int32_t len, uint32_t location)
{
    int ret = 0;

    ret = hal_spi_flash_erase(location, len);
    CHECK_RET_RETURN(ret);
    ret = hal_spi_flash_write(buf, len, &location);

    return ret;
}

int hal_spi_flash_read(void* buf, int32_t len, uint32_t location)
{
    int ret = 0;
    int i;
    uint8_t* pbuf = (uint8_t*)buf;

    if (NULL == pbuf
        || len < 0
        || location > SPI_FLASH_TOTAL_SIZE
        || len + location > SPI_FLASH_TOTAL_SIZE)
    {
        return -1;
    }

    SPI_FLASH_CS_ENABLE();

    if ((ret = prv_spi_flash_send_cmd(SPI_FLASH_ReadData, location)) != -1)
    {
        for (i = 0; i < len; ++i)
        {
            if (prv_spi_flash_send_byte(SPI_FLASH_DUMMY_BYTE, pbuf + i) == -1)
            {
                ret = -1;
                break;
            }
        }
    }

    SPI_FLASH_CS_DISABLE();

    return ret;
}

int hal_spi_flash_get_id(void)
{
    uint8_t tmp1 = 0;
    uint8_t tmp2 = 0;
    uint8_t tmp3 = 0;

    SPI_FLASH_CS_ENABLE();

    if (prv_spi_flash_send_byte(SPI_FLASH_JedecDeviceID, NULL) != -1)
    {
        (void)prv_spi_flash_send_byte(SPI_FLASH_DUMMY_BYTE, &tmp1);
        (void)prv_spi_flash_send_byte(SPI_FLASH_DUMMY_BYTE, &tmp2);
        (void)prv_spi_flash_send_byte(SPI_FLASH_DUMMY_BYTE, &tmp3);
    }

    SPI_FLASH_CS_DISABLE();

    return (tmp1 << CHOOSE_BIT_16) | (tmp2 << CHOOSE_BIT_8) | tmp3;
}

void hal_spi_flash_power_down(void)
{
    SPI_FLASH_CS_ENABLE();
    (void)prv_spi_flash_send_byte(SPI_FLASH_PowerDown, NULL);
    SPI_FLASH_CS_DISABLE();
}

void hal_spi_flash_wake_up(void)
{
    SPI_FLASH_CS_ENABLE();
    (void)prv_spi_flash_send_byte(SPI_FLASH_ReleasePowerDown, NULL);
    SPI_FLASH_CS_DISABLE();
}

#endif /* HAL_SPI_MODULE_ENABLED */