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
/* Freescale includes. */
#include "fsl_device_registers.h"
#include "board.h"

#include "pin_mux.h"
#include <stdbool.h>
//#include "los_inspect_entry.h"
#include "los_base.h" 
#include "los_task.h"
#include "los_typedef.h"
#include "osport.h"
#include<stdlib.h>

//import the uart here
extern void uart_init(void);
extern s32_t uart_read(u8_t *buf,s32_t len,s32_t timeout);
extern s32_t uart_write(u8_t *buf,s32_t len,s32_t timeout);

int atiny_random(void* output, size_t len)
{
    int ret = -1;
    unsigned char *dst;
    int value;
    dst = output;
    if((NULL == output)||(len <= 0))
    {
        return ret;
    }
    while(len-- > 0)
    {
        value = rand();
        *dst++= (uint8_t)value;
    }

    return 0;
}

void atiny_reboot(void)
{
    while(1);
}


#if USE_PPPOS
//the uart is used as the pppos interface
#if defined ( __CC_ARM ) || defined ( __ICCARM__ ) 
int fputc(int ch, FILE *f)
{
    int ret = 0;
    //ret =uart_write(( char *)&ch,1,0);
    return ret;
}
int fgetc(FILE *f)
{
    char ch;
    //uart_read((char *)&ch,1,0xFFFFFF);
    return ch;
}
#elif defined ( __GNUC__ ) 
__attribute__((used)) int _write(int fd, char *ptr, int len)
{
    //(void)uart_write((uint8_t *)ptr, len, 0xFFFF);
    return len;
}
#endif

#else
int fputc(int ch, FILE *f)
{
    int ret = 0;
    ret =uart_write((unsigned char *)&ch,1,0);
    return ret;
}

int fgetc(FILE *f)
{
    char ch;
    uart_read((unsigned char *)&ch,1,0xFFFFFF);
    return ch;
}
#endif
VOID HardWare_Init(VOID)
{
    /* Init board hardware. */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    /* attach 12 MHz clock to SPI3 */
    //CLOCK_AttachClk(kMCLK_to_FLEXCOMM3);
    /* reset FLEXCOMM for UART */
    RESET_PeripheralReset(kFC0_RST_SHIFT_RSTn);
    /* reset FLEXCOMM for SPI */
    RESET_PeripheralReset(kFC3_RST_SHIFT_RSTn);
    BOARD_InitPins();
    //BOARD_BootClockFROHF48M();
    BOARD_BootClockFROHF96M();
   // BOARD_InitDebugConsole();
    uart_init();
}

volatile int gRamSize = 0;
volatile int gRamMax = 0;
VOID osTaskMemUsedInc(UINT32 uwUsedSize)
{
    gRamSize += uwUsedSize;
    if(gRamSize > gRamMax)
    {
        gRamMax = gRamSize;
    }
}
VOID osTaskMemUsedDec(UINT32 uwUsedSize)
{
    gRamSize -= uwUsedSize;
}


VOID tmp_task(VOID)
{
    for(;;)
    {
        printf("hhh");
        LOS_TaskDelay(1000);
    }
}

UINT32 ggg;
extern VOID *main_ppp(UINT32  args);
UINT32 creat_tmp_task()
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 0;
    task_init_param.pcName = "main_ppp";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)main_ppp;
    printf("func is %p\n",main_ppp);

#ifdef CONFIG_FEATURE_FOTA
    task_init_param.uwStackSize = 0x2000; /* fota use mbedtls bignum to verify signature  consuming more stack  */
#else
    task_init_param.uwStackSize = 0x1000;
#endif

    uwRet = LOS_TaskCreate(&ggg, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}



int main(void)
{
    UINT32 uwRet = LOS_OK;
    HardWare_Init();
    uwRet = LOS_KernelInit();
    if (uwRet != LOS_OK)
    {
        return LOS_NOK;
    }
    printf("come into main\n");
    
    creat_tmp_task();
    LOS_Start();
    return 0;
}





