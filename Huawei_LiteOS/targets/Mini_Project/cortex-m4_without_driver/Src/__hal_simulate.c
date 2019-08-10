
#include <stdio.h>


unsigned int SystemCoreClock = 16000000;
__attribute__((weak)) void SystemInit(void)
{
}

__attribute__((weak)) void HardWare_Init(void)
{
    
}

__attribute__((weak)) int hal_rng_generate_buffer(void* buf, size_t len)
{
	return 0;
}

__attribute__((weak)) void HAL_NVIC_SystemReset(void)
{
}

__attribute__((weak)) void delayus(uint32_t usec)
{
}

__attribute__((weak)) void hieth_hw_init(void)
{
}

#ifdef CONFIG_FEATURE_FOTA
#include "ota/ota_api.h"
__attribute__((weak)) void hal_init_ota(void)
{
}
__attribute__((weak)) void hal_get_ota_opt(ota_opt_s *opt)
{
}
#endif

