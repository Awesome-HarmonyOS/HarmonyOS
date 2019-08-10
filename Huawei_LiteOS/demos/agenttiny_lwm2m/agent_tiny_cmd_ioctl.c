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

#include "agent_tiny_cmd_ioctl.h"
#include "atiny_lwm2m/agenttiny.h"
#include "osdepends/atiny_osdep.h"
#ifdef CONFIG_FEATURE_FOTA
#include "ota/ota_api.h"
#include "ota_port.h"
#endif

#if defined(WITH_AT_FRAMEWORK) && defined(USE_NB_NEUL95)
#include "at_device/bc95.h"
#endif


#define ATINY_POWER_VOLTAGE     3800
#define ATINY_BATTERY_LEVEL     90
#define ATINY_MEMORY_FREE       50
#define ATINY_NETWORK_BEARER    5
#define ATINY_SIGNAL_STRENGTH   90
#define ATINY_CELL_ID           21103
#define ATINY_LINK_QUALITY      98
#define ATINY_LINK_UTRILIZATION 10
#define ATINY_POWER_SOURCE      1
#define ATINY_POWER_CURRENT     125
#define ATINY_LATITUDE          27.986065f
#define ATINY_LONGITUDE         86.922623f
#define ATINY_ALTITUDE          8495.0000f
#define ATINY_RADIUS            0.0f
#define ATINY_SPEED             0.0f
#define ATINY_TIME_CODE         1367491215

int atiny_get_manufacturer(char *manu, int len)
{
    (void)atiny_snprintf(manu, len, "Open Mobile Alliance");
    return ATINY_OK;
}

int atiny_get_model_number(char *mode, int len)
{
    (void)atiny_snprintf(mode, len, "Lightweight M2M Client");
    return ATINY_OK;
}

int atiny_get_serial_number(char *num, int len)
{
    (void)atiny_snprintf(num, len, "345000123");
    return ATINY_OK;
}

int atiny_get_firmware_ver(char *version, int len)
{
    (void)atiny_snprintf(version, len, "example_ver001");
    return ATINY_OK;
}

int atiny_do_dev_reboot(void)
{
    (void)atiny_printf("device is rebooting......\r\n");
    //LOS_TaskDelay(1000);
    atiny_reboot();
    return ATINY_OK;
}

int atiny_do_factory_reset(void)
{
    (void)atiny_printf("\n\t FACTORY RESET\r\n\n");
    return ATINY_OK;
}

int atiny_get_power_source(int *arg)
{
    *arg = ATINY_POWER_SOURCE;
    return ATINY_OK;
}

int atiny_get_source_voltage(int *voltage)
{
    *voltage = ATINY_POWER_VOLTAGE;
    return ATINY_OK;
}

int atiny_get_power_current(int *arg)
{
    *arg = ATINY_POWER_CURRENT;
    return ATINY_OK;
}

int atiny_get_baterry_level(int *voltage)
{
    *voltage = ATINY_BATTERY_LEVEL;
    return ATINY_OK;
}

int atiny_get_memory_free(int *voltage)
{
    int tmp;
    (void)atiny_random(&tmp, sizeof(tmp));
    tmp %= 30;
    *voltage = ATINY_MEMORY_FREE + tmp;
    return ATINY_OK;
}

static int err_code = ATINY_OK;

int atiny_get_dev_err(int *arg)
{
    *arg = err_code;
    return ATINY_OK;
}

int atiny_do_reset_dev_err(void)
{
    err_code = ATINY_OK;
    return ATINY_OK;
}

static int64_t g_current_time = ATINY_TIME_CODE;

int atiny_get_current_time(int64_t *arg)
{
    *arg = g_current_time + (int64_t)atiny_gettime_ms() / 1000;
    return ATINY_OK;
}

int atiny_set_current_time(const int64_t *arg)
{
    g_current_time = *arg - (int64_t)atiny_gettime_ms() / 1000;
    return ATINY_OK;
}

#define UTC_OFFSET_MAX_LEN 7
static char g_UTC_offset[UTC_OFFSET_MAX_LEN] = "+01:00";

int atiny_get_UTC_offset(char *offset, int len)
{
    if(len > strlen(g_UTC_offset) + 1)
    {
        (void)atiny_snprintf(offset, len, g_UTC_offset);
    }
    return ATINY_OK;
}

int atiny_set_UTC_offset(const char *offset, int len)
{
    (void)atiny_snprintf(g_UTC_offset, len + 1, offset);
    return ATINY_OK;
}

#define TIMEZONE_MAXLEN 25
static char g_timezone[TIMEZONE_MAXLEN] = "Europe/Berlin";

int atiny_get_timezone(char *timezone, int len)
{
    if(len > strlen(g_timezone) + 1)
    {
        (void)atiny_snprintf(timezone, len, g_timezone);
    }
    return ATINY_OK;
}

int atiny_set_timezone(const char *timezone, int len)
{
    (void)atiny_snprintf(g_timezone, len + 1, timezone);
    return ATINY_OK;
}

int atiny_get_bind_mode(char *mode, int len)
{
    (void)atiny_printf("bind type is UQ......\r\n");
    (void)atiny_snprintf(mode, len, "UQ");
    return ATINY_OK;
}

int atiny_trig_firmware_update(void)
{
    (void)atiny_printf("firmware is updating......\r\n");
    return ATINY_OK;
}

int atiny_get_firmware_result(int *result)
{
    *result = 0;
    return ATINY_OK;
}

int atiny_get_firmware_state(int *state)
{
    *state = 0;
    return ATINY_OK;
}

int atiny_get_network_bearer(int *network_brearer)
{
    *network_brearer = ATINY_NETWORK_BEARER;
    return ATINY_OK;
}

int atiny_get_signal_strength(int *singal_strength)
{
    *singal_strength = ATINY_SIGNAL_STRENGTH;
    return ATINY_OK;
}

int atiny_get_cell_id(long *cell_id)
{
    *cell_id = ATINY_CELL_ID;
    return ATINY_OK;
}

int atiny_get_link_quality(int *quality)
{
    *quality = ATINY_LINK_QUALITY;
    return ATINY_OK;
}

int atiny_get_link_utilization(int *utilization)
{
    *utilization = ATINY_LINK_UTRILIZATION;
    return ATINY_OK;
}

int atiny_write_app_write(void *user_data, int len)
{
    (void)atiny_printf("write num19 object success\r\n");
    return ATINY_OK;
}

int atiny_update_psk(char *psk_id, int len)
{
    //memcpy_s(g_psk_value,psk_id,len,16);
    (void)atiny_printf("update psk success\r\n");
    return ATINY_OK;
}

int atiny_get_latitude(float *latitude)
{
    *latitude = ATINY_LATITUDE;
    return ATINY_OK;
}

int atiny_get_longitude(float *longitude)
{
    *longitude = ATINY_LONGITUDE;
    return ATINY_OK;
}

int atiny_get_altitude(float *altitude)
{
    *altitude = ATINY_ALTITUDE;
    return ATINY_OK;
}

int atiny_get_radius(float *radius)
{
    *radius = ATINY_RADIUS;
    return ATINY_OK;
}

int atiny_get_speed(float *speed)
{
    *speed = ATINY_SPEED;
    return ATINY_OK;
}

int atiny_get_timestamp(uint64_t *timestamp)
{
    *timestamp = atiny_gettime_ms() / 1000 + ATINY_TIME_CODE;
    return ATINY_OK;
}

//-----  3GPP TS 23.032 V11.0.0(2012-09) ---------
#define HORIZONTAL_VELOCITY                  0
#define HORIZONTAL_VELOCITY_VERTICAL         1
#define HORIZONTAL_VELOCITY_WITH_UNCERTAINTY 2

#define VELOCITY_OCTETS                      5

void location_get_velocity(atiny_velocity_s *velocity,
                           uint16_t bearing,
                           uint16_t horizontal_speed,
                           uint8_t speed_uncertainty)
{
    uint8_t tmp[VELOCITY_OCTETS];
    int copy_len;

    tmp[0] = HORIZONTAL_VELOCITY_WITH_UNCERTAINTY << 4;
    tmp[0] |= (bearing & 0x100) >> 8;
    tmp[1] = (bearing & 0xff);
    tmp[2] = horizontal_speed >> 8;
    tmp[3] = horizontal_speed & 0xff;
    tmp[4] = speed_uncertainty;

    copy_len = MAX_VELOCITY_LEN > sizeof(tmp) ? sizeof(tmp) : MAX_VELOCITY_LEN;
    memcpy(velocity->opaque, tmp, copy_len);
    velocity->length = copy_len;
}

int atiny_get_velocity(atiny_velocity_s *velocity)
{
    location_get_velocity(velocity, 0, 0, 255);
    return ATINY_OK;
}

int atiny_cmd_ioctl(atiny_cmd_e cmd, char *arg, int len)
{
    int result = ATINY_OK;
    switch(cmd)
    {
    case ATINY_GET_MANUFACTURER:
        result = atiny_get_manufacturer(arg, len);
        break;
    case ATINY_GET_MODEL_NUMBER:
        result = atiny_get_model_number(arg, len);
        break;
    case ATINY_GET_SERIAL_NUMBER:
        result = atiny_get_serial_number(arg, len);
        break;
    case ATINY_GET_FIRMWARE_VER:
        result = atiny_get_firmware_ver(arg, len);
        break;
    case ATINY_DO_DEV_REBOOT:
        result = atiny_do_dev_reboot();
        break;
    case ATINY_DO_FACTORY_RESET:
        result = atiny_do_factory_reset();
        break;
    case ATINY_GET_POWER_SOURCE:
        result = atiny_get_power_source((int *)arg);
        break;
    case ATINY_GET_SOURCE_VOLTAGE:
        result = atiny_get_source_voltage((int *)arg);
        break;
    case ATINY_GET_POWER_CURRENT:
        result = atiny_get_power_current((int *)arg);
        break;
    case ATINY_GET_BATERRY_LEVEL:
        result = atiny_get_baterry_level((int *)arg);
        break;
    case ATINY_GET_MEMORY_FREE:
        result = atiny_get_memory_free((int *)arg);
        break;
    case ATINY_GET_DEV_ERR:
        result = atiny_get_dev_err((int *)arg);
        break;
    case ATINY_DO_RESET_DEV_ERR:
        result = atiny_do_reset_dev_err();
        break;
    case ATINY_GET_CURRENT_TIME:
        result = atiny_get_current_time((int64_t *)arg);
        break;
    case ATINY_SET_CURRENT_TIME:
        result = atiny_set_current_time((const int64_t *)arg);
        break;
    case ATINY_GET_UTC_OFFSET:
        result = atiny_get_UTC_offset(arg, len);
        break;
    case ATINY_SET_UTC_OFFSET:
        result = atiny_set_UTC_offset(arg, len);
        break;
    case ATINY_GET_TIMEZONE:
        result = atiny_get_timezone(arg, len);
        break;
    case ATINY_SET_TIMEZONE:
        result = atiny_set_timezone(arg, len);
        break;
    case ATINY_GET_BINDING_MODES:
        result = atiny_get_bind_mode(arg, len);
        break;
    case ATINY_GET_FIRMWARE_STATE:
        result = atiny_get_firmware_state((int *)arg);
        break;
    case ATINY_GET_NETWORK_BEARER:
        result = atiny_get_network_bearer((int *)arg);
        break;
    case ATINY_GET_SIGNAL_STRENGTH:
        result = atiny_get_signal_strength((int *)arg);
        break;
    case ATINY_GET_CELL_ID:
        result = atiny_get_cell_id((long *)arg);
        break;
    case ATINY_GET_LINK_QUALITY:
        result = atiny_get_link_quality((int *)arg);
        break;
    case ATINY_GET_LINK_UTILIZATION:
        result = atiny_get_link_utilization((int *)arg);
        break;
    case ATINY_WRITE_APP_DATA:
        result = atiny_write_app_write((int *)arg, len);
        break;
    case ATINY_UPDATE_PSK:
        result = atiny_update_psk(arg, len);
        break;
    case ATINY_GET_LATITUDE:
        result = atiny_get_latitude((float *)arg);
        break;
    case ATINY_GET_LONGITUDE:
        result = atiny_get_longitude((float *)arg);
        break;
    case ATINY_GET_ALTITUDE:
        result = atiny_get_altitude((float *)arg);
        break;
    case ATINY_GET_RADIUS:
        result = atiny_get_radius((float *)arg);
        break;
    case ATINY_GET_SPEED:
        result = atiny_get_speed((float *)arg);
        break;
    case ATINY_GET_TIMESTAMP:
        result = atiny_get_timestamp((uint64_t *)arg);
        break;
    case ATINY_GET_VELOCITY:
        result = atiny_get_velocity((atiny_velocity_s *)arg);
        break;

#ifdef CONFIG_FEATURE_FOTA
    case ATINY_GET_OTA_OPT:
    {
        ota_opt_s *opt = (ota_opt_s *)arg;
        hal_get_ota_opt(opt);
        opt->key.rsa_N = "C94BECB7BCBFF459B9A71F12C3CC0603B11F0D3A366A226FD3E73D453F96EFBBCD4DFED6D9F77FD78C3AB1805E1BD3858131ACB5303F61AF524F43971B4D429CB847905E68935C1748D0096C1A09DD539CE74857F9FDF0B0EA61574C5D76BD9A67681AC6A9DB1BB22F17120B1DBF3E32633DCE34F5446F52DD7335671AC3A1F21DC557FA4CE9A4E0E3E99FED33A0BAA1C6F6EE53EDD742284D6582B51E4BF019787B8C33C2F2A095BEED11D6FE68611BD00825AF97DB985C62C3AE0DC69BD7D0118E6D620B52AFD514AD5BFA8BAB998332213D7DBF5C98DC86CB8D4F98A416802B892B8D6BEE5D55B7E688334B281E4BEDDB11BD7B374355C5919BA5A9A1C91F";
        opt->key.rsa_E = "10001";
        result = ATINY_OK;
        break;
    }
#endif

#if defined(WITH_AT_FRAMEWORK) && defined(USE_NB_NEUL95)
    case ATINY_TRIGER_SERVER_INITIATED_BS:
        nb_reattach();
        result = ATINY_OK;
        break;
#endif

    default:
        break;
    }
    return result;
}

void atiny_event_notify(atiny_event_e event, const char *arg, int len)
{
    (void)atiny_printf("notify:stat:%d\r\n", event);
}

