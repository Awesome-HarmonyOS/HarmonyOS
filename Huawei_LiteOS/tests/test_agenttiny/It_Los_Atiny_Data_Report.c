#include "test_agenttiny.h"

#define DATA_LOG    printf("[%s] data report ret: %d\n",__FUNCTION__,ret);
UINT32 test_report_handle;

void test_atiny_report(void)
{
    const UnitTest It_Los_Atiny_DataReport[] =
    {
        unit_test(It_Los_Atiny_Data_Report_000),
        unit_test(It_Los_Atiny_Data_Report_001),
        unit_test(It_Los_Atiny_Data_Report_002),
        unit_test(It_Los_Atiny_Data_Report_003),
        unit_test(It_Los_Atiny_Data_Report_004),
        unit_test(It_Los_Atiny_Data_Report_005),
        unit_test(It_Los_Atiny_Data_Report_006),
    };
    (void)LOS_TaskDelay(250 * 4 * 5);  // ÑÓÊ±ÓÃÓÚatiny_bind()
    if(!is_reg_ok())
        return;
    run_tests(It_Los_Atiny_DataReport);
    atiny_deinit(test_phandle);
}

UINT32 test_creat_report_task()
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 1;
    task_init_param.pcName = "atiny_report_task";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)test_atiny_report;
    task_init_param.uwStackSize = 0x4000;

    uwRet = LOS_TaskCreate(&test_report_handle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}

int It_Los_Atiny_Data_Report(void)
{
    UINT32 uwRet = LOS_OK;
    atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 10;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;
#ifndef WITH_DTLS
    uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwDevice_info.endpoint_name = "66660006";
#else
    uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);

    uwDevice_info.endpoint_name = "99990009";
#endif
    uwDevice_info.manufacturer = "LiteOS";
    uwDevice_info.dev_type = "IOT";
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return ret;
    }
    
    uwRet = test_creat_report_task();
    if(LOS_OK != uwRet)
    {
        ATINY_LOG(LOG_INFO,">>>TEST:Deinit task create fail!");
        return -1;
    }
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    
    uwRet = LOS_TaskDelete(test_report_handle);
    if(LOS_OK != uwRet)
    {
        ATINY_LOG(LOG_INFO,">>>TEST:Deinit task delete fail!");
        return -1;
    }
    
    return 0;
}

void test_ack(atiny_report_type_e type, int cookie, data_send_status_e status)
{
    ATINY_LOG(LOG_DEBUG, "type:%d cookie:%d status:%d\n", type, cookie, status);
}
void It_Los_Atiny_Data_Report_000 (void **state)
{
    int ret = 0;
    uint8_t buf[5] = {0,1,2,3,4};
    data_report_t report_data;
    memset(&report_data,0,sizeof(data_report_t));
    report_data.buf = buf;
    report_data.callback = test_ack;
    report_data.cookie = 0;
    report_data.len = sizeof(buf);
    report_data.type = APP_DATA;
    
    ret = atiny_data_report(test_phandle, &report_data);
    DATA_LOG;
    assert_int_equal(ret, ATINY_OK);
    (void)LOS_TaskDelay(250 * 8);   //valid case
}

void It_Los_Atiny_Data_Report_001 (void **state)
{
    int ret = 0;
    uint8_t buf[5] = {0,1,2,3,4};
    data_report_t report_data;
    memset(&report_data,0,sizeof(data_report_t));
    report_data.buf = buf;
    report_data.callback = test_ack;
    report_data.cookie = 0;
    report_data.len = sizeof(buf);
    report_data.type = APP_DATA;
    
    ret = atiny_data_report(NULL, &report_data);    // phandle == NULL
    DATA_LOG;
    assert_int_equal(ret, ATINY_ARG_INVALID);
    (void)LOS_TaskDelay(250 * 8);
    
    ret = atiny_data_report(test_phandle, NULL);    // report_data == NULL
    DATA_LOG;
    assert_int_equal(ret, ATINY_ARG_INVALID);
    (void)LOS_TaskDelay(250 * 8);
    
    report_data.buf = NULL;                         // report_data->buf == NULL
    ret = atiny_data_report(test_phandle, &report_data);
    DATA_LOG;
    assert_int_equal(ret, ATINY_ARG_INVALID);
    (void)LOS_TaskDelay(250 * 8);
    
    report_data.buf = buf;
    report_data.len = 0;                            // report_data->len <= 0
    DATA_LOG;
    assert_int_equal(ret, ATINY_ARG_INVALID);
    (void)LOS_TaskDelay(250 * 8);
    
    report_data.len = 1024 + 1;                     // report_data->len > 1024
    DATA_LOG;
    assert_int_equal(ret, ATINY_ARG_INVALID);
    (void)LOS_TaskDelay(250 * 8);
}

void It_Los_Atiny_Data_Report_002 (void **state)
{
    int ret = 0;
    uint8_t buf[5] = {0,1,2,3,4};
    data_report_t report_data;
    memset(&report_data,0,sizeof(data_report_t));
    report_data.buf = buf;
    report_data.callback = NULL;    // calback = NULL
    report_data.cookie = 0;
    report_data.len = sizeof(buf);
    report_data.type = APP_DATA;
    
    ret = atiny_data_report(test_phandle, &report_data);
    DATA_LOG;
    assert_int_equal(ret, ATINY_OK);
    (void)LOS_TaskDelay(250 * 8);
}

void It_Los_Atiny_Data_Report_003 (void **state)
{
    int ret = 0;
    uint8_t buf[5] = {0,1,2,3,4};
    data_report_t report_data;
    memset(&report_data,0,sizeof(data_report_t));
    report_data.buf = buf;
    report_data.callback = test_ack;
    report_data.cookie = 0;
    report_data.len = sizeof(buf);
    report_data.type = FIRMWARE_UPDATE_STATE;  // report_data.type = FIRMWARE_UPDATE_STATE
    
    ret = atiny_data_report(test_phandle, &report_data);
    DATA_LOG;
    assert_int_equal(ret, ATINY_RESOURCE_NOT_FOUND);
    (void)LOS_TaskDelay(250 * 8);
}

void It_Los_Atiny_Data_Report_004 (void **state)
{
    int ret = 0;
    uint8_t buf[5] = {0,1,2,3,4}; 
    data_report_t report_data;
    memset(&report_data,0,sizeof(data_report_t));
    int cnt = 0;
    report_data.buf = buf;
    report_data.callback = test_ack;
    report_data.cookie = 0;
    report_data.len = sizeof(buf) - 1;  // unmatch with buf
    report_data.type = APP_DATA;
    ret = atiny_data_report(test_phandle, &report_data);
    DATA_LOG;
    assert_int_equal(ret, ATINY_OK);
    (void)LOS_TaskDelay(250 * 4);
}

void It_Los_Atiny_Data_Report_005 (void **state)
{
    int ret = 0;
    uint8_t buf[5] = {0,1,2,3,4}; 
    data_report_t report_data;
    memset(&report_data,0,sizeof(data_report_t));
    int cnt = 0;
    report_data.buf = buf;
    report_data.callback = test_ack;
    report_data.cookie = 4294967295;   // boundary value
    report_data.len = sizeof(buf);
    report_data.type = APP_DATA;
    ret = atiny_data_report(test_phandle, &report_data);
    DATA_LOG;
    assert_int_equal(ret, ATINY_OK);
    (void)LOS_TaskDelay(250 * 4);
}

void It_Los_Atiny_Data_Report_006 (void **state)
{
    int ret = 0;
    uint8_t buf[5] = {0,1,2,3,4}; 
    data_report_t report_data;
    memset(&report_data,0,sizeof(data_report_t));
    int cnt = 0;
    report_data.buf = buf;
    report_data.callback = test_ack;
    report_data.cookie = 4294967295;   // boundary value
    report_data.len = 0;   // boundary value
    report_data.type = APP_DATA;
    ret = atiny_data_report(test_phandle, &report_data);
    DATA_LOG;
    assert_int_equal(ret, ATINY_ARG_INVALID);
    (void)LOS_TaskDelay(250 * 4);
}
