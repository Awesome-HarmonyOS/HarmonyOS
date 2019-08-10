#include "test_agenttiny.h"

UINT32 test_deinit_handle;
void *test_phandle;
extern atiny_event_e g_RegState;
static int reg_suss;

int is_reg_ok()
{
    handle_data_t *handle = (handle_data_t *)test_phandle;
    if (STATE_READY == handle->lwm2m_context->state)
    {
        reg_suss++;
        printf(">>>Bind register is OK!\n");
        return 1;
    }
    else
        return 0;
}

static void test_atiny_deinit(void)
{
    static int deinit_cnt = 0;
    const int CON_TIME_CNT = 10; // seconds
    static int del_handle_time = 0;
    
    while(1)
    {
        if(NULL != test_phandle && is_reg_ok())
        {    
//            atiny_deinit(test_phandle);
            del_handle_time++;
            printf(">>>TEST:deinit success. use time %d seconds\n",deinit_cnt);
            printf(">>>TEST:deinit success. del_handle_time = %d \n",del_handle_time);
            deinit_cnt = 0;
            atiny_deinit(test_phandle);
        }
        else
        {
            deinit_cnt++;
            if(deinit_cnt >= CON_TIME_CNT)
            {
                if(NULL != test_phandle)
                {
//                    atiny_deinit(test_phandle);
//                    del_handle_time++;
                    printf(">>>TEST:deinit success. use time %d seconds\n",deinit_cnt);
                    printf(">>>TEST:deinit success. del_handle_time = %d \n",del_handle_time);
                    deinit_cnt = 0;
                    atiny_deinit(test_phandle);
                }
            }
        }
        LOS_TaskDelay(4*250);
    }
}

UINT32 test_creat_deinit_task()
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 1;
    task_init_param.pcName = "atiny_deinit_task";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)test_atiny_deinit;
    task_init_param.uwStackSize = 0x4000;

    uwRet = LOS_TaskCreate(&test_deinit_handle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}

int It_Los_Atiny_Bind(void)
{
    UINT32 uwRet;
    uwRet = test_creat_deinit_task();
    if(LOS_OK != uwRet)
    {
        ATINY_LOG(LOG_INFO,">>>TEST:Deinit task create fail!");
        return -1;
    }
    const UnitTest It_Los_Atiny_Bind[] =
    {
#ifdef WITH_DTLS
        unit_test(It_Los_Atiny_Bind_030),
        unit_test(It_Los_Atiny_Bind_031),
        unit_test(It_Los_Atiny_Bind_032),
        unit_test(It_Los_Atiny_Bind_033),
        unit_test(It_Los_Atiny_Bind_034),
        unit_test(It_Los_Atiny_Bind_035),
        unit_test(It_Los_Atiny_Bind_036),
        unit_test(It_Los_Atiny_Bind_037),
        unit_test(It_Los_Atiny_Bind_038),
        unit_test(It_Los_Atiny_Bind_039),
        unit_test(It_Los_Atiny_Bind_040),
        unit_test(It_Los_Atiny_Bind_041),
        unit_test(It_Los_Atiny_Bind_042),
        unit_test(It_Los_Atiny_Bind_043),
        unit_test(It_Los_Atiny_Bind_044),
#else
        unit_test(It_Los_Atiny_Bind_000),
        unit_test(It_Los_Atiny_Bind_001),
        unit_test(It_Los_Atiny_Bind_002),
        unit_test(It_Los_Atiny_Bind_003),
        unit_test(It_Los_Atiny_Bind_004),
        unit_test(It_Los_Atiny_Bind_005),
        unit_test(It_Los_Atiny_Bind_006),
        unit_test(It_Los_Atiny_Bind_007),
        unit_test(It_Los_Atiny_Bind_008),
        unit_test(It_Los_Atiny_Bind_009),
        unit_test(It_Los_Atiny_Bind_010),
        unit_test(It_Los_Atiny_Bind_011),
        unit_test(It_Los_Atiny_Bind_012),
        unit_test(It_Los_Atiny_Bind_013),
        unit_test(It_Los_Atiny_Bind_014),
        unit_test(It_Los_Atiny_Bind_015),
        unit_test(It_Los_Atiny_Bind_016),
        unit_test(It_Los_Atiny_Bind_017),
        unit_test(It_Los_Atiny_Bind_018),
        unit_test(It_Los_Atiny_Bind_019),
        unit_test(It_Los_Atiny_Bind_020),
        unit_test(It_Los_Atiny_Bind_021),
        unit_test(It_Los_Atiny_Bind_022),
        unit_test(It_Los_Atiny_Bind_023),
        unit_test(It_Los_Atiny_Bind_024),
        unit_test(It_Los_Atiny_Bind_025),
        unit_test(It_Los_Atiny_Bind_026),
#endif
    };
    
    run_tests(It_Los_Atiny_Bind);
    
    printf("********** reg success: %d ***********\n",reg_suss);
    uwRet = LOS_TaskDelete(test_deinit_handle);
    if(LOS_OK != uwRet)
    {
        ATINY_LOG(LOG_INFO,">>>TEST:Deinit task delete fail!");
        return -1;
    }
    
    return 0;
}

void It_Los_Atiny_Bind_000 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;
#ifdef WITH_DTLS
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
    
#else
	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;
#endif

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &phandle);
    assert_int_equal(ret, ATINY_OK);
    
    ret = atiny_bind(NULL,NULL);
    assert_int_equal(ret, ATINY_ARG_INVALID);
    
    ret = atiny_bind(&uwDevice_info,NULL);
    assert_int_equal(ret, ATINY_ARG_INVALID);
    
	ret = atiny_bind(NULL,phandle);
	assert_int_equal(ret, ATINY_ARG_INVALID);
    
    uwDevice_info.endpoint_name = NULL;
    ret = atiny_bind(&uwDevice_info,&phandle);
    assert_int_equal(ret, ATINY_ARG_INVALID);
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = NULL;
    assert_int_equal(ret, ATINY_ARG_INVALID);
}

void It_Los_Atiny_Bind_001 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_002 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;
    
    uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;    // exist regsitered DTLS endpoint
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_003 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;
    
    uwAtiny_params.security_params[0].server_ip = "not a ip addr";  // invalid ip addr
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_004 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = "240.1.1.1";  //invalid ip addr  E??IP
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_005 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = "192.168.1.103";  //invalid ip addr ?????????ip
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_006 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 4294967295;  // MAX UINT

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_007 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 0;  // boundary value

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 0;     // boundary value

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_008 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = -1;    // not normal value

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_009 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"SQ";    //unsupport model
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_010 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"S";     // unsupport model
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = -1;  // compiler warning

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_011 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQS";   // unsupport model
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = "77770007";
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_012 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"US";    // unsupport model
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_013 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"else";  // invalid param
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_MALLOC_FAILED);
}

void It_Los_Atiny_Bind_014 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = "not a port";       // invalid port
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_015 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = "5684";      //DTLS port
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_016 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = "65535";      //incorrect port
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_017 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_CLIENT_INITIATED;  // unsuporrt leshan server
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_018 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;  // BOOTSTRAP_SEQUENCE
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_019 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"else";  // invalid model
	uwAtiny_params.server_params.life_time = 5000;
	uwAtiny_params.server_params.storing_cnt = 4294967295;  //boundary value

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;  // BOOTSTRAP_SEQUENCE
	uwAtiny_params.server_params.hold_off_time = 0;

	uwAtiny_params.security_params[0].server_ip = "not a ip addr";  // invalid ip
	uwAtiny_params.security_params[0].server_port = "not a port";   // invalid port
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_MALLOC_FAILED);
}

void It_Los_Atiny_Bind_020 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 0;     // boundary value
	uwAtiny_params.server_params.storing_cnt = 4294967295;  // boundary value

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 2147483647;    // boundary value

	uwAtiny_params.security_params[0].server_ip = "not a ip addr";  //invalid ip
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_021 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"SQ";    // unsupport model
	uwAtiny_params.server_params.life_time = 2147483647;    // boundary value
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE
;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_022 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 1;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;   // DTLS endpoint
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_023 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"U";
	uwAtiny_params.server_params.life_time = 2147483647;    //boundary value
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 0;

	uwAtiny_params.security_params[0].server_ip = "not a ip addr";  // invalid ip
	uwAtiny_params.security_params[0].server_port = "0";    //"incorrect port"
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = "not a endpoint";     // invalid endpoint
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_024 (void **state)
{
    atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = NULL;    //unsupport model
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_025 (void **state)
{
    atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"S";     // unsupport model
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = -1;  // compiler warning

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = NULL;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_PORT;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_026 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"SQ";    // unsupport model
	uwAtiny_params.server_params.life_time = 2147483647;    // boundary value
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = NULL;
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PORT;

    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if(ATINY_OK != ret)
    {
        return;
    }
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

/* DTLS case */
void It_Los_Atiny_Bind_030 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

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
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return ;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_031 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

    uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = "hello";     // invalid psk value
	uwAtiny_params.security_params[0].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_032 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

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
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;   // mismatching endpoint_name 和psk不匹配的设备
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_033 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

    uwAtiny_params.security_params[0].server_ip = "not a ip addr";      //invalid ip 
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_034 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

    uwAtiny_params.security_params[0].server_ip = "240.1.1.1";  // 不能ping通的E类ip
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_035 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

    uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 0;    // invalid value
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_036 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

    uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = "88880008";      // 和endpoint_name psk不匹配
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_037 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"S";     // unsupport binding model
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

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
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_038 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"SQ";    // unsupport binding model
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

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
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_039 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQS";   // unsupport binding model
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

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
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_040 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"US";    // unsupport binding model
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

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
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_041 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"ELSE";      // unsupport binding model
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

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
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_MALLOC_FAILED);
}

void It_Los_Atiny_Bind_042 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"U";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

    uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[0].server_port = "5683";         // 非DTLS端口
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_OK);
}

void It_Los_Atiny_Bind_043 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"ELSE";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

    uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[0].server_port = "not a port";       // invalid port
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_DTLS_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_MALLOC_FAILED);
}

void It_Los_Atiny_Bind_044 (void **state)
{
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
    atiny_device_info_t uwDevice_info;
    memset(&uwDevice_info, 0, sizeof(atiny_device_info_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"ELSE";
	uwAtiny_params.server_params.life_time = 20;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

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
    
    uwDevice_info.endpoint_name = DEFAULT_EP_NAME_S;
    uwDevice_info.manufacturer = DEFAULT_MFT;
    uwDevice_info.dev_type = DEFAULT_DEV_TYPE;
    
    ret = atiny_init(&uwAtiny_params, &test_phandle);
    if (ATINY_OK != ret)
        return;
    
    ret = atiny_bind(&uwDevice_info, test_phandle);
    assert_int_equal(ret, ATINY_MALLOC_FAILED);
}

