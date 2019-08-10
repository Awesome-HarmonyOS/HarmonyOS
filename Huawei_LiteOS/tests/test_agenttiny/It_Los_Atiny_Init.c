#include "test_agenttiny.h"

int It_Los_Atiny_Init()
{
    const UnitTest It_Los_Atiny_Init[] =
	{
		unit_test(It_Los_Atiny_Init_000),
	    unit_test(It_Los_Atiny_Init_001),
		unit_test(It_Los_Atiny_Init_002),
		unit_test(It_Los_Atiny_Init_003),
		unit_test(It_Los_Atiny_Init_004),
		unit_test(It_Los_Atiny_Init_005),
		unit_test(It_Los_Atiny_Init_006),
		unit_test(It_Los_Atiny_Init_007),
		unit_test(It_Los_Atiny_Init_008),
		unit_test(It_Los_Atiny_Init_009),
		unit_test(It_Los_Atiny_Init_010),
		unit_test(It_Los_Atiny_Init_011),
		unit_test(It_Los_Atiny_Init_012),
		unit_test(It_Los_Atiny_Init_013),
		unit_test(It_Los_Atiny_Init_014),
		unit_test(It_Los_Atiny_Init_015),
		unit_test(It_Los_Atiny_Init_016),
		unit_test(It_Los_Atiny_Init_017),
		unit_test(It_Los_Atiny_Init_018),
		unit_test(It_Los_Atiny_Init_019),
		unit_test(It_Los_Atiny_Init_020),
		unit_test(It_Los_Atiny_Init_021),
		unit_test(It_Los_Atiny_Init_022),
		unit_test(It_Los_Atiny_Init_023),
		unit_test(It_Los_Atiny_Init_024),
		unit_test(It_Los_Atiny_Init_025),
		unit_test(It_Los_Atiny_Init_026),
		unit_test(It_Los_Atiny_Init_027),
		unit_test(It_Los_Atiny_Init_028),
		unit_test(It_Los_Atiny_Init_029),
		unit_test(It_Los_Atiny_Init_030),
		unit_test(It_Los_Atiny_Init_031),
		unit_test(It_Los_Atiny_Init_032),
		unit_test(It_Los_Atiny_Init_033),
		unit_test(It_Los_Atiny_Init_034),
		unit_test(It_Los_Atiny_Init_035),
		unit_test(It_Los_Atiny_Init_036),
		unit_test(It_Los_Atiny_Init_037),
	};
    
    run_tests(It_Los_Atiny_Init);
    
    return 0;
}

void It_Los_Atiny_Init_000(void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));

	int ret = 0;

	ret = atiny_init(NULL, NULL);
	assert_int_equal(ret, ATINY_ARG_INVALID);

	ret = atiny_init(&uwAtiny_params, NULL);
	assert_int_equal(ret, ATINY_ARG_INVALID);

	ret = atiny_init(NULL, &phandle);
	assert_int_equal(ret, ATINY_ARG_INVALID);

}

void It_Los_Atiny_Init_001 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"Q";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = sizeof(TEST_LWM2M_SERVER_PSK);
	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);
    
    atiny_destroy(phandle);

	atiny_deinit(phandle);
}

void It_Los_Atiny_Init_002(void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"Q";
	uwAtiny_params.server_params.life_time = 2147483647;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 65535;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 0;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);
    
    atiny_destroy(phandle);

	atiny_deinit(phandle);
}

void It_Los_Atiny_Init_003 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"S";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 1024;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 0;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);
    
    atiny_destroy(phandle);

	atiny_deinit(phandle);

}

void It_Los_Atiny_Init_004 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 4294967295;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_CLIENT_INITIATED;
	uwAtiny_params.server_params.hold_off_time = 2147483647;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 16;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 32;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
	atiny_deinit(phandle);
}

void It_Los_Atiny_Init_005 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQs";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 4294967295;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_CLIENT_INITIATED;
	uwAtiny_params.server_params.hold_off_time = 2147483647;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 32;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 16;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
  	atiny_deinit(phandle);
}

void It_Los_Atiny_Init_006 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"ELSE";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 16;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 1024;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
  	atiny_deinit(phandle);
}

void It_Los_Atiny_Init_007 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"Q";
	uwAtiny_params.server_params.life_time = 2147483647;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 16;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 65535;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_008 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"S";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_CLIENT_INITIATED;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 0;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 0;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_009 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 0;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 0;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_010 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"SQ";
	uwAtiny_params.server_params.life_time = 2147483647;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 1024;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 65535;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_011 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"S";
	uwAtiny_params.server_params.life_time = 2147483647;
	uwAtiny_params.server_params.storing_cnt = 4294967295;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 32;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 65535;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_012 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"ELSE";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 2147483647;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 1024;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 32;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_013 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"Q";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 4294967295;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_CLIENT_INITIATED;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 32;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 1024;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_014 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"S";
	uwAtiny_params.server_params.life_time = 2147483647;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 2147483647;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 65535;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 32;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_015 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"Q";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 4294967295;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 2147483647;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 0;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 32;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_016 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"ELSE";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 4294967295;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_CLIENT_INITIATED;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 65535;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 65535;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_017 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"ELSE";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 0;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 16;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_018 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"ELSE";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 32;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 0;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_019 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"U";
	uwAtiny_params.server_params.life_time = 2147483647;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_CLIENT_INITIATED;
	uwAtiny_params.server_params.hold_off_time = 2147483647;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 0;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 65535;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_020 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"U";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 32;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 32;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_021 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"S";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 2147483647;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 16;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 1024;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}


void It_Los_Atiny_Init_022 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"SQ";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 4294967295;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_CLIENT_INITIATED;
	uwAtiny_params.server_params.hold_off_time = 2147483647;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 16;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 0;

	ret = atiny_init(&uwAtiny_params, &phandle);	
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_023 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"U";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 4294967295;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 1024;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 1024;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_024 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 2147483647;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 65535;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 1024;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_025 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"SQ";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 0;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 1024;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_026 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQs";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 65535;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 1024;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_027 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"SQ";
	uwAtiny_params.server_params.life_time = 5000;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 65535;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 16;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_028 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"U";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 4294967295;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 16;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 16;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_029 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"SQ";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 32;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 32;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_030 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQs";
	uwAtiny_params.server_params.life_time = 2147483647;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_CLIENT_INITIATED;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 1024;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 65535;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_031 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQs";
	uwAtiny_params.server_params.life_time = 2147483647;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 0;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 32;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_032 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 0;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 1024;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 16;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_033 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"U";
	uwAtiny_params.server_params.life_time = 2147483647;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 65535;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 0;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_034 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQ";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_SEQUENCE;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[0].psk_len = 32;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 65535;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_035 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"Q";
	uwAtiny_params.server_params.life_time = 50000;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 1024;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 16;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_036 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQs";
	uwAtiny_params.server_params.life_time = -2147483648;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
	uwAtiny_params.server_params.hold_off_time = 10;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 1024;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK_INVALID;
	uwAtiny_params.security_params[1].psk_len = 0;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

void It_Los_Atiny_Init_037 (void **state)
{
	void *phandle;
	atiny_param_t uwAtiny_params;
	memset(&uwAtiny_params, 0, sizeof(atiny_param_t));
	int ret = 0;

	uwAtiny_params.server_params.binding = (char *)"UQs";
	uwAtiny_params.server_params.life_time = 2147483647;
	uwAtiny_params.server_params.storing_cnt = 1024;

	uwAtiny_params.server_params.bootstrap_mode = BOOTSTRAP_CLIENT_INITIATED;
	uwAtiny_params.server_params.hold_off_time = -2147483648;

	uwAtiny_params.security_params[0].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[0].server_port = TEST_LWM2M_SERVER_DTLS_PORT_INVALID;
	uwAtiny_params.security_params[0].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[0].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[0].psk_len = 16;

	uwAtiny_params.security_params[1].server_ip = TEST_LWM2M_SERVER_IP_INVALID;
	uwAtiny_params.security_params[1].server_port = TEST_LWM2M_SERVER_DTLS_PORT;
	uwAtiny_params.security_params[1].psk_Id = TEST_LWM2M_SERVER_PSK_ID_INVALID;
	uwAtiny_params.security_params[1].psk = TEST_LWM2M_SERVER_PSK;
	uwAtiny_params.security_params[1].psk_len = 0;

	ret = atiny_init(&uwAtiny_params, &phandle);
	assert_int_equal(ret, ATINY_OK);

    atiny_destroy(phandle);
    atiny_deinit(phandle);
}

