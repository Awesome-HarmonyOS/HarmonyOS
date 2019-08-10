#ifndef TEST_AGENTTINY_H_
#define TEST_AGENTTINY_H_

#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include "los_base.h"
#include "los_task.ph"
#include "los_typedef.h"
#include "los_sys.h"
#include "atiny_lwm2m/agenttiny.h"
#include "osdepends/atiny_osdep.h"
#include <cmockery.h>
//#include "regresstest.h"
#include "atiny_lwm2m/agenttiny.h"
#include "liblwm2m.h"
#include "object_comm.h"

#define TEST_LOG    printf("Atiny_Test_Log : %s : %d \n",__FUNCTION__, __LINE__)

//#define TEST_LWM2M_SERVER_IP					"192.168.1.102"     //leshan server
//#define TEST_LWM2M_SERVER_DTLS_IP               "192.168.1.102"
#define TEST_LWM2M_SERVER_IP					"180.101.147.115"	// dianxin
#define TEST_LWM2M_SERVER_DTLS_IP				"180.101.147.115"
#define TEST_LWM2M_SERVER_IP_INVALID			"not server ip"

#define TEST_LWM2M_SERVER_PORT                  "5683"
#define TEST_LWM2M_SERVER_DTLS_PORT				"5684"
#define TEST_LWM2M_SERVER_DTLS_PORT_INVALID 	"not a port"

#define TEST_LWM2M_SERVER_PSK_ID				"6666dtls"
#define TEST_LWM2M_SERVER_PSK_ID_INVALID		"not a psk id"

#define TEST_LWM2M_SERVER_PSK_INVALID			"not a psk"
#define DEFAULT_EP_NAME							"6666wifi"
#define DEFAULT_EP_NAME_S						"6666dtls"
#define DEFAULT_MFT								"lwm2mFota"
#define DEFAULT_DEV_TYPE						"lwm2mFota"

extern void *test_phandle;
extern char TEST_LWM2M_SERVER_PSK[16];
extern char otherPSK[4];
extern int is_reg_ok();
extern int It_Los_Atiny_Init(void);
extern int It_Los_Atiny_Bind(void);
extern int It_Los_Atiny_Data_Report(void);
typedef struct
{
    lwm2m_context_t  *lwm2m_context;
    atiny_param_t     atiny_params;
    client_data_t     client_data;
    lwm2m_object_t   *obj_array[8];
    int atiny_quit;
    int reconnect_flag;
    void *quit_sem;
    int reboot_flag;
    uint8_t *recv_buffer;
} handle_data_t;


//const UnitTest tests;
extern int test_agenttyiny(void);

/********* atiny_init **********/
extern void It_Los_Atiny_Init_000(void **state);
extern void It_Los_Atiny_Init_001(void **state);
extern void It_Los_Atiny_Init_002(void **state);
extern void It_Los_Atiny_Init_003(void **state);
extern void It_Los_Atiny_Init_004(void **state);
extern void It_Los_Atiny_Init_005(void **state);
extern void It_Los_Atiny_Init_006(void **state);
extern void It_Los_Atiny_Init_007(void **state);
extern void It_Los_Atiny_Init_008(void **state);
extern void It_Los_Atiny_Init_009(void **state);
extern void It_Los_Atiny_Init_010(void **state);
extern void It_Los_Atiny_Init_011(void **state);
extern void It_Los_Atiny_Init_012(void **state);
extern void It_Los_Atiny_Init_013(void **state);
extern void It_Los_Atiny_Init_014(void **state);
extern void It_Los_Atiny_Init_015(void **state);
extern void It_Los_Atiny_Init_016(void **state);
extern void It_Los_Atiny_Init_017(void **state);
extern void It_Los_Atiny_Init_018(void **state);
extern void It_Los_Atiny_Init_019(void **state);
extern void It_Los_Atiny_Init_020(void **state);
extern void It_Los_Atiny_Init_021(void **state);
extern void It_Los_Atiny_Init_022(void **state);
extern void It_Los_Atiny_Init_023(void **state);
extern void It_Los_Atiny_Init_024(void **state);
extern void It_Los_Atiny_Init_025(void **state);
extern void It_Los_Atiny_Init_026(void **state);
extern void It_Los_Atiny_Init_027(void **state);
extern void It_Los_Atiny_Init_028(void **state);
extern void It_Los_Atiny_Init_029(void **state);
extern void It_Los_Atiny_Init_030(void **state);
extern void It_Los_Atiny_Init_031(void **state);
extern void It_Los_Atiny_Init_032(void **state);
extern void It_Los_Atiny_Init_033(void **state);
extern void It_Los_Atiny_Init_034(void **state);
extern void It_Los_Atiny_Init_035(void **state);
extern void It_Los_Atiny_Init_036(void **state);
extern void It_Los_Atiny_Init_037(void **state);

/********* atiny_bind **********/
extern void It_Los_Atiny_Bind_000(void **state);
extern void It_Los_Atiny_Bind_001(void **state);
extern void It_Los_Atiny_Bind_002(void **state);
extern void It_Los_Atiny_Bind_003(void **state);
extern void It_Los_Atiny_Bind_004(void **state);
extern void It_Los_Atiny_Bind_005(void **state);
extern void It_Los_Atiny_Bind_006(void **state);
extern void It_Los_Atiny_Bind_007(void **state);
extern void It_Los_Atiny_Bind_008(void **state);
extern void It_Los_Atiny_Bind_009(void **state);
extern void It_Los_Atiny_Bind_010(void **state);
extern void It_Los_Atiny_Bind_011(void **state);
extern void It_Los_Atiny_Bind_012(void **state);
extern void It_Los_Atiny_Bind_013(void **state);
extern void It_Los_Atiny_Bind_014(void **state);
extern void It_Los_Atiny_Bind_015(void **state);
extern void It_Los_Atiny_Bind_016(void **state);
extern void It_Los_Atiny_Bind_017(void **state);
extern void It_Los_Atiny_Bind_018(void **state);
extern void It_Los_Atiny_Bind_019(void **state);
extern void It_Los_Atiny_Bind_020(void **state);
extern void It_Los_Atiny_Bind_021(void **state);
extern void It_Los_Atiny_Bind_022(void **state);
extern void It_Los_Atiny_Bind_023(void **state);
extern void It_Los_Atiny_Bind_024(void **state);
extern void It_Los_Atiny_Bind_025(void **state);
extern void It_Los_Atiny_Bind_026(void **state);

extern void It_Los_Atiny_Bind_030(void **state);
extern void It_Los_Atiny_Bind_031(void **state);
extern void It_Los_Atiny_Bind_032(void **state);
extern void It_Los_Atiny_Bind_033(void **state);
extern void It_Los_Atiny_Bind_034(void **state);
extern void It_Los_Atiny_Bind_035(void **state);
extern void It_Los_Atiny_Bind_036(void **state);
extern void It_Los_Atiny_Bind_037(void **state);
extern void It_Los_Atiny_Bind_038(void **state);
extern void It_Los_Atiny_Bind_039(void **state);
extern void It_Los_Atiny_Bind_040(void **state);
extern void It_Los_Atiny_Bind_041(void **state);
extern void It_Los_Atiny_Bind_042(void **state);
extern void It_Los_Atiny_Bind_043(void **state);
extern void It_Los_Atiny_Bind_044(void **state);

extern void It_Los_Atiny_Data_Report_000(void **state);
extern void It_Los_Atiny_Data_Report_001(void **state);
extern void It_Los_Atiny_Data_Report_002(void **state);
extern void It_Los_Atiny_Data_Report_003(void **state);
extern void It_Los_Atiny_Data_Report_004(void **state);
extern void It_Los_Atiny_Data_Report_005(void **state);
extern void It_Los_Atiny_Data_Report_006(void **state);

#endif

