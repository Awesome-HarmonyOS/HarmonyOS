/*
 * Copyright 2008 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 #ifndef __SOTA_TEST_H_
 #define __SOTA_TEST_H_
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include "../cmockery/src/google/cmockery.h"
#include "at_device/bc95.h"
#include "ota_port.h"
#include "sota/sota.h"
#include "../../components/ota/sota/sota_hal.h"
#include "ota/package.h"
#include "../../include/at_frame/at_main.h"
#include "los_swtmr.h"

#define VER_LEN  16
#define DEVICE_VER "V0.0"


static sota_arg_s * g_flash_op_t;



#define htons_ota(x) ((((x) & 0x00ff) << 8) | (((x) & 0xff00) >> 8))
#define PCP_HEAD 0xFFFE
#define BLOCK_HEAD 3


extern int nb_send_str(const char* buf, int len);

void * arg = NULL;

extern int read_ver(char* buf, uint32_t len);
extern int set_ver(const char* buf, uint32_t len);
extern int sota_log(const char *fmt, ...);

int sota_test_main(void);



#endif
