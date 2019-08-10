/*
 *  Temporary target-specific config.h for entropy collection
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */

#if defined(TARGET_LIKE_MBED)
#define MBEDTLS_NO_PLATFORM_ENTROPY
#undef MBEDTLS_HAVE_TIME_DATE
#undef MBEDTLS_FS_IO
#endif

/*
 * WARNING: this is temporary!
 * This should be in a separate yotta module which would be a target
 * dependency of mbedtls (see IOTSSL-313)
 */
#if defined(TARGET_LIKE_K64F)
#define MBEDTLS_ENTROPY_HARDWARE_ALT
#endif
