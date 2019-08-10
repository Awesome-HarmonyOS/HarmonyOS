/**
 * \file error.h
 *
 * \brief Error to string translation
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
#ifndef MBEDTLS_ERROR_H
#define MBEDTLS_ERROR_H

#include <stddef.h>

/**
 * Error code layout.
 *
 * Currently we try to keep all error codes within the negative space of 16
 * bits signed integers to support all platforms (-0x0001 - -0x7FFF). In
 * addition we'd like to give two layers of information on the error if
 * possible.
 *
 * For that purpose the error codes are segmented in the following manner:
 *
 * 16 bit error code bit-segmentation
 *
 * 1 bit  - Unused (sign bit)
 * 3 bits - High level module ID
 * 5 bits - Module-dependent error code
 * 7 bits - Low level module errors
 *
 * For historical reasons, low-level error codes are divided in even and odd,
 * even codes were assigned first, and -1 is reserved for other errors.
 *
 * Low-level module errors (0x0002-0x007E, 0x0003-0x007F)
 *
 * Module   Nr  Codes assigned
 * MPI       7  0x0002-0x0010
 * GCM       2  0x0012-0x0014
 * BLOWFISH  2  0x0016-0x0018
 * THREADING 3  0x001A-0x001E
 * AES       2  0x0020-0x0022
 * CAMELLIA  2  0x0024-0x0026
 * XTEA      1  0x0028-0x0028
 * BASE64    2  0x002A-0x002C
 * OID       1  0x002E-0x002E   0x000B-0x000B
 * PADLOCK   1  0x0030-0x0030
 * DES       1  0x0032-0x0032
 * CTR_DBRG  4  0x0034-0x003A
 * ENTROPY   3  0x003C-0x0040   0x003D-0x003F
 * NET      11  0x0042-0x0052   0x0043-0x0045
 * ASN1      7  0x0060-0x006C
 * PBKDF2    1  0x007C-0x007C
 * HMAC_DRBG 4  0x0003-0x0009
 * CCM       2                  0x000D-0x000F
 *
 * High-level module nr (3 bits - 0x0...-0x7...)
 * Name      ID  Nr of Errors
 * PEM       1   9
 * PKCS#12   1   4 (Started from top)
 * X509      2   20
 * PKCS5     2   4 (Started from top)
 * DHM       3   9
 * PK        3   14 (Started from top)
 * RSA       4   9
 * ECP       4   8 (Started from top)
 * MD        5   4
 * CIPHER    6   6
 * SSL       6   17 (Started from top)
 * SSL       7   31
 *
 * Module dependent error code (5 bits 0x.00.-0x.F8.)
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Translate a mbed TLS error code into a string representation,
 *        Result is truncated if necessary and always includes a terminating
 *        null byte.
 *
 * \param errnum    error code
 * \param buffer    buffer to place representation in
 * \param buflen    length of the buffer
 */
void mbedtls_strerror( int errnum, char *buffer, size_t buflen );

#ifdef __cplusplus
}
#endif

#endif /* error.h */
