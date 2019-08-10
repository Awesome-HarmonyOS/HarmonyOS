/**
 * @file
 * Dummy SNMPv3 functions.
 */

/*
 * Copyright (c) 2016 Elias Oenal.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * Author: Elias Oenal <lwip@eliasoenal.com>
 *         Dirk Ziegelmeier <dirk@ziegelmeier.net>
 */

#include "lwip/apps/snmpv3.h"
#include "snmpv3_priv.h"
#include <string.h>
#include "lwip/err.h"

#if LWIP_SNMP && LWIP_SNMP_V3

/**
 *  @param username is a pointer to a string.
 * @param auth_algo is a pointer to u8_t. The implementation has to set this if user was found.
 * @param auth_key is a pointer to a pointer to a string. Implementation has to set this if user was found.
 * @param priv_algo is a pointer to u8_t. The implementation has to set this if user was found.
 * @param priv_key is a pointer to a pointer to a string. Implementation has to set this if user was found.
 */
err_t
snmpv3_get_user(const char* username, u8_t *auth_algo, u8_t *auth_key, u8_t *priv_algo, u8_t *priv_key)
{
  const char* engine_id;
  u8_t engine_id_len;
  
  if(strlen(username) == 0) {
    return ERR_OK;
  }
  
  if(memcmp(username, "lwip", 4) != 0) {
    return ERR_VAL;
  }
  
  snmpv3_get_engine_id(&engine_id, &engine_id_len);
  
  if(auth_key != NULL) {
    snmpv3_password_to_key_sha((const u8_t*)"maplesyrup", 10,
      (const u8_t*)engine_id, engine_id_len,
      auth_key);
    *auth_algo = SNMP_V3_AUTH_ALGO_SHA;
  }
  if(priv_key != NULL) {
    snmpv3_password_to_key_sha((const u8_t*)"maplesyrup", 10,
      (const u8_t*)engine_id, engine_id_len,
      priv_key);
    *priv_algo = SNMP_V3_PRIV_ALGO_DES;
  }
  return ERR_OK;
}

/**
 * Get engine ID from persistence
 * @param id
 * @param len
 */
void
snmpv3_get_engine_id(const char **id, u8_t *len)
{
  *id = "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02";
  *len = 12;
}

/**
 * Store engine ID in persistence
 * @param id
 * @param len
 */
err_t
snmpv3_set_engine_id(const char *id, u8_t len)
{
  LWIP_UNUSED_ARG(id);
  LWIP_UNUSED_ARG(len);
  return ERR_OK;
}

/**
 * Get engine boots from persistence. Must be increased on each boot.
 * @return 
 */
u32_t
snmpv3_get_engine_boots(void)
{
  return 0;
}

/**
 * Store engine boots in persistence
 * @param boots
 */
void 
snmpv3_set_engine_boots(u32_t boots)
{
  LWIP_UNUSED_ARG(boots);
}

/**
 * RFC3414 2.2.2.
 * Once the timer reaches 2147483647 it gets reset to zero and the
 * engine boot ups get incremented.
 */
u32_t
snmpv3_get_engine_time(void)
{
  return 0;
}

/**
 * Reset current engine time to 0
 */
void
snmpv3_reset_engine_time(void)
{
}

#endif /* LWIP_SNMP && LWIP_SNMP_V3 */
