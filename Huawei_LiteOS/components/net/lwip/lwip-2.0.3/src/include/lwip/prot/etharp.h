/**
 * @file
 * ARP protocol definitions
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
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
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef LWIP_HDR_PROT_ETHARP_H
#define LWIP_HDR_PROT_ETHARP_H

#include "lwip/arch.h"
#include "lwip/prot/ethernet.h"
#include "lwip/ip4_addr.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ETHARP_HWADDR_LEN
#define ETHARP_HWADDR_LEN     ETH_HWADDR_LEN
#endif

#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
/** the ARP message, see RFC 826 ("Packet format") */
struct etharp_hdr {
  PACK_STRUCT_FIELD(u16_t hwtype);
  PACK_STRUCT_FIELD(u16_t proto);
  PACK_STRUCT_FLD_8(u8_t  hwlen);
  PACK_STRUCT_FLD_8(u8_t  protolen);
  PACK_STRUCT_FIELD(u16_t opcode);
  PACK_STRUCT_FLD_S(struct eth_addr shwaddr);
  PACK_STRUCT_FLD_S(struct ip4_addr2 sipaddr);
  PACK_STRUCT_FLD_S(struct eth_addr dhwaddr);
  PACK_STRUCT_FLD_S(struct ip4_addr2 dipaddr);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

#define SIZEOF_ETHARP_HDR 28

/* ARP hwtype values */
enum etharp_hwtype {
  HWTYPE_ETHERNET = 1
  /* others not used */
};

/* ARP message types (opcodes) */
enum etharp_opcode {
  ARP_REQUEST = 1,
  ARP_REPLY   = 2
};

#ifdef __cplusplus
}
#endif

#endif /* LWIP_HDR_PROT_ETHARP_H */
