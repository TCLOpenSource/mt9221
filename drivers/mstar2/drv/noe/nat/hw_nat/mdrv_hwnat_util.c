/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2019 MediaTek Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @file   MDRV_HWNAT_UTIL.h
/// @brief  NOE Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/in.h>
#include <linux/ipv6.h>
#include "mdrv_hwnat_foe.h"
#include "mdrv_hwnat_util.h"

void MacReverse(uint8_t * Mac)
{
    uint8_t tmp;
    uint8_t i;

    for (i = 5; i > 2; i--) {
        tmp = Mac[i];
        Mac[i] = Mac[5 - i];
        Mac[5 - i] = tmp;
    }
}

static int _GetNext(char *src, int separator, char *dest)
{
    char *c;
    int len = 0;

    if ((src == NULL) || (dest == NULL)) {
        return -1;
    }

    c = strchr(src, separator);
    if (c == NULL) {
        strncpy(dest, src, len);
        return -1;
    }
    len = c - src;
    strncpy(dest, src, len);
    dest[len] = '\0';
    return len + 1;
}

static inline int atoi(char *s)
{
    int i = 0;
    while (isdigit(*s)) {
        i = i * 10 + *(s++) - '0';
    }
    return i;
}



unsigned int MDrv_HWNAT_Str2Ip(char *str)
{
    int len;
    char *ptr = str;
    char buf[128];
    unsigned char c[4];
    int i;
    for (i = 0; i < 3; ++i) {
        if ((len = _GetNext(ptr, '.', buf)) == -1) {
            return 1;   /* parsing error */
        }
        c[i] = atoi(buf);
        ptr += len;
    }
    c[3] = atoi(ptr);
    return ((c[0] << 24) + (c[1] << 16) + (c[2] << 8) + c[3]);
}

void reg_modify_bits(unsigned int *Addr, uint32_t Data, uint32_t Offset, uint32_t Len)
{
#if 0
    unsigned int mask = 0;
    unsigned int value;
    unsigned int i;

    for (i = 0; i < len; i++) {
        mask |= 1 << (offset + i);
    }

    value = sysregread(addr);
    value &= ~mask;
    value |= (data << offset) & mask;;
    sysregwrite(addr, value);
#endif
}
static inline uint16_t CsumPart(uint32_t o, uint32_t n, uint16_t old)
{
    uint32_t d[] = { o, n };
    return csum_fold(csum_partial((char *)d, sizeof(d), old ^ 0xFFFF));
}

/*
 * KeepAlive with new header mode will pass the modified packet to cpu.
 * We must change to original packet to refresh NAT table.
 */

/*
 * Recover TCP Src/Dst Port and recalculate tcp checksum
 */
void foe_to_org_tcphdr(struct foe_entry *entry, struct iphdr *iph,
            struct tcphdr *th)
{
    /* TODO: how to recovery 6rd/dslite packet */
    th->check =
        CsumPart((th->source) ^ 0xffff,
             htons(entry->ipv4_hnapt.sport), th->check);
    th->check =
        CsumPart((th->dest) ^ 0xffff,
             htons(entry->ipv4_hnapt.dport), th->check);
    th->check =
        CsumPart(~(iph->saddr), htonl(entry->ipv4_hnapt.sip),
             th->check);
    th->check =
        CsumPart(~(iph->daddr), htonl(entry->ipv4_hnapt.dip),
             th->check);
    th->source = htons(entry->ipv4_hnapt.sport);
    th->dest = htons(entry->ipv4_hnapt.dport);
}

/*
 * Recover UDP Src/Dst Port and recalculate udp checksum
 */
void
foe_to_org_udphdr(struct foe_entry *entry, struct iphdr *iph,
            struct udphdr *uh)
{
    /* TODO: how to recovery 6rd/dslite packet */

    uh->check =
        CsumPart((uh->source) ^ 0xffff,
             htons(entry->ipv4_hnapt.sport), uh->check);
    uh->check =
        CsumPart((uh->dest) ^ 0xffff,
             htons(entry->ipv4_hnapt.dport), uh->check);
    uh->check =
        CsumPart(~(iph->saddr), htonl(entry->ipv4_hnapt.sip),
             uh->check);
    uh->check =
        CsumPart(~(iph->daddr), htonl(entry->ipv4_hnapt.dip),
             uh->check);
    uh->source = htons(entry->ipv4_hnapt.sport);
    uh->dest = htons(entry->ipv4_hnapt.dport);
}

/*
 * Recover Src/Dst IP and recalculate ip checksum
 */
void foe_to_org_iphdr(struct foe_entry *entry, struct iphdr *iph)
{
    /* TODO: how to recovery 6rd/dslite packet */
    iph->saddr = htonl(entry->ipv4_hnapt.sip);
    iph->daddr = htonl(entry->ipv4_hnapt.dip);
    iph->check = 0;
    iph->check = ip_fast_csum((unsigned char *)(iph), iph->ihl);
}

void hwnat_memcpy(void *dest, void *src, u32 n)
{
    memcpy(dest, src, n);
}










/* Convert IP address from Hex to string */
uint8_t *MDrv_HWNAT_Util_Ip_To_Str(uint32_t Ip)
{
    static uint8_t Buf[32];
    uint8_t *ptr = (char *)&Ip;
    uint8_t c[4];

    c[0] = *(ptr);
    c[1] = *(ptr + 1);
    c[2] = *(ptr + 2);
    c[3] = *(ptr + 3);
    sprintf(Buf, "%d.%d.%d.%d", c[3], c[2], c[1], c[0]);
    return Buf;
}


unsigned int MDrv_HWNAT_Util_Str_To_Ip(char *str)
{
    int len;
    char *ptr = str;
    char buf[128];
    unsigned char c[4];
    int i;
    for (i = 0; i < 3; ++i) {
        if ((len = _GetNext(ptr, '.', buf)) == -1) {
            return 1;   /* parsing error */
        }
        c[i] = atoi(buf);
        ptr += len;
    }
    c[3] = atoi(ptr);
    return ((c[0] << 24) + (c[1] << 16) + (c[2] << 8) + c[3]);
}

void MDrv_HWNAT_Util_Calc_Tcphdr(struct foe_entry *entry, struct iphdr *iph, struct tcphdr *th)
{
    /* TODO: how to recovery 6rd/dslite packet */
    th->check = CsumPart((th->source) ^ 0xffff, htons(entry->ipv4_hnapt.sport), th->check);
    th->check = CsumPart((th->dest) ^ 0xffff, htons(entry->ipv4_hnapt.dport), th->check);
    th->check = CsumPart(~(iph->saddr), htonl(entry->ipv4_hnapt.sip), th->check);
    th->check = CsumPart(~(iph->daddr), htonl(entry->ipv4_hnapt.dip), th->check);
    th->source = htons(entry->ipv4_hnapt.sport);
    th->dest = htons(entry->ipv4_hnapt.dport);

}

void MDrv_HWNAT_Util_Calc_Udphdr(struct foe_entry *entry, struct iphdr *iph, struct udphdr *uh)
{
    /* TODO: how to recovery 6rd/dslite packet */
    uh->check = CsumPart((uh->source) ^ 0xffff, htons(entry->ipv4_hnapt.sport), uh->check);
    uh->check = CsumPart((uh->dest) ^ 0xffff, htons(entry->ipv4_hnapt.dport), uh->check);
    uh->check = CsumPart(~(iph->saddr), htonl(entry->ipv4_hnapt.sip), uh->check);
    uh->check = CsumPart(~(iph->daddr), htonl(entry->ipv4_hnapt.dip), uh->check);
    uh->source = htons(entry->ipv4_hnapt.sport);
    uh->dest = htons(entry->ipv4_hnapt.dport);
}

void MDrv_HWNAT_Util_Calc_Iphdr(struct foe_entry *entry, struct iphdr *iph)
{
    /* TODO: how to recovery 6rd/dslite packet */
    iph->saddr = htonl(entry->ipv4_hnapt.sip);
    iph->daddr = htonl(entry->ipv4_hnapt.dip);
    iph->check = 0;
    iph->check = ip_fast_csum((unsigned char *)(iph), iph->ihl);

}


void MDrv_HWNAT_Util_Memcpy(void *dest, void *src, u32 n)
{
#if 0
    ether_addr_copy(dest, src);
#else
    memcpy(dest, src, n);
#endif
}


