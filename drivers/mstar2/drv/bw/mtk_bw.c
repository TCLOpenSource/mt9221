// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019-2020 MediaTek Inc.
 * Author: Max Tsai <Max-CH.Tsai@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/limits.h>

#include <linux/sysfs.h>
#include <linux/delay.h>
#include "mtk_bw.h"

#define MAX_CHANNEL (2)
#define BCLK (218844185) //(133 * 0x191B85) (0x19)
#define MPLL_T (9481) //2.3148 ns(432MHz) shift 12bit

static enum modulelist module_id = all;
static struct module_map mlist[] = {
	PORT_DEFINE(all),
	PORT_DEFINE(bist),
	PORT_DEFINE(cpu),
	PORT_DEFINE(gpu),
	PORT_DEFINE(misc),
	PORT_DEFINE(ai),
	PORT_DEFINE(audio),
	PORT_DEFINE(vdec),
	PORT_DEFINE(vdecr2),
	PORT_DEFINE(mvop),
	PORT_DEFINE(mfdec),
	PORT_DEFINE(vd),
	PORT_DEFINE(mfe),
	PORT_DEFINE(pq),
	PORT_DEFINE(od),
	PORT_DEFINE(memc),
	PORT_DEFINE(dip),
	PORT_DEFINE(gop),
	PORT_DEFINE(ge),
};

static void putbuf(char *buf, u32 *len, const char *fmt, ...)
{
        va_list args;

        va_start(args, fmt);
        *len += vscnprintf(buf+(*len), PAGE_SIZE-(*len), fmt, args);
        va_end(args);
}

static void *find_device(u8 type, u32 id)
{
        char dev_name[32];
	struct device_node *np;
	struct platform_device *pdev;
	struct device *dev;
	struct mtk_miu *pdata = NULL;

        if (type == MIU_DIG) {
		snprintf(dev_name, 31, "dig%d", id);
        } else if (type == MIU_DIG_E) {
		snprintf(dev_name, 31, "dige%d", id);
        } else if (type == MIU_ATOP) {
		snprintf(dev_name, 31, "atop%d", id);
        } else if (type == MIU_CTRL_ARB) {
		snprintf(dev_name, 31, "ctrl_arb%d", id);
        } else if (type == MIU_BLK_ARB) {
		snprintf(dev_name, 31, "blk_arb%d", id);
	} else {
		return NULL;
	}

	np = of_find_node_by_name(NULL, dev_name);
	if (!np)
		return NULL;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return NULL;

	dev = &pdev->dev;
	pdata = dev_get_drvdata(dev);

	return pdata;
}

static int mtk_miu_get_latency(u16 ctrlarb, u16 blkarb)
{
	unsigned int ret = 0;
	struct mtk_miu *miu_barb = find_device(MIU_BLK_ARB, ctrlarb);

	if (!miu_barb)
		return ret;

	if (!(readb_relaxed(miu_barb->base + 0x90) & 0x1))
		return ret;

	if (readb_relaxed(miu_barb->base + 0x91) != blkarb)
		return ret;

	return readw_relaxed(miu_barb->base + 0x94);
}

static int mtk_miu_get_max_service_number(u16 ctrlarb, u16 blkarb)
{
	int ret = -1;
	unsigned int ctrloffset = 0;
	unsigned int blkmaxsel = 0;
	unsigned int ipmaxsel = 0;
	struct mtk_miu *miu_barb = find_device(MIU_BLK_ARB, ctrlarb);
	struct mtk_miu *miu_carb = find_device(MIU_CTRL_ARB, 0);

	if (!miu_carb)
		return ret;

	if (!miu_barb) {
		if (((ctrlarb << 4) | blkarb) == 0xF0) {
			ret = readb_relaxed(miu_carb->base + 0x54);
		}else if (((ctrlarb << 4) | blkarb) == 0xF1) {
			ret = readb_relaxed(miu_carb->base + 0x55);
		}else if (((ctrlarb << 4) | blkarb) == 0xF2) {
			ret = readb_relaxed(miu_carb->base + 0x58);
		}else if (((ctrlarb << 4) | blkarb) == 0xF3) {
			ret = readb_relaxed(miu_carb->base + 0x59);
		} else {
			ctrloffset = (((ctrlarb / 2) * 0x4) + (ctrlarb % 2));
			ret = readb_relaxed(miu_carb->base + 0x40 + ctrloffset);
		}
		return ret;
	}

	if (readw_relaxed(miu_barb->base + 0x38) & (1 << blkarb))
		return ret;

	blkmaxsel = ((readw_relaxed(miu_barb->base + 0x48) & 0xFFFF) |
		((readw_relaxed(miu_barb->base + 0x4C) << 16) & 0xFFFF0000));

	ipmaxsel = ((blkmaxsel >> (blkarb * 2)) & 0x3);

	return readb_relaxed(miu_barb->base + 0x40 +
			(ipmaxsel / 2) * 4 + (ipmaxsel % 2));
}

static int mtk_miu_get_priority(u16 ctrlarb, u16 blkarb)
{
	int ret = -1;
	u16 prio = 0;
	struct mtk_miu *miu_barb = find_device(MIU_BLK_ARB, ctrlarb);
	struct mtk_miu *miu_carb = find_device(MIU_CTRL_ARB, 0);

	if (!miu_carb)
		return ret;

	//CPU,GPU
	if (ctrlarb == 0xF)
		ctrlarb = 10 + blkarb;

	if (!(readw_relaxed(miu_carb->base + 0x1A0) & (1 << ctrlarb))) {
		prio =
		readw_relaxed(miu_carb->base + 0x60 + (ctrlarb / 8) * 0x4);
		ret = ((prio >> ((ctrlarb % 8) << 1)) & 0x3);
		return ret;
	}

	if (!miu_barb)
		return ret;

	if (!(readw_relaxed(miu_barb->base + 0x1BC) & (1 << blkarb))) {
		if ((readw_relaxed(miu_barb->base + 0x70) & (1 << blkarb))) {
			return ret;
		}
		prio =
		readw_relaxed(miu_barb->base + 0x30 + (blkarb / 8) * 0x4);
		ret = ((prio >> ((blkarb % 8) << 1)) & 0x3);
	}

	return ret;
}

static unsigned int mtk_miu_get_hpmask(u16 ctrlarb, u16 blkarb)
{
	unsigned int ret = 0;
	struct mtk_miu *miu_barb = find_device(MIU_BLK_ARB, ctrlarb);
	struct mtk_miu *miu_carb = find_device(MIU_CTRL_ARB, 0);

	if (!miu_carb)
		return ret;

	ret = (readw_relaxed(miu_carb->base + 0x1D0) & (1 << ctrlarb)) ? 1 : 0;

	if (!miu_barb)
		return ret ? 0 : 1;

	ret |= (readw_relaxed(miu_barb->base + 0x10) & (1 << blkarb)) ? 1 : 0;

	return ret ? 0 : 1;
}

static unsigned int mtk_miu_get_mask(u16 ctrlarb, u16 blkarb)
{
	unsigned int ret = 0;
	struct mtk_miu *miu_barb = find_device(MIU_BLK_ARB, ctrlarb);
	struct mtk_miu *miu_carb = find_device(MIU_CTRL_ARB, 0);

	if (!miu_carb)
		return 0;

	ret = (readw_relaxed(miu_carb->base + 0x1CC) & (1 << ctrlarb)) ? 1 : 0;

	if (!miu_barb)
		return ret;

	ret |= (readw_relaxed(miu_barb->base + 0xC) & (1 << blkarb)) ? 1 : 0;

	return ret;
}

static int mtk_miu_start_mon(void)
{
	struct mtk_miu *miu_dige = find_device(MIU_DIG_E, 0);

	if (!miu_dige) {
                pr_err("Get miu_dige device fail\n");
		return -ENXIO;
	}
	bw_setb(0, true, miu_dige->base + 0x1C8);
	bw_setb(11, true, miu_dige->base + 0x1CC);
	bw_setb(11, false, miu_dige->base + 0x1CC);

	return 0;
}

static int mtk_miu_reset_mon(void)
{
	int ret = 0;
	struct mtk_miu *miu_dige = find_device(MIU_DIG_E, 0);

	if (!miu_dige) {
                pr_err("Get miu_dige device fail\n");
		return -ENXIO;
	}

	bw_setb(0, false, miu_dige->base + 0x1C8);
	bw_setb(0, true, miu_dige->base + 0x1C8);
	return ret;
}

static int mtk_miu_set_mon_ch(enum miu_channel mon_ch)
{
	int ret = 0;
	struct mtk_miu *miu_dige = find_device(MIU_DIG_E, 0);

	if (!miu_dige) {
                pr_err("Get miu_dige device fail\n");
		return -ENXIO;
	}

	bw_setb(12, false, miu_dige->base + 0x1CC);
	bw_setb(13, false, miu_dige->base + 0x1CC);
	if (mon_ch == MIU_CH0)
		bw_setb(13, true, miu_dige->base + 0x1CC);
	else if(mon_ch == MIU_CH1)
		bw_setb(12, true, miu_dige->base + 0x1CC);
	else
		return -ENXIO;

	return ret;
}

static int mtk_miu_get_mon_ch(enum miu_channel *mon_ch)
{
	int ret = 0;
	u8 chan_value;
	struct mtk_miu *miu_dige = find_device(MIU_DIG_E, 0);

	if (!miu_dige) {
                pr_err("Get miu_dige device fail\n");
		return -ENXIO;
	}

	chan_value = ((readb_relaxed(miu_dige->base + 0x1CD) >> 4) & 0x3);
	if (chan_value == 0)
		*mon_ch = MIU_CH_ALL;
	else if (chan_value == 1)
		*mon_ch = MIU_CH1;
	else if (chan_value == 2)
		*mon_ch = MIU_CH0;
	else
		ret = -EFAULT;


	return ret;
}

static int mtk_miu_set_mon_id(struct mtk_miu *miu_dige, u64 client,
				u8 mon_idx, u8 client_idx)
{
	int ret = 0;
	void __iomem *base = miu_dige->base;

	writeb_relaxed(mon_idx * 8 + client_idx, base + 0x1C9);
	writeb_relaxed((u8)client, base + 0x1CC);
	bw_setb(15, true, base + 0x1C8);
	bw_setb(15, false, base + 0x1C8);

	return ret;
}

static int mtk_miu_set_mon_freq(u32 mon_freq)
{
	int ret = 0;
	struct mtk_miu *miu_dige = find_device(MIU_DIG_E, 0);
	u64 freq_value;

	if (mon_freq > 620) {
		pr_err("overbound MAX frequency 620 ms\n");
		return -EINVAL;
	} else if (mon_freq < 1) {
		pr_err("overbound MIN frequency 1 ms\n");
		return -EINVAL;
	}

	if (!miu_dige)
		return ret;

	freq_value = ((u64)mon_freq * 1000000) / MPLL_T;
	writew_relaxed((u16)freq_value, miu_dige->base + 0xFC);

	return ret;
}

static void mtk_miu_get_mon_freq(u32 *mon_freq)
{
	struct mtk_miu *miu_dige = find_device(MIU_DIG_E, 0);
	u64 freq_value;

	if (!miu_dige)
		return;

	freq_value = (u64)readw_relaxed(miu_dige->base + 0xFC);
	*mon_freq = (u32)(((freq_value * MPLL_T) / 1000000) + 1);
}

static int mtk_miu_set_mon(struct mtk_mon *mon)
{
	int ret = 0;
	u32 idx;
	void __iomem *base;
	struct mtk_miu *miu_dige = find_device(MIU_DIG_E, 0);

	if (!miu_dige) {
                pr_err("Get miu_dige device fail\n");
		ret = -ENXIO;
		goto RET_FUN;
	}
	base = miu_dige->base;

	if (mon->client_cnt > 8) {
		pr_err("client count more than 8\n");
		ret = -EINVAL;
		goto RET_FUN;
	}

	//freeze miu monitor
	bw_setb(11, true, base + 0x1CC);

	for (idx = 0; idx < mon->client_cnt; idx++)
		mtk_miu_set_mon_id(miu_dige, mon->client[idx], mon->mid, idx);

	//un-freeze miu monitor
	bw_setb(11, false, base + 0x1CC);

RET_FUN:
	return ret;
}

static int mtk_miu_get_mon(struct mtk_mon_res *mres, int mcnt)
{
	int idx;
	void __iomem *base;
	struct mtk_miu *miu_dige = find_device(MIU_DIG_E, 0);
	u32 rtmp1, rtmp2 = 0;

	if (!miu_dige) {
                pr_err("Get miu_dige device fail\n");
		return -ENXIO;
	}
	base = miu_dige->base;

	bw_setb(11, true, base + 0x1CC);

	writeb_relaxed(0x01, base + 0x1C8);
	for (idx = 0; idx < mcnt; idx++) {
		writeb_relaxed(0x01, base + 0x1C8);
		rtmp1 = readw_relaxed(base + 0x1D0 + (idx * 0x4));
		writeb_relaxed(0x11, base + 0x1C8);
		rtmp2 = readw_relaxed(base + 0x1D0 + (idx * 0x4));
		mres[idx].avg_ocpy =
		(u64)((rtmp1 & 0xFFFF) | ((rtmp2 & 0xFFFF) << 16));

		writeb_relaxed(0x21, base + 0x1C8);
		rtmp1 = readw_relaxed(base + 0x1D0 + (idx * 0x4));
		writeb_relaxed(0x31, base + 0x1C8);
		rtmp2 = readw_relaxed(base + 0x1D0 + (idx * 0x4));
		mres[idx].max_ocpy =
		(u64)((rtmp1 & 0xFFFF) | ((rtmp2 & 0xFFFF) << 16));

		writeb_relaxed(0x41, base + 0x1C8);
		rtmp1 = readw_relaxed(base + 0x1D0 + (idx * 0x4));
		writeb_relaxed(0x51, base + 0x1C8);
		rtmp2 = readw_relaxed(base + 0x1D0 + (idx * 0x4));
		mres[idx].avg_bw =
		(u64)((rtmp1 & 0xFFFF) | ((rtmp2 & 0xFFFF) << 16));

		writeb_relaxed(0x61, base + 0x1C8);
		rtmp1 = readw_relaxed(base + 0x1D0 + (idx * 0x4));
		writeb_relaxed(0x71, base + 0x1C8);
		rtmp2 = readw_relaxed(base + 0x1D0 + (idx * 0x4));
		mres[idx].max_bw =
		(u64)((rtmp1 & 0xFFFF) | ((rtmp2 & 0xFFFF) << 16));

		writeb_relaxed(0xD1, base + 0x1C8);
		rtmp1 = readw_relaxed(base + 0x1D0 + (idx * 0x4));
		writeb_relaxed(0xE1, base + 0x1C8);
		rtmp2 = readw_relaxed(base + 0x1D0 + (idx * 0x4));
		mres[idx].time =
		(u64)((rtmp1 & 0xFFFF) | ((rtmp2 & 0xFFFF) << 16));
	}

	bw_setb(11, false, base + 0x1CC);

	return 0;
}

static int mtk_miu_get_dram_clock(u8 ch, u32 *ch_freq)
{
	int ret = 0;
	u64 div_fst, div_sec;
	u64 lbclk = 0;
	struct mtk_miu *miu_atop = find_device(MIU_ATOP, ch);

	if (!miu_atop) {
		pr_err("Get miu_atop device fail\n");
		*ch_freq = 0;
		ret = -ENXIO;
		goto RET_FUN;
	}

	lbclk = ((((u64)readb_relaxed(miu_atop->base + 0x64)) << 16) |
	((u64)readw_relaxed(miu_atop->base + 0x60)));
	if (lbclk == 0) {
		*ch_freq = 0;
		return ret;
	}

	div_fst = (u64)((readb_relaxed(miu_atop->base + 0x6D) & 0xC) >> 2);
	div_sec = (u64)(readb_relaxed(miu_atop->base + 0x68) & 0x1F);
	if ((readb_relaxed(miu_atop->base + 0xF1) & 0x1))
		*ch_freq = ((8 * div_sec * BCLK) / lbclk);
	else
		*ch_freq = (((2 << div_fst) * div_sec * BCLK) / lbclk);

RET_FUN:
	return ret;
}

static int mtk_miu_get_dram_width(u8 ch, u32 *ch_wid)
{
	int ret = 0;
	u8 dram_wid;
	void __iomem *base;
	struct mtk_miu *miu_dige0 = find_device(MIU_DIG_E, 0);
	struct mtk_miu *miu_dige1 = find_device(MIU_DIG_E, 1);

	if(!miu_dige0 || !miu_dige1) {
                pr_err("Get miu_dig_e device fail\n");
		*ch_wid = 0;
		ret = -ENXIO;
		goto RET_FUN;
	}

	if ((ch == 1) && !(readb_relaxed(miu_dige0->base + 0x4C) & 0x3)) {
		*ch_wid = 0;
		goto RET_FUN;
	}

	if (ch == 0)
		base = miu_dige0->base;
	else
		base = miu_dige1->base;

	dram_wid = (readb_relaxed(base + 0x41) & 0x3);
	if (dram_wid == 0)
		*ch_wid = 16;
	else if (dram_wid == 1)
		*ch_wid = 32;
	else if (dram_wid == 2)
		*ch_wid = 64;
	else
		*ch_wid = 0;

RET_FUN:
	return ret;
}

static int mkt_miu_get_mon_cnt(unsigned int pcnt)
{
	return ((pcnt / 8) + ((pcnt % 8) ? 1 : 0));
}

static void mtk_miu_get_mon_cid(int mid, unsigned int *clist, unsigned int pcnt)
{
	int idx, idx1, ctmp = 0;
	u32 id = 0;

	for (idx = 0; idx < mlist[mid].larb_num; idx++) {
		for (idx1 = 0; idx1 < mlist[mid].plist[idx].pcount; idx1++) {
			id = (mlist[mid].llist[idx] << 4) |
			(mlist[mid].plist[idx].barb_plist[idx1]);
			*(clist + ctmp) = id;
			ctmp++;
		}
	}
}

static int mtk_miu_store_total_bw(struct bw_res *res, struct mtk_mon_res *mres,
				int *mon_id)
{
	int idx, idx1;
	u64 taocpy = 0, tmocpy = 0, tabw = 0, tmbw = 0, ttime = 0;
	int mcnt = 0, idcnt = 0;
	int bw_norm = 1;
	enum miu_channel mon_chan = MIU_CH_ALL;

	mtk_miu_get_mon_ch(&mon_chan);
	if (mon_chan == MIU_CH_ALL)
		bw_norm = 2;
	for (idx = 0; idx < MODULE_CNT; idx++) {
		for (idx1 = 0; idx1 < *(mon_id + idx * 2); idx1++, idcnt++) {
			taocpy += mres[mcnt * 8 + idcnt].avg_ocpy;
			tmocpy += mres[mcnt * 8 + idcnt].max_ocpy;
			tabw += mres[mcnt * 8 + idcnt].avg_bw;
			tmbw += mres[mcnt * 8 + idcnt].max_bw;
			ttime = mres[mcnt * 8 + idcnt].time;
		}
		res[idx].effi = (taocpy == 0) ? 0 : ((tabw * 1000) / taocpy);
		res[idx].meffi = (tmocpy == 0) ? 0 : ((tabw * 1000) / tmocpy);
		res[idx].util =
		(ttime == 0) ? 0 : (((tabw / bw_norm) * 1000) / ttime);
		res[idx].mutil =
		(ttime == 0) ? 0 : (((tmbw / bw_norm) * 1000) / ttime);
		res[idx].bw =
		(ttime == 0) ? 0 : (((taocpy / bw_norm) * 1000) / ttime);
		res[idx].mbw =
		(ttime == 0) ? 0 : (((tmocpy / bw_norm) * 1000) / ttime);

		if (idx + 1 !=
		MODULE_CNT && *(mon_id + (idx + 1) * 2 + 1) == 0) {
			mcnt++;
			idcnt = 0;
		}
		taocpy = tmocpy = tabw = tmbw = ttime = 0;
	}
	return 0;
}

static int mtk_miu_store_module_bw(struct blk_bw_res *bres,
				struct mtk_mon_res *mres)
{
        int idx;
        int mcnt = 0;
        int bw_norm = 1;
        enum miu_channel mon_chan = MIU_CH_ALL;

        mtk_miu_get_mon_ch(&mon_chan);
        if (mon_chan == MIU_CH_ALL)
                bw_norm = 2;
	for (idx = 0; idx < *mlist[module_id].pcnt; idx++) {
                bres[idx].res.effi = (mres[idx].avg_ocpy == 0) ? 0 :
                                ((mres[idx].avg_bw * 1000) /
				mres[idx].avg_ocpy);
                bres[idx].res.meffi = (mres[idx].max_ocpy == 0) ? 0 :
                                ((mres[idx].avg_bw * 1000) /
				mres[idx].max_ocpy);
                bres[idx].res.util = (mres[idx].time == 0) ? 0 :
                                (((mres[idx].avg_bw /
				bw_norm) * 1000) / mres[idx].time);
                bres[idx].res.mutil = (mres[idx].time == 0) ? 0 :
                                (((mres[idx].max_bw /
				bw_norm) * 1000) / mres[idx].time);
                bres[idx].res.bw = (mres[idx].time == 0) ? 0 :
                                (((mres[idx].avg_ocpy /
				bw_norm) * 1000) / mres[idx].time);
                bres[idx].res.mbw = (mres[idx].time == 0) ? 0 :
                                (((mres[idx].max_ocpy /
				bw_norm) * 1000) / mres[idx].time);

		if (idx % 8 == 7)
			mcnt++;
        }
        return 0;
}

static int mtk_miu_get_total_bw(struct bw_res *res)
{
	int ret = 0;
	int idx = 0, idx1 = 0, idx2 = 0;
	int mcnt = 0, mtimes = 1;
	u32 mon_freq = 32;
	unsigned int *clist = NULL;
	struct mtk_mon *monitor = NULL;
	struct mtk_mon_res *mres = NULL;
	int mon_id[MODULE_CNT][2];

	monitor = kmalloc(sizeof(struct mtk_mon), GFP_KERNEL);
	if (!monitor)
		goto RET_FUN;
	monitor->client = kcalloc(8, sizeof(u64), GFP_KERNEL);
	if (!monitor->client)
		goto RET_FUN;
	for (idx = 0; idx < MODULE_CNT; idx++) {
		mon_id[idx][0] = mkt_miu_get_mon_cnt(*mlist[idx].pcnt);
		if (mcnt + mon_id[idx][0] > 8) {
			mtimes++;
			mcnt = 0;
		}
		mon_id[idx][1] = mcnt;
		mcnt += mon_id[idx][0];
	}

	mres = kcalloc(mtimes, sizeof(struct mtk_mon_res) * 8, GFP_KERNEL);
	for (idx = 0; idx < 8 * mtimes; idx++)
		mres[idx] = mtk_mon_res_def;

	mtk_miu_get_mon_freq(&mon_freq);
	for (idx = 0, mtimes = 0; idx < MODULE_CNT; idx++) {
		if (mon_id[idx][1] == 0)
			mtk_miu_reset_mon();
		if (clist)
			kzfree(clist);
		clist = kcalloc(*mlist[idx].pcnt, sizeof(u32), GFP_KERNEL);
		if (!clist)
			goto RET_FUN;
		mtk_miu_get_mon_cid(idx, clist, *mlist[idx].pcnt);
		memset(monitor->client, 0, sizeof(u64) * 8);
		for (idx1 = 0; idx1 < mon_id[idx][0]; idx1++) {
			monitor->mid = mon_id[idx][1] + idx1;
			for (idx2 = 0; idx2 < 8; idx2++) {
				if ((idx1 * 8 + idx2) >= *mlist[idx].pcnt)
					break;
				monitor->client[idx2] = clist[idx1 * 8 + idx2];
			}
			monitor->client_cnt = idx2;
			mtk_miu_set_mon(monitor);
		}
		if (idx + 1 != MODULE_CNT && mon_id[idx + 1][1] != 0)
			continue;

		if (mtk_miu_start_mon() < 0)
			goto RET_FUN;

		(mon_freq > 20) ? msleep(mon_freq) :
		usleep_range(mon_freq * 1000, mon_freq * 1000);

		mtk_miu_get_mon(&mres[mtimes * 8],
		(mon_id[idx][0] + mon_id[idx][1]));
		mtimes++;
	}
	mtk_miu_store_total_bw(res, mres, (int *)mon_id);

RET_FUN:
	if (clist)
		kfree(clist);
	if (mres)
		kfree(mres);
	if (monitor) {
		if (monitor->client)
			kfree(monitor->client);
		kfree(monitor);
	}
	return ret;
}

static void put_buf_module(struct mtk_dram_cfg *d_cfg,
                           struct bw_res *res,
                           char *buf,
                           unsigned int *len)
{
        u32 ratio;
        u32 m_idx, idx;
	u32 total_bw = 0;
	enum miu_channel mon_chan = MIU_CH_ALL;

	mtk_miu_get_mon_ch(&mon_chan);
	if (mon_chan == MIU_CH_ALL) {
		for (idx = 0; idx < MAX_CHANNEL; idx++) {
			total_bw += (d_cfg[idx].clock * d_cfg[idx].width / 8);
		}
	} else {
		total_bw = d_cfg[mon_chan].clock * d_cfg[mon_chan].width / 8;
	}

        putbuf(buf, len, "Starting Monitor......\033[K\n\033[K\n");
        putbuf(buf, len, "\033[7m%4s%10s%10s%10s%10s%10s%10s%10s",
                        "ID",
                        "MODULE",
                        "DOMAIN",
                        "BW",
                        "EFFI",
                        "USAGE",
                        "UF",
                        "OF");
        putbuf(buf, len, "\033[0m\033[K\n");
        for (m_idx = 0; m_idx < MODULE_CNT; m_idx++) {
                putbuf(buf, len, "%4u%10s%10s%10u%10u%10u%10u%10u",
			m_idx,
			mlist[m_idx].name,
			mlist[m_idx].domain,
			(unsigned int)(res[m_idx].util * total_bw / 1000),
			(unsigned int)res[m_idx].effi,
			(unsigned int)(res[m_idx].bw * total_bw / 1000),
			0,
			0);
                putbuf(buf, len, "\033[K\n");
        }
        putbuf(buf, len, "\033[K\n");

        /* print ratio */
        for (m_idx = 0; m_idx < MODULE_CNT; m_idx++) {
                ratio = (res[m_idx].bw/10/2);
                putbuf(buf, len,
                        "%-8s :%6u o/oo [\033[7m%*s\033[0m%*s]",
                        mlist[m_idx].name,
                        (unsigned int)res[m_idx].bw,
                        ratio,
                        "",
                        50-ratio,
                        "");
                putbuf(buf, len, "\033[K\n");
        }
        putbuf(buf, len, "\033[K\n");
        putbuf(buf, len, "\033[4m%34s\033[K\n","");
        putbuf(buf, len, "\033[7m|%8s  |%8s  |%8s  |\033[K\n",
			"Channel",
                        "Width",
                        "Clock");
	if (mon_chan == MIU_CH_ALL) {
		for (idx = 0; idx < MAX_CHANNEL; idx++) {
        		putbuf(buf, len,
				"\033[0m\033[4m|%7u   |%7u   |%8u  |\033[K\n",
				idx,
	                        d_cfg[idx].width,
        	                d_cfg[idx].clock);
		}
	} else {
		putbuf(buf, len,
			"\033[0m\033[4m|%7u   |%7u   |%8u  |\033[K\n",
			mon_chan,
			d_cfg[mon_chan].width,
			d_cfg[mon_chan].clock);
	}

        putbuf(buf, len, "\033[0m\033[K\n");

        putbuf(buf, len, "\033[4m%34s\033[K\n","");
        putbuf(buf, len, "\033[7m|%8s  |%8s  |%8s  |\033[K\n",
                        "TotalBW",
                        "Usage",
                        "Slack");
        putbuf(buf, len, "\033[0m\033[4m|%8u  |%8u  |%8u  |\033[K\n",
                    total_bw,
                    (unsigned int)(res[0].bw * total_bw / 1000),
                    (unsigned int)(total_bw-(res[0].bw * total_bw / 1000)));
        putbuf(buf, len, "|%7u%%  |%7u%%  |%6u%%   |\033[K\n",
                        100,
                        (unsigned int)res[0].bw / 10,
                        (unsigned int)(100 - res[0].bw / 10));
        putbuf(buf, len, "\033[0m\033[K\n\033[K\n");

        /* print status for all*/

        if ((res[0].bw / 10) > 90)
                putbuf(buf, len, "Status: \033[41;30m  DANGER  ");
        else if ((res[0].bw / 10) > 80)
                putbuf(buf, len, "Status: \033[43;30m WARNNING ");
        else
                putbuf(buf, len, "Status: \033[42;30m   SAFE   ");

        putbuf(buf, len, "\033[0m\033[K\n\033[K\n\033[K\n\033[K\n");
}

static ssize_t mtk_miu_total_show(struct device_driver *drv, char *buf)
{
	u32 len = 0;
	int idx;
	struct mtk_dram_cfg *d_cfg;
	struct bw_res *res;

	/* Get DRAM config */
	d_cfg = kcalloc(MAX_CHANNEL, sizeof(struct mtk_dram_cfg), GFP_KERNEL);
	res = kcalloc(MODULE_CNT, sizeof(struct bw_res), GFP_KERNEL);
	if (!d_cfg || !res) {
		pr_err("no memory\n");
		goto RET_FUN;
	}
	for (idx = 0; idx < MAX_CHANNEL; idx++) {
		d_cfg[idx].channel = idx;
		if (mtk_miu_get_dram_width(idx, &d_cfg[idx].width) < 0) {
			pr_err("no dram device\n");
			goto RET_FUN;
		}
		if (mtk_miu_get_dram_clock(idx, &d_cfg[idx].clock) < 0) {
			pr_err("no dram device\n");
			goto RET_FUN;
		}
	}
	for (idx = 0; idx < MODULE_CNT; idx++)
		res[idx] = bw_res_def;

	mtk_miu_get_total_bw(res);
	put_buf_module(d_cfg, res, buf, &len);

RET_FUN:
	if (d_cfg)
		kfree(d_cfg);
	if (res)
		kfree(res);
        return len;
}

static int mtk_miu_get_module_bw(struct blk_bw_res *bres)
{
	int ret = 0;
	int idx = 0;
	int mtimes = 0;
	bool get_res = true;
	u32 mon_freq = 32;
	unsigned int *clist = NULL;
	struct mtk_mon *monitor = NULL;
	struct mtk_mon_res *mres = NULL;

	monitor = kmalloc(sizeof(struct mtk_mon), GFP_KERNEL);
	if (!monitor)
		goto RET_FUN;
	monitor->client = kmalloc(sizeof(u64), GFP_KERNEL);
	if (!monitor->client)
		goto RET_FUN;

	mtimes = mkt_miu_get_mon_cnt(*mlist[module_id].pcnt);
	mres = kcalloc(mtimes, sizeof(struct mtk_mon_res) * 8, GFP_KERNEL);
	for (idx = 0; idx < 8 * mtimes; idx++)
		mres[idx] = mtk_mon_res_def;

	clist = kcalloc(*mlist[module_id].pcnt, sizeof(u32), GFP_KERNEL);
	if (!clist)
		goto RET_FUN;
	mtk_miu_get_mon_cid(module_id, clist, *mlist[module_id].pcnt);

	mtk_miu_get_mon_freq(&mon_freq);
	for (idx = 0, mtimes = 0; idx < *mlist[module_id].pcnt; idx++) {
		if (get_res) {
			mtk_miu_reset_mon();
			memset(monitor->client, 0, sizeof(u64));
			get_res = false;
		}
		monitor->mid = idx % 8;
		monitor->client[0] = clist[idx];
		monitor->client_cnt = 1;
		mtk_miu_set_mon(monitor);
		if (idx + 1 != *mlist[module_id].pcnt && monitor->mid != 7)
			continue;

		if (mtk_miu_start_mon() < 0)
			goto RET_FUN;

		(mon_freq > 20) ? msleep(mon_freq) :
		usleep_range(mon_freq * 1000, mon_freq * 1000);

		mtk_miu_get_mon(&mres[mtimes * 8], ((idx % 8) + 1));
		get_res = true;
		mtimes++;
	}
	mtk_miu_store_module_bw(bres, mres);

	for (idx = 0; idx < *mlist[module_id].pcnt; idx++) {
		bres[idx].mask = mtk_miu_get_mask(((clist[idx] >> 4) & 0xF),
						(clist[idx] & 0xF));

		bres[idx].hpmask = mtk_miu_get_hpmask(((clist[idx] >> 4) & 0xF),
							(clist[idx] & 0xF));
		bres[idx].prior =
		mtk_miu_get_priority(((clist[idx] >> 4) & 0xF),
					(clist[idx] & 0xF));

		bres[idx].maxserivcenum =
		mtk_miu_get_max_service_number(((clist[idx] >> 4) & 0xF),
					(clist[idx] & 0xF));

		bres[idx].latency =
		mtk_miu_get_latency(((clist[idx] >> 4) & 0xF),
		(clist[idx] & 0xF));
	}
RET_FUN:
	if (clist)
		kfree(clist);
	if (mres)
		kfree(mres);
	if (monitor) {
		if (monitor->client)
			kfree(monitor->client);
		kfree(monitor);
	}
	return ret;
}

static unsigned int put_buf_larb_left(u32 total_bw,
				struct blk_bw_res *bres,
				char *buf,
				unsigned int *len,
				unsigned int larb_num)
{
        u32 idx, idx1 = 0;
        u32 left_pos = 0;
        u32 next_p_nr;
        struct module_map m = mlist[module_id];
	enum miu_channel mon_chan = MIU_CH_ALL;

	mtk_miu_get_mon_ch(&mon_chan);
	if (mon_chan == MIU_CH_ALL) {
	        putbuf(buf, len, "BLOCK_ARB %d - channel all\033[K\n",
			m.llist[larb_num]);
	} else {
	        putbuf(buf, len, "BLOCK_ARB %d - channel%d\033[K\n",
			m.llist[larb_num], mon_chan);
	}
        putbuf(buf, len, "\033[7m");
        putbuf(buf, len, "%3s%3s%3s%3s%4s%6s%7s%7s%7s%7s",
                        "ID", "M", "HP", "P", "LM", "LA",
                        "UT", "EF", "US", "USP");
        putbuf(buf, len, "\033[K\n");

	for (idx = 0; idx < larb_num; idx ++)
		idx1 += m.plist[idx].pcount;

        for (idx = 0; idx < m.plist[larb_num].pcount; idx++, idx1++) {
                putbuf(buf, len, "\033[0m");
                putbuf(buf, len, "%3u%3u%3u",
			m.plist[larb_num].barb_plist[idx],
			bres[idx1].mask,
			bres[idx1].hpmask);
		if (bres[idx1].prior < 0) {
                	putbuf(buf, len, "%3s", "D");
		} else {
                	putbuf(buf, len, "%3d", bres[idx1].prior);
		}
		if (bres[idx1].maxserivcenum < 0) {
                	putbuf(buf, len, "%4s", "N");
		} else {
                	putbuf(buf, len, "%4d", bres[idx1].maxserivcenum);
		}
                putbuf(buf, len, "%6u%7lu%7lu%7lu%7lu",
		bres[idx1].latency,
		(unsigned long int)(bres[idx1].res.util * total_bw / 1000),
		(unsigned long int)bres[idx1].res.effi,
		(unsigned long int)(bres[idx1].res.bw * total_bw / 1000),
		(unsigned long int)bres[idx1].res.bw);
                putbuf(buf, len, "\033[K\n");
        }
        left_pos = idx;
        next_p_nr = m.plist[larb_num+1].pcount;
        if (larb_num != m.larb_num-1) {
                if (m.plist[larb_num].pcount < next_p_nr) {
                        for (idx = 0; idx < next_p_nr - left_pos; idx++)
                                putbuf(buf, len, "\033[K\n");
                        for (idx = 0; idx < next_p_nr+2; idx++)
                                putbuf(buf, len, "\033[1A");
                } else {
                        for (idx = 0; idx < m.plist[larb_num].pcount+2; idx++)
                                putbuf(buf, len, "\033[1A");
                }
        } else {
                putbuf(buf, len, "\033[K\n");
        }

        return left_pos;

}

static void put_buf_larb_right(u32 total_bw, struct blk_bw_res *bres, char *buf,
		unsigned int *len, unsigned int larb_num, unsigned int left_pos)
{
        u32 idx, idx1 = 0;
        struct module_map m = mlist[module_id];
	enum miu_channel mon_chan = MIU_CH_ALL;

	mtk_miu_get_mon_ch(&mon_chan);
        putbuf(buf, len, "\033[60C");
	if (mon_chan == MIU_CH_ALL) {
	        putbuf(buf, len, "BLOCK_ARB %d - channel all\033[K\n",
			m.llist[larb_num]);
	} else {
	        putbuf(buf, len, "BLOCK_ARB %d - channel%d\033[K\n",
			m.llist[larb_num], mon_chan);
	}
        putbuf(buf, len, "\033[60C\033[7m");
        putbuf(buf, len, "%3s%3s%3s%3s%4s%6s%7s%7s%7s%7s",
                        "ID", "M", "HP", "P", "LM", "LA",
                        "UT", "EF", "US", "USP");
        putbuf(buf, len, "\033[K\n");

	for (idx = 0; idx < larb_num; idx ++)
		idx1 += m.plist[idx].pcount;

        for (idx = 0; idx < m.plist[larb_num].pcount; idx++, idx1++) {
                putbuf(buf, len, "\033[60C\033[0m");
                putbuf(buf, len, "%3u%3u%3u",
			m.plist[larb_num].barb_plist[idx],
			bres[idx1].mask,
			bres[idx1].hpmask);
		if (bres[idx1].prior < 0) {
                	putbuf(buf, len, "%3s", "D");
		} else {
                	putbuf(buf, len, "%3d", bres[idx1].prior);
		}
		if (bres[idx1].maxserivcenum < 0) {
                	putbuf(buf, len, "%4s", "N");
		} else {
                	putbuf(buf, len, "%4d", bres[idx1].maxserivcenum);
		}
                putbuf(buf, len, "%6u%7lu%7lu%7lu%7lu",
		bres[idx1].latency,
		(unsigned long int)(bres[idx1].res.util * total_bw / 1000),
		(unsigned long int)bres[idx1].res.effi,
		(unsigned long int)(bres[idx1].res.bw * total_bw / 1000),
		(unsigned long int)bres[idx1].res.bw);
                putbuf(buf, len, "\033[K\n");
        }
        if (idx < left_pos) {
                for (idx1 = 0; idx1 < (left_pos - idx); idx1++)
                        putbuf(buf, len, "\033[1B");
                        putbuf(buf, len, "\033[60C\033[K");
        }
        putbuf(buf, len, "\033[K\n");
}

static void put_buf_larb(struct blk_bw_res *bres,
			struct mtk_dram_cfg *d_cfg,
			char *buf,
			unsigned int *len)
{
        u32 idx;
        u32 left_pos = 0;
	u32 total_bw = 0;
        struct module_map m = mlist[module_id];
	enum miu_channel mon_chan = MIU_CH_ALL;

	mtk_miu_get_mon_ch(&mon_chan);
	if (mon_chan == MIU_CH_ALL) {
		for (idx = 0; idx < MAX_CHANNEL; idx++) {
			total_bw += (d_cfg[idx].clock * d_cfg[idx].width / 8);
		}
	} else {
		total_bw = d_cfg[mon_chan].clock * d_cfg[mon_chan].width / 8;
	}

        putbuf(buf, len, "Starting Moniter......\033[K\n\033[K\n");
        putbuf(buf, len, "Module : \033[46;30m  %s  ", m.name);
        putbuf(buf, len, "\033[0m\033[K\n\033[K\n");
        putbuf(buf, len, "M: Mask/Unmask\033[K\n");
        putbuf(buf, len, "HP: High priority\033[K\n");
        putbuf(buf, len, "P: Priority, 0>1>2>3; D: dynamic priority\033[K\n");
        putbuf(buf, len, "LM: Limit mask size; N: no limit mask\033[K\n");
        putbuf(buf, len, "LA: Latency(T/MIU clock)\033[K\n");
        putbuf(buf, len, "UT: Utilization(MB/s)\033[K\n");
        putbuf(buf, len, "EF: Efficiency(o/oo)\033[K\n");
        putbuf(buf, len, "US: Usage(MB/s)\033[K\n");
	putbuf(buf, len, "USP: Usage percentage(o/oo)\033[K\n\033[K\n");
        for (idx = 0; idx < m.larb_num; idx++) {
                if (idx % 2 == 0)
                        left_pos = put_buf_larb_left(total_bw, bres,
						buf, len, idx);
                else
                        put_buf_larb_right(total_bw, bres, buf, len,
						idx, left_pos);
        }
}

static ssize_t mtk_miu_module_show(struct device_driver *drv, char *buf)
{
	u32 len = 0;
	int idx;
	struct mtk_dram_cfg *d_cfg;
	struct blk_bw_res *bres;

	/* Get DRAM config */
	d_cfg = kcalloc(MAX_CHANNEL, sizeof(struct mtk_dram_cfg), GFP_KERNEL);
	bres =
	kcalloc(*mlist[module_id].pcnt, sizeof(struct blk_bw_res), GFP_KERNEL);
	if (!d_cfg || !bres) {
		pr_err("no memory\n");
		goto RET_FUN;
	}
	for (idx = 0; idx < MAX_CHANNEL; idx++) {
		d_cfg[idx].channel = idx;
		if (mtk_miu_get_dram_width(idx, &d_cfg[idx].width) < 0) {
			pr_err("no dram device\n");
			goto RET_FUN;
		}
		if (mtk_miu_get_dram_clock(idx, &d_cfg[idx].clock) < 0) {
			pr_err("no dram device\n");
			goto RET_FUN;
		}
	}
	for (idx = 0; idx < *mlist[module_id].pcnt; idx++)
		bres[idx] = blk_bw_res_def;

	mtk_miu_get_module_bw(bres);
	put_buf_larb(bres, d_cfg, buf, &len);
RET_FUN:
	if (d_cfg)
		kfree(d_cfg);
	if (bres)
		kfree(bres);
        return len;
}

static ssize_t mtk_miu_module_store(struct device_driver *drv,
				const char *buf, size_t count)
{
	int idx = 0;
	char module_name[16];

	if (sscanf(buf, "%s", module_name) == 1) {
		for (idx = 0; idx < MODULE_CNT; idx++) {
			if (strcasecmp(module_name, mlist[idx].name) == 0) {
				module_id = idx;
				break;
			}
		}
		if (idx == MODULE_CNT)
			pr_emerg("Set Module wronge\n");
	} else {
		pr_emerg("Set Module wronge\n");
	}
	return count;
}

static ssize_t mtk_miu_mon_chan_store(struct device_driver *drv,
				const char *buf, size_t count)
{
	char channel[16];

	if (sscanf(buf, "%s", channel) == 1) {
		if (strcmp(channel, "0") == 0)
			mtk_miu_set_mon_ch(MIU_CH0);
		else if (strcmp(channel, "1") == 0)
			mtk_miu_set_mon_ch(MIU_CH1);
		else if (strcasecmp(channel, "all") == 0)
			mtk_miu_set_mon_ch(MIU_CH_ALL);
		else
			pr_err("error parameter\n");
	}

	return count;
}

static ssize_t mtk_miu_mon_chan_show(struct device_driver *drv, char *buf)
{
	u32 len = 0;
	enum miu_channel mon_chan;

	if (mtk_miu_get_mon_ch(&mon_chan) < 0) {
		putbuf(buf, &len, "something wrong\n");
		goto RET_FUN;
	}

	if (mon_chan == MIU_CH0)
		putbuf(buf, &len, "channel 0\n");
	else if (mon_chan == MIU_CH1)
		putbuf(buf, &len, "channel 1\n");
	else if (mon_chan == MIU_CH_ALL)
		putbuf(buf, &len, "channel 0  + channel 1\n");
	else
		putbuf(buf, &len, "something wrong\n");

RET_FUN:
        return len;
}

static ssize_t mtk_miu_mon_freq_store(struct device_driver *drv,
				const char *buf, size_t count)
{
	u32 mon_freq;

	if (sscanf(buf, "%d", &mon_freq) == 1)
		mtk_miu_set_mon_freq(mon_freq);

	return count;
}

static ssize_t mtk_miu_mon_freq_show(struct device_driver *drv, char *buf)
{
	u32 len = 0;
	u32 mon_freq = 32;

	mtk_miu_get_mon_freq(&mon_freq);

	putbuf(buf, &len, "monitor frequency:(%d)ms\n", mon_freq);

        return len;
}

static ssize_t mtk_miu_mon_area_store(struct device_driver *drv,
				const char *buf, size_t count)
{
	struct mtk_miu *miu_dige = find_device(MIU_DIG_E, 0);
	u32 mon_area;

	if (!miu_dige)
		return count;

	if (sscanf(buf, "%d", &mon_area) == 1) {
		if (mon_area == 0)
			bw_setb(14, false, miu_dige->base + 0x1CC);
		else if(mon_area == 1)
			bw_setb(14, true, miu_dige->base + 0x1CC);
	}

	return count;
}

static ssize_t mtk_miu_mon_area_show(struct device_driver *drv, char *buf)
{
	u32 len = 0;
	struct mtk_miu *miu_dige = find_device(MIU_DIG_E, 0);

	if (!miu_dige)
		return len;

	if ((readw_relaxed(miu_dige->base + 0x1CC) & 0x4000))
		putbuf(buf, &len, "enable monitor area\n");
	else
		putbuf(buf, &len, "disable monitor area\n");

        return len;
}

static DRIVER_ATTR_RO(mtk_miu_total);
static DRIVER_ATTR_RW(mtk_miu_module);
static DRIVER_ATTR_RW(mtk_miu_mon_chan);
static DRIVER_ATTR_RW(mtk_miu_mon_freq);
static DRIVER_ATTR_RW(mtk_miu_mon_area);

static int mtk_miu_dig_probe(struct platform_device *pdev)
{
	s32 err = 0;
	char name[32];
	struct resource *res = NULL;
	struct mtk_miu *miu_dig = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	miu_dig = devm_kzalloc(dev, sizeof(*miu_dig), GFP_KERNEL);
	if (!miu_dig)
		return -ENOMEM;

	miu_dig->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	miu_dig->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(miu_dig->base))
		return PTR_ERR(miu_dig->base);

	err = of_property_read_u32(node, "mediatek,miu-dig-id", &(miu_dig->id));
	if (err || miu_dig->id > 2) {
		dev_err(dev, "missing mediatek,miu-dig-id property\n");
		return err;
	}

	miu_dig->type = MIU_DIG;
	platform_set_drvdata(pdev, miu_dig);

	return 0;
}

static int mtk_miu_dig_remove(struct platform_device *pdev)
{
	struct mtk_miu *miu_dig = NULL;
	int idx;

	for (idx = 0; idx < 2; idx++) {
		miu_dig = find_device(MIU_DIG, idx);
		if (miu_dig)
			devm_kfree(&pdev->dev, (void *)miu_dig);
	}
	return 0;
}

static int mtk_miu_dig_e_probe(struct platform_device *pdev)
{
	s32 err = 0;
	char name[32];
	struct resource *res = NULL;
	struct mtk_miu *miu_dig_e = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	miu_dig_e = devm_kzalloc(dev, sizeof(*miu_dig_e), GFP_KERNEL);
	if (!miu_dig_e)
		return -ENOMEM;

	miu_dig_e->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	miu_dig_e->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(miu_dig_e->base))
		return PTR_ERR(miu_dig_e->base);

	err = of_property_read_u32(node,
		"mediatek,miu-dig-e-id", &(miu_dig_e->id));
	if (err || miu_dig_e->id > 2) {
		dev_err(dev, "missing mediatek,miu-dig-id-e property\n");
		return err;
	}

	miu_dig_e->type = MIU_DIG_E;
	platform_set_drvdata(pdev, miu_dig_e);

	return 0;
}

static int mtk_miu_dig_e_remove(struct platform_device *pdev)
{
	struct mtk_miu *miu_dig_e = NULL;
	int idx;

	for (idx = 0; idx < 2; idx++) {
		miu_dig_e = find_device(MIU_DIG_E, idx);
		if (miu_dig_e)
			devm_kfree(&pdev->dev, (void *)miu_dig_e);
	}
	return 0;
}

static int mtk_miu_atop_probe(struct platform_device *pdev)
{
	s32 err = 0;
	char name[32];
	struct resource *res = NULL;
	struct mtk_miu *miu_atop = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	miu_atop = devm_kzalloc(dev, sizeof(*miu_atop), GFP_KERNEL);
	if (!miu_atop)
		return -ENOMEM;

	miu_atop->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	miu_atop->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(miu_atop->base))
		return PTR_ERR(miu_atop->base);

	err = of_property_read_u32(node,
		"mediatek,miu-atop-id", &(miu_atop->id));
	if (err || miu_atop->id > 2) {
		dev_err(dev, "missing mediatek,miu-atop-id property\n");
		return err;
	}

	miu_atop->type = MIU_ATOP;
	platform_set_drvdata(pdev, miu_atop);

	return 0;
}

static int mtk_miu_atop_remove(struct platform_device *pdev)
{
	struct mtk_miu *miu_atop = NULL;
	int idx;

	for (idx = 0; idx < 2; idx++) {
		miu_atop = find_device(MIU_ATOP, idx);
		if (miu_atop)
			devm_kfree(&pdev->dev, (void *)miu_atop);
	}
	return 0;
}

static int mtk_miu_ctrl_arb_probe(struct platform_device *pdev)
{
	s32 err = 0;
	char name[32];
	struct resource *res = NULL;
	struct mtk_miu *miu_carb = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	miu_carb = devm_kzalloc(dev, sizeof(*miu_carb), GFP_KERNEL);
	if (!miu_carb)
		return -ENOMEM;

	miu_carb->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	miu_carb->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(miu_carb->base))
		return PTR_ERR(miu_carb->base);

	err = of_property_read_u32(node,
		"mediatek,miu-ctrl-arb-id", &(miu_carb->id));
	if (err || miu_carb->id > 2) {
		dev_err(dev, "missing mediatek,miu-ctrl-arb-id property\n");
		return err;
	}

	miu_carb->type = MIU_CTRL_ARB;
	platform_set_drvdata(pdev, miu_carb);

	return 0;
}

static int mtk_miu_ctrl_arb_remove(struct platform_device *pdev)
{
	struct mtk_miu *miu_carb = NULL;
	int idx;

	for (idx = 0; idx < 2; idx++) {
		miu_carb = find_device(MIU_CTRL_ARB, idx);
		if (miu_carb)
			devm_kfree(&pdev->dev, (void *)miu_carb);
	}
	return 0;
}

static int mtk_miu_blk_arb_probe(struct platform_device *pdev)
{
	s32 err = 0;
	char name[32];
	struct resource *res = NULL;
	struct mtk_miu *miu_barb = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	miu_barb = devm_kzalloc(dev, sizeof(*miu_barb), GFP_KERNEL);
	if (!miu_barb)
		return -ENOMEM;

	miu_barb->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	miu_barb->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(miu_barb->base))
		return PTR_ERR(miu_barb->base);

	err = of_property_read_u32(node,
		"mediatek,miu-blk-arb-id", &(miu_barb->id));
	if (err || miu_barb->id > 10) {
		dev_err(dev, "missing mediatek,miu-blk-arb-id property\n");
		return err;
	}

	miu_barb->type = MIU_BLK_ARB;
	platform_set_drvdata(pdev, miu_barb);

	return 0;
}

static int mtk_miu_blk_arb_remove(struct platform_device *pdev)
{
	struct mtk_miu *miu_barb = NULL;
	int idx;

	for (idx = 0; idx < 2; idx++) {
		miu_barb = find_device(MIU_BLK_ARB, idx);
		if (miu_barb)
			devm_kfree(&pdev->dev, (void *)miu_barb);
	}
	return 0;
}

static int mtk_bw_measure_init(void)
{
	struct device_driver *drv;
	int err = 0;

	drv = driver_find("mtk-bw", &platform_bus_type);
        err = driver_create_file(drv, &driver_attr_mtk_miu_total);
        if (err) {
		pr_notice("mtk_miu_total create attribute error\n");
                return err;
        }

        err = driver_create_file(drv, &driver_attr_mtk_miu_module);
        if (err) {
		pr_notice("mtk_miu_module create attribute error\n");
                return err;
        }

        err = driver_create_file(drv, &driver_attr_mtk_miu_mon_chan);
        if (err) {
		pr_notice("mtk_miu_mon_chan create attribute error\n");
                return err;
        }

        err = driver_create_file(drv, &driver_attr_mtk_miu_mon_freq);
        if (err) {
		pr_notice("mtk_miu_mon_freq create attribute error\n");
                return err;
        }

        err = driver_create_file(drv, &driver_attr_mtk_miu_mon_area);
        if (err) {
		pr_notice("mtk_miu_mon_area create attribute error\n");
                return err;
        }

	return err;
}


static const struct of_device_id mtk_miu_dig_of_match[] = {
	{ .compatible = "mediatek,mt5889-miu-dig",},
	{/* sentinel */},
};

static const struct of_device_id mtk_miu_dig_e_of_match[] = {
	{ .compatible = "mediatek,mt5889-miu-dig-e",},
	{/* sentinel */},
};

static const struct of_device_id mtk_miu_atop_of_match[] = {
	{ .compatible = "mediatek,mt5889-miu-atop",},
	{/* sentinel */},
};

static const struct of_device_id mtk_miu_ctrl_arb_of_match[] = {
	{ .compatible = "mediatek,mt5889-miu-ctrl-arb",},
	{/* sentinel */},
};

static const struct of_device_id mtk_miu_blk_arb_of_match[] = {
	{ .compatible = "mediatek,mt5889-miu-blk-arb",},
	{/* sentinel */},
};

static struct platform_driver mtk_miu_dig_driver = {
	.probe  = mtk_miu_dig_probe,
	.remove = mtk_miu_dig_remove,
	.driver = {
                .name = "mtk-miu-dig",
		.owner	= THIS_MODULE,
		.of_match_table = mtk_miu_dig_of_match,
	},
};

static struct platform_driver mtk_miu_dig_e_driver = {
	.probe  = mtk_miu_dig_e_probe,
	.remove = mtk_miu_dig_e_remove,
	.driver = {
                .name = "mtk-miu-dig-e",
		.owner	= THIS_MODULE,
		.of_match_table = mtk_miu_dig_e_of_match,
	},
};

static struct platform_driver mtk_miu_atop_driver = {
	.probe  = mtk_miu_atop_probe,
	.remove = mtk_miu_atop_remove,
	.driver = {
                .name = "mtk-miu-atop",
		.owner	= THIS_MODULE,
		.of_match_table = mtk_miu_atop_of_match,
	},
};

static struct platform_driver mtk_miu_ctrl_arb_driver = {
	.probe  = mtk_miu_ctrl_arb_probe,
	.remove = mtk_miu_ctrl_arb_remove,
	.driver = {
                .name = "mtk-miu-ctrl-arb",
		.owner	= THIS_MODULE,
		.of_match_table = mtk_miu_ctrl_arb_of_match,
	},
};

static struct platform_driver mtk_miu_blk_arb_driver = {
	.probe  = mtk_miu_blk_arb_probe,
	.remove = mtk_miu_blk_arb_remove,
	.driver = {
                .name = "mtk-miu-blk-arb",
		.owner	= THIS_MODULE,
		.of_match_table = mtk_miu_blk_arb_of_match,
	},
};

static struct platform_driver mtk_miu_bw_driver = {
	.driver = {
		.name = "mtk-bw",
	},
};

static int __init mtk_miu_init(void)
{
        int ret;

        ret = platform_driver_register(&mtk_miu_dig_driver);
        if (ret != 0) {
                pr_err("Failed to register mtk_miu_dig driver\n");
                return ret;
        }
        ret = platform_driver_register(&mtk_miu_dig_e_driver);
        if (ret != 0) {
                pr_err("Failed to register mtk_miu_dig_e driver\n");
                return ret;
        }
        ret = platform_driver_register(&mtk_miu_atop_driver);
        if (ret != 0) {
                pr_err("Failed to register mtk_miu_atop driver\n");
                return ret;
        }
        ret = platform_driver_register(&mtk_miu_ctrl_arb_driver);
        if (ret != 0) {
                pr_err("Failed to register mtk_miu_ctrl_arb driver\n");
                return ret;
        }
        ret = platform_driver_register(&mtk_miu_blk_arb_driver);
        if (ret != 0) {
                pr_err("Failed to register mtk_miu_blk_arb driver\n");
                return ret;
        }
	ret = platform_driver_register(&mtk_miu_bw_driver);
	if (ret != 0) {
                pr_err("Failed to register mtk miu bw driver\n");
		return ret;
	}
	ret = mtk_bw_measure_init();
	if (ret != 0) {
                pr_err("Failed to init bw measure\n");
		return ret;
	}
	mtk_miu_set_mon_freq((u32)32);

        return ret;
}

static void __exit mtk_miu_exit(void)
{
	platform_driver_unregister(&mtk_miu_dig_driver);
	platform_driver_unregister(&mtk_miu_dig_e_driver);
	platform_driver_unregister(&mtk_miu_atop_driver);
	platform_driver_unregister(&mtk_miu_ctrl_arb_driver);
	platform_driver_unregister(&mtk_miu_blk_arb_driver);
	platform_driver_unregister(&mtk_miu_bw_driver);
	return;
}

module_init(mtk_miu_init);
module_exit(mtk_miu_exit);

