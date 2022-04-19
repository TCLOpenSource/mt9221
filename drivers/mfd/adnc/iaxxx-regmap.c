/*
 * iaxxx-regmap.c -- Register map data for IAxxx series devices
 *
 * Copyright 2016 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */
#define pr_fmt(fmt) "iaxxx: %s:%d, " fmt "\n", __func__, __LINE__

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-register-map.h>
#include "iaxxx.h"
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-regmap.h>

/* HMD and DMX 2 blocks are in ia8x01 */
#define IAXXX_PLUGIN_UPDATE_BLOCKS_MAX 2

#define IAXXX_VIRTUAL_ADDR_START				      0x01000000
#define IAXXX_VIRTUAL_ADDR_END					      0x10FFFFFF

#define SRB_BLOCK_SELECT_REG	IAXXX_SRB_HOST_DEFINED_1_ADDR
#define SRB_BLOCK_SELECT_MASK	0x1F
#define SRB_BLOCK_SELECT_SHIFT	0

/* Double it for Header and Group of each ARB */
#define IAXXX_REGMAP_NO_PM_NUM_ARBS     6

#define IAXXX_PHYSICAL_ADDRESS_BASE	0x50000000
#define IAXXX_I2S_SIZE			0x00001000
#define IAXXX_PCM_SIZE			0x00001000
#define IAXXX_CNR_SIZE			0x00001000
#define IAXXX_IOCTRL_SIZE		0x00001000
#define IAXXX_PAD_CTRL_SIZE		0x00001000
#define IAXXX_GPIO_SIZE			0x00001000
#define IAXXX_CNR_SIZE			0x00001000
#define IAXXX_PCTRL_SIZE		0x00001000
#define IAXXX_SRB_BASE			IAXXX_SRB_REGS_ADDR
#define IAXXX_BOSS_DRAM_BASE		0xA0000000

#define IAXXX_I2S_REGS_END_ADDR	 (IAXXX_I2S_REGS_ADDR + IAXXX_I2S_SIZE)
#define IAXXX_AO_REGS_END_ADDR	 (IAXXX_AO_REGS_ADDR + (4 * IAXXX_AO_REG_NUM))
#define IAXXX_CNR0_REGS_END_ADDR (IAXXX_CNR0_REGS_ADDR + IAXXX_CNR_SIZE)
#define IAXXX_IOCTRL_REGS_END_ADDR (IAXXX_IO_CTRL_REGS_ADDR + IAXXX_IOCTRL_SIZE)
#define IAXXX_PAD_CTRL_REGS_END_ADDR (IAXXX_PAD_CTRL_REGS_ADDR + \
	IAXXX_PAD_CTRL_SIZE)
#define IAXXX_GPIO_REGS_END_ADDR (IAXXX_GPIO_REGS_ADDR + IAXXX_GPIO_SIZE)
#define IAXXX_PCTRL_REGS_END_ADDR (IAXXX_PCTRL_REGS_ADDR + IAXXX_PCTRL_SIZE)
#define IAXXX_CNR_REGS_END_ADDR (IAXXX_CNR_REGS_ADDR + IAXXX_CNR_SIZE)

#define PCM_REG_NAME(N)  IAXXX_PCM ## N ## _REGS_ADDR
#define IAXXX_PCM_BASE_ADDR(N) PCM_REG_NAME(N)
#define IAXXX_PCM_REGS_END_ADDR(N) (IAXXX_PCM_BASE_ADDR(N) + IAXXX_PCM_SIZE)

/* Virtual addresses */
#define IAXXX_REG_CHANNEL_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_CHANNEL)
#define IAXXX_REG_STREAM_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_STREAM)
#define IAXXX_REG_PACKAGE_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_PACKAGE)
#define IAXXX_REG_PLUGIN_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_PLUGIN)
#define IAXXX_REG_TUNNEL_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_TUNNEL)
#define IAXXX_REG_DBGLOG_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_DBGLOG)
#define IAXXX_REG_POWER_BASE    IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_POWER)
#define IAXXX_REG_EVENT_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_EVENT)
#define IAXXX_REG_SCRIPT_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_SCRIPT)

#define IAXXX_ARB_GRP_INDEX	(1 << 23)

/* Relocatable block read retry max */
#define IAXXX_RDBT_RETRY_COUNT 5
#define IAXXX_RDBT_READ_DELAY (5*1000) /* 5 ms */
#define IAXXX_RDBT_READ_RANGE (1*1000) /* 1 ms */


static const uint32_t iaxxx_phy_addr_ranges[][2] = {
	/* Start phy address */  /* End phy address */
	{IAXXX_SRB_BASE, (IAXXX_SRB_BASE + IAXXX_SRB_SIZE)},
	{IAXXX_I2S_REGS_ADDR, IAXXX_I2S_REGS_END_ADDR},
	{IAXXX_AF_WCPT_WALL_CLOCK_RD_0_ADDR,
		IAXXX_AF_WCPT_WALL_CLOCK_RD_1_ADDR},
	{IAXXX_AO_REGS_ADDR, IAXXX_AO_REGS_END_ADDR},
	{IAXXX_CNR0_REGS_ADDR, IAXXX_CNR0_REGS_END_ADDR},
	{IAXXX_PCM_BASE_ADDR(0), IAXXX_PCM_REGS_END_ADDR(0)},
	{IAXXX_PCM_BASE_ADDR(1), IAXXX_PCM_REGS_END_ADDR(1)},
	{IAXXX_PCM_BASE_ADDR(2), IAXXX_PCM_REGS_END_ADDR(2)},
	{IAXXX_IO_CTRL_REGS_ADDR, IAXXX_IOCTRL_REGS_END_ADDR},
	{IAXXX_PAD_CTRL_REGS_ADDR, IAXXX_PAD_CTRL_REGS_END_ADDR},
	{IAXXX_GPIO_REGS_ADDR, IAXXX_GPIO_REGS_END_ADDR},
	{IAXXX_PCTRL_REGS_ADDR, IAXXX_PCTRL_REGS_END_ADDR},
	{IAXXX_CNR_REGS_ADDR, IAXXX_CNR_REGS_END_ADDR},
};

/* Returns true if register is in the supported physical address range */
static inline bool iaxxx_is_physical_address(uint32_t reg)
{
	int i;
	int phy_addr_size = sizeof(iaxxx_phy_addr_ranges) /
					sizeof(iaxxx_phy_addr_ranges[0]);

	for (i = 0; i < phy_addr_size; i++) {
		if (reg >= iaxxx_phy_addr_ranges[i][0] &&
				reg < iaxxx_phy_addr_ranges[i][1])
			return true;
	}
	return false;
}

/*
 * readable_register needed to optimize access and bus usage
 */
static bool iaxxx_readable_register(struct device *dev, unsigned int reg)
{
	int i;
	const struct regmap_range_cfg *cfg;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv) {
		dev_err(dev, "priv structure is NULL\n");
		return false;	/* Something went wrong */
	}

	/* ONLY if FW is booted, allow reads to this regmap */
	if (!iaxxx_is_firmware_ready(priv))
		return false;

	cfg = priv->regmap_config->ranges;

	/* Virtual addresses are only supported when mapped */
	if (reg >= IAXXX_VIRTUAL_ADDR_START && reg <= IAXXX_VIRTUAL_ADDR_END) {
		if (!cfg)
			return false;

		/* TODO: use lookup table Block Index -> *regmap_range_cfg */
		/* For now, walk through the ranges and check the address */
		for (i = 0; i < priv->regmap_config->num_ranges; ++i, ++cfg)
			if ((reg >= cfg->range_min) && (reg <= cfg->range_max))
				return true;
	}

	/* Support physical address */
	return iaxxx_is_physical_address(reg);
}

static bool iaxxx_writeable_register(struct device *dev, unsigned int reg)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv) {
		dev_err(dev, "priv structure is NULL\n");
		return false;	/* Something went wrong */
	}

	/* ONLY if FW is booted allow writes to this regmap */
	if (!iaxxx_is_firmware_ready(priv))
		return false;

	return true;
}

static bool iaxxx_volatile_register(struct device *dev, unsigned int reg)
{
		return true;
}

static bool iaxxx_application_volatile_reg(struct device *dev, unsigned int reg)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int i;

	if (!priv) {
		dev_err(dev, "priv structure is NULL\n");
		return false;	/* Something went wrong */
	}

	switch (reg) {
	/* I2S Registers */
	case IAXXX_I2S_I2S_TRIGGER_GEN_ADDR:
	/* PCM registers */
	case IAXXX_PCM0_RIS_ADDR:
	case IAXXX_PCM1_RIS_ADDR:
	case IAXXX_PCM2_RIS_ADDR:
	/* Wall clock registers */
	case IAXXX_AF_WCPT_WALL_CLOCK_RD_0_ADDR:
	case IAXXX_AF_WCPT_WALL_CLOCK_RD_1_ADDR:
	/* SRB registers */
	case IAXXX_SRB_SYS_BLK_UPDATE_ADDR:
	case IAXXX_SRB_SYS_BLK_UPDATE_1_ADDR:
	case IAXXX_SRB_SYS_BLK_UPDATE_2_ADDR:
	case IAXXX_SRB_SYS_STATUS_ADDR:
	case IAXXX_SRB_FLASHUPD_STATUS_ADDR:
	case IAXXX_SRB_SYS_APP_VER_NUM_ADDR:
	case IAXXX_SRB_SYS_APP_VER_STR_ADDR:
	case IAXXX_SRB_SYS_ROM_VER_NUM_ADDR:
	case IAXXX_SRB_SYS_ROM_VER_STR_ADDR:
	case IAXXX_SRB_SYS_AUDIO_CLOCK_ADDR:
	case IAXXX_SRB_SYS_I2SM_CLOCK_ADDR:
	case IAXXX_SRB_SYS_POWER_CTRL_ADDR:
	/* Power registers */
	case IAXXX_SRB_PROC_ACTIVE_STATUS_ADDR:
	case IAXXX_SRB_DED_MEM_PWR_STATUS_ADDR:
	/* Checksum Regs */
	case IAXXX_SRB_CHKSUM_VAL1_ADDR:
	case IAXXX_SRB_CHKSUM_VAL2_ADDR:
	case IAXXX_SRB_CHKSUM_ADDR:
	/* Channel ARB Regs */
	case IAXXX_CH_HDR_CH_CNT_ADDR:
	case IAXXX_CH_HDR_CH_GAIN_ADDR:
	/* Stream ARB regs */
	case IAXXX_STR_HDR_STR_CNT_ADDR:
	/* Package ARB regs */
	case IAXXX_PKG_MGMT_PKG_REQ_ADDR:
	case IAXXX_PKG_MGMT_PKG_ERROR_ADDR:
	case IAXXX_PKG_MGMT_PKG_IADDR_P_ADDR:
	case IAXXX_PKG_MGMT_PKG_DADDR_P_ADDR:
	/* Tunnel ARB Regs */
	case IAXXX_TNL_HDR_TNL_COUNT_ADDR:
	/* Endpoints ARB Regs */
	case IAXXX_IN_EP_GRP_STATUS_ADDR:
	/* fallthrough */
	case IAXXX_PCTRL_REG_TRIG_DMX_INTR_ADDR:
	/* fallthrough */
	case IAXXX_PCTRL_REG_TRIG_HMD_INTR_ADDR:
	case IAXXX_CNR_SYS_CLK_SRC_SEL_ADDR:
		return true;
	}

	/* PCM Registers */
	if (reg >= IAXXX_PCM_BASE_ADDR(0) && reg < IAXXX_PCM_REGS_END_ADDR(0))
		return true;
	if (reg >= IAXXX_PCM_BASE_ADDR(1) && reg < IAXXX_PCM_REGS_END_ADDR(1))
		return true;
	if (reg >= IAXXX_PCM_BASE_ADDR(2) && reg < IAXXX_PCM_REGS_END_ADDR(2))
		return true;

	/* I2S Registers */
	if (reg >= IAXXX_I2S_I2S0_HL_ADDR &&
			reg <= IAXXX_I2S_I2S_TRIGGER_GEN_ADDR)
		return true;
	/* SRB event registers */
	if (reg >= IAXXX_EVT_MGMT_EVT_ADDR &&
		reg <= IAXXX_EVT_MGMT_EVT_DST_OPAQUE_ADDR)
		return true;
	/* AO registers */
	if (reg >= IAXXX_AO_REGS_ADDR && reg <= IAXXX_AO_REGS_END_ADDR)
		return true;
	/* CNR registers */
	if (reg >= IAXXX_CNR0_REGS_ADDR && reg <= IAXXX_CNR0_REGS_END_ADDR)
		return true;
	/* IOCTL registers */
	if (reg >= IAXXX_IO_CTRL_REGS_ADDR && reg <= IAXXX_IOCTRL_REGS_END_ADDR)
		return true;
	/* PAD CTRL registers */
	if (reg >= IAXXX_PAD_CTRL_REGS_ADDR && reg <
		IAXXX_PAD_CTRL_REGS_END_ADDR)
		return true;
	/* GPIO registers */
	if (reg >= IAXXX_GPIO_REGS_ADDR && reg < IAXXX_GPIO_REGS_END_ADDR)
		return true;
	/* Tunnel header registers */
	if (reg >= IAXXX_TNL_HDR_TNL_OUT_BUF_SIZE_ADDR &&
			reg <= IAXXX_TNL_HDR_TNL_OUT_BUF_TAIL_ADDR)
		return true;

	for (i = 0; i < priv->rbdt_info[IAXXX_BLOCK_PACKAGE].num_of_grp;
	     i++) {
		if (reg == IAXXX_GET_GRP_ADDR(priv, IAXXX_PKG_GRP_PKG_INFO_ADDR,
					      i))
			return true;

		if (reg == IAXXX_GET_GRP_ADDR(priv,
				IAXXX_PKG_GRP_PKG_VER_STR_ADDR, i))
			return true;
	}

	/* Stream group registers */
	for (i = 0; i < priv->rbdt_info[IAXXX_BLOCK_STREAM].num_of_grp;
	     i++) {
		if (reg == IAXXX_GET_GRP_ADDR(priv,
				IAXXX_STR_GRP_STR_STATUS_ADDR, i))
			return true;
		if ((reg >= IAXXX_GET_GRP_ADDR(priv,
			IAXXX_STR_GRP_STR_AF_ERR_AFS_FIFO_OVERFLOW_CNT_ADDR,
			i)) && (reg <= IAXXX_GET_GRP_ADDR(priv,
				IAXXX_STR_GRP_STR_AF_ERR_ACCESS_CNT_ADDR, i)))
			return true;
	}

	/* Plugin group registers */
	for (i = 0; i < priv->rbdt_info[IAXXX_BLOCK_PLUGIN].num_of_grp;
	     i++) {
		if (reg >= IAXXX_GET_GRP_ADDR(priv,
			IAXXX_PLUGIN_INS_GRP_PARAM_ADDR, i) &&
			reg <= IAXXX_GET_GRP_ADDR(priv,
			IAXXX_PLUGIN_INS_GRP_PLUGINLOG_SIZE_ADDR, i))
			return true;
	}

	/* Plugin Header registers*/
	for (i = 0; i < IAXXX_PLUGIN_UPDATE_BLOCKS_MAX; i++) {
		if (reg >= IAXXX_PLUGIN_HDR_ENABLE_BLOCK_ADDR(i) &&
			reg <= IAXXX_PLUGIN_HDR_ERROR_INS_ID_BLOCK_ADDR(i))
			return true;
	}

	/* Channel group registers */
	for (i = 0; i < priv->rbdt_info[IAXXX_BLOCK_CHANNEL].num_of_grp;
	     i++) {
		if (reg == IAXXX_GET_GRP_ADDR(priv,
				IAXXX_OUT_CH_GRP_CH_GAIN_STATUS_ADDR, i))
			return true;
		if (reg == IAXXX_GET_GRP_ADDR(priv,
				IAXXX_IN_CH_GRP_CH_GAIN_STATUS_ADDR, i))
			return true;
		if ((reg >= IAXXX_GET_GRP_ADDR(priv,
			IAXXX_OUT_CH_GRP_CH_PEAK_ADDR, i)) &&
			(reg <= IAXXX_GET_GRP_ADDR(priv,
			IAXXX_OUT_CH_GRP_CH_DROP_CNT_ADDR, i)))
			return true;
		if ((reg >= IAXXX_GET_GRP_ADDR(priv,
			IAXXX_IN_CH_GRP_CH_PEAK_ADDR, i)) &&
			(reg <= IAXXX_GET_GRP_ADDR(priv,
			IAXXX_IN_CH_GRP_CH_DROP_CNT_ADDR, i)))
			return true;
	}

	/* Tunnel group registers */
	for (i = 0; i < priv->rbdt_info[IAXXX_BLOCK_TUNNEL].num_of_grp;
	     i++) {
		if ((reg >= IAXXX_GET_GRP_ADDR(priv,
			IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_ADDR, i)) &&
			(reg <= IAXXX_GET_GRP_ADDR(priv,
			IAXXX_IN_TNL_GRP_TNL_NRECVD_ADDR, i)))
			return true;

		if ((reg >= IAXXX_GET_GRP_ADDR(priv,
			IAXXX_OUT_TNL_GRP_TNL_NFRAME_DROPS_ADDR, i)) &&
			(reg <= IAXXX_GET_GRP_ADDR(priv,
			IAXXX_OUT_TNL_GRP_TNL_NRECVD_ADDR, i)))
			return true;
	}

	if (reg >= IAXXX_DEBUG_REGS_ADDR && reg <=
			IAXXX_DEBUG_DEBUGREG_HISTLOG_SIZE_ADDR)
		return true;

	if (reg >= IAXXX_SRB_ARB_0_BASE_ADDR_ADDR && reg <=
	    IAXXX_SRB_ARB_31_INFO_ADDR)
		return true;
	/* All Power Management Registers */
	if (reg >= IAXXX_PWR_MGMT_REGS_ADDR && reg <
	    (IAXXX_PWR_MGMT_REGS_ADDR + (IAXXX_PWR_MGMT_REG_NUM * 4)))
		return true;

	if (reg == IAXXX_SCRIPT_MGMT_SCRIPT_ADDR_ADDR)
		return true;

	return false;
}

static bool iaxxx_application_volatile_reg_no_pm
		(struct device *dev, unsigned int reg)
{
	int i;

	/* All SRB Registers except SRB Block Selct Register.
	 * SRB Block Select register is excluded because FW does
	 * not require Page Select to be able to read/write
	 */
	if ((reg != SRB_BLOCK_SELECT_REG) && reg >= IAXXX_SRB_BASE &&
		reg < (IAXXX_SRB_BASE + IAXXX_SRB_SIZE))
		return true;

	/* All Power Management Registers */
	if (reg >= IAXXX_PWR_MGMT_REGS_ADDR && reg <
		(IAXXX_PWR_MGMT_REGS_ADDR + IAXXX_PWR_MGMT_REG_NUM*4))
		return true;

	/* All event Management Registers */
	if (reg >= IAXXX_EVT_MGMT_EVT_ADDR &&
		reg < (IAXXX_EVT_MGMT_EVT_ADDR + (IAXXX_EVT_MGMT_REG_NUM*4)))
		return true;

	/* Plugin header register */
	for (i = 0; i < IAXXX_PLUGIN_UPDATE_BLOCKS_MAX; i++) {
		if (reg == IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(i))
			return true;
	}

	return false;
}

/*
 * readable_register needed to optimize access and bus usage
 */
static bool iaxxx_readable_register_no_pm(struct device *dev, unsigned int reg)
{

	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int i;

	if (WARN_ON(!priv))
		return false;	/* Something went wrong */

	/* All SRB Registers */
	if (reg >= IAXXX_SRB_BASE && reg < (IAXXX_SRB_BASE + IAXXX_SRB_SIZE))
		return true;

	/* All Power Management Registers */
	if (reg >= IAXXX_PWR_MGMT_REGS_ADDR && reg <=
		(IAXXX_PWR_MGMT_REGS_ADDR + IAXXX_PWR_MGMT_REG_NUM*4))
		return true;

	/* All event Management Registers */
	if (reg >= IAXXX_EVT_MGMT_EVT_ADDR &&
		reg < (IAXXX_EVT_MGMT_EVT_ADDR + (IAXXX_EVT_MGMT_REG_NUM*4)))
		return true;

	/* Electrical control register used during boot */
	if (reg == IAXXX_AO_EFUSE_BOOT_ADDR)
		return true;

	if (reg >= IAXXX_PCTRL_REGS_ADDR &&
	    reg <= IAXXX_PCTRL_REG_TRIG_DMX_INTR_ADDR)
		return true;

	if (reg == IAXXX_CNR_SYS_CLK_SRC_SEL_ADDR)
		return true;

	/* Plugin header register */
	for (i = 0; i < IAXXX_PLUGIN_UPDATE_BLOCKS_MAX; i++) {
		if (reg == IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(i))
			return true;
	}

	return false;
}

/* To reset the ARB, SRB and HW regmap cache */
int iaxxx_regmap_drop_regions(struct iaxxx_priv *priv)
{
	uint32_t start_addr, end_addr;
	int rc;
	int i;
	int phy_addr_size = sizeof(iaxxx_phy_addr_ranges) /
					sizeof(iaxxx_phy_addr_ranges[0]);

	for (i = 0; i < phy_addr_size; i++) {
		rc = regcache_drop_region(priv->regmap,
				iaxxx_phy_addr_ranges[i][0],
				iaxxx_phy_addr_ranges[i][1]);
		if (rc != 0) {
			dev_err(priv->dev,
			"%s: drop physical[%d] registers failed : %d\n",
			__func__, i, rc);
			goto drop_regions_exit;
		}
	}

	/* regmap cache drop */
	for (i = 0 ; i < priv->regmap_config->num_ranges; i++) {
		start_addr = priv->regmap_config->ranges[i].range_min;
		end_addr = priv->regmap_config->ranges[i].range_min +
			priv->regmap_config->ranges[i].window_len;
		rc = regcache_drop_region(priv->regmap, start_addr, end_addr);
		if (rc != 0) {
			dev_err(priv->dev,
			"%s: drop arb range[%d] registers failed : %d\n",
			__func__, i, rc);
			goto drop_regions_exit;
		}
	}

drop_regions_exit:
	return rc;
}

/*
 * Use ranges to define the Application Register Blocks
 * This needs to be populated at boot-time after firmware download.
 * This is just a template, we copy this over to allocated memory
 */
static const struct regmap_range_cfg iaxxx_ranges[] = {
	{
		.name = "Channel",
		.range_min = IAXXX_REG_CHANNEL_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Stream",
		.range_min = IAXXX_REG_STREAM_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Tunnel",
		.range_min = IAXXX_REG_TUNNEL_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Package Management",
		.range_min = IAXXX_REG_PACKAGE_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Plugin Instance",
		.range_min = IAXXX_REG_PLUGIN_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Crash Log",
		.range_min = IAXXX_REG_DBGLOG_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Event",
		.range_min = IAXXX_REG_EVENT_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Power",
		.range_min = IAXXX_REG_POWER_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Script",
		.range_min = IAXXX_REG_SCRIPT_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},

};

static struct regmap_config iaxxx_regmap_config = {
	.name = IAXXX_REGMAP_NAME,
	.reg_bits = 32,		/* 32-bit register offsets */
	.val_bits = 32,		/* 32-bit register values */
	.max_register = IAXXX_MAX_REGISTER,
	.readable_reg = iaxxx_readable_register,
	.writeable_reg = iaxxx_writeable_register,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.volatile_reg = iaxxx_volatile_register,
	.cache_type = REGCACHE_RBTREE,

	/* These will be set before the second regmap_init() */
	/*.ranges = iaxxx_ranges,
	 * .num_ranges = ARRAY_SIZE(iaxxx_ranges),
	 */
};

static struct regmap_config iaxxx_regmap_no_pm_config = {
	.name = IAXXX_REGMAP_NO_PM_NAME,
	.reg_bits = 32,		/* 32-bit register offsets */
	.val_bits = 32,		/* 32-bit register values */
	.max_register = IAXXX_MAX_REGISTER,
	.readable_reg = iaxxx_readable_register_no_pm,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.volatile_reg = iaxxx_volatile_register,
	.cache_type = REGCACHE_NONE,
};

/**
 * get_rbdt_block_count: determines the number of RBDT blocks to be mapped
 *
 * @priv: iaxxx private data
 *
 * The iaxxx_ranges table above defines the known Application Register
 * Blocks. This function will search through the Register Block
 * Descriptor Table and count the number of blocks that match the blocks
 * in iaxxx_ranges. This is used to determine how much space to allocate
 * for the range configuration table.
 *
 * Returns the number of blocks in the RBDT that match the range table.
 */
static inline unsigned int get_rbdt_block_count(struct iaxxx_priv *priv)
{
	int i;
	unsigned int num_blks = 0;
	unsigned int blk_index;
	const struct regmap_range_cfg *def_range_cfg;

	/* Count the number of known relocated blocks */
	for (i = 0; i < ARRAY_SIZE(iaxxx_ranges); ++i) {
		def_range_cfg = &iaxxx_ranges[i];
		blk_index = IAXXX_INDEX_FROM_VIRTUAL(def_range_cfg->range_min);
		WARN_ON(blk_index >= IAXXX_RBDT_NUM_ENTRIES);

		/*
		 * Number of blocks is equal to num of ARBs header having
		 * non-zero size + num of ARBs group having non-zero group size
		 * Num of blocks is those many regmap ranges has to be defined
		 */
		if (priv->rbdt_info[blk_index].hdr_size)
			++num_blks;

		if (priv->rbdt_info[blk_index].grp_size)
			++num_blks;
	}

	WARN_ON(num_blks > IAXXX_RBDT_NUM_ENTRIES);
	return num_blks;
}

/**
 * iaxxx_update_relocatable_blocks: builds regmap range configuration data
 *
 * @priv: iaxxx private data
 *
 * The regmap framework provides the concept of range configurations that can
 * be used for indirectly accessed or paged registers. This mechanism will be
 * used to let regmap handle the virtual-to-physical address translation.
 * The range_min and range_max fields specify the min and max virtual register
 * addresses for a given Application Register Block. The window_start and
 * window_len fields specify the physical address and length of the physical
 * register block.
 *
 * The Register Descriptor Block Table (RBDT) is index by Block Index and
 * provides the physical start address and length for each Application
 * Register Block (ARB).
 *
 * The range configuration table is allocated, initialized, and then assigned
 * to the regmap configuration.
 */
static int iaxxx_update_relocatable_blocks(struct iaxxx_priv *priv)
{
	int i;
	uint32_t hdr_addr, hdr_size, grp_addr, grp_size;
	unsigned int nblks;
	unsigned long blk_index;

	struct device *dev = priv->dev;
	struct regmap_range_cfg *range_cfg;
	struct regmap_range_cfg *range_cfg_1;
	struct regmap_range_cfg *curr_range_cfg;
	struct regmap_range_cfg *curr_range_cfg_1;
	const struct regmap_range_cfg *def_range_cfg;

	/* Get the count of the number of valid RBDT entries */
	nblks = get_rbdt_block_count(priv);
	if (!nblks) {
		dev_warn(dev, "%s: No relocatable blocks\n", __func__);
		return -EIO;
	}

	/* Allocate a range configuration for the number of blocks */
	range_cfg = devm_kzalloc(dev, nblks * sizeof(*range_cfg), GFP_KERNEL);
	if (!range_cfg)
		return -ENOMEM;

	/* Allocate a range configuration for second regmap */
	range_cfg_1 = devm_kzalloc(dev,
			IAXXX_REGMAP_NO_PM_NUM_ARBS * sizeof(*range_cfg),
			GFP_KERNEL);
	if (!range_cfg_1) {
		devm_kfree(priv->dev, range_cfg);
		return -ENOMEM;
	}

	/* Build the range configuration table */
	curr_range_cfg = range_cfg;
	curr_range_cfg_1 = range_cfg_1;

	for (i = 0; i < ARRAY_SIZE(iaxxx_ranges); ++i) {
		def_range_cfg = &iaxxx_ranges[i];
		blk_index = IAXXX_INDEX_FROM_VIRTUAL(def_range_cfg->range_min);

		/* Read the physical address from the RBDT */
		hdr_addr = priv->rbdt_info[blk_index].hdr_base_addr;
		hdr_size = priv->rbdt_info[blk_index].hdr_size *
			priv->rbdt_info[blk_index].num_of_hdr;
		dev_info(priv->dev, "ARB Header[%lu] = 0x%.08x, size = %d bytes",
				 blk_index, hdr_addr, hdr_size);

		grp_addr = priv->rbdt_info[blk_index].grp_base_addr;
		grp_size = priv->rbdt_info[blk_index].grp_size *
			priv->rbdt_info[blk_index].num_of_grp;
		dev_info(priv->dev, "ARB Group[%lu] = 0x%.08x, size = %d bytes",
				 blk_index, grp_addr, grp_size);

		/* Define range for ARB Header */
		if (hdr_size) {
			/* Copy over the defaults */
			*curr_range_cfg = *def_range_cfg;

			curr_range_cfg->window_len = hdr_size;
			curr_range_cfg->window_start = hdr_addr;

			curr_range_cfg->range_max =
				curr_range_cfg->range_min +
				hdr_size - sizeof(u32);

			++curr_range_cfg;

			/*
			 * Second regmap only has range for below ARBS, which
			 * might be needed in pm ops
			 */
			if ((blk_index == IAXXX_BLOCK_POWER) ||
				(blk_index == IAXXX_BLOCK_EVENT) ||
				(blk_index == IAXXX_BLOCK_STREAM) ||
				(blk_index == IAXXX_BLOCK_PLUGIN)) {
				*curr_range_cfg_1 = *def_range_cfg;
				curr_range_cfg_1->window_len = hdr_size;
				curr_range_cfg_1->window_start = hdr_addr;
				curr_range_cfg_1->range_max =
						curr_range_cfg_1->range_min +
						hdr_size - sizeof(u32);
				++curr_range_cfg_1;
			}
		}

		/* Define range for ARB Group */
		if (grp_size) {
			/* Copy over the defaults */
			*curr_range_cfg = *def_range_cfg;

			curr_range_cfg->window_len = grp_size;
			curr_range_cfg->window_start = grp_addr;
			/*
			 * Both header and group base address starts with 0
			 * To differentiate between them, 23rd bit is set for
			 * group addresses. By default range_min is set to
			 * header base address. As this range is for group,
			 * setting the 23rd bit in the base address.
			 */
			curr_range_cfg->range_min |= IAXXX_ARB_GRP_INDEX;
			curr_range_cfg->range_max =
			curr_range_cfg->range_min + grp_size - sizeof(u32);

			++curr_range_cfg;

			/* Second regmap only has range for below ARBS, which
			 * might be needed in pm ops
			 */
			if ((blk_index == IAXXX_BLOCK_POWER) ||
				(blk_index == IAXXX_BLOCK_EVENT) ||
				(blk_index == IAXXX_BLOCK_STREAM) ||
				(blk_index == IAXXX_BLOCK_PLUGIN)) {
				*curr_range_cfg_1 = *def_range_cfg;
				curr_range_cfg_1->window_len = grp_size;
				curr_range_cfg_1->window_start = grp_addr;
				/*
				 * Min. address for Group i.e. header min
				 * address with 23rd bit set
				 */
				curr_range_cfg_1->range_min |=
					IAXXX_ARB_GRP_INDEX;
				curr_range_cfg_1->range_max =
					curr_range_cfg_1->range_min +
					grp_size - sizeof(u32);
				++curr_range_cfg_1;
			}
		}
	}

	priv->regmap_config->ranges = range_cfg;
	priv->regmap_config->num_ranges = nblks;

	if ((curr_range_cfg - range_cfg) != nblks) {
		dev_err(dev, "%s: current range cfg check fail\n", __func__);
		dev_err(dev,
			"%s: (curr_range_cfg - range_cfg) %d != nblks %d\n",
			__func__, (u32)(curr_range_cfg - range_cfg), nblks);
		return -EINVAL;
	}

	priv->regmap_no_pm_config->ranges = range_cfg_1;
	priv->regmap_no_pm_config->num_ranges = IAXXX_REGMAP_NO_PM_NUM_ARBS;

	if ((curr_range_cfg_1 - range_cfg_1) != IAXXX_REGMAP_NO_PM_NUM_ARBS) {
		dev_err(dev, "%s: current range cfg 1 check fail\n", __func__);
		dev_err(dev,
			"%s: (curr_range_cfg_1 - range_cfg_1) %d != nblks %d\n",
			__func__, (u32)(curr_range_cfg_1 - range_cfg_1),
			IAXXX_REGMAP_NO_PM_NUM_ARBS);
		return -EINVAL;
	}

	return 0;
}

/**
 * iaxxx_conv_physical_to_virtual_register_address
 *   - Converts register physical address to its virtual address
 *
 * @priv: iaxxx private data
 * @phy_addr: register physical address
 *
 * Loops through all configured regmap ranges and see where
 * the physical address falls in. If so, calculate the virtual
 * address using range start address and offset from start address.
 *
 * For example: Channel count virtual address is 0x01000000 and
 * physical address is 0x40090120, so for 0x40090120 address
 * this API will return 0x01000000
 *
 */
uint32_t iaxxx_conv_physical_to_virtual_register_address(
		struct iaxxx_priv *priv,
		const uint32_t phy_addr)
{
	int i;
	uint32_t virt_addr;
	uint32_t hdr_addr, hdr_size, grp_addr, grp_size;
	unsigned int blk_index;
	const struct regmap_range_cfg *def_range_cfg;

	/* Using the array of virtual address ranges and
	 * ARB header base address, size of each header, no. of headers
	 * and group base address, size of each group, no. of groups
	 * array,
	 * find the virtual address for the physical address
	 */
	for (i = 0; i < ARRAY_SIZE(iaxxx_ranges); ++i) {
		def_range_cfg = &iaxxx_ranges[i];
		blk_index = IAXXX_INDEX_FROM_VIRTUAL(def_range_cfg->range_min);
		if (blk_index < IAXXX_RBDT_NUM_ENTRIES) {
			hdr_addr = priv->rbdt_info[blk_index]. hdr_base_addr;
			hdr_size = priv->rbdt_info[blk_index].hdr_size *
				priv->rbdt_info[blk_index].num_of_hdr;

			/* If physical address is in header address range */
			if (hdr_addr && hdr_size && (phy_addr >= hdr_addr) &&
				(phy_addr < hdr_addr + hdr_size)) {

				virt_addr = phy_addr - hdr_addr;
				virt_addr += def_range_cfg->range_min;
				return virt_addr;
			} else {
				/*
				 * If physical address is not in header address
				 * range, then look into group address range
				 */
				grp_addr =
				priv->rbdt_info[blk_index].grp_base_addr;
				grp_size = priv->rbdt_info[blk_index].grp_size *
					priv->rbdt_info[blk_index].num_of_grp;

				if (grp_addr && grp_size &&
				    (phy_addr >= grp_addr) &&
				    (phy_addr < grp_addr + grp_size)) {

					virt_addr = phy_addr - grp_addr;
					/*
					 * Set the Grp addr identification bit
					 */
					virt_addr += def_range_cfg->range_min |
						IAXXX_ARB_GRP_INDEX;
					return virt_addr;
				}
			}
		}
	}

	return phy_addr;
}

/**
 * iaxxx_conv_virtual_to_physical_register_address
 *   - Converts register virtual address to its physical address
 *
 * @priv: iaxxx private data
 * @phy_addr: register virtual address
 *
 */
uint32_t iaxxx_conv_virtual_to_physical_register_address(
		struct iaxxx_priv *priv,
		const uint32_t virt_addr)
{
	/* Get ARB block index to get the physical address
	 * it is mapped to
	 */
	int blk_index = IAXXX_INDEX_FROM_VIRTUAL(virt_addr);
	bool is_group = virt_addr & IAXXX_ARB_GRP_INDEX;
	uint32_t virt_base_addr = IAXXX_VIRTUAL_BASE_ADDR(blk_index) | is_group;
	uint32_t offset = virt_addr - virt_base_addr;

	/* Return the physical address from the RBDT +
	 * offset of the virtual address from its base
	 */
	if (is_group)
		return priv->rbdt_info[blk_index].grp_base_addr + offset;
	else
		return priv->rbdt_info[blk_index].hdr_base_addr + offset;
}

/**
 * iaxxx_update_rbdt - updates the cached copy of the RBDT
 *
 * @priv: iaxxx private data
 *
 * Saves a copy of the Register Block Descriptor Table (RBDT) to the sys_rbdt
 * fields of the driver private data.
 */
static int iaxxx_update_rbdt(struct iaxxx_priv *priv)
{
	int rc, i;
	struct device *dev = priv->dev;
	uint32_t sys_rbdt[2 * IAXXX_RBDT_NUM_ENTRIES];

	rc = regmap_bulk_read(priv->regmap_no_pm,
			      IAXXX_SRB_ARB_0_BASE_ADDR_ADDR,
				&sys_rbdt, ARRAY_SIZE(sys_rbdt));
	if (rc) {
		dev_err(dev, "%s: regmap_read failed : %d\n", __func__, rc);
		return rc;
	}

	for (i = 0; i < IAXXX_RBDT_NUM_ENTRIES; i++) {
		/*
		 * sys_rbdt has each ARB Address and Info values.
		 * Even index has address and odd index has info value.
		 * Thats why 2 * i is used
		 */
		priv->rbdt_info[i].hdr_base_addr = sys_rbdt[2 * i];
		priv->rbdt_info[i].hdr_size = (sys_rbdt[2 * i + 1] &
				IAXXX_SRB_ARB_0_INFO_HDR_SIZE_MASK) >>
				IAXXX_SRB_ARB_0_INFO_HDR_SIZE_POS;
		priv->rbdt_info[i].num_of_hdr = (sys_rbdt[2 * i + 1] &
				IAXXX_SRB_ARB_0_INFO_NUM_HDR_MASK) >>
				IAXXX_SRB_ARB_0_INFO_NUM_HDR_POS;
		priv->rbdt_info[i].grp_size = (sys_rbdt[2 * i + 1] &
				IAXXX_SRB_ARB_0_INFO_GRP_SIZE_MASK) >>
				IAXXX_SRB_ARB_0_INFO_GRP_SIZE_POS;
		priv->rbdt_info[i].num_of_grp = (sys_rbdt[2 * i + 1] &
				IAXXX_SRB_ARB_0_INFO_NUM_GRP_MASK) >>
				IAXXX_SRB_ARB_0_INFO_NUM_GRP_POS;
		if (priv->rbdt_info[i].grp_size)
			priv->rbdt_info[i].grp_base_addr =
				priv->rbdt_info[i].hdr_base_addr +
				priv->rbdt_info[i].hdr_size;
		pr_debug("dev id-%d: ARB %d Hdr addr 0x%x Hdr size %d Num of hdr %d, Grp addr 0x%x Grp size %d Num of Grp %d",
			priv->dev_id, i, priv->rbdt_info[i].hdr_base_addr,
			priv->rbdt_info[i].hdr_size,
			priv->rbdt_info[i].num_of_hdr,
			priv->rbdt_info[i].grp_base_addr,
			priv->rbdt_info[i].grp_size,
			priv->rbdt_info[i].num_of_grp);
	}

	return rc;
}

/* Register map initialization */
int iaxxx_regmap_init(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s: enter\n", __func__);

	if (!priv->regmap_init_bus) {
		dev_err(dev, "%s: missing regmap_init func\n", __func__);
		return -EINVAL;
	}

	priv->regmap_config = &iaxxx_regmap_config;
	priv->regmap_no_pm_config = &iaxxx_regmap_no_pm_config;

	return priv->regmap_init_bus(priv);
}

int iaxxx_sbl_regmap_init(struct iaxxx_priv *priv)
{
	/* Free the existing regmaps */
	if (priv->regmap) {
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}

	if (priv->regmap_no_pm) {
		regmap_exit(priv->regmap_no_pm);
		priv->regmap_no_pm = NULL;
	}

	priv->regmap_config->volatile_reg = iaxxx_volatile_register;
	priv->regmap_no_pm_config->volatile_reg = iaxxx_volatile_register;

	priv->is_application_mode = false;

	return iaxxx_regmap_init(priv);
}

/**
 * iaxxx_application_regmap_init: application register map initialization
 *
 * @priv: iaxxx private data
 *
 * Once the firmware download has completed and the device has booted into
 * application mode, the regmap needs to be updated to include the application
 * registers that have virtual addresses. These registers are not available in
 * SBL mode.
 *
 * 1) Update the cached SRB register to include the new RBDT settings
 * 2) Release the SBL regmap (it served us well)
 * 3) Setup the range configurations for the Application Register Blocks
 * 4) Initialize a new regmap that includes the range configurations
 */
int iaxxx_application_regmap_init(struct iaxxx_priv *priv)
{
	int rc, retry_count = 0;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s: enter\n", __func__);

retry_update_rdbt:
	/* Update System Register Block using the SBL regmap. */
	rc = iaxxx_update_rbdt(priv);
	if (rc) {
		dev_err(dev, "%s: update SRB failed : %d\n", __func__, rc);
		goto err;
	}

	/* Update regmap configuration for the relocatable blocks */
	rc = iaxxx_update_relocatable_blocks(priv);
	if (rc) {
		/*
		 * We tried to populate the ARBs but it failed. This indicate
		 * that the fw is not ready. Lets wait for a while and try
		 * reading it again.
		 */
		usleep_range(IAXXX_RDBT_READ_DELAY,
				IAXXX_RDBT_READ_DELAY + IAXXX_RDBT_READ_RANGE);

		/* retry the ARB population by reading it again. */
		if (++retry_count < IAXXX_RDBT_RETRY_COUNT) {
			dev_info(dev, "%s: retyring the iaxxx_update_rbdt\n",
							__func__);
			goto retry_update_rdbt;
		}

		dev_err(dev, "%s: range cfg retry failed : %d\n", __func__, rc);
		goto err;
	}

	/* Free the existing regmaps */
	if (priv->regmap) {
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}

	if (priv->regmap_no_pm) {
		regmap_exit(priv->regmap_no_pm);
		priv->regmap_no_pm = NULL;
	}

	priv->regmap_config->volatile_reg = iaxxx_application_volatile_reg;
	priv->regmap_no_pm_config->volatile_reg =
			iaxxx_application_volatile_reg_no_pm;

	priv->is_application_mode = true;

	/* Initialize the "Application" regmap */
	rc = iaxxx_regmap_init(priv);
	if (rc) {
		devm_kfree(dev, (void *)priv->regmap_config->ranges);
		priv->regmap_config->ranges = &iaxxx_ranges[0];
		priv->regmap_config->num_ranges = 0;
		devm_kfree(priv->dev,
				(void *)priv->regmap_no_pm_config->ranges);
		priv->regmap_no_pm_config->ranges = NULL;
	}

err:
	return rc;
}
