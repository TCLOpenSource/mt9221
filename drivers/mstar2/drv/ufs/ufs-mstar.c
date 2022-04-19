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

#include <linux/of.h>
#include <linux/platform_device.h>

#include "ufshcd.h"
#include "ufshcd-pltfrm.h"
#include "unipro.h"
#include "ufshci.h"
#include "ufs.h"
#include "ufs-mstar.h"

static struct ufs_mstar_host ufs_mstar_hosts[MAX_UFS_MSTAR_HOSTS];

void dumpMemory4(const char *str, const void *data, size_t size)
{
	print_hex_dump(KERN_ERR, str, DUMP_PREFIX_OFFSET, 16, 4, data, size, false);
}

#ifdef CONFIG_MSTAR_UFS_DEBUG
static inline void print_prd(struct ufshcd_lrb *lrbp)
{
	dumpMemory4("PRDT : ", lrbp->ucd_prdt_ptr,
		    sizeof(lrbp->ucd_prdt_ptr[0]) *
		    le16_to_cpu(lrbp->utr_descriptor_ptr->prd_table_length));
}

static void print_query_function(const void *upiu)
{
	const unsigned char *req_upiu = upiu;
	u8 opcode = req_upiu[12];
	u8 idn = req_upiu[13];
	u8 index = req_upiu[14];
	char *opcode_name;

	switch (opcode) {
	case UPIU_QUERY_OPCODE_READ_DESC:
		opcode_name = "read descriptor";
		break;
	case UPIU_QUERY_OPCODE_WRITE_DESC:
		opcode_name = "write descriptor";
		break;
	case UPIU_QUERY_OPCODE_READ_ATTR:
		opcode_name = "read attribute";
		break;
	case UPIU_QUERY_OPCODE_WRITE_ATTR:
		opcode_name = "write attribute";
		break;
	case UPIU_QUERY_OPCODE_READ_FLAG:
		opcode_name = "read flag";
		break;
	case UPIU_QUERY_OPCODE_SET_FLAG:
		opcode_name = "set flag";
		break;
	case UPIU_QUERY_OPCODE_CLEAR_FLAG:
		opcode_name = "clear flag";
		break;
	case UPIU_QUERY_OPCODE_TOGGLE_FLAG:
		opcode_name = "toggle flag";
		break;
	default:
		opcode_name = "unknown opcde";
		break;
	}
	pr_err("%s: IDN %02x INDEX %02x\n", opcode_name, idn, index);
}

void ufs_mstar_print_request(struct ufshcd_lrb *lrbp)
{
	struct utp_upiu_req *ucd_req_ptr = lrbp->ucd_req_ptr;
	struct utp_transfer_req_desc *req_desc = lrbp->utr_descriptor_ptr;
	u8 req = be32_to_cpu(ucd_req_ptr->header.dword_0) >> 24;

	pr_err("-------------------------------------------\n");
	pr_err("                [request]\n");
	dumpMemory4("UTRD : ", req_desc, sizeof(*req_desc));
	dumpMemory4("CMD UPIU : ", ucd_req_ptr, sizeof(*ucd_req_ptr));

	switch (req) {
	case UPIU_TRANSACTION_COMMAND:
			//scsi_print_command(ucd_req_ptr->sc.cdb);
			print_prd(lrbp);
		break;
	case UPIU_TRANSACTION_QUERY_REQ:
			print_query_function(ucd_req_ptr);
		break;
	}
}

void ufs_mstar_print_response(struct ufshcd_lrb *lrbp)
{
	struct utp_upiu_rsp *ucd_rsp_ptr = lrbp->ucd_rsp_ptr;
	struct utp_transfer_req_desc *req_desc = lrbp->utr_descriptor_ptr;
	u8 req = be32_to_cpu(lrbp->ucd_req_ptr->header.dword_0) >> 24;

	pr_err("                [response]\n");
	dumpMemory4("UTRD : ", req_desc, sizeof(*req_desc));
	dumpMemory4("RSP UPIU: ", ucd_rsp_ptr, sizeof(*ucd_rsp_ptr));

	if (req == UPIU_TRANSACTION_COMMAND)
		print_prd(lrbp);

	pr_err("-------------------------------------------\n");
}

void dump_ufshci_reg(struct ufs_hba *hba)
{
	int reg;
	unsigned int ua32_hci_reg[0xA0>>2];

	for (reg=0 ; reg<0xA0 ; reg+=4)
		ua32_hci_reg[reg>>2] = ufshcd_readl(hba, reg);

	pr_err("\n-------------- UFSHCI REGISTER --------------\n");
	print_hex_dump(KERN_ERR, "",
		DUMP_PREFIX_OFFSET, 16, 4, ua32_hci_reg, 0xA0, false);
}
#endif

static int ufs_mstar_link_startup_pre_change(struct ufs_hba *hba)
{
	//struct ufs_mstar_host *host = ufshcd_get_variant(hba);

	int err = 0;

	return err;
}

static int ufs_mstar_link_startup_post_change(struct ufs_hba *hba)
{
	//struct ufs_mstar_host *host = ufshcd_get_variant(hba);

	int err = 0;

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x3000), 0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x3001), 1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x4025), 6);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x4021), 1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x4020), 1);

	return err;
}

static int ufs_mstar_link_startup_notify(struct ufs_hba *hba, enum ufs_notify_change_status status)
{
	int err = 0;
	//struct ufs_mstar_host *host = ufshcd_get_variant(hba);

	switch (status) {
	case PRE_CHANGE:
		err = ufs_mstar_link_startup_pre_change(hba);
		break;
	case POST_CHANGE:
		err = ufs_mstar_link_startup_post_change(hba);
		break;
	default:
		dev_err((const struct device *)&hba->dev, "%s: invalid status %d\n", __func__, status);
		err = -EINVAL;
		break;
	}

	return err;
}

static int ufs_mstar_hce_enable_notify(struct ufs_hba *hba, enum ufs_notify_change_status status)
{
	//struct ufs_mstar_host *host = ufshcd_get_variant(hba);
	int err = 0;

	switch (status) {
	case PRE_CHANGE:

		break;
	case POST_CHANGE:

		break;
	default:
		dev_err((const struct device *)&hba->dev, "%s: invalid status %d\n", __func__, status);
		err = -EINVAL;
		break;
	}
	return err;
}

static int ufs_mstar_pwr_change_notify(struct ufs_hba *hba,
				enum ufs_notify_change_status status,
				struct ufs_pa_layer_attr *desired,
				struct ufs_pa_layer_attr *final)
{
	u8 buf1[16], buf2[16];

	if (PRE_CHANGE == status) {  /* before power mode change */
		memcpy(final, desired, sizeof(struct ufs_pa_layer_attr));
		if(final->pwr_rx == SLOWAUTO_MODE || final->pwr_rx == SLOW_MODE)
		{
			if(final->gear_rx > CAP_MSTAR_MAX_RX_PWM_GEAR)
				final->gear_rx = CAP_MSTAR_MAX_RX_PWM_GEAR;
		}
		else if (final->pwr_rx == FASTAUTO_MODE || final->pwr_rx == FAST_MODE)
		{
			if(final->gear_rx > CAP_MSTAR_MAX_RX_HS_GEAR)
				final->gear_rx = CAP_MSTAR_MAX_RX_HS_GEAR;
		}

		if(final->pwr_tx == SLOWAUTO_MODE || final->pwr_tx == SLOW_MODE)
		{
			if(final->gear_tx > CAP_MSTAR_MAX_TX_PWM_GEAR)
				final->gear_tx = CAP_MSTAR_MAX_TX_PWM_GEAR;
		}
		else if (final->pwr_tx == FASTAUTO_MODE || final->pwr_tx == FAST_MODE)
		{
			if(final->gear_tx > CAP_MSTAR_MAX_TX_HS_GEAR)
				final->gear_tx = CAP_MSTAR_MAX_TX_HS_GEAR;
		}
	}
	else
	{
		if(final->pwr_rx == FASTAUTO_MODE || final->pwr_rx == FAST_MODE)
			ufs_mstar_pltfrm_clock(UFSHCI_SW_DEFAULT_CLK, final->gear_rx, final->hs_rate);

		if(final->pwr_rx == FASTAUTO_MODE || final->pwr_rx == FAST_MODE)
			sprintf(buf1, "HS-GEAR%d", final->gear_rx);
		else
			sprintf(buf1, "PWM-GEAR%d", final->gear_rx);

		if(final->pwr_tx == FASTAUTO_MODE || final->pwr_tx == FAST_MODE)
			sprintf(buf2, "HS-GEAR%d", final->gear_tx);
		else
			sprintf(buf2, "PWM-GEAR%d", final->gear_tx);

		dev_err((const struct device *)hba->dev, "[%s] RX:%s, TX:%s\n", __func__, buf1, buf2);
	}

	return 0;
}

/**
 * ufs_mstar_advertise_quirks - advertise the known MSTAR UFS controller quirks
 * @hba: host controller instance
 *
 * MSTAR UFS host controller might have some non standard behaviours (quirks)
 * than what is specified by UFSHCI specification. Advertise all such
 * quirks to standard UFS host controller driver so standard takes them into
 * account.
 */
static void ufs_mstar_advertise_quirks(struct ufs_hba *hba)
{
	//struct ufs_mstar_host *host = ufshcd_get_variant(hba);
}


/**
 * ufs_mstar_init - bind phy with controller
 * @hba: host controller instance
 *
 * Binds PHY with controller and powers up PHY enabling clocks
 * and regulators.
 *
 * Returns -EPROBE_DEFER if binding fails, returns negative error
 * on phy power up failure and returns zero on success.
 */
static int ufs_mstar_init(struct ufs_hba *hba)
{
	int err = 0;
	struct ufs_mstar_host *host;

	err = ufs_mstar_pltfrm_init();
	if (err)
	{
		dev_err((const struct device *)&hba->dev, "ufs_mstar_init() failed %d\n", err);
		return err;
	}

	host = &ufs_mstar_hosts[0];

	/* Make a two way bind between the qcom host and the hba */
	host->hba = hba;
	ufshcd_set_variant(hba, host);

	ufs_mstar_advertise_quirks(hba);

	return err;
}

/**
 * struct ufs_hba_mstar_vops - UFS MSTAR specific variant operations
 *
 * The variant operations configure the necessary controller and PHY
 * handshake during initialization.
 */
static struct ufs_hba_variant_ops ufs_hba_mstar_vops = {
	.name					= "mstar",
	.init					= ufs_mstar_init,
	.exit					= NULL,
	.get_ufs_hci_version	= NULL,
	.clk_scale_notify		= NULL,
	.setup_clocks			= NULL,
	.setup_regulators		= NULL,
	.hce_enable_notify		= ufs_mstar_hce_enable_notify,
	.link_startup_notify	= ufs_mstar_link_startup_notify,
	.pwr_change_notify		= ufs_mstar_pwr_change_notify,
	.apply_dev_quirks		= NULL,
	.suspend				= NULL,
	.resume					= NULL,
	.dbg_register_dump		= NULL,
	.phy_initialization		= NULL,
	
};

/**
 * ufs_mstar_probe - probe routine of the driver
 * @pdev: pointer to Platform device handle
 *
 * Return zero for success and non-zero for failure
 */
static int ufs_mstar_probe(struct platform_device *pdev)
{
	int err;

	/* Perform generic probe */
	err = ufshcd_pltfrm_init(pdev, &ufs_hba_mstar_vops);
	if (err)
		dev_err((const struct device *)&pdev->dev, "ufshcd_pltfrm_init() failed %d\n", err);

	return err;
}

/**
 * ufs_qcom_remove - set driver_data of the device to NULL
 * @pdev: pointer to platform device handle
 *
 * Always returns 0
 */
static int ufs_mstar_remove(struct platform_device *pdev)
{
	struct ufs_hba *hba =  platform_get_drvdata(pdev);

	pm_runtime_get_sync(&(pdev)->dev);
	ufshcd_remove(hba);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id ufs_mstar_of_match[] = {
	{ .compatible = "mstar-ufs"},
	{},
};
#endif

static const struct dev_pm_ops ufs_mstar_pm_ops = {
	.suspend	= ufshcd_pltfrm_suspend,
	.resume		= ufshcd_pltfrm_resume,
	.runtime_suspend = ufshcd_pltfrm_runtime_suspend,
	.runtime_resume  = ufshcd_pltfrm_runtime_resume,
	.runtime_idle    = ufshcd_pltfrm_runtime_idle,
};

static struct platform_driver ufs_mstar_pltform = {
	.probe	= ufs_mstar_probe,
	.remove	= ufs_mstar_remove,
	.shutdown = ufshcd_pltfrm_shutdown,
	.driver	= {
		.name	= "mstar-ufs",
		.pm	= &ufs_mstar_pm_ops,
		#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(ufs_mstar_of_match),
		#endif
	},
};
module_platform_driver(ufs_mstar_pltform);
