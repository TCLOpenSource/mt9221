/*
 * iaxxx-regmap.h
 *
 * Copyright (c) 2020 Knowles, inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __IAXXX_REGMAP_H__
#define __IAXXX_REGMAP_H__

#define IAXXX_SRB_ARB_N_SIZE(N)		(IAXXX_SRB_ARB_0_INFO_ADDR+(N*8))
#define IAXXX_SRB_ARB_N_BASE_ADDRESS(N)	(IAXXX_SRB_ARB_0_BASE_ADDR_ADDR+(N*8))

#define IAXXX_MAX_REGISTER	  (IAXXX_SRB_ARB_N_SIZE(31))
#define IAXXX_SRB_SIZE		  ((IAXXX_MAX_REGISTER+4) - IAXXX_SRB_REGS_ADDR)

#define IAXXX_VIRTUAL_BASE_ADDR(B)				     ((B) << 24)
#define IAXXX_INDEX_FROM_VIRTUAL(A) ((A) >> 24)

#define IAXXX_BLOCK_CHANNEL				1
#define IAXXX_BLOCK_STREAM				2
#define IAXXX_BLOCK_TUNNEL				3
#define IAXXX_BLOCK_PACKAGE				4
#define IAXXX_BLOCK_PLUGIN				5
#define IAXXX_BLOCK_MEM_STAT				7
#define IAXXX_BLOCK_MIPS				8
#define IAXXX_BLOCK_DBGLOG				10
#define IAXXX_BLOCK_POWER                               14
#define IAXXX_BLOCK_EVENT				15
#define IAXXX_BLOCK_SCRIPT				16

/* To check group instance id is valid */
#define IAXXX_IS_VALID_GRP_ID(priv, index, inst_id) ({\
	int ret; \
	if (inst_id < priv->rbdt_info[index].num_of_grp) \
		ret = true; \
	else { \
		pr_err( \
		"dev id-%d: For ARB %d, invalid inst id %d, valid is < %d", \
		priv->dev_id, index, inst_id, \
		priv->rbdt_info[index].num_of_grp); \
		ret = false; \
	} \
	ret; \
})

/* To get group instance virtual address */
#define IAXXX_GET_GRP_ADDR(priv, virt_addr, \
			    inst_id) \
({ \
	int blk_index = IAXXX_INDEX_FROM_VIRTUAL(virt_addr); \
	uint32_t addr; \
	\
	if (!IAXXX_IS_VALID_GRP_ID(priv, blk_index, inst_id)) \
		addr = 0; \
	else \
		addr = (virt_addr + \
			(priv->rbdt_info[blk_index].grp_size * inst_id)); \
	addr; \
})

/* To check header instance id is valid */
#define IAXXX_IS_VALID_HDR_ID(priv, index, inst_id) ({\
	int ret; \
	if (inst_id < priv->rbdt_info[index].num_of_hdr) \
		ret = true; \
	else { \
		pr_err( \
		"dev id-%d: For ARB %d, invalid inst id %d, valid is < %d", \
		priv->dev_id, index, inst_id, \
		priv->rbdt_info[index].num_of_hdr); \
		ret = false; \
	} \
	ret; \
})

/* To get header instance virtual address */
#define IAXXX_GET_HDR_ADDR(priv, virt_addr, \
			    inst_id) \
({ \
	int blk_index = IAXXX_INDEX_FROM_VIRTUAL(virt_addr); \
	uint32_t addr; \
	\
	if (!IAXXX_IS_VALID_HDR_ID(priv, blk_index, inst_id)) \
		addr = 0; \
	else \
		addr = (virt_addr + \
			(priv->rbdt_info[blk_index].hdr_size * inst_id)); \
	addr; \
})

#endif /* __IAXXX_REGMAP_H__ */
