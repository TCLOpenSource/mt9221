/*
 * iaxxx-register-internal.h  --  Knowles ALSA SoC Audio PCM driver header
 *
 * Copyright 2016 Knowles Corporation.
 *
 * Author: Sharada Kumar <Sharada.Kumar@knowles.com>
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

#ifndef __IAXXX_REGISTER_INTERNAL_H__
#define __IAXXX_REGISTER_INTERNAL_H__

enum {
	PDM_PORTA = 0,
	PDM_PORTB = 1,
	PDM_PORTC = 2,
	PDM_PORTD = 3,
	PDM_PORTE = 4,
	PDM_CDC = 6,
};

#define IAXXX_PCM_CTRL_FSP			0
#define IAXXX_PCM_CTRL_FSP_MASK			(IAXXX_PCM0_MC_FSP_MASK)
#define IAXXX_PCM_CTRL_FSP_HIGH			(0 << IAXXX_PCM_CTRL_FSP)
#define IAXXX_PCM_CTRL_FSP_LOW			(1 << IAXXX_PCM_CTRL_FSP)

#define IAXXX_PCM_CTRL_FSE			1
#define IAXXX_PCM_CTRL_FSE_MASK			(IAXXX_PCM0_MC_FSE_MASK)
#define IAXXX_PCM_CTRL_FSE_POS			(0 << IAXXX_PCM_CTRL_FSE)
#define IAXXX_PCM_CTRL_FSE_NEG			(1 << IAXXX_PCM_CTRL_FSE)

#define IAXXX_PCM_CTRL_TX			2
#define IAXXX_PCM_CTRL_TX_MASK			(IAXXX_PCM0_MC_TCP_MASK)
#define IAXXX_PCM_CTRL_TX_POS			(0 << IAXXX_PCM_CTRL_TX)
#define IAXXX_PCM_CTRL_TX_NEG			(1 << IAXXX_PCM_CTRL_TX)

#define IAXXX_PCM_CTRL_RX			3
#define IAXXX_PCM_CTRL_RX_MASK			(IAXXX_PCM0_MC_RCP_MASK)
#define IAXXX_PCM_CTRL_RX_POS			(0 << IAXXX_PCM_CTRL_RX)
#define IAXXX_PCM_CTRL_RX_NEG			(1 << IAXXX_PCM_CTRL_RX)

#define IAXXX_PCM_CTRL_END			4
#define IAXXX_PCM_CTRL_END_MASK			(IAXXX_PCM0_MC_END_MASK)
#define IAXXX_PCM_CTRL_LE			(0 << IAXXX_PCM_CTRL_END)
#define IAXXX_PCM_CTRL_BE			(1 << IAXXX_PCM_CTRL_END)

#define IAXXX_PCM_CTRL_TRI			5
#define IAXXX_PCM_CTRL_TRI_MASK			(IAXXX_PCM0_MC_TRI_MASK)
#define IAXXX_PCM_CTRL_TRISTATE_DISABLE		(0 << IAXXX_PCM_CTRL_TRI)
#define IAXXX_PCM_CTRL_TRISTATE_ENABLE		(1 << IAXXX_PCM_CTRL_TRI)

#define IAXXX_PCM_CTRL_DL			6
#define IAXXX_PCM_CTRL_SL_MASK			(IAXXX_PCM0_MC_DL_MASK)
#define IAXXX_PCM_CTRL_LAST_TRISTATE_NORMAL	(0 << IAXXX_PCM_CTRL_DL)
#define IAXXX_PCM_CTRL_LAST_TRISTATE_EARLY	(1 << IAXXX_PCM_CTRL_DL)

#define IAXXX_PCM_CTRL_ADIS			8
#define IAXXX_PCM_CTRL_ADIS_MASK		(IAXXX_PCM0_MC_ADIS_MASK)
#define IAXXX_PCM_CTRL_ASYNC_ENABLE		(0 << IAXXX_PCM_CTRL_ADIS)
#define IAXXX_PCM_CTRL_ASYNC_DISABLE		(1 << IAXXX_PCM_CTRL_ADIS)

/* NOTE: Missing bits are unused */

#define IAXXX_PCM_CTRL_DDLYEN			12
#define IAXXX_PCM_CTRL_DDLYEN_MASK		(IAXXX_PCM0_MC_DDLYEN_MASK)
#define IAXXX_PCM_CTRL_RX_DATA_DELAY_0		(0 << IAXXX_PCM_CTRL_DDLYEN)
#define IAXXX_PCM_CTRL_RX_DATA_DELAY_1		(1 << IAXXX_PCM_CTRL_DDLYEN)

#define IAXXX_PCM_CTRL_DLNACT			13
#define IAXXX_PCM_CTRL_DLNACT_MASK		(IAXXX_PCM0_MC_DLNACT_MASK)
#define IAXXX_PCM_CTRL_TRISTATE_DIS_NORMAL	(0 << IAXXX_PCM_CTRL_DLNACT)
#define IAXXX_PCM_CTRL_TRISTATE_DIS_LATE	(1 << IAXXX_PCM_CTRL_DLNACT)

#define IAXXX_PCM_CTRL_LEFTJUST			14
#define IAXXX_PCM_CTRL_LEFTJUST_MASK		(IAXXX_PCM0_MC_LEFTJUST_MASK)
#define IAXXX_PCM_CTRL_RIGHT_JUSTIFY_DATA	(0 << IAXXX_PCM_CTRL_LEFTJUST)
#define IAXXX_PCM_CTRL_LEFT_JUSTIFY_DATA	(1 << IAXXX_PCM_CTRL_LEFTJUST)

#define IAXXX_PCM_CTRL_DEFAULT_I2SFMT	(IAXXX_PCM_CTRL_FSP_LOW |\
					IAXXX_PCM_CTRL_FSE_POS | \
					IAXXX_PCM_CTRL_TX_NEG | \
					IAXXX_PCM_CTRL_RX_POS | \
					IAXXX_PCM_CTRL_BE | \
					IAXXX_PCM_CTRL_TRISTATE_DISABLE | \
					IAXXX_PCM_CTRL_LAST_TRISTATE_NORMAL | \
					IAXXX_PCM_CTRL_ASYNC_DISABLE | \
					IAXXX_PCM_CTRL_RX_DATA_DELAY_0      | \
					IAXXX_PCM_CTRL_TRISTATE_DIS_NORMAL  | \
					IAXXX_PCM_CTRL_LEFT_JUSTIFY_DATA)

#define IAXXX_PCM_CTRL_DEFAULT_DSPFMT	(IAXXX_PCM_CTRL_FSP_HIGH |\
					IAXXX_PCM_CTRL_FSE_NEG | \
					IAXXX_PCM_CTRL_TX_POS | \
					IAXXX_PCM_CTRL_RX_NEG | \
					IAXXX_PCM_CTRL_BE | \
					IAXXX_PCM_CTRL_TRISTATE_DISABLE | \
					IAXXX_PCM_CTRL_LAST_TRISTATE_NORMAL | \
					IAXXX_PCM_CTRL_ASYNC_ENABLE | \
					IAXXX_PCM_CTRL_RX_DATA_DELAY_0      | \
					IAXXX_PCM_CTRL_TRISTATE_DIS_NORMAL  | \
					IAXXX_PCM_CTRL_LEFT_JUSTIFY_DATA)

#define IAXXX_PCM_CTRL_DEFAULT_TDMFMT	(IAXXX_PCM0_MC_FSE_MASK | \
					IAXXX_PCM0_MC_RCP_MASK | \
					IAXXX_PCM0_MC_END_MASK | \
					IAXXX_PCM0_MC_TRI_MASK | \
					IAXXX_PCM0_MC_LEFTJUST_MASK)

#define IAXXX_PCM_CTRL_DEFAULT IAXXX_PCM_CTRL_DEFAULT_I2SFMT


#define IAXXX_PCM_WORD_LEN_MASK		(IAXXX_PCM0_SWLR_WORD_LEN_MASK)
#define IAXXX_PCM_FRAME_LEN_MASK		(IAXXX_PCM0_SFLR_FRAME_LEN_MASK)
#define IAXXX_PCM_TX_DELAY_MASK		(IAXXX_PCM0_STDD_TX_DATA_DLY_MASK)
#define IAXXX_PCM_RX_DELAY_MASK		(IAXXX_PCM0_SRDD_RX_DATA_DLY_MASK)

#define IAXXX_IO_CTRL_CLK_MUX_SEL_SLAVE		0x0000001f
#define IAXXX_IO_CTRL_CLK_MUX_SEL_MASTER	0x00000005
#define IAXXX_IO_CTRL_MUX_SEL_I2S_VAL		0x5
#define IAXXX_IO_CTRL_MUX_SEL_I2S_CLK_VAL	0x6
#define IAXXX_IO_CTRL_CDC_MCLK_MUX_SEL_MASK	0x0000001f
#define IAXXX_IO_CTRL_PCM_BCLK_AND_SEL_SLAVE  0x00000100
#define IAXXX_IO_CTRL_PCM_BCLK_AND_SEL_MASTER  0x0000100
#define IAXXX_IO_CTRL_CLK_GPIO_PAD_REC_ENABLE 0x00800000
#define IAXXX_IO_CTRL_PDM_BCLK_AND_SEL_SLAVE	0x00000200
#define IAXXX_IO_CTRL_PDM_BCLK_AND_SEL_MASTER	0x00000200

#define IAXXX_IO_CTRL_CLK_PDM_SLAVE ((IAXXX_IO_CTRL_CLK_MUX_SEL_SLAVE) | \
				(IAXXX_IO_CTRL_PDM_BCLK_AND_SEL_SLAVE) |\
				(IAXXX_IO_CTRL_CLK_GPIO_PAD_REC_ENABLE))

#define IAXXX_IO_CTRL_CLK_PDM_MASTER ((IAXXX_IO_CTRL_CLK_MUX_SEL_MASTER) | \
				(IAXXX_IO_CTRL_PDM_BCLK_AND_SEL_MASTER))


#define PDM_CLK_SLAVE ((IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_RESET_VAL <<\
			IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTA_CLK_PDM0_CLK_AND_SEL_POS))
#define PDM_CLK_MASTER ((IAXXX_IO_CTRL_MUX_SEL_I2S_VAL <<\
			IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTA_CLK_PDM0_CLK_AND_SEL_POS))

#define CDC_CLK_SLAVE ((IAXXX_IO_CTRL_PORTB_CLK_MUX_SEL_RESET_VAL <<\
			IAXXX_IO_CTRL_PORTB_CLK_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTB_CLK_CDC0_CLK_AND_SEL_POS))
#define CDC_CLK_MASTER ((IAXXX_IO_CTRL_MUX_SEL_I2S_VAL <<\
			IAXXX_IO_CTRL_PORTB_CLK_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTB_CLK_CDC0_CLK_AND_SEL_POS))

#define PDM0_OUT_CLK_SLAVE ((IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_RESET_VAL <<\
			IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTC_CLK_PDM0_CLK_AND_SEL_POS))
#define PDM1_OUT_CLK_SLAVE ((IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_RESET_VAL <<\
			IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTC_DO_PDM1_CLK_AND_SEL_POS))
#define PDM0_OUT_CLK_MASTER ((IAXXX_IO_CTRL_MUX_SEL_I2S_VAL <<\
			IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTC_CLK_PDM0_CLK_AND_SEL_POS))
#define PDM1_OUT_CLK_MASTER ((IAXXX_IO_CTRL_MUX_SEL_I2S_VAL <<\
			IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTC_DO_PDM1_CLK_AND_SEL_POS))

#define CDC0_OUT_CLK_SLAVE ((IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_RESET_VAL <<\
			IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTC_CLK_PDM0_CLK_AND_SEL_POS))
#define CDC1_OUT_CLK_SLAVE ((IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_RESET_VAL <<\
			IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTC_DO_PDM1_CLK_AND_SEL_POS))
#define CDC0_OUT_CLK_MASTER ((IAXXX_IO_CTRL_MUX_SEL_I2S_VAL <<\
			IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTC_CLK_PDM0_CLK_AND_SEL_POS))
#define CDC1_OUT_CLK_MASTER ((IAXXX_IO_CTRL_MUX_SEL_I2S_VAL <<\
			IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_POS) |\
			(1 << IAXXX_IO_CTRL_PORTC_DO_PDM1_CLK_AND_SEL_POS))

#define DMIC_CLK_FWD ((IAXXX_IO_CTRL_MUX_SEL_I2S_CLK_VAL << \
			IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_POS) | \
			(1 << IAXXX_IO_CTRL_PORTC_CLK_PDM0_CLK_AND_SEL_POS))
#define PORTD_CLK_FWD ((IAXXX_IO_CTRL_MUX_SEL_I2S_CLK_VAL << \
			IAXXX_IO_CTRL_PORTD_CLK_MUX_SEL_POS) | \
			(1 << IAXXX_IO_CTRL_PORTD_CLK_CDC1_CLK_AND_SEL_POS))

#define IAXXX_IO_CTRL_DO (IAXXX_IO_CTRL_MUX_SEL_PCM_VAL << \
			  IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_POS)

#define IAXXX_IO_CTRL_DI (1 << IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_POS)

#define IAXXX_IO_CTRL_FS_SLAVE ((1 << \
				 IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_POS) | \
				 IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK)

#define IAXXX_IO_CTRL_CLK_SLAVE ((1 << \
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_POS) | \
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK)

#define IAXXX_IO_CTRL_FS_MASTER ((IAXXX_IO_CTRL_MUX_SEL_I2S_VAL << \
				IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_POS) | (1 << \
				IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_POS))

#define IAXXX_IO_CTRL_CLK_MASTER ((IAXXX_IO_CTRL_MUX_SEL_I2S_VAL << \
				   IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_POS) | \
			(1 << IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_POS))

/* I2S default values */
#define IAXXX_CLK_DEFAULT_HL_DIV 4
#define IAXXX_CLK_PERIOD_SIZE 3

#define IAXXX_CLK_DEFAULT_PERIOD 16
#define IAXXX_CLK_DIV_SIZE 3

#define IAXXX_MAX_NUMERATOR ((1 << 12) - 1)
#define IAXXX_MAX_DENOMINATOR ((1 << 12) - 1)

#define IAXXX_I2S_TRIGGER_LOW  0x00000000
#define IAXXX_I2S_TRIGGER_HIGH  0x00000001

#define IAXXX_I2S_TRIGGER_LOW  0x00000000
#define IAXXX_I2S_TRIGGER_HIGH  0x00000001
#define IAXXX_I2S_GEN_CFG_FS_POL_LOW (0 << \
				IAXXX_I2S_I2S0_GEN_CFG_PCM_FS_POL_POS)

#define IAXXX_I2S_GEN_CFG_FS_POL_HIGH ((1 << \
				IAXXX_I2S_I2S0_GEN_CFG_PCM_FS_POL_POS) & \
				IAXXX_I2S_I2S0_GEN_CFG_PCM_FS_POL_MASK)
#define IAXXX_I2S_GEN_CFG_CLK_POL_LOW (0 << \
				IAXXX_I2S_I2S0_GEN_CFG_I2S_CLK_POL_POS)
#define IAXXX_I2S_GEN_CFG_CLK_POL_HIGH (1 << \
				IAXXX_I2S_I2S0_GEN_CFG_I2S_CLK_POL_POS)
#define IAXXX_I2S_GEN_CFG_FS_POL_I2S_MODE ((1 << \
				IAXXX_I2S_I2S0_GEN_CFG_I2S_FS_POL_POS)& \
				IAXXX_I2S_I2S0_GEN_CFG_I2S_FS_POL_MASK)
#define IAXXX_I2S_GEN_CFG_FS_POL_DSP_MODE ((0 << \
				IAXXX_I2S_I2S0_GEN_CFG_I2S_FS_POL_POS)& \
				IAXXX_I2S_I2S0_GEN_CFG_I2S_FS_POL_MASK)
#define IAXXX_I2S_GEN_CFG_ABORT_ON_SYNC_DISABLE (0 << \
				IAXXX_I2S_I2S0_GEN_CFG_ABORT_ON_SYNC_POS)
#define IAXXX_I2S_GEN_CFG_ABORT_ON_SYNC_ENABLE ((1 << \
				IAXXX_I2S_I2S0_GEN_CFG_ABORT_ON_SYNC_POS)&\
				IAXXX_I2S_I2S0_GEN_CFG_ABORT_ON_SYNC_MASK)
#define IAXXX_I2S_GEN_CFG_GEN_MASTER_MODE (1 << \
				IAXXX_I2S_I2S0_GEN_CFG_GEN_MASTER_POS)
#define IAXXX_I2S_GEN_CFG_GEN_PSEUDO_MODE ((0 << \
				IAXXX_I2S_I2S0_GEN_CFG_GEN_MASTER_POS)& \
				IAXXX_I2S_I2S0_GEN_CFG_GEN_MASTER_MASK)
#define IAXXX_I2S_FS_ALIGN_MASTER_MODE 0x00000000

#define IAXXX_I2S_GEN_CFG_FS_VALID_I2S_MODE ((0x00 << \
				IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_POS) &\
				IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_MASK)
/* CNR0 default values */
#define IAXXX_CNR0_I2S_ENABLE_MASK(id)  (0x00000001 << (id))

#define IAXXX_CNR0_I2S0_MASK_POS 0
#define IAXXX_CNR0_I2S1_MASK_POS 1
#define IAXXX_CNR0_I2S2_MASK_POS 2
#define IAXXX_CNR0_I2S3_MASK_POS 3
#define IAXXX_CNR0_I2S4_MASK_POS 4
#define IAXXX_CNR0_I2S5_MASK_POS 5

#define IAXXX_CNR0_I2S_ENABLE_HIGH 1
#define IAXXX_CNR0_I2S0_ENABLE_HIGH 0x00000001
#define IAXXX_CNR0_I2S1_ENABLE_HIGH 0x00000002
#define IAXXX_CNR0_I2S2_ENABLE_HIGH 0x00000004
#define IAXXX_CNR0_I2S3_ENABLE_HIGH 0x00000008
#define IAXXX_CNR0_I2S4_ENABLE_HIGH 0x00000010
#define IAXXX_CNR0_I2S5_ENABLE_HIGH 0x00000020

#define IAXXX_CNR0_I2S_ENABLE_LOW 0

#define IAXXX_I2S_I2S0_HL_DISABLE (0 << IAXXX_I2S_I2S0_HL_EN_POS)
#define IAXXX_I2S_I2S0_HL_ENABLE (1 << IAXXX_I2S_I2S0_HL_EN_POS)
#define IAXXX_I2S_I2S0_NR_DISABLE (0 << IAXXX_I2S_I2S0_NR_EN_POS)
#define IAXXX_I2S_I2S0_NR_ENABLE (1 << IAXXX_I2S_I2S0_NR_EN_POS)

#define IAXXX_CNR0_PCM_ENABLE 1
#define IAXXX_CNR0_PCM_DISABLE 0
#define IAXXX_AO_BCLK_ENABLE 1
#define IAXXX_AO_DO_ENABLE 1
#define IAXXX_AO_FS_ENABLE 1
#define IAXXX_AO_BCLK_DISABLE 0
#define IAXXX_AO_FS_DISABLE 0
#define IAXXX_AO_DO_DISABLE 0

#define IAXXX_CIC_RX_RESET 1
#define IAXXX_CIC_ADTL_RX_G 0
#define IAXXX_CIC_ADTL_RX_DEBUG 0
#define IAXXX_CIC_ADTL_RX_0 ((IAXXX_CIC_ADTL_RX_G << \
			IAXXX_CNR0_CIC_ADTL_CTRL_G_RX_0_POS) | \
			(IAXXX_CIC_ADTL_RX_DEBUG << \
			IAXXX_CNR0_CIC_ADTL_CTRL_DBG_RX_0_POS))
#define IAXXX_CIC_ADTL_RX(id) ((IAXXX_CIC_ADTL_RX_G << \
			(IAXXX_CNR0_CIC_ADTL_CTRL_G_RX_0_POS + (id) * 2)) | \
			(IAXXX_CIC_ADTL_RX_DEBUG << \
			(IAXXX_CNR0_CIC_ADTL_CTRL_DBG_RX_0_POS + (id) * 2)))
#define IAXXX_CIC_RX_RT_CTRL_OPT_SIZE     (4)

#define IAXXX_PCM_ISEN_WMASK_VAL (IAXXX_PCM0_ISEN_WMASK_VAL & \
				  (~IAXXX_PCM0_ISEN_ON_ENABLE_EN_MASK))

#define IAXXX_CNR0_CIC_CTRL_RX_MASK(id) \
			(IAXXX_CNR0_CIC_CLOCK_CTRL_RX_AC_0_MASK << (id))
#define IAXXX_CNR0_CIC_CTRL_RX_POL_MASK(id) \
			 (IAXXX_CNR0_CIC_POL_CTRL_RX_POL_0_MASK << (id))
#define IAXXX_CIC_ADTL_RX_MASK(id)\
		((IAXXX_CNR0_CIC_RX_ADTL_CTRL_G_RX_0_MASK << ((id) * 2)) |\
		(IAXXX_CNR0_CIC_RX_ADTL_CTRL_DBG_RX_0_MASK << ((id) * 2)))
#define IAXXX_CNR0_CIC_CTRL_RX_POL_POS(id) \
			(IAXXX_CNR0_CIC_POL_CTRL_RX_POL_0_POS + (id))
#define IAXXX_CNR0_CIC_HB_CIC_RX_MASK(id) \
			(IAXXX_CNR0_CIC_RX_HB_CIC_RX_0_MASK << ((id) * 2))
#define IAXXX_CNR0_CIC_HB_CIC_RX_POS(id) \
			(IAXXX_CNR0_CIC_RX_HB_CIC_RX_0_POS + (id) * 2)
#define IAXXX_CIC_RX_RT_CTRL_OPT_MASK \
		(IAXXX_CNR0_CIC_RX_RT_CTRL_MIC_0_MASK  |  \
		IAXXX_CNR0_CIC_RX_RT_CTRL_S_0_MASK | \
		IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_0_MASK)

#define IAXXX_CIC_ADTL_TX_MASK(id)\
		((id == 0 ? (IAXXX_CNR0_CIC_TX_ADTL_CTRL_DBG_TX_0_MASK \
		| IAXXX_CNR0_CIC_TX_ADTL_CTRL_G_TX_0_MASK) : \
		(IAXXX_CNR0_CIC_TX_ADTL_CTRL_DBG_TX_1_MASK | \
		 IAXXX_CNR0_CIC_TX_ADTL_CTRL_G_TX_1_MASK)))

#define IAXXX_CIC_TX_PHASE_POS(id) \
	((id == 0) ? IAXXX_CNR0_CIC_TX_0_1_PHASE_0_POS : \
	 IAXXX_CNR0_CIC_TX_0_1_PHASE_1_POS)
#define IAXXX_CIC_TX_PHASE_MASK(id) \
	((id == 0) ? IAXXX_CNR0_CIC_TX_0_1_PHASE_0_MASK : \
	 IAXXX_CNR0_CIC_TX_0_1_PHASE_1_MASK)
#define IAXXX_CIC_TX_DSEL_POS(id) \
	((id == 0) ? IAXXX_CNR0_CIC_TX_0_1_DSEL_0_POS : \
	 IAXXX_CNR0_CIC_TX_0_1_DSEL_1_POS)
#define IAXXX_CIC_TX_DSEL_MASK(id) \
	((id == 0) ? IAXXX_CNR0_CIC_TX_0_1_DSEL_0_MASK : \
	 IAXXX_CNR0_CIC_TX_0_1_DSEL_1_MASK)
#define IAXXX_CIC_TX_L_POS(id) ((id == 0) ? IAXXX_CNR0_CIC_TX_0_1_L_0_POS : \
				IAXXX_CNR0_CIC_TX_0_1_L_1_POS)
#define IAXXX_CIC_TX_L_MASK(id) ((id == 0) ? IAXXX_CNR0_CIC_TX_0_1_L_0_MASK  : \
				 IAXXX_CNR0_CIC_TX_0_1_L_1_MASK)

#define IAXXX_CIC_POL_CTRL_TX_POL_MASK(id) \
	((id == 0) ? IAXXX_CNR0_CIC_POL_CTRL_TX_POL_0_MASK : \
	 IAXXX_CNR0_CIC_POL_CTRL_TX_POL_1_MASK)
#define IAXXX_CIC_POL_CTRL_TX_POL_POS(id) \
	((id == 0) ? IAXXX_CNR0_CIC_POL_CTRL_TX_POL_0_POS : \
	 IAXXX_CNR0_CIC_POL_CTRL_TX_POL_1_POS)
#define IAXXX_CIC_HB_CIC_TX_MASK(id) \
	((id == 0) ? IAXXX_CNR0_CIC_TX_HB_CIC_TX_0_MASK : \
	 IAXXX_CNR0_CIC_TX_HB_CIC_TX_1_MASK)
#define IAXXX_CIC_HB_CIC_TX_SIZE(id) \
	((id == 0) ? IAXXX_CNR0_CIC_TX_HB_CIC_TX_0_SIZE : \
	 IAXXX_CNR0_CIC_TX_HB_CIC_TX_1_SIZE)
#define IAXXX_CIC_HB_CIC_TX_POS(id) \
	((id == 0) ? IAXXX_CNR0_CIC_TX_HB_CIC_TX_0_POS : \
	 IAXXX_CNR0_CIC_TX_HB_CIC_TX_1_POS)
#define IAXXX_CIC_TX_CLR_POS(id) \
	((id == 0) ? IAXXX_CNR0_CIC_TX_0_1_CLR_0_POS : \
	 IAXXX_CNR0_CIC_TX_0_1_CLR_1_POS)
#define IAXXX_CIC_TX_CLR_MASK(id) \
	((id == 0) ? IAXXX_CNR0_CIC_TX_0_1_CLR_0_MASK : \
	 IAXXX_CNR0_CIC_TX_0_1_CLR_1_MASK)
#define IAXXX_CIC_POL_CTRL_TX_POL_POS(id) \
	((id == 0) ? IAXXX_CNR0_CIC_POL_CTRL_TX_POL_0_POS : \
	 IAXXX_CNR0_CIC_POL_CTRL_TX_POL_1_POS)
#define IAXXX_CIC_CLOCK_CTRL_TX_AC_MASK(id) \
	((id == 0) ? IAXXX_CNR0_CIC_CLOCK_CTRL_TX_AC_0_MASK : \
	 IAXXX_CNR0_CIC_CLOCK_CTRL_TX_AC_1_MASK)
#define IAXXX_CIC_CLOCK_CTRL_TX_AC_POS(id) \
	((id == 0) ? IAXXX_CNR0_CIC_CLOCK_CTRL_TX_AC_0_POS : \
	 IAXXX_CNR0_CIC_CLOCK_CTRL_TX_AC_1_POS)

#define IAXXX_DMIC1_CLK 1
#define IAXXX_DMIC0_CLK 0
#define IAXXX_PDM_POLARITY 0 /* TODO check */
#define IAXXX_CIC_MIC_ENABLE 0 /* 0 - using DMIC */
#define IAXXX_CIC_S_DMIC_ENABLE 1 /* 1 - using DMIC */
#define IAXXX_CLK_ENABLE 1
#define IAXXX_CLK_DISABLE 0
#define IAXXX_DMIC_ENABLE 1
#define IAXXX_DMIC_ENABLE_MASK 1
#define I2S_OFFSET 8
#define PCM_OFFSET 0x1000
#define CIC_MIC_OFFSET 4

#define IAXXX_PCM_SWLR_ADDR(id) (IAXXX_PCM0_SWLR_ADDR + \
					((id) * PCM_OFFSET))
#define IAXXX_PCM_SFLR_ADDR(id) (IAXXX_PCM0_SFLR_ADDR + \
					((id) * PCM_OFFSET))
#define IAXXX_PCM_MC_ADDR(id) (IAXXX_PCM0_MC_ADDR + \
					((id) * PCM_OFFSET))
#define IAXXX_PCM_STSA_ADDR(id) (IAXXX_PCM0_STSA_ADDR + \
					((id) * PCM_OFFSET))
#define IAXXX_PCM_SRSA_ADDR(id) (IAXXX_PCM0_SRSA_ADDR + \
					((id) * PCM_OFFSET))
#define IAXXX_PCM_STDD_ADDR(id) (IAXXX_PCM0_STDD_ADDR + \
					((id) * PCM_OFFSET))
#define IAXXX_PCM_SRDD_ADDR(id) (IAXXX_PCM0_SRDD_ADDR + \
					((id) * PCM_OFFSET))
#define IAXXX_PCM_ISEN_ADDR(id) (IAXXX_PCM0_ISEN_ADDR + \
					((id) * PCM_OFFSET))

#define IAXXX_I2S_I2S_GEN_CFG_ADDR(id) (IAXXX_I2S_I2S0_GEN_CFG_ADDR + \
					(4 * (id) * I2S_OFFSET))

#define IAXXX_I2S_I2S_FS_ALIGN_ADDR(id) (IAXXX_I2S_I2S0_FS_ALIGN_ADDR + \
					(4 * (id) * I2S_OFFSET))

#define IAXXX_I2S_I2S_HL_ADDR(id) (IAXXX_I2S_I2S0_HL_ADDR + \
					(4 * (id) * I2S_OFFSET))

#define IAXXX_I2S_I2S_NR_ADDR(id) (IAXXX_I2S_I2S0_NR_ADDR + \
					(4 * (id) * I2S_OFFSET))
#define IAXXX_I2S_I2S_CLK_CTRL_ADDR(id) (IAXXX_I2S_I2S0_CLK_CTRL_ADDR + \
						(4 * (id) * I2S_OFFSET))

#define IAXXX_CIC_TX_PHASE_OP_ENABLE 1
#define IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(id) \
			(IAXXX_AO_CLK_CFG_PCM_PORT0_CLK_OE_POS + (id))

#define IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(id) \
			(IAXXX_AO_CLK_CFG_PCM_PORT0_CLK_OE_MASK << (id))

#define IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(id) \
			(IAXXX_AO_CLK_CFG_PCM_PORT0_FS_OE_POS + (id))
#define IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(id) \
			(IAXXX_AO_CLK_CFG_PCM_PORT0_FS_OE_MASK << (id))

#define IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(id) \
			(IAXXX_AO_CLK_CFG_PCM_PORT0_DO_OE_POS + (id))

#define IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(id) \
			(IAXXX_AO_CLK_CFG_PCM_PORT0_DO_OE_MASK << (id))

#define IAXXX_AO_CLK_CFG_PORT_DI_OE_POS(id) \
			(IAXXX_AO_CLK_CFG_PORTA_DI_OE_POS + (id))
#define IAXXX_AO_CLK_CFG_PORT_DI_OE_MASK(id) \
			(IAXXX_AO_CLK_CFG_PORTA_DI_OE_MASK << (id))
#define IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(id) \
			(IAXXX_CNR0_PCM_ACTIVE_PCM_0_ACT_MASK << (id))
#define IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(id) \
			(IAXXX_CNR0_PCM_ACTIVE_PCM_0_ACT_POS + (id))
#define IAXXX_CNR0_CIC_RX_RT_CTRL_MIC_POS(id) \
		(IAXXX_CNR0_CIC_RX_RT_CTRL_MIC_0_POS + ((id) * CIC_MIC_OFFSET))
#define IAXXX_CNR0_CIC_RX_RT_CTRL_S_POS(id) \
		(IAXXX_CNR0_CIC_RX_RT_CTRL_S_0_POS + ((id) * CIC_MIC_OFFSET))
#define IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_POS(id) \
		(IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_0_POS +\
		((id) * CIC_MIC_OFFSET))

#define IAXXX_CNR0_CIC_RX_RT_0_MASK (IAXXX_CNR0_CIC_RX_RT_CTRL_MIC_0_POS|\
				IAXXX_CNR0_CIC_RX_RT_CTRL_S_0_MASK | \
				IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_0_MASK)
#define IAXXX_CNR0_CIC_RX_RT_MASK(id) (IAXXX_CNR0_CIC_RX_RT_0_MASK << (id * 4))

/*TODO: Add def for interrupt status and RIS */
#endif /* IAXXX_REGISTER_INTERNAL_H__ */
