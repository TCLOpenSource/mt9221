/*
 * iaxxx-pwr-mgmt.h -- iaxxx power management
 *
 * Copyright 2019 Knowles Corporation
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

#ifndef _IAXXX_PWR_MGMT_H
#define _IAXXX_PWR_MGMT_H

enum {
	PROC_PWR_DOWN, /* Processor Power Down */
	PROC_PWR_UP, /* Processor Power Up */
	PROC_STALL_ENABLE, /* Processor Stall Enable */
	PROC_STALL_DISABLE, /* Processor Stall Disable */
};

enum {
	MEM_PWR_DOWN, /* Power Down */
	MEM_PWR_UP, /* Power Up */
	MEM_RETN_ON, /* Retention ON */
	MEM_RETN_OFF, /* Retention OFF */
};

enum {
	PLL_SRC_NONE,
	PLL_SRC_SYS_CLK,       /* System Clock - supported */
	PLL_SRC_PORTA_CLK,     /* io_ref0_clk2cnr_i - supported */
	PLL_SRC_PORTA_DO,      /* io_ref1_clk2cnr_i - Not supported */
	PLL_SRC_PORTB_CLK,     /* io_ref2_clk2cnr_i - supported */
	PLL_SRC_PORTB_DO,      /* io_ref3_clk2cnr_i - Not supported */
	PLL_SRC_PORTC_CLK,     /* io_ref4_clk2cnr_i - supported */
	PLL_SRC_PORTC_DO,      /* io_ref5_clk2cnr_i - Not supported */
	PLL_SRC_COMMB_3,       /* io_ref6_clk2cnr_i - Not supported */
	PLL_SRC_OSC_CLK,       /* Internal Oscillator - supported */
	PLL_SRC_EXT_CLK,       /* External clock - supported */

	PLL_SRC_SRC_MAX,
};

enum {
	APLL_SRC_FREQ_NONE,

	APLL_SRC_FREQ_512,          /*!< 512     KHz */
	APLL_SRC_FREQ_768,          /*!< 768     KHz */
	APLL_SRC_FREQ_1024,         /*!< 1.024   MHz */
	APLL_SRC_FREQ_1536,         /*!< 1.536   MHz */
	APLL_SRC_FREQ_2048,         /*!< 2.048   MHz */
	APLL_SRC_FREQ_2400,         /*!< 2.400   MHz */
	APLL_SRC_FREQ_3072,         /*!< 3.072   MHz */
	APLL_SRC_FREQ_4608,         /*!< 4.608   MHz */
	APLL_SRC_FREQ_4800,         /*!< 4.800   MHz */
	APLL_SRC_FREQ_6144,         /*!< 6.144   MHz */
	APLL_SRC_FREQ_9600,         /*!< 9.600   MHz */
	APLL_SRC_FREQ_12288,        /*!< 12.288  MHz */
	APLL_SRC_FREQ_19200,        /*!< 19.200  MHz */
	APLL_SRC_FREQ_24576,        /*!< 24.576  MHz */

	NUM_APLL_SRC_FREQ,
};

enum iaxxx_power_state {
	IAXXX_NOCHANGE     = 0,
	IAXXX_OPTIMAL_MODE = 1,
	IAXXX_SLEEP_MODE   = 2,
	IAXXX_NORMAL_MODE  = 4,
};

int iaxxx_config_apll(struct iaxxx_priv *priv,
		       u32 apll_src, u32 apll_out_freq, u32 apll_in_freq);
int iaxxx_get_system_clk_src(struct iaxxx_priv *priv);
int iaxxx_suspend_chip(struct iaxxx_priv *priv);
int iaxxx_wakeup_chip(struct iaxxx_priv *priv);
int iaxxx_enable_optimal_to_normal_transition(struct iaxxx_priv *priv);
void iaxxx_pm_enable(struct iaxxx_priv *priv);
void iaxxx_pm_disable(struct iaxxx_priv *priv);

int iaxxx_check_and_powerup_proc(struct iaxxx_priv *priv, uint32_t proc_id);

#endif /* _IAXXX_PWR_MGMT_H */
