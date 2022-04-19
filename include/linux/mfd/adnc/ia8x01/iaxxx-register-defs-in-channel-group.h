/*
 * iaxxx-register-defs-in-channel-group.h
 *
 * Copyright (c) 2018 Knowles, inc.
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

/**********************************************************
 * This file is generated by running a format script
 * on header files shared by Firmware.
 *
 * DO NOT EDIT.
 *
 *********************************************************/

#ifndef __IAXXX_REGISTER_DEFS_IN_CH_GRP_H__
#define __IAXXX_REGISTER_DEFS_IN_CH_GRP_H__

/*** The base address for this set of registers ***/
#define IAXXX_IN_CH_GRP_REGS_ADDR (0x01800000)

/*** IN_CH_GRP_CH_GAIN_CTRL (0x01800000) ***/
/*
 * Channel Gain Control.
 */
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_ADDR (0x01800000)
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_MASK_VAL 0xffff01ff
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_RMASK_VAL 0xffff01ff
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_WMASK_VAL 0xffff01ff
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_RESET_VAL 0x00000000

/*
 * Target gain (in dB) (-128 dB to 127 dB)
 */
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK 0x000000ff
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_POS 0
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_SIZE 8

/*
 * Generate an event when current gain reaches target gain.
 * (1 - enabled, 0 - disabled)
 */
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_MASK 0x00000100
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS 8
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_SIZE 1

/*
 * Gain ramp (in dB per second).
 * 0xFFFF represents instantaneous gain change.
 */
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK 0xffff0000
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS 16
#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_SIZE 16

/*** IN_CH_GRP_CH_GAIN_STATUS (0x01800004) ***/
/*
 * Channel Gain Status.
 */
#define IAXXX_IN_CH_GRP_CH_GAIN_STATUS_ADDR (0x01800004)
#define IAXXX_IN_CH_GRP_CH_GAIN_STATUS_MASK_VAL 0x000000ff
#define IAXXX_IN_CH_GRP_CH_GAIN_STATUS_RMASK_VAL 0x000000ff
#define IAXXX_IN_CH_GRP_CH_GAIN_STATUS_WMASK_VAL 0x000000ff
#define IAXXX_IN_CH_GRP_CH_GAIN_STATUS_RESET_VAL 0x00000000

/*
 * Current gain (in dB) (-128 dB to 127 dB)
 */
#define IAXXX_IN_CH_GRP_CH_GAIN_STATUS_GAIN_CURRENT_MASK 0x000000ff
#define IAXXX_IN_CH_GRP_CH_GAIN_STATUS_GAIN_CURRENT_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_GAIN_STATUS_GAIN_CURRENT_POS 0
#define IAXXX_IN_CH_GRP_CH_GAIN_STATUS_GAIN_CURRENT_SIZE 8

/*** IN_CH_GRP_OUT_FMT (0x01800008) ***/
/*
 * Output Endpoint Format
 */
#define IAXXX_IN_CH_GRP_OUT_FMT_ADDR (0x01800008)
#define IAXXX_IN_CH_GRP_OUT_FMT_MASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_OUT_FMT_RMASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_OUT_FMT_WMASK_VAL 0x00000000
#define IAXXX_IN_CH_GRP_OUT_FMT_RESET_VAL 0x00000000

/*
 * Encoding of the data contained in the frame
 * buffer coming out of the endpoint.
 */
#define IAXXX_IN_CH_GRP_OUT_FMT_ENCODING_MASK 0x000000ff
#define IAXXX_IN_CH_GRP_OUT_FMT_ENCODING_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_OUT_FMT_ENCODING_POS 0
#define IAXXX_IN_CH_GRP_OUT_FMT_ENCODING_SIZE 8

/*
 * Sample rate of the data contained in the frame
 * buffer coming out of the endpoint
 */
#define IAXXX_IN_CH_GRP_OUT_FMT_SAMPLE_RATE_MASK 0x0000ff00
#define IAXXX_IN_CH_GRP_OUT_FMT_SAMPLE_RATE_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_OUT_FMT_SAMPLE_RATE_POS 8
#define IAXXX_IN_CH_GRP_OUT_FMT_SAMPLE_RATE_SIZE 8

/*
 * Number of bytes contained in the
 * frame buffer coming out of the endpoint.
 */
#define IAXXX_IN_CH_GRP_OUT_FMT_LENGTH_MASK 0xffff0000
#define IAXXX_IN_CH_GRP_OUT_FMT_LENGTH_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_OUT_FMT_LENGTH_POS 16
#define IAXXX_IN_CH_GRP_OUT_FMT_LENGTH_SIZE 16

/*** IN_CH_GRP_CH_PEAK (0x0180000c) ***/
/*
 * Channel Peak Value
 */
#define IAXXX_IN_CH_GRP_CH_PEAK_ADDR (0x0180000c)
#define IAXXX_IN_CH_GRP_CH_PEAK_MASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_PEAK_RMASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_PEAK_WMASK_VAL 0x00000000
#define IAXXX_IN_CH_GRP_CH_PEAK_RESET_VAL 0x00000000

/*
 * Channel peak value
 */
#define IAXXX_IN_CH_GRP_CH_PEAK_CH_PEAK_MASK 0xffffffff
#define IAXXX_IN_CH_GRP_CH_PEAK_CH_PEAK_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_PEAK_CH_PEAK_POS 0
#define IAXXX_IN_CH_GRP_CH_PEAK_CH_PEAK_SIZE 32

/*** IN_CH_GRP_CH_RMS (0x01800010) ***/
/*
 * Channel RMS Value.
 */
#define IAXXX_IN_CH_GRP_CH_RMS_ADDR (0x01800010)
#define IAXXX_IN_CH_GRP_CH_RMS_MASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_RMS_RMASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_RMS_WMASK_VAL 0x00000000
#define IAXXX_IN_CH_GRP_CH_RMS_RESET_VAL 0x00000000

/*
 * Channel RMS Value
 */
#define IAXXX_IN_CH_GRP_CH_RMS_CH_RMS_MASK 0xffffffff
#define IAXXX_IN_CH_GRP_CH_RMS_CH_RMS_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_RMS_CH_RMS_POS 0
#define IAXXX_IN_CH_GRP_CH_RMS_CH_RMS_SIZE 32

/*** IN_CH_GRP_CH_MTR_SMPL (0x01800014) ***/
/*
 * Channel Meter Sample Count.
 */
#define IAXXX_IN_CH_GRP_CH_MTR_SMPL_ADDR (0x01800014)
#define IAXXX_IN_CH_GRP_CH_MTR_SMPL_MASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_MTR_SMPL_RMASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_MTR_SMPL_WMASK_VAL 0x00000000
#define IAXXX_IN_CH_GRP_CH_MTR_SMPL_RESET_VAL 0x00000000

/*
 * Accumulated sample count for rms value and peak.
 */
#define IAXXX_IN_CH_GRP_CH_MTR_SMPL_CH_MTR_SMPL_CNT_MASK 0xffffffff
#define IAXXX_IN_CH_GRP_CH_MTR_SMPL_CH_MTR_SMPL_CNT_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_MTR_SMPL_CH_MTR_SMPL_CNT_POS 0
#define IAXXX_IN_CH_GRP_CH_MTR_SMPL_CH_MTR_SMPL_CNT_SIZE 32

/*** IN_CH_GRP_CH_NSENT (0x01800018) ***/
/*
 * Number of Frames sent.
 */
#define IAXXX_IN_CH_GRP_CH_NSENT_ADDR (0x01800018)
#define IAXXX_IN_CH_GRP_CH_NSENT_MASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_NSENT_RMASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_NSENT_WMASK_VAL 0x00000000
#define IAXXX_IN_CH_GRP_CH_NSENT_RESET_VAL 0x00000000

/*
 * Number of Frames sent.
 */
#define IAXXX_IN_CH_GRP_CH_NSENT_CH_NSENT_MASK 0xffffffff
#define IAXXX_IN_CH_GRP_CH_NSENT_CH_NSENT_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_NSENT_CH_NSENT_POS 0
#define IAXXX_IN_CH_GRP_CH_NSENT_CH_NSENT_SIZE 32

/*** IN_CH_GRP_CH_NRECVD (0x0180001c) ***/
/*
 * Number of Frames received.
 */
#define IAXXX_IN_CH_GRP_CH_NRECVD_ADDR (0x0180001c)
#define IAXXX_IN_CH_GRP_CH_NRECVD_MASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_NRECVD_RMASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_NRECVD_WMASK_VAL 0x00000000
#define IAXXX_IN_CH_GRP_CH_NRECVD_RESET_VAL 0x00000000

/*
 * Number of Frames received.
 */
#define IAXXX_IN_CH_GRP_CH_NRECVD_CH_NRECVD_MASK 0xffffffff
#define IAXXX_IN_CH_GRP_CH_NRECVD_CH_NRECVD_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_NRECVD_CH_NRECVD_POS 0
#define IAXXX_IN_CH_GRP_CH_NRECVD_CH_NRECVD_SIZE 32

/*** IN_CH_GRP_CH_ENDPOINT_STATE (0x01800020) ***/
/*
 * Channel Endpoint state.
 */
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_ADDR (0x01800020)
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_MASK_VAL 0x00000007
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_RMASK_VAL 0x00000007
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_WMASK_VAL 0x00000000
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_RESET_VAL 0x00000000

/*
 * State of the endpoint
 *   0 - ENDPOINT_STATE_OFFLINE
 *   1 - ENDPOINT_STATE_ONLINE
 */
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_CH_STATE_MASK 0x00000001
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_CH_STATE_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_CH_STATE_POS 0
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_CH_STATE_SIZE 1

/*
 * Combined state of sink endpoint(s).
 *   0 - ENDPOINT_STATE_OFFLINE
 *   1 - ENDPOINT_STATE_ONLINE
 */
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_CH_SNK_STATE_MASK 0x00000002
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_CH_SNK_STATE_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_CH_SNK_STATE_POS 1
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_CH_SNK_STATE_SIZE 1

/*
 * Frame state.
 *   0 - Frame Unavailable
 *   1 - Frame Available
 */
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_FRAME_STATE_MASK 0x00000004
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_FRAME_STATE_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_FRAME_STATE_POS 2
#define IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_FRAME_STATE_SIZE 1

/*** IN_CH_GRP_CH_INTR_CNT (0x01800024) ***/
/*
 * Channel Interrupt Count.
 */
#define IAXXX_IN_CH_GRP_CH_INTR_CNT_ADDR (0x01800024)
#define IAXXX_IN_CH_GRP_CH_INTR_CNT_MASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_INTR_CNT_RMASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_INTR_CNT_WMASK_VAL 0x00000000
#define IAXXX_IN_CH_GRP_CH_INTR_CNT_RESET_VAL 0x00000000

/*
 * Channel interrupt count.
 */
#define IAXXX_IN_CH_GRP_CH_INTR_CNT_CH_INTR_CNT_MASK 0xffffffff
#define IAXXX_IN_CH_GRP_CH_INTR_CNT_CH_INTR_CNT_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_INTR_CNT_CH_INTR_CNT_POS 0
#define IAXXX_IN_CH_GRP_CH_INTR_CNT_CH_INTR_CNT_SIZE 32

/*** IN_CH_GRP_CH_DROP_CNT (0x01800028) ***/
/*
 * Channel Drop Count.
 */
#define IAXXX_IN_CH_GRP_CH_DROP_CNT_ADDR (0x01800028)
#define IAXXX_IN_CH_GRP_CH_DROP_CNT_MASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_DROP_CNT_RMASK_VAL 0xffffffff
#define IAXXX_IN_CH_GRP_CH_DROP_CNT_WMASK_VAL 0x00000000
#define IAXXX_IN_CH_GRP_CH_DROP_CNT_RESET_VAL 0x00000000

/*
 * Channel drop count.
 */
#define IAXXX_IN_CH_GRP_CH_DROP_CNT_CH_DROP_CNT_MASK 0xffffffff
#define IAXXX_IN_CH_GRP_CH_DROP_CNT_CH_DROP_CNT_RESET_VAL 0x0
#define IAXXX_IN_CH_GRP_CH_DROP_CNT_CH_DROP_CNT_POS 0
#define IAXXX_IN_CH_GRP_CH_DROP_CNT_CH_DROP_CNT_SIZE 32

/* Number of registers in the module */
#define IAXXX_IN_CH_GRP_REG_NUM 11

#endif /* __IAXXX_REGISTER_DEFS_IN_CH_GRP_H__ */
