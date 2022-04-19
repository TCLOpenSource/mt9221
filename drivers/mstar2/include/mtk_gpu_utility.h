/*------------------------------------------------------------------------------
 * MediaTek Inc. (C) 2019. All rights reserved.
 *
 * Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to MediaTek Inc. and/or its licensors. Without
 * the prior written permission of MediaTek inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of MediaTek Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE
 * MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO MEDIATEK FOR SUCH MEDIATEKBSOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek
 * Software") have been modified by MediaTek Inc. All revisions are subject to
 * any receiver's applicable license agreements with MediaTek Inc.
 *----------------------------------------------------------------------------*/

#ifndef _MTK_GPU_UTILITY_H_
#define _MTK_GPU_UTILITY_H_

extern int mali_gpu_clock;
extern int mali_gpu_utilization;
extern int mali_gpu_power_on;

extern unsigned int mtk_dtv_get_gpu_loading(void);
extern unsigned int mtk_dtv_get_gpu_cur_freq(void);
extern unsigned int mtk_dtv_get_gpu_max_freq(void);
extern unsigned int mtk_dtv_get_gpu_boost_freq(void);

/* MET for GPU_LOADING, GPU_DVFS */
bool mtk_get_gpu_loading(unsigned int *loading);
bool mtk_get_gpu_cur_freq(unsigned int *freq);
bool mtk_get_gpu_max_freq(unsigned int *freq);
bool mtk_get_gpu_boost_freq(unsigned int *freq);

/* MET for GPU_PMU */
typedef struct {
	int id;
	const char *name;
	unsigned int value;
	int overflow;
} GPU_PMU;
bool mtk_get_gpu_pmu_init(GPU_PMU *pmus, int pmu_size, int *ret_size);
bool mtk_get_gpu_pmu_deinit(void);
bool mtk_get_gpu_pmu_swapnreset(GPU_PMU *pmus, int pmu_size);
bool mtk_get_gpu_pmu_swapnreset_stop(void);

typedef void (*gpu_power_change_notify_fp)(int power_on);

bool mtk_register_gpu_power_change(const char *name, gpu_power_change_notify_fp callback);
bool mtk_unregister_gpu_power_change(const char *name);
void mtk_notify_gpu_power_change(int power_on);

#endif /* _MTK_GPU_UTILITY_H_ */
