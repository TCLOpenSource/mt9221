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

#ifndef _PROTOCOLS_COMMON_
#define _PROTOCOLS_COMMON_

/**************************** IR Common Definition ********************************/
//IR shotcount Status  enum
typedef enum{
    STATE_INACTIVE = 0,
    STATE_HEADER_SPACE,
    STATE_BIT_DATA,
    STATE_BIT_PULSE,
    STATE_BIT_SPACE,
    STATE_BIT_TRAILER,
}IR_Status_e;

typedef enum{
	IR_KEYTYPE_8 = 0,
	IR_KEYTYPE_16,
}IRKEY_type;
#define XTAL_CLOCK_FREQ             12000000    //12 MHz
#define IR_CKDIV_NUM          ((XTAL_CLOCK_FREQ+500000)/1000000)
#define IR_CLK                (XTAL_CLOCK_FREQ/1000000)
#define irGetCnt(time) ((u32)((double)time*((double)IR_CLK)/(IR_CKDIV_NUM+1)))

#define irGetCntValSplit(a,b,v)         (irGetCnt(a + (b - a + 1)*v))
#define irGetMinCnt(time, tolerance)   ((u32)(((double)time*((double)IR_CLK)/(IR_CKDIV_NUM+1))*((double)1-tolerance)))
#define irGetMaxCnt(time, tolerance)   ((u32)(((double)time*((double)IR_CLK)/(IR_CKDIV_NUM+1))*((double)1+tolerance)))


static inline bool eq_margin(unsigned d, unsigned d1, unsigned d2)
{
    return ((d > d1) && (d < d2));
}
#define FALSE           0
#define TRUE            1

/***********************************************************************************/
//&step 1: IR KEYMAPS  Define
#define NAME_KEYMAP_MSTAR_TV                  "ir-mstar-tv"
#define NAME_KEYMAP_TCL_RCA_TV                "ir-tcl-rca-tv"
#define NAME_KEYMAP_TCL_TV                    "ir-tcl-tv"
#define NAME_KEYMAP_HISENSE_TV                "ir-hisense-tv"
#define NAME_KEYMAP_CHANGHONG_TV              "ir-changhong-tv"
#define NAME_KEYMAP_HAIER_TV                  "ir-haier-tv"
#define NAME_KEYMAP_KONKA_TV                  "ir-konka-tv"
#define NAME_KEYMAP_SKYWORTH_TV               "ir-skyworth-tv"
#define NAME_KEYMAP_P7051_STB                 "ir-p7051-stb"  //panasonic 7051 stb ir keymap
#define NAME_KEYMAP_KATHREIN_TV               "ir-Kathrein-tv"
#define NAME_KEYMAP_RC5_TV                    "ir-rc5-tv"
#define NAME_KEYMAP_METZ_RM18_TV              "ir-metz-rm18-tv"
#define NAME_KEYMAP_METZ_RM19_TV              "ir-metz-rm19-tv"
#define NAME_KEYMAP_GOME_TV                   "ir-gome-tv"
#define NAME_KEYMAP_BEKO_RC5_TV               "ir-beko-rc5-tv"

#define    NUM_KEYMAP_MSTAR_TV                0x807F
#define    NUM_KEYMAP_TCL_TV                  0x00F0
#define    NUM_KEYMAP_GOME_TV0                0x6D92
#define    NUM_KEYMAP_CHANGHONG_TV            0x40BF
#define    NUM_KEYMAP_HISENSE_TV              0x00BF
#define    NUM_KEYMAP_HAIER_TV                0x1818
#define    NUM_KEYMAP_KONKA_TV                0x0002
#define    NUM_KEYMAP_SKYWORTH_TV             0x0E0E
#define    NUM_KEYMAP_P7051_STB               0x484E5958
#define    NUM_KEYMAP_KATHREIN_TV             0x8046
#define    NUM_KEYMAP_TCL_RCA_TV              0xF000
#define    NUM_KEYMAP_RC5_TV                  0x0008
#define    NUM_KEYMAP_METZ_RM18_TV            0x0609
#define    NUM_KEYMAP_METZ_RM19_TV            0x0708
#define    NUM_KEYMAP_RC311_SAMPO0            0x003F
#define    NUM_KEYMAP_RC311_SAMPO1            0x0A2D
#define    NUM_KEYMAP_BEKO_RC5_TV             0x0000
#define    NUM_KEYMAP_MAX                     0xFFFFFFFF

typedef enum{
    IR_TYPE_FULLDECODE_MODE = 0,   /* NEC full deocoder mode */
    IR_TYPE_RAWDATA_MODE,          /* NEC raw decoder mode*/
    IR_TYPE_HWRC5_MODE,            /* RC5 decoder mode*/
    IR_TYPE_HWRC5X_MODE,           /* RC5_ext decoder mode*/
    IR_TYPE_HWRC6_MODE,            /* RC6_mode0 decoder mode*/
    IR_TYPE_SWDECODE_MODE,         /* SW decoder mode*/
    IR_TYPE_MAX_MODE
}IR_Mode_e;

/***********************************************************************************/
//&step2:  IR  protocols Type enum
typedef enum{
    IR_TYPE_NEC = 0,        /* NEC protocol */
    IR_TYPE_RC5,            /* RC5 protocol*/
    IR_TYPE_RC6,            /* RC6 protocol*/
    IR_TYPE_RCMM,           /* RCMM protocol*/
    IR_TYPE_KONKA,          /* Konka protocol*/
    IR_TYPE_HAIER,          /* Haier protocol*/
    IR_TYPE_RCA,            /*TCL RCA protocol**/
    IR_TYPE_P7051,          /*Panasonic 7051 protocol**/
    IR_TYPE_TOSHIBA,        /*Toshiba protocol*/
    IR_TYPE_RC5X,           /* RC5 ext protocol*/
    IR_TYPE_RC6_MODE0,      /* RC6  mode0 protocol*/
    IR_TYPE_METZ,           /*Metz remote control unit protocol*/
    IR_TYPE_PANASONIC,      /* PANASONIC protocol*/
    IR_TYPE_RC311,          /*RC311(RC-328ST) remote control */
    IR_TYPE_BEKO_RC5,       /* Beko protocal(customized RC5) */
    IR_TYPE_SHARP,          /* Sharp protocol*/
    IR_TYPE_MAX,
}IR_Type_e;

#define NEC_PERIOD       108+5
#define RC5_PERIOD       114+5
#define RC6_PERIOD       106+5
#define RCMM_PERIOD      28+2
#define KONKA_PERIOD     60+3
#define HAIER_PERIOD     150+5    //discuss
#define RCA_PERIOD       64+3
#define P7051_PERIOD     50+3
#define TOSHIBA_PERIOD   108+5
#define RC5X_PERIOD      114+5
#define RC6_MODE0_PERIOD 106+5
#define METZ_PERIOD      125+6
#define PANASONIC_PERIOD 130+6
#define RC311_PERIOD     100+5
#define BEKO_RC5_PERIOD  114+5
#define SHARP_PERIOD     135+4

//Description  of IR
typedef struct IR_Profile_s {
    IR_Type_e eIRType;
    u32 u32HeadCode;
    u32 u32IRSpeed;
    u8 u8Enable;
}IR_Profile_t;

/***********************************************************************************/
//&step3:  Add protocol spec Description
//------------------------- NEC [ Do not modify ]----------------------------//
//Standard NEC Timming Spec
#define NEC_NBITS       32
#define NEC_HEADER_PULSE    9000//us
#define NEC_HEADER_SPACE    4500
#define NEC_REPEAT_SPACE    2250
#define NEC_BIT_PULSE       560
#define NEC_BIT_0_SPACE     560
#define NEC_BIT_1_SPACE     1680

#define NEC_REPEAT_TIMEOUT 140UL   // > nec cycle 110ms

//low/up bound Coefficient   [Calculate The Specific Timming Data According To The Differ Coefficient]
#define NEC_HEADER_PULSE_LWB    irGetMinCnt(NEC_HEADER_PULSE,0.3)   //9000  *(1-0.2) us
#define NEC_HEADER_PULSE_UPB    irGetMaxCnt(NEC_HEADER_PULSE,0.3)   //9000 *(1+0.2) us

#define NEC_HEADER_SPACE_LWB    irGetMinCnt(NEC_HEADER_SPACE,0.25)   //4500 *(1-0.2) us
#define NEC_HEADER_SPACE_UPB    irGetMaxCnt(NEC_HEADER_SPACE,0.25)   //4500 *(1+0.2) us

#define NEC_REPEAT_SPACE_LWB    irGetMinCnt(NEC_REPEAT_SPACE,0.2)   //2250*(1-0.2) us
#define NEC_REPEAT_SPACE_UPB    irGetMaxCnt(NEC_REPEAT_SPACE,0.2)   //2250*(1+0.2) us

#define NEC_BIT_PULSE_LWB       irGetMinCnt(NEC_BIT_PULSE,0.35)      //560*(1-0.35) us
#define NEC_BIT_PULSE_UPB       irGetMaxCnt(NEC_BIT_PULSE,0.35)      //560*(1+0.35) us

#define NEC_BIT_0_SPACE_LWB     irGetMinCnt(NEC_BIT_0_SPACE,0.35)    //560*(1-0.2) us
#define NEC_BIT_0_SPACE_UPB     irGetMaxCnt(NEC_BIT_0_SPACE,0.35)    //560*(1+0.2) us

#define NEC_BIT_1_SPACE_LWB     irGetMinCnt(NEC_BIT_1_SPACE,0.2)    //1680*(1-0.2) us
#define NEC_BIT_1_SPACE_UPB     irGetMaxCnt(NEC_BIT_1_SPACE,0.2)    //1680*(1+0.2) us

typedef struct IR_NEC_Spec_s {
    IR_Status_e eStatus;    //nec decoder status
    u8 u8BitCount;          //nec bitcount
    u8 u8RepeatTimes;       //nec repeat filter times [if u8RepeatTimes == speed , return repeat flag=1 &return ture , otherwise return false]
    u32 u32DataBits;        //nec data record  [ex:power key 0x46=0100,0110]
} IR_NEC_Spec_t;

int nec_decode_init(void);
void nec_decode_exit(void);

//--------------------------- RCA  [ Do not modify ]---------------------------//

#define RCA_NBITS       24
#define RCA_UNIT        500     //us
#define RCA_HEADER_PULSE  4000  //(8  * RCA_UNIT)
#define RCA_HEADER_SPACE  4000  //(8  * RCA_UNIT)
#define RCA_BIT_PULSE     500   //(1  * RCA_UNIT)
#define RCA_BIT_0_SPACE   1000  //(2  * RCA_UNIT)
#define RCA_BIT_1_SPACE   2000  //(4  * RCA_UNIT)

#define RCA_REPEAT_TIMEOUT 100UL   // > nec cycle 80ms

//low/up bound Coefficient   [Calculate The Specific Timming Data According To The Differ Coefficient]
#define RCA_HEADER_PULSE_LWB    irGetMinCnt(RCA_HEADER_PULSE,0.25)
#define RCA_HEADER_PULSE_UPB    irGetMaxCnt(RCA_HEADER_PULSE,0.25)

#define RCA_HEADER_SPACE_LWB    irGetMinCnt(RCA_HEADER_SPACE,0.25)
#define RCA_HEADER_SPACE_UPB    irGetMaxCnt(RCA_HEADER_SPACE,0.25)

#define RCA_BIT_PULSE_LWB       irGetMinCnt(RCA_BIT_PULSE,0.3)
#define RCA_BIT_PULSE_UPB       irGetMaxCnt(RCA_BIT_PULSE,0.3)

#define RCA_BIT_0_SPACE_LWB     irGetMinCnt(RCA_BIT_0_SPACE,0.3)
#define RCA_BIT_0_SPACE_UPB     irGetMaxCnt(RCA_BIT_0_SPACE,0.3)

#define RCA_BIT_1_SPACE_LWB     irGetMinCnt(RCA_BIT_1_SPACE,0.25)
#define RCA_BIT_1_SPACE_UPB     irGetMaxCnt(RCA_BIT_1_SPACE,0.25)

/*add TCL RCA protocol*/
typedef struct IR_RCA_Spec_s{
    IR_Status_e eStatus;
    u8 u8BitCount;
    u8 u8RepeatTimes;
    u32 u32DataBits;
} IR_RCA_Spec_t;
int rca_decode_init(void);
void rca_decode_exit(void);

//--------------------------- Panasonic 7051 [ Do not modify ]---------------------------//

#define P7051_NBITS             48
#define P7051_HEADER_PULSE      3640
#define P7051_HEADER_SPACE      1800
#define P7051_BIT_PULSE         380
#define P7051_BIT_0_SPACE       380
#define P7051_BIT_1_SPACE       1350

//low/up bound Coefficient   [Calculate The Specific Timming Data According To The Differ Coefficient]
#define P7051_HEADER_PULSE_LWB  irGetMinCnt(P7051_HEADER_PULSE,0.2)
#define P7051_HEADER_PULSE_UPB  irGetMaxCnt(P7051_HEADER_PULSE,0.2)

#define P7051_HEADER_SPACE_LWB  irGetMinCnt(P7051_HEADER_SPACE,0.2)
#define P7051_HEADER_SPACE_UPB  irGetMaxCnt(P7051_HEADER_SPACE,0.2)

#define P7051_BIT_PULSE_LWB     irGetMinCnt(P7051_BIT_PULSE,0.3)
#define P7051_BIT_PULSE_UPB     irGetMaxCnt(P7051_BIT_PULSE,0.3)

#define P7051_BIT_0_SPACE_LWB   irGetMinCnt(P7051_BIT_0_SPACE,0.3)
#define P7051_BIT_0_SPACE_UPB   irGetMaxCnt(P7051_BIT_0_SPACE,0.3)

#define P7051_BIT_1_SPACE_LWB   irGetMinCnt(P7051_BIT_1_SPACE,0.2)
#define P7051_BIT_1_SPACE_UPB   irGetMaxCnt(P7051_BIT_1_SPACE,0.2)

/*add Panasonic 7051 protocol*/
typedef struct IR_P7051_Spec_s{
    IR_Status_e eStatus;
    u8 u8BitCount;
    u8 u8RepeatTimes;
    u64 u64DataBits;
} IR_P7051_Spec_t;

int p7051_decode_init(void);
void  p7051_decode_exit(void);

//--------------------------- RC5  [ Do not modify ]---------------------------//

#define RC5_NBITS           14
#define RC5_BIT_MIN         889  //us
#define RC5_BIT_MAX         1778 //us

#define RC5_BIT_MIN_LWB     irGetMinCnt(RC5_BIT_MIN,0.2)  //us
#define RC5_BIT_MIN_UPB     irGetMaxCnt(RC5_BIT_MIN,0.2)  //us

#define RC5_BIT_MAX_LWB     irGetMinCnt(RC5_BIT_MAX,0.2) //us
#define RC5_BIT_MAX_UPB     irGetMaxCnt(RC5_BIT_MAX,0.2) //us


/*add Philips RC5 protocol*/
typedef struct IR_RC5_Spec_s{
    IR_Status_e eStatus;
    u8 u8BitCount;
    u8 u8RepeatTimes;
    u64 u64DataBits;
} IR_RC5_Spec_t;

int rc5_decode_init(void);
void  rc5_decode_exit(void);

//--------------------------- RC6  [ Do not modify ]---------------------------//

#define RC6_MODE0_NBITS      16
#define RC6_MODE6A_32_NBITS  32
#define MODE_BITS            8
#define TRAILER_BITS         12

#define RC6_HEADER_PULSE  2666 //us
#define RC6_HEADER_SPACE  889  //us
#define RC6_BIT_MIN       444  //us
#define RC6_BIT_MAX       889  //us
#define RC6_TRAILER_MIN   889  //us
#define RC6_TRAILER_MAX   1333 //us

#define RC6_HEADER_PULSE_LWB    irGetMinCnt(RC6_HEADER_PULSE,0.2)
#define RC6_HEADER_PULSE_UPB    irGetMaxCnt(RC6_HEADER_PULSE,0.2)

#define RC6_HEADER_SPACE_LWB    irGetMinCnt(RC6_HEADER_SPACE,0.2)
#define RC6_HEADER_SPACE_UPB    irGetMaxCnt(RC6_HEADER_SPACE,0.2)

#define RC6_BIT_MIN_LWB         irGetMinCnt(RC6_BIT_MIN,0.3)  //us
#define RC6_BIT_MIN_UPB         irGetMaxCnt(RC6_BIT_MIN,0.3)  //us

#define RC6_BIT_MAX_LWB         irGetMinCnt(RC6_BIT_MAX,0.25) //us
#define RC6_BIT_MAX_UPB         irGetMaxCnt(RC6_BIT_MAX,0.25) //us

#define RC6_TRAILER_MIN_LOB     irGetMinCnt(RC6_TRAILER_MIN,0.2) //us
#define RC6_TRAILER_MIN_UPB     irGetMaxCnt(RC6_TRAILER_MIN,0.2) //us

#define RC6_TRAILER_MAX_LOB     irGetMinCnt(RC6_TRAILER_MAX,0.2) //us
#define RC6_TRAILER_MAX_UPB     irGetMaxCnt(RC6_TRAILER_MAX,0.2) //us




/*add Philips RC6 protocol*/
typedef struct IR_RC6_Spec_s{
    IR_Status_e eStatus;
    u8 u8DataFlag;
    u8 u8RC6Mode;
    u8 u8Toggle;
    u8 u8BitCount;
    u8 u8RepeatTimes;
    u64 u64DataBits;
} IR_RC6_Spec_t;

int rc6_decode_init(void);
void  rc6_decode_exit(void);
#ifndef TCL_FACTORY_DRIVER
#define TCL_FACTORY_DRIVER    1
#endif
//--------------------------- Toshiba [ Do not modify ]---------------------------//

#define TOSHIBA_NBITS            32
#define TOSHIBA_HEADER_PULSE     4500
#define TOSHIBA_HEADER_SPACE     4500
#define TOSHIBA_BIT_PULSE        560
#define TOSHIBA_BIT_0_SPACE      560
#define TOSHIBA_BIT_1_SPACE      1690

#define TOSHIBA_REPEAT_TIMEOUT   140UL   // > toshiba  cycle 110ms

//low/up bound Coefficient   [Calculate The Specific Timming Data According To The Differ Coefficient]
#define TOSHIBA_HEADER_PULSE_LWB    irGetMinCnt(TOSHIBA_HEADER_PULSE,0.2)
#define TOSHIBA_HEADER_PULSE_UPB    irGetMaxCnt(TOSHIBA_HEADER_PULSE,0.2)

#define TOSHIBA_HEADER_SPACE_LWB    irGetMinCnt(TOSHIBA_HEADER_SPACE,0.2)
#define TOSHIBA_HEADER_SPACE_UPB    irGetMaxCnt(TOSHIBA_HEADER_SPACE,0.2)

#define TOSHIBA_BIT_PULSE_LWB       irGetMinCnt(TOSHIBA_BIT_PULSE,0.3)
#define TOSHIBA_BIT_PULSE_UPB       irGetMaxCnt(TOSHIBA_BIT_PULSE,0.3)

#define TOSHIBA_BIT_0_SPACE_LWB     irGetMinCnt(TOSHIBA_BIT_0_SPACE,0.3)
#define TOSHIBA_BIT_0_SPACE_UPB     irGetMaxCnt(TOSHIBA_BIT_0_SPACE,0.3)

#define TOSHIBA_BIT_1_SPACE_LWB     irGetMinCnt(TOSHIBA_BIT_1_SPACE,0.2)
#define TOSHIBA_BIT_1_SPACE_UPB     irGetMaxCnt(TOSHIBA_BIT_1_SPACE,0.2)

/*add Panasonic 7051 protocol*/
typedef struct IR_Toshiba_Spec_s{
    IR_Status_e eStatus;    //Toshiba decoder status
    u8 u8BitCount;          //Toshiba bitcount
    u8 u8RepeatTimes;       //Toshiba repeat filter times [if u8RepeatTimes == speed , return repeat flag=1 &return ture , otherwise return false]
    u32 u32DataBits;        //Toshiba data record  [ex:power key 0x46=0100,0110]
} IR_Toshiba_Spec_t;

int toshiba_decode_init(void);
void  toshiba_decode_exit(void);
//--------------------------- Metz [ Do not modify ]---------------------------//

#define METZ_NBITS           23
#define METZ_HEADER_PULSE    845
#define METZ_HEADER_SPACE    2355
#define METZ_BIT_PULSE       422
#define METZ_BIT_0_SPACE     978
#define METZ_BIT_1_SPACE     1678

#define METZ_REPEAT_TIMEOUT  250UL   // metz  repeat timeout

//low/up bound Coefficient   [Calculate The Specific Timming Data According To The Differ Coefficient]
#define METZ_HEADER_PULSE_LWB   irGetMinCnt(METZ_HEADER_PULSE,0.2)
#define METZ_HEADER_PULSE_UPB   irGetMaxCnt(METZ_HEADER_PULSE,0.2)

#define METZ_HEADER_SPACE_LWB   irGetMinCnt(METZ_HEADER_SPACE,0.2)
#define METZ_HEADER_SPACE_UPB   irGetMaxCnt(METZ_HEADER_SPACE,0.2)

#define METZ_BIT_PULSE_LWB      irGetMinCnt(METZ_BIT_PULSE,0.3)
#define METZ_BIT_PULSE_UPB      irGetMaxCnt(METZ_BIT_PULSE,0.3)

#define METZ_BIT_0_SPACE_LWB    irGetMinCnt(METZ_BIT_0_SPACE,0.3)
#define METZ_BIT_0_SPACE_UPB    irGetMaxCnt(METZ_BIT_0_SPACE,0.3)

#define METZ_BIT_1_SPACE_LWB    irGetMinCnt(METZ_BIT_1_SPACE,0.2)
#define METZ_BIT_1_SPACE_UPB    irGetMaxCnt(METZ_BIT_1_SPACE,0.2)

/*add Panasonic 7051 protocol*/
typedef struct IR_Metz_Spec_s{
    IR_Status_e eStatus;    //Metz decoder status
    u8 u8BitCount;          //Metz bitcount
    u8 u8RepeatTimes;       //Metz repeat filter times [if u8RepeatTimes == speed , return repeat flag=1 &return ture , otherwise return false]
    u8 u8PrevToggle;
    u32 u32DataBits;        //Metz data record  [ex:power key 0x46=0100,0110]
} IR_Metz_Spec_t;

int metz_decode_init(void);
void metz_decode_exit(void);
//------------------------- RCMM [ Do not modify ]----------------------------//
//Standard RCMM Timming Spec
#define RCMM_NBITS              32
#define RCMM_HEADER_PULSE       416 //us
#define RCMM_HEADER_SPACE       278  //us
//#define RCMM_BIT_START
#define RCMM_BIT_00             278  //us
#define RCMM_BIT_01             444  //us
#define RCMM_BIT_10             611  //us
#define RCMM_BIT_11             778 //us
#define RCMM_HEADER_PULSE_LWB   irGetMinCnt(RCMM_HEADER_PULSE,0.3)
#define RCMM_HEADER_PULSE_UPB   irGetMaxCnt(RCMM_HEADER_PULSE,0.3)
#define RCMM_HEADER_SPACE_LWB   irGetMinCnt(RCMM_HEADER_SPACE,0.3)
#define RCMM_HEADER_SPACE_UPB   irGetMaxCnt(RCMM_HEADER_SPACE,0.3)
#define RCMM_BIT_00_LWB         irGetCnt(RCMM_BIT_00/2)
#define RCMM_BIT_00_UPB         irGetCntValSplit(RCMM_BIT_00,RCMM_BIT_01,0.55)
#define RCMM_BIT_01_LWB         irGetCntValSplit(RCMM_BIT_00,RCMM_BIT_01,0.55)
#define RCMM_BIT_01_UPB         irGetCntValSplit(RCMM_BIT_01,RCMM_BIT_10,0.5)
#define RCMM_BIT_10_LWB         irGetCntValSplit(RCMM_BIT_01,RCMM_BIT_10,0.4)
#define RCMM_BIT_10_UPB         irGetCntValSplit(RCMM_BIT_10,RCMM_BIT_11,0.4)
#define RCMM_BIT_11_LWB         irGetCntValSplit(RCMM_BIT_10,RCMM_BIT_11,0.5)
#define RCMM_BIT_11_UPB         irGetCnt(RCMM_BIT_11*2)
typedef struct IR_RCMM_Spec_s
{
    IR_Status_e eStatus;
    u8 u8BitCount;
    u8 u8RepeatTimes;
    u32 u32DataBits;
} IR_RCMM_Spec_t;

int rcmm_decode_init(void);
void  rcmm_decode_exit(void);
//------------------------- KONKA [ Do not modify ]----------------------------//
//Standard KONKA Timming Spec
#define KONKA_NBITS               16
#define KONKA_HEADER_PULSE        3000//us
#define KONKA_HEADER_SPACE        3000
#define KONKA_BIT_PULSE           500
#define KONKA_BIT_0_SPACE         1500
#define KONKA_BIT_1_SPACE         2500
#define KONKA_BIT_TRAILER         4000

#define KONKA_REPEAT_TIMEOUT      100UL   // > konka cycle 110ms

//low/up bound Coefficient   [Calculate The Specific Timming Data According To The Differ Coefficient]
#define KONKA_HEADER_PULSE_LWB     irGetMinCnt(KONKA_HEADER_PULSE,0.3)   //3000  *(1-0.3) us
#define KONKA_HEADER_PULSE_UPB     irGetMaxCnt(KONKA_HEADER_PULSE,0.3)   //3000 *(1+0.3) us

#define KONKA_HEADER_SPACE_LWB     irGetMinCnt(KONKA_HEADER_SPACE,0.25)   //3000 *(1-0.25) us
#define KONKA_HEADER_SPACE_UPB     irGetMaxCnt(KONKA_HEADER_SPACE,0.25)   //3000 *(1+0.25) us

#define KONKA_BIT_PULSE_LWB        irGetMinCnt(KONKA_BIT_PULSE,0.2)      //500*(1-0.2) us
#define KONKA_BIT_PULSE_UPB        irGetMaxCnt(KONKA_BIT_PULSE,0.2)      //500*(1+0.2) us

#define KONKA_BIT_0_SPACE_LWB      irGetMinCnt(KONKA_BIT_0_SPACE,0.2)    //1500*(1-0.2) us
#define KONKA_BIT_0_SPACE_UPB      irGetMaxCnt(KONKA_BIT_0_SPACE,0.2)    //1500*(1+0.2) us

#define KONKA_BIT_1_SPACE_LWB      irGetMinCnt(KONKA_BIT_1_SPACE,0.2)    //2500*(1-0.2) us
#define KONKA_BIT_1_SPACE_UPB      irGetMaxCnt(KONKA_BIT_1_SPACE,0.2)    //2500*(1+0.2) us

#define KONKA_BIT_TRAILER_LWB      irGetMinCnt(KONKA_BIT_TRAILER,0.2)    //4000*(1-0.2) us
#define KONKA_BIT_TRAILER_UPB      irGetMaxCnt(KONKA_BIT_TRAILER,0.2)    //4000*(1+0.2) us

typedef struct IR_KONKA_Spec_s {
    IR_Status_e eStatus;    //konka decoder status
    u8 u8BitCount;          //konka bitcount
    u8 u8RepeatTimes;       //konka repeat filter times [if u8RepeatTimes == speed , return repeat flag=1 &return ture , otherwise return false]
    u16 u16DataBits;        //konka data record  [ex:power key 0x46=0100,0110]
} IR_KONKA_Spec_t;

int konka_decode_init(void);
void konka_decode_exit(void);

//------------------------- PANASONIC [ Do not modify ]----------------------------//
//Standard PANASONIC Timming Spec
#define PANASONIC_NBITS          48
#define PANASONIC_HEADER_PULSE   3488//us
#define PANASONIC_HEADER_SPACE   1744
#define PANASONIC_BIT_PULSE      436
#define PANASONIC_BIT_0_SPACE    436
#define PANASONIC_BIT_1_SPACE    1308

//panasonic repeat_time 74.62ms,repeat_cycle 119.964ms--140.892ms
#define PANASONIC_REPEAT_TINEOUT 140UL

//low/up bound Coefficient   [Calculate The Specific Timming Data According To The Differ Coefficient]
#define PANASONIC_HEADER_PULSE_LWB    irGetMinCnt(PANASONIC_HEADER_PULSE,0.3)     //3488*(1-0.3) us
#define PANASONIC_HEADER_PULSE_UPB    irGetMaxCnt(PANASONIC_HEADER_PULSE,0.3)     //3488*(1+0.3) us

#define PANASONIC_HEADER_SPACE_LWB    irGetMinCnt(PANASONIC_HEADER_SPACE,0.25)    //1744*(1-0.25) us
#define PANASONIC_HEADER_SPACE_UPB    irGetMaxCnt(PANASONIC_HEADER_SPACE,0.25)    //1744*(1+0.25) us

#define PANASONIC_BIT_PULSE_LWB       irGetMinCnt(PANASONIC_BIT_PULSE,0.25)        //436*(1-0.25) us
#define PANASONIC_BIT_PULSE_UPB       irGetMaxCnt(PANASONIC_BIT_PULSE,0.25)        //436*(1+0.25) us

#define PANASONIC_BIT_0_SPACE_LWB     irGetMinCnt(PANASONIC_BIT_0_SPACE,0.35)     //436*(1-0.35) us
#define PANASONIC_BIT_0_SPACE_UPB     irGetMaxCnt(PANASONIC_BIT_0_SPACE,0.35)     //436*(1+0.35) us

#define PANASONIC_BIT_1_SPACE_LWB     irGetMinCnt(PANASONIC_BIT_1_SPACE,0.25)      //1308*(1-0.25) us
#define PANASONIC_BIT_1_SPACE_UPB     irGetMaxCnt(PANASONIC_BIT_1_SPACE,0.25)      //1308*(1+0.25) us

typedef struct IR_PANASONIC_Spec_s {
    IR_Status_e eStatus;    //panasonic decoder status
    u8 u8BitCount;          //panasonic bitcount
    u8 u8RepeatTimes;       //panasonic repeat filter times [if u8RepeatTimes == speed , return repeat flag=1 &return ture , otherwise return false]
    u64 u64DataBits;        //panasonic data record  [ex:]
} IR_PANASONIC_Spec_t;

int panasonic_decode_init(void);
void panasonic_decode_exit(void);

//------------------------- RC311 [ Do not modify ]----------------------------//
//Standard RC311 Timming Spec
#define RC311_NBITS               24
#define RC311_HEADER_PULSE        3360  //(420us * 8) 8T
#define RC311_HEADER_SPACE        3360  //(420us * 8) 8T
#define RC311_BIT_PULSE           840   //(420us * 2)2T
#define RC311_BIT_0_SPACE         840   //(420us * 2)2T
#define RC311_BIT_1_SPACE         2520  //(420us * 6)6T

#define RC311_REPEAT_TIMEOUT      140UL   // > nec cycle 110ms

//low/up bound Coefficient   [Calculate The Specific Timming Data According To The Differ Coefficient]
#define RC311_HEADER_PULSE_LWB  irGetMinCnt(RC311_HEADER_PULSE,0.3)   //3360  *(1-0.3) us
#define RC311_HEADER_PULSE_UPB  irGetMaxCnt(RC311_HEADER_PULSE,0.3)   //3360 *(1+0.3) us

#define RC311_HEADER_SPACE_LWB  irGetMinCnt(RC311_HEADER_SPACE,0.25)   //3360 *(1-0.25) us
#define RC311_HEADER_SPACE_UPB  irGetMaxCnt(RC311_HEADER_SPACE,0.25)   //3360 *(1+0.25) us

#define RC311_BIT_PULSE_LWB     irGetMinCnt(RC311_BIT_PULSE,0.2)      //840*(1-0.2) us
#define RC311_BIT_PULSE_UPB     irGetMaxCnt(RC311_BIT_PULSE,0.2)      //840*(1+0.2) us

#define RC311_BIT_0_SPACE_LWB   irGetMinCnt(RC311_BIT_0_SPACE,0.3)    //840*(1-0.3) us
#define RC311_BIT_0_SPACE_UPB   irGetMaxCnt(RC311_BIT_0_SPACE,0.3)    //840*(1+0.3) us

#define RC311_BIT_1_SPACE_LWB   irGetMinCnt(RC311_BIT_1_SPACE,0.2)    //2520*(1-0.2) us
#define RC311_BIT_1_SPACE_UPB   irGetMaxCnt(RC311_BIT_1_SPACE,0.2)    //2520*(1+0.2) us

typedef struct IR_RC311_Spec_s
{
    IR_Status_e eStatus;
    u8 u8BitCount;
    u8 u8RepeatTimes;
    u32 u32DataBits;
} IR_RC311_Spec_t;

int rc311_decode_init(void);
void rc311_decode_exit(void);

//--------------------------- Beko RC5 ---------------------------//

#define BEKO_RC5_NBITS              14
#define BEKO_RC5_BIT_MIN          889  //us
#define BEKO_RC5_BIT_MAX       1778 //us

#define BEKO_RC5_BIT_MIN_LWB        irGetMinCnt(BEKO_RC5_BIT_MIN,0.2)  //us
#define BEKO_RC5_BIT_MIN_UPB        irGetMaxCnt(BEKO_RC5_BIT_MIN,0.2)  //us

#define BEKO_RC5_BIT_MAX_LWB        irGetMinCnt(BEKO_RC5_BIT_MAX,0.2) //us
#define BEKO_RC5_BIT_MAX_UPB        irGetMaxCnt(BEKO_RC5_BIT_MAX,0.2) //us

/*add Beko RC5 protocol*/
typedef struct IR_Beko_RC5_Spec_s{
    IR_Status_e eStatus;
    u8 u8BitCount;
    u8 u8RepeatTimes;
    u64 u64DataBits;
} IR_Beko_RC5_Spec_t;

int beko_rc5_decode_init(void);
void  beko_rc5_decode_exit(void);
//--------------------------- sharp [ Do not modify ]---------------------------//

#define SHARP_NBITS		    15
#define SHARP_BIT_0_LOW     420
#define SHARP_BIT_0_HIGH    1580
#define SHARP_BIT_1_LOW	    1580
#define SHARP_BIT_1_HIGH	4000

/*add sharp protocol*/
typedef struct IR_Sharp_Spec_s{
    IR_Status_e eStatus;    //sharp decoder status
    u8 u8BitCount;          //sharp bitcount
    u32 u32DataBits;
} IR_sharp_Spec_t;

int sharp_decode_init(void);
void sharp_decode_exit(void);
#endif
