#ifndef MZC_HAL_H
#define MZC_HAL_H

#include <linux/wait.h>
#include <linux/irqreturn.h>
#include <linux/zsmalloc.h>
#include <linux/workqueue.h>
#include <asm/io.h>
#include "mhal_mzc_hal_diff.h"


u16 LZC_REG_READ(u16 _offset);

void LZC_REG_WRITE(u16 _offset, u16 val);

void LZC_REG_WRITE_32(u16 _offset, u32 val);

unsigned int LZC_REG_READ_32(u16 _offset);

void ACP_REG_WRITE(u16 _offset,u16 val);

u16 ACP_REG_READ(u16 _offset);

void ACACHE_REG_WRITE(u16 _offset,u16 val);

void mhal_lzc_cmdq_init(int enc_out_size_thr);

void mhal_lzc_cmdq_exit(void);

int mhal_lenc_cmdq_set(unsigned long in_addr, unsigned long out_addr, unsigned int process_id);

int mhal_ldec_cmdq_set(unsigned long in_addr, unsigned long out_addr, unsigned int process_id);

void mhal_lzc_single_init(int enc_out_size_thr);

void mhal_lzc_single_exit(void);

void mhal_lzc_irq_mask_all_except_timeout(void);

void mhal_lzc_irq_mask_all(void);

int mhal_lenc_single_set(unsigned long in_addr,unsigned long out_addr, unsigned int process_id);

int mhal_ldec_single_set(unsigned long in_addr,unsigned long out_addr, unsigned int process_id);

void mhal_ldec_cycle_info(int *active_cycle, int *free_cycle, int *page_count);

void mhal_lenc_cycle_info(int *active_cycle, int *free_cycle, int *page_count);

int mhal_lenc_cmdq_done(int *target_id);

int mhal_ldec_cmdq_done(int *target_id);

int mhal_lenc_single_done(void);

int mhal_ldec_single_done(void);

unsigned int mhal_lzc_cmdq_handler(int *w_outputcount, int *r_outputcount, int *is_write, int *is_read);

void mhal_lzc_cmdq_clear_irq(unsigned int irq_status);

void mhal_lenc_cmdq_handler(int *outlen,int *process_id);

void mhal_ldec_cmdq_handler(int *outlen, int *process_id);

void mhal_lzc_single_handler(int *w_outlen, int *r_outlen, int *is_write, int *is_read);

void mhal_regptr_set(void);

void mhal_regptr_unset(void);

void mhal_acp_common_init(void);

void mhal_ldec_state_save(int wait);

void mhal_lenc_state_save(int wait);

int mhal_lenc_state_check(void);

int mhal_ldec_state_check(void);

void mhal_lzc_common_init(int enc_out_size_thr);

void mhal_lenc_cmdq_init(void);

void mhal_ldec_cmdq_init(void);

unsigned long mhal_lenc_crc_get(void);

int mhal_lenc_cmdq_done(int *target_id);

int mhal_ldec_cmdq_done(int *target_id);

void mhal_regptr_set(void);

void mhal_acp_common_init(void);

void mhal_lzc_cmdq_init(int enc_out_size_thr);

void mhal_lzc_irq_mask_all(void);


#define _BIT(x)                      (1<<(x))
/*bank and irq */

#define regoffsetnum 0x80

#define SINGLE_MODE 0

#define CMDQ_MODE 1

#define CMDQ_LENC_NORMAL_END 1

#define CMDQ_LDEC_NORMAL_END 1

#define CMDQ_LENC_TIME_OUT 4

#define CMDQ_LDEC_TIME_OUT 16

#define RESET 0

#define NOT_RESET 1

#define LZC_BANK 0x1810

#define ACP_BANK 0x101d

#define FREQ_BANK 0x1033

#define MZC_CLK_BANK 0x1001

#define ACACHE_BANK 0x100e

#define L4_AXI_BANK 0x1018

#define E_IRQ_LZC 216

#define NONPM_BASE 0x1f000000

#define NON_SECURE_ACCESS 0x3

#define SECURE_ACCESS 0x0

#define MASK_IRQ_ALL 0xffff
#define LDEC_CMDQ_IRQ_IN_NEAR_EMPTY     _BIT(0)
#define LDEC_CMDQ_IRQ_IN_NEAR_FULL      _BIT(1)
#define LDEC_CMDQ_IRQ_IN_OVERFLOW       _BIT(2)
#define LDEC_CMDQ_IRQ_OUT_NEAR_EMPTY    _BIT(3)
#define LDEC_CMDQ_IRQ_OUT_NEAR_FULL     _BIT(4)
#define LDEC_CMDQ_IRQ_OUT_UNDERFLOW     _BIT(5)
#define LENC_CMDQ_IRQ_IN_NEAR_EMPTY     _BIT(6)
#define LENC_CMDQ_IRQ_IN_NEAR_FULL      _BIT(7)
#define LENC_CMDQ_IRQ_IN_OVERFLOW       _BIT(8)
#define LENC_CMDQ_IRQ_OUT_NEAR_EMPTY    _BIT(9)
#define LENC_CMDQ_IRQ_OUT_NEAR_FULL     _BIT(10)
#define LENC_CMDQ_IRQ_OUT_UNDERFLOW     _BIT(11)
#define LDEC_CMDQ_IRQ_TIMER   			_BIT(12)
#define LDEC_CMDQ_IRQ_NUMBER_THRESHOLD  _BIT(13)
#define LENC_CMDQ_IRQ_TIMER   			_BIT(14)
#define LENC_CMDQ_IRQ_NUMBER_THRESHOLD  _BIT(15)
#define LDEC_IRQ_NORMAL_END             _BIT(16-16)
#define LDEC_IRQ_CABAC_ERROR            _BIT(17-16)
#define LDEC_IRQ_SIZE_ERROR             _BIT(18-16)
#define LDEC_IRQ_DICT_ERROR             _BIT(19-16)
#define LDEC_IRQ_TIMEOUT                _BIT(20-16)
#define LDEC_IRQ_PKT_MARKER_ERROR       _BIT(21-16)
#define LENC_IRQ_NORMAL_END             _BIT(22-16)
#define LENC_IRQ_SIZE_ERROR             _BIT(23-16)
#define LENC_IRQ_TIMEOUT                _BIT(24-16)
#define ACP_RRESP_ERROR					_BIT(25-16)
#define ACP_BRESP_ERROR					_BIT(26-16)
#define LDEC_WRITE_INVALID_ERROR		_BIT(27-16)
#define LENC_WRITE_INVALID_ERROR		_BIT(28-16)






#define MASK_IRQ_ALL_EXCEPT_TIMER_AND_NUMBER (MASK_IRQ_ALL &  ~(LDEC_CMDQ_IRQ_TIMER | LDEC_CMDQ_IRQ_NUMBER_THRESHOLD | LENC_CMDQ_IRQ_TIMER | LENC_CMDQ_IRQ_NUMBER_THRESHOLD))

#define MASK_IRQ_ALL_EXCEPT_HW_TIMEOUT (MASK_IRQ_ALL & ~(LENC_IRQ_TIMEOUT | LDEC_IRQ_TIMEOUT))

#define SINGLE_MODE_LENC_IRQ        (LENC_IRQ_NORMAL_END | LENC_IRQ_SIZE_ERROR | LENC_IRQ_TIMEOUT)

#define SINGLE_MODE_LDEC_IRQ        (LDEC_IRQ_NORMAL_END | LDEC_IRQ_CABAC_ERROR | LDEC_IRQ_SIZE_ERROR | LDEC_IRQ_DICT_ERROR | LDEC_IRQ_TIMEOUT | LDEC_IRQ_PKT_MARKER_ERROR)

#define MASK_SINGLE_MODE_IRQ        (~(SINGLE_MODE_LENC_IRQ | SINGLE_MODE_LDEC_IRQ))

#define CLEAR_IRQ(irq)              (irq)

#define LENC_RELATED (LENC_CMDQ_IRQ_TIMER | LENC_CMDQ_IRQ_NUMBER_THRESHOLD)

#define LDEC_RELATED (LDEC_CMDQ_IRQ_TIMER | LDEC_CMDQ_IRQ_NUMBER_THRESHOLD)

#define LENC_CMDQ_TIMER_THR_LO 0xffff

#define LENC_CMDQ_TIMER_THR_HI 0xffff

#define LDEC_CMDQ_TIMER_THR_LO 0xffff

#define LDEC_CMDQ_TIMER_THR_HI 0xffff

#define LENC_TIMEOUT_THRESHOLD (0xfffff)

#define LDEC_TIMEOUT_THRESHOLD (0xfffff)

#define LENC_CRC_DEFAULT_MODE 1

#define LENC_CRC_DEFAULT_ENABLE_STATE 1

#define LENC_OUTPUT_THRESHOLD 0

#define LDEC_CRC_DEFAULT_MODE 0 // 0: input, 1: output

#define LDEC_CRC_DEFAULT_ENABLE_STATE 0

#define LDEC_OUTPUT_THRESHOLD 0

#define CMDQ_LENC_SIZE_ERROR 0x2

#define CMDQ_LDEC_ERROR_TYPE_BIT_POS ((LDEC_IRQ_CABAC_ERROR) | (LDEC_IRQ_SIZE_ERROR) | (LDEC_IRQ_DICT_ERROR) | (LDEC_IRQ_TIMEOUT) | (LDEC_IRQ_PKT_MARKER_ERROR))

#define CLEAR_LENC_IRQ (LENC_RELATED)

#define CLEAR_LDEC_IRQ (LDEC_RELATED)

#define LENC_TIMEOUT (LENC_IRQ_TIMEOUT)

#define LDEC_TIMEOUT (LDEC_IRQ_TIMEOUT)

#define CLEAR_LENC_IRQ_TIMEOUT (LENC_TIMEOUT)

#define CLEAR_LDEC_IRQ_TIMEOUT (LDEC_TIMEOUT)

#define CMDQ_TOP_SETTING 0

#define CMDQ_TOP_SETTING_002 0x2

#define MZC_VERSION_SETTING 0xd

#define RO_LDEC_OUT_INQ_LV1 0x06

#define RO_LENC_OUT_INQ_LV1 0x07

#define LDEC_INQ_IN_ST_ADR_LSB_HI 0x09

#define LDEC_INQ_IN_ST_ADR_LSB_LO 0x08

#define LDEC_INQ_DATA 0x0a

#define LDEC_INQ_OUT_ST_ADR 0x0b

#define LDEC_OUTQ_INFO 0x0e

#define LENC_INQ_DATA 0x10

#define LENC_INQ_IN_ST_ADR 0x11

#define LENC_INQ_OUT_ST_ADR_LSB_LO 0x12

#define LENC_INQ_OUT_ST_ADR_LSB_HI 0x13

#define LENC_OUTQ_INFO_LO 0x16

#define LENC_OUTQ_INFO_HI 0x17

#define LZC_OUTQ_NUM_TH 0x18

#define LDEC_OUTQ_TIMER_TH_LO 0x19

#define LDEC_OUTQ_TIMER_TH_HI 0x1a

#define LENC_OUTQ_TIMER_TH_LO 0x1b

#define LENC_OUTQ_TIMER_TH_HI 0x1c

#define LDEC_GENERAL_SETTING 0x1d

#define LDEC_IN_ST_ADR_LSB_LO 0x1e

#define LDEC_IN_ST_ADR_LSB_HI 0x1f

#define LDEC_OUT_ST_ADR_LO 0x20

#define LDEC_OUT_ST_ADR_HI 0x21

#define LDEC_TIMEOUT_THR 0x23

#define LDEC_TIMEOUT_EN 0x24

#define MZC_IRQ_MASK_HI 0x31

#define MZC_IRQ_MASK_LO 0x30

#define LZC_IRQ_STATUS_LO 0x34

#define LZC_IRQ_STATUS_HI 0x35

#define MZC_ST_IRQ_CPU_LO 0x36

#define MZC_ST_IRQ_CPU_HI 0x37

#define MZC_ST_IRQ_IP_LO 0x38

#define MZC_ST_IRQ_IP_HI 0x39

#define LENC_GENERAL_SETTING 0x40

#define LENC_IN_ST_ADR 0x41

#define LENC_OUT_ST_ADR_MSB 0x42

#define LENC_OUT_ST_ADR_LSB_HI 0x44

#define LENC_OUT_ST_ADR_LSB_LO 0x43

#define LENC_OUTSIZE_THR 0x45

#define LENC_TIMEOUT_THR 0x46

#define LENC_TIMEOUT_EN 0x47

#define LENC_PAGE_SYNC 0x50

#define RO_LENC_BYTE_CNT 0x56

#define ACP_OUTSTAND 0x60

#define ACP_AR 0x61

#define ACP_AW 0x62

#define MZC_RSV7C  0x7c

#define MZC_RSV7D  0x7d

#define ACP_IDLE_SW_FORCE 0xdfff

#define MZC_FREQ_REGISTER 0x19

#define MZC_CLCK_SETTING 0x1e

#define MZC_FULL_SPEED 0x0

#define MZC_CLCK_FIRE 0x80

#define DFS_ENABLE_ADDR 0x6a

#define DFS_FREQ_DIVISION_ADDR 0x69

#define DFS_UPDATE_ADDR 0x6b

#define MZC_RESERVED_7C 0x7c

#ifndef GET_LOWORD
#define GET_LOWORD(value)    ((unsigned short)(((unsigned int)(value)) & 0xffff))
#endif
#ifndef GET_HIWORD
#define GET_HIWORD(value)    ((unsigned short)((((unsigned int)(value)) >> 16) & 0xffff))
#endif

#endif