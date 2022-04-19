////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2007 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (¡§MStar Confidential Information¡¨) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////


//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/kernel.h>
#include "mdrv_iic_io.h"
#define I2C_SLAVE_ID (1<<4 | 0xF2)
static U8 TunerEWBSBusNumber=0;

int _I2C_WriteBytes(U16 u16BusNumSlaveID, U8 u8addrcount, U8* pu8addr, U16 u16size, U8* pu8data);
int _I2C_ReadBytes(U16 u16BusNumSlaveID, U8 u8AddrNum, U8* paddr, U16 u16size, U8* pu8data); 
int _I2C_Channel_Set(U8 ch_num);
int _I2C_Channel_Change(U8 ch_num);

int _I2C_WriteBytes(U16 u16BusNumSlaveID, U8 u8addrcount, U8* pu8addr, U16 u16size, U8* pu8data)
{
    if(MDrv_SW_IIC_WriteBytes(TunerEWBSBusNumber,0xF2, u8addrcount, pu8addr, u16size, pu8data)!=0)
        return FALSE;
    else
        return TRUE;

}
int _I2C_ReadBytes(U16 u16BusNumSlaveID, U8 u8AddrNum, U8* paddr, U16 u16size, U8* pu8data)
{
    if(MDrv_SW_IIC_ReadBytes(TunerEWBSBusNumber,0xF2,u8AddrNum, paddr, u16size, pu8data)!=0)
        return FALSE;
    else
        return TRUE;

}

static int ReadMSB3400Reg(U16 u16Addr, U8 *pu8Data)
{
    int bRet = TRUE;
    U8 u8MsbData[6] = {0};
    
    u8MsbData[0] = 0x10;
    u8MsbData[1] = 0x00;
    u8MsbData[2] = 0x00;
    u8MsbData[3] = (u16Addr >> 8) &0xff;
    u8MsbData[4] = u16Addr &0xff;

    u8MsbData[0] = 0x35;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, 0, 1, u8MsbData);

    u8MsbData[0] = 0x10;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, 0, 5, u8MsbData);
    bRet &= _I2C_ReadBytes(I2C_SLAVE_ID, 0, 0, 1, pu8Data);

    u8MsbData[0] = 0x34;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, 0, 1, u8MsbData);

    return bRet;
}

static int WriteMSB3400Reg(U16 u16Addr, U8 u8Data)
{
    int bRet = TRUE;
    U8   u8MsbData[6] = {0};

    u8MsbData[0] = 0x10;
    u8MsbData[1] = 0x00;
    u8MsbData[2] = 0x00;
    u8MsbData[3] = (u16Addr >> 8) &0xff;
    u8MsbData[4] = u16Addr &0xff;
    u8MsbData[5] = u8Data;

    u8MsbData[0] = 0x35;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, 0, 1, u8MsbData);
    u8MsbData[0] = 0x10;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, 0, 6, u8MsbData);
    u8MsbData[0] = 0x34;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, 0, 1, u8MsbData);

    return bRet;
}


int _I2C_Channel_Set(U8 ch_num)
{
    int bRet = TRUE;
    U8 Data[5] = {0x53, 0x45, 0x52, 0x44, 0x42};

    //Exit
    Data[0] = 0x34;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, NULL, 1, Data);
    Data[0]=(ch_num & 0x01)? 0x36 : 0x45;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, NULL, 1, Data);

    //Init
    Data[0] = 0x53;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, NULL, 5, Data);
    Data[0]=(ch_num & 0x04)? 0x80 : 0x81;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, NULL, 1, Data);
    
    if ((ch_num==4)||(ch_num==5)||(ch_num==1))
        Data[0] = 0x82;
    else
        Data[0] = 0x83;
    
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, NULL, 1, Data);

    if ((ch_num==4)||(ch_num==5))
        Data[0]=0x85;
    else
        Data[0] = 0x84;

    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, NULL, 1, Data);
    Data[0]=(ch_num & 0x01)? 0x51 : 0x53;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, NULL, 1, Data);
    Data[0]=(ch_num & 0x01)? 0x37 : 0x7F;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, NULL, 1, Data);
    Data[0] = 0x35;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, NULL, 1, Data);
    Data[0] = 0x71;
    bRet &= _I2C_WriteBytes(I2C_SLAVE_ID, 0, NULL, 1, Data);

    return bRet;
}

int _I2C_Channel_Change(U8 ch_num)
{
    int bRet = TRUE;
    U8 Data[5] = {0x53, 0x45, 0x52, 0x44, 0x42};

    Data[0] = (ch_num & 0x01)? 0x81 : 0x80;
    bRet&= _I2C_WriteBytes(I2C_SLAVE_ID, 0, 0, 1, Data);
    Data[0] = (ch_num & 0x02)? 0x83 : 0x82;
    bRet&= _I2C_WriteBytes(I2C_SLAVE_ID, 0, 0, 1, Data);
    Data[0] = (ch_num & 0x04)? 0x85 : 0x84;
    bRet&= _I2C_WriteBytes(I2C_SLAVE_ID, 0, 0, 1, Data);

    return bRet;
}

int BYPASS_EWBS_CHANNEL(int isEnable)
{
    U8 u8Data=0;
    _I2C_Channel_Set(0);
    _I2C_Channel_Change(3);
    if(isEnable==1)
    {
        ReadMSB3400Reg(0x20C1,&u8Data);
        printk("\n\r [Current EWBS Status =0x%x]",u8Data);
        ReadMSB3400Reg(0x20C0,&u8Data);
        printk("\n\r [Current EWBS Control Bit =0x%x]",u8Data);
        ReadMSB3400Reg(0x20C2, &u8Data);
        if ((u8Data & 0x14) == 0x14) // For EWBS mode
        {
            if (WriteMSB3400Reg(0x20C2, 0x07)==FALSE) return FALSE;
            if (WriteMSB3400Reg(0x20C0, 0x00)==FALSE) return FALSE;

            while (u8Data!=0x02) // Waiting to enter FW idle state
            {
                if (ReadMSB3400Reg(0x20C1, &u8Data)==FALSE) return FALSE;
            }
        }    
        WriteMSB3400Reg(0x0910,0x10);
        printk("\n\r ENABLE BYPASSS MODE ON");
    }
    else
    {
        WriteMSB3400Reg(0x0910,0x00);
        printk("\n\r DISABLE BYPASSS MODE OFF");
    }
    return TRUE;
}

int CHECK_EWBS_DEMOD_EXIST(U8 TunerBusNumber)
{  
    
    U8 Chipdata=0;
    TunerEWBSBusNumber=TunerBusNumber;
    _I2C_Channel_Set(0);
    _I2C_Channel_Change(3);
    ReadMSB3400Reg(0x0900,&Chipdata);
    if(Chipdata==0xD2)
    {
        printk("\n\rCurrent Board use EWBS demod ");
        return TRUE;
    }
    else
    {
        printk("\n\rCurrent Board didn't use EWBS demod");
        return FALSE;
    }
}