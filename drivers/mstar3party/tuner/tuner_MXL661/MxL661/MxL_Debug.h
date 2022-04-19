/*******************************************************************************
 *
 * FILE NAME          : MxL_Debug.h
 * 
 * AUTHOR             : Brenndon Lee
 *
 * DATE CREATED       : 10/25/2010
 *
 * DESCRIPTION        : Debug header files 
 *
 *******************************************************************************
 *                Copyright (c) 2010, MaxLinear, Inc.
 ******************************************************************************/

#ifndef __MXL_DEBUG_H__
#define __MXL_DEBUG_H__

/******************************************************************************
    Include Header Files
    (No absolute paths - paths handled by make file)
******************************************************************************/
#if 0//kdrv
#include <stdio.h>
#endif
#include "MaxLinearDataTypes.h"

/******************************************************************************
    Macros
******************************************************************************/

//#define MxL_DLL_DEBUG0 printf
#define MxL_DLL_DEBUG0(fmt, args...)          //printk("[%s][%05d] " fmt, "Mstar-MXL661", __LINE__, ## args)

#endif /* __MXL_DEBUG_H__*/
