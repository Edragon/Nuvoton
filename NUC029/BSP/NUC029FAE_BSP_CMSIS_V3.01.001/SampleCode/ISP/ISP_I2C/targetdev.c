/******************************************************************************
 * @file     targetdev.c
 * @brief    ISP sample source file
 * @version  0x27
 * @date     31, December, 2014
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"
#include "ISP_USER.h"

//the smallest of APROM size is 16K
uint32_t GetApromSize()
{
    return 16 * 1024;
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    *size = 0;
    FMC_Read_User(Config0, &uData);

    if ((uData & 0x01) == 0)   //DFEN enable
    {
        FMC_Read_User(Config1, &uData);
        // filter the reserved bits in CONFIG1
        uData &= 0x000FFFFF;


        if (uData > g_apromSize || (uData & 0x1FF))   //avoid config1 value from error
        {
            uData = g_apromSize;
        }

        *addr = uData;
        *size = g_apromSize - uData;
    }
    else
    {
        *addr = g_apromSize;
        *size = 0;
    }
}
