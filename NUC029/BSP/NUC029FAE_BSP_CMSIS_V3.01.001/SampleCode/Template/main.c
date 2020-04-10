/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/06/11 10:13a $
 * @brief    A project template for NUC029FAE MCU
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC029FAE.h"


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    // Enable UART clock
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P1_MFP = SYS_MFP_P12_RXD | SYS_MFP_P13_TXD;

    /* Lock protected registers */
    SYS_LockReg();

}


int main()
{

    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART, 115200);

    printf("Hello World\n");


    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
