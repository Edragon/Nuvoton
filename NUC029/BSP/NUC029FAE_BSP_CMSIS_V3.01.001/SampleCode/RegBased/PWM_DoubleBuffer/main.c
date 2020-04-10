/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/06/11 10:13a $
 * @brief    Demonstrate the PWM double buffer feature
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC029FAE.h"

void PWM_IRQHandler(void)
{
    static int toggle = 0;

    // Update PWM channel 0 period and duty
    if(toggle == 0)
    {
        PWM->CNR[2] = 110;
        PWM->CMR[2] = 50;
    }
    else
    {
        PWM->CNR[2] = 200;
        PWM->CMR[2] = 100;
    }
    toggle ^= 1;
    // Clear channel 0 period interrupt flag;
    PWM->PIIR |= PWM_PIIR_PWMPIF2_Msk;
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->RegLockAddr != 1)
    {
        SYS->RegLockAddr = 0x59;
        SYS->RegLockAddr = 0x16;
        SYS->RegLockAddr = 0x88;
    }

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC22M_STB_Msk));

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_PWM23_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P1_MFP = SYS_MFP_P12_RXD | SYS_MFP_P13_TXD;

    /* Set P2 multi-function pins for PWM Channel */
    SYS->P2_MFP = SYS_MFP_P24_PWM2;


    /* Lock protected registers */
    SYS->RegLockAddr = 0;
}

void UART_Init(void)
{
    // Set UART to 8 bit character length, 1 stop bit, and no parity
    UART->LCR = UART_LCR_WLS_Msk;
    // 22.1184 MHz reference clock input, for 115200 bps
    // 22118400 / 115200 = 192. Using mode 2 to calculate baudrate, 192 - 2 = 190 = 0xBE
    UART->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (0xBE);
}

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Init();

    printf("\nThis sample code will output PWM channel 2 to output waveform\n");
    printf("using double buffer feature.\n");

    /*
        PWM channel 2 wave form of this sample shown below:

        |<-        CNR + 1  clk     ->|  CNR + 1 = 199 + 1 CLKs
                       |<-CMR+1 clk ->|  CMR + 1 = 99 + 1 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                               |<CMR+1>|  CMR + 1 = 39 + 1 CLKs
         ______________                ________         _____
      __|      100     |_____100______|  60    |__40___|     PWM waveform

    */



    // Set channel 2 prescaler to 2. Actual value fill into register needs to minus 1.
    PWM->PPR = 0x1 << PWM_PPR_CP23_Pos;
    // Set channel 2 clock divider to 1
    PWM->CSR = PWM_CLK_DIV_1 << PWM_CSR_CSR2_Pos;
    // Enable PWM channel 0 auto-reload mode
    PWM->PCR = PWM_PCR_CH2MOD_Msk;
    /*
      Configure PWM channel 2 init period and duty.
      Period is HCLK / (prescaler * clock divider * (CNR + 1))
      Duty ratio = (CMR + 1) / (CNR + 1)
      Period = 22.1184 MHz / (2 * 1 * (199 + 1)) =  55296 Hz
      Duty ratio = (99 + 1) / (199 + 1) = 50%
    */
    PWM->CMR[2] = 99;
    PWM->CNR[2] = 199;


    // Enable PWM channel 2 output
    PWM->POE = PWM_POE_PWM2_Msk;

    // Enable PWM channel 2 period interrupt
    PWM->PIER = PWM_PIER_PWMPIE2_Msk;
    NVIC_EnableIRQ(PWM_IRQn);

    // Start
    PWM->PCR |= PWM_PCR_CH2EN_Msk;

    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


