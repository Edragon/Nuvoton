/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/06/11 10:13a $
 * @brief    Demonstrate ADC conversion and comparison function by monitoring
 *           the conversion result of channel 0.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC029FAE.h"


void ADC_IRQHandler(void)
{
    uint32_t u32Flag;

    // Get ADC comparator interrupt flag
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_CMP0_INT | ADC_CMP1_INT);

    if(u32Flag & ADC_CMP0_INT)
        printf("Channel 0 input < 0x200\n");
    if(u32Flag & ADC_CMP1_INT)
        printf("Channel 0 input >= 0x200\n");

    ADC_CLR_INT_FLAG(ADC, u32Flag);
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12MHz XTAL (UART), internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Enable UART and ADC clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_ADC_EN_Msk;

    /* Select UART clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_S_Msk) | CLK_CLKSEL1_UART_S_XTAL;

    /* ADC clock source is 22.1184MHz, set divider to (3 + 1), ADC clock is 22.1184/4 MHz */
    CLK->CLKDIV |= (3 << CLK_CLKDIV_ADC_N_Pos);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD, and  ADC channel 5 */
    SYS->P1_MFP = SYS_MFP_P12_RXD | SYS_MFP_P13_TXD | SYS_MFP_P15_AIN5;

    /* Analog pin OFFD to prevent leakage */
    P1->OFFD |= (1 << 5) << GPIO_OFFD_OFFD_Pos;

    /* Lock protected registers */
    SYS_LockReg();
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
    UART_Open(UART, 115200);

    printf("\nThis sample code demonstrate ADC conversion and comparison function\n");
    printf("by monitoring the conversion result of channel 5 (P1.5)\n");

    // Enable channel 0
    ADC_Open(ADC, 0, 0, (1 << 5));

    // Power on ADC
    ADC_POWER_ON(ADC);

    // Configure and enable Comparator 0 to monitor channel 5 input less than 0x200
    ADC_ENABLE_CMP0(ADC, 5, ADC_CMP_LESS_THAN, 0x200, 16);
    // Configure and enable Comparator 1 to monitor channel 5 input greater or equal to 0x200
    ADC_ENABLE_CMP1(ADC, 5, ADC_CMP_GREATER_OR_EQUAL_TO, 0x200, 16);

    // Enable ADC comparator 0 and 1 interrupt
    ADC_EnableInt(ADC, ADC_CMP0_INT);
    ADC_EnableInt(ADC, ADC_CMP1_INT);
    NVIC_EnableIRQ(ADC_IRQn);

    while(1)
    {
        // Trigger ADC conversion if it is idle
        if(!ADC_IS_BUSY(ADC))
        {
            ADC_START_CONV(ADC);
        }
    }

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


