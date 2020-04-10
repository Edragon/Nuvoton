/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/06/11 10:13a $
 * @brief    Toggle P2.4 to turn on / off the board LED
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC029FAE.h"
#include "gpio.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON |= CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_XTAL,CLK_CLKDIV_HCLK(1));

    /* STCLK to XTL STCLK to XTL */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLK_S_XTAL);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART_MODULE,CLK_CLKSEL1_UART_S_XTAL,CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD and TXD */
    SYS->P1_MFP = SYS_MFP_P12_RXD | SYS_MFP_P13_TXD;

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(SYS_IPRSTC2_UART_RST_Msk);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART, 115200);

}

void delay_loop(void)
{
    __IO uint32_t j;

    for(j=0; j<60000; j++);
    for(j=0; j<60000; j++);
    for(j=0; j<60000; j++);
    for(j=0; j<60000; j++);
}



int main(void)
{
    /* If define INIT_SYSCLK_AT_BOOTING in system_M051Series.h, HCLK will be set to 50MHz in SystemInit(void)*/
    if(SYS->RegLockAddr == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART_Init();

    printf("+---------------------------------+\n");
    printf("|  NUC029FAE NuTiny Sample Code   |\n");
    printf("+---------------------------------+\n");

    /*set P2.4 to output mode */
    GPIO_SetMode(P2, BIT4, GPIO_PMD_OUTPUT);

    while(1)
    {
        P24 = 0;
        delay_loop();
        P24 = 1;
        delay_loop();
    }
}

#ifdef USE_ASSERT
/**
  * @brief  The function prints the source file name and line number where the assert_param() error
      occurs, and then stops in an infinite loop. User can add his own codes here if necessary.
  * @param  file: source file name.
  * @param  line: line number.
  * @retval None.
  */

void assert_error(uint8_t * file, uint32_t line)
{
    /*en: Infinite loop */
    while(1) ;

}
#endif
/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
