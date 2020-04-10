/**************************************************************************//**
 * @file     main.c
 * @version  V2.10
 * $Date: 14/06/11 10:13a $
 * @brief    Use GPIO driver to control the GPIO pin direction, control
 *           their high/low state, and how to use GPIO interrupts
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC029FAE.h"

/**
 * @brief       Port0/Port1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port0/Port1 default IRQ, declared in startup_NUC029FAE.s.
 */
void GPIO01_IRQHandler(void)
{
    uint32_t reg;
    /* To check if P0.7 interrupt occurred */
    if (P0->ISRC & BIT7)
    {
        P0->ISRC = BIT7;
        printf("P0.7 INT occurred. \n");

    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT0, PORT1 interrupts */
        reg = P0->ISRC;
        P0->ISRC = reg;
        reg = P1->ISRC;
        P1->ISRC = reg;
        printf("Un-expected interrupts. \n");
    }
}


/**
 * @brief       Port2/Port3/Port4 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port2/Port3/Port4 default IRQ, declared in startup_NUC029FAE.s.
 */
void GPIO234_IRQHandler(void)
{
    uint32_t reg;
    /* To check if P2.5 interrupt occurred */
    if (P2->ISRC & BIT5)
    {
        P2->ISRC = BIT5;
        printf("P2.5 INT occurred. \n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT2, PORT3 and PORT4 interrupts */
        reg = P2->ISRC;
        P2->ISRC = reg;
        reg = P3->ISRC;
        P3->ISRC = reg;
        reg = P4->ISRC;
        P4->ISRC = reg;
        printf("Un-expected interrupts. \n");
    }
}


/**
 * @brief       External INT0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT0(P3.2) default IRQ, declared in startup_NUC029FAE.s.
 */
void EINT0_IRQHandler(void)
{
    /* For P3.2, clear the INT flag */
    P3->ISRC = BIT2;
    P30 = P30 ^ 1;
    printf("P3.2 EINT0 occurred. \n");
}

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
    SYS_ResetModule(UART_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART, 115200);

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    int32_t i32Err;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------+ \n");
    printf("|      NUC029FAE Driver Sample Code   | \n");
    printf("+-------------------------------------+ \n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect P1.5 and P3.4 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Configure P1.5 as Output mode and P3.4 as Input mode then close it */
    GPIO_SetMode(P1, BIT5, GPIO_PMD_OUTPUT);
    GPIO_SetMode(P3, BIT4, GPIO_PMD_INPUT);

    i32Err = 0;
    printf("  GPIO Output/Input test ...... \n");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    P15 = 0;
    if (P34 != 0)
    {
        i32Err = 1;
    }

    P15 = 1;
    if (P34 != 1)
    {
        i32Err = 1;
    }

    if ( i32Err )
    {
        printf("  [FAIL] --- Please make sure P1.5 and P3.4 are connected. \n");
    }
    else
    {
        printf("  [OK] \n");
    }

    /* Configure P1.5 and P3.4 to default Quasi-bidirectional mode */
    GPIO_SetMode(P1, BIT5, GPIO_PMD_QUASI);
    GPIO_SetMode(P3, BIT4, GPIO_PMD_QUASI);


    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("\n  P07, P25, and P32(INT0) are used to test interrupt\n");

    /* Configure P0.7 as Quasi-bidirection mode and enable interrupt by rising and falling edge trigger */
    GPIO_SetMode(P0, BIT7, GPIO_PMD_QUASI);
    GPIO_EnableEINT1(P0, 7, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(GPIO01_IRQn);


    /*  Configure P2.5 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(P2, BIT5, GPIO_PMD_QUASI);
    GPIO_EnableInt(P2, 5, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPIO234_IRQn);

    /* Configure P3.2 as EINT0 pin and enable interrupt by falling edge trigger */
    GPIO_SetMode(P3, BIT2, GPIO_PMD_QUASI);
    GPIO_EnableEINT0(P3, 2, GPIO_INT_FALLING);
    NVIC_EnableIRQ(EINT0_IRQn);


    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBNCECON_DBCLKSRC_HCLK, GPIO_DBNCECON_DBCLKSEL_1);
    GPIO_ENABLE_DEBOUNCE(P0, BIT7);
    GPIO_ENABLE_DEBOUNCE(P2, BIT5);
    GPIO_ENABLE_DEBOUNCE(P3, BIT2);

    /* Waiting for interrupts */
    while (1);

}


