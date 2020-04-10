/**************************************************************************//**
* @file     main.c
* @version  V2.00
* $Revision: 2 $
* $Date: 14/06/11 10:13a $
* @brief    Show how to print and get character with IDE console window
*
* @note
* Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*
******************************************************************************/

#include <stdio.h>
#include "NUC029FAE.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Main Function                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main()
{
    int8_t item;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\n Start SEMIHOST test: \n");

    while(1)
    {
        item = getchar();
        printf("%c\n",item);
    }

}





/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/



