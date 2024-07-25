//##############################################################################
//      filename:        uC_main.c
//
//      main file for demo projects
//
//##############################################################################
//
//  Author:             V.SchK
//  Company:            HS-Ulm
//
//  Revision:           3.0 (XC8 compatible)
//  Date:               May 2019
//  Assembled using     XC8 2.00+
//
//  todo    - add comments ;-)
//          -
//
//##############################################################################

#include "AddOnBoards.h"


//--- P R I V A T E   P R O T O T Y P E S --------------------------------------
void __init(void);

//##############################################################################
// Function:        void __main(void)
//                      called from the startup code
// PreCondition:    None
// Input:
// Output:
// Side Effects:
// Overview:
//##############################################################################
void main()
{
//    __init();
//    while (1) {...}
    TEST_ADDON();        // addOn board is selected in AddOnBoards.h
}

//##############################################################################
// Function:        void __init(void)
//
// PreCondition:    None
// Input:
// Output:
// Side Effects:
// Overview:
//##############################################################################
void __init(void)
{
//    ANSELA = 0x01;  // all pins but A0  digital IO
//    ANSELB = 0x00;
//    ANSELC = 0x00;
}
