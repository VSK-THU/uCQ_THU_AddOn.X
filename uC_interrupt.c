//##############################################################################
//    filename:        uC_intrrupt.c
//##############################################################################
//    interrupt functions for uCQuick demo project
//##############################################################################
//
//      Author:            	V.SchK
//      Company:        	HS-Ulm
//
//      Revision:        	2.0 (XC8 compatibility)
//      Date:               November 2014
//     	Assembled using		MPLAB X  1.4 / C18 3.40+ / XC8 1.32+
//
//   	todo	- add comments ;-)
//             	-
//
//##############################################################################

/** I N C L U D E S ***********************************************************/
#include "uCQuick/uCQ_2018.h"
#include "AddOnBoards.h"
#include <stdint.h>


/** P R I V A T E  P R O T O T Y P E S ****************************************/
//void __interrupt(high_priority) high_isr(void);
//void __interrupt(low_priority) low_isr(void);

/** D E C L A R A T I O N S ***************************************************/

//##############################################################################
// Function:        void high_isr(void)
// PreCondition:    None
// Input:
// Output:
// Side Effects:
// Overview:
//##############################################################################
void __interrupt(high_priority) high_isr(void)
{
    if (ENC_IR){
        if(ENC_DIR == ENC_DIR_UP){ flags.encUp = 1; }
          else{ flags.encDown = 1; }
        mENC_IR_RST();
        return;
    }

#if defined(ADDON_SRF05)||defined(ADDON_SRF04)
    static uint16_t timeSRF_edg1;

    if (SRF_TMR_IR)   //------------------------------------------------------
    {
        mSRF_TMR_IR_CLR();
        mSRF_TRIGGER();
        CCP1CONbits.CCP1M = CAPTURE_RISING_EDGE;    // wait for rising edge
        flags.all = 0;
        return;
    }
    if (SRF_IR) // -------------------------------------------- SRF_IR
    {
        mSRF_IR_CLR();
        if (CCP1CONbits.CCP1M == CAPTURE_RISING_EDGE){
            CCP1CONbits.CCP1M = CAPTURE_FALLING_EDGE;
            timeSRF_edg1 = (uint16_t)CCPR1;
        }
        else {
            CCP1CONbits.CCP1M = CAPTURE_MODULE_OFF;
            timeSRF = (uint16_t)CCPR1 - timeSRF_edg1;
            flags.newSRF = 1;
        }
        return;
    }
#endif

#if defined(ADDON_RGB_LED)
    if (RGB_TMR_IR){   //-------------------------------------------------------
        mRGB_TMR_IR_CLR();

        if(rgbSeq){
            if(++rgbTime == 0) if(dutyR) LED_R_VAL = rgbLED_ON;
            if(rgbTime == dutyR) LED_R_VAL = rgbLED_OFF;
            if(rgbTime == startG)  if(dutyG) LED_G_VAL = rgbLED_ON;
            if(rgbTime == (startG + dutyG)) LED_G_VAL = rgbLED_OFF;
            if(rgbTime == startB)  if(dutyB) LED_B_VAL = rgbLED_ON;
            if(rgbTime == (startB + dutyB)) LED_B_VAL = rgbLED_OFF;
            if(rgbTime >= endRGB) rgbTime = 0xFFFF;
        }
        else{
            if(++rgbTime == 0){
                if(dutyR) LED_R_VAL = rgbLED_ON;
                if(dutyG) LED_G_VAL = rgbLED_ON;
                if(dutyB) LED_B_VAL = rgbLED_ON;
            }
            if(rgbTime == dutyR) LED_R_VAL = rgbLED_OFF;
            if(rgbTime == dutyG) LED_G_VAL = rgbLED_OFF;
            if(rgbTime == dutyB) LED_B_VAL = rgbLED_OFF;
            if(rgbTime >= 99) rgbTime = 0xFFFF;
        }
        return;
    }
#endif

    while(1){;}                         // (detect unexpected IR sources)
}

//##############################################################################
// Function:        void low_isr(void)
// PreCondition:    None
// Input:
// Output:
// Side Effects:
// Overview:
//##############################################################################
#ifdef USE_IR_PRIORITIES
    void __interrupt(low_priority) low_isr(void)
    {
        while(1){;}                         // (detect unexpected IR sources)
    }
#endif
