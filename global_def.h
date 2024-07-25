//##############################################################################
//	filename:		global_def.h    (Projekt uCQ-addOn)
//##############################################################################
//
//  	Author:			VSchK
//  	Company:		HS-Ulm
//
//##############################################################################

#ifndef GLOBAL_DEF_H
#define GLOBAL_DEF_H

#include <xc.h>
#include "uCQuick/uCQ_2018.h"
#include "PLIB/plib18fxxk22.h"
#include "AddOnBoards.h"

//## D E F I N I T I O N S #####################################################

//-- module HSU_PROT (USART / handshake not used) ------------------------------
//#define PCK_SIZE	20

////## Pins (if USART not used)
//#define RS_nRES		1	// out request to send
//#define RS_nRTS		1	// out request to send
//#define RS_nCTS		0	// in clear to send
//#define RS_nDTR		1	// out data terminal ready
//#define RS_nDCD		1	// in data carrier detected
//#define RS_nDSR		1	// in data set ready
//#define RS_nRI		1	// in ring index
//
////#define RS_nRTS		LATBbits.LATB5	// out request to send
////#define RS_nCTS		PORTBbits.RB2	// in clear to send

#define BusyUSART()     Busy1USART()
#define TXREG           TXREG1
#define RCREG           RCREG1


//-- module MPU6000 / APA102 ---------------------------------------------------
#define CS_SPI          LATCbits.LATC2
//#define SCL             LATCbits.LATC3
//#define SDO             LATCbits.LATC5
#define CS_SPI_TRI      TRISCbits.TRISC2
#define SCL_TRI         TRISCbits.TRISC3
#define SCL_ANS         ANSELCbits.ANSC3
#define SDO_TRI         TRISCbits.TRISC5
#define SDI_TRI         TRISCbits.TRISC4
#define SDI_ANS         ANSELCbits.ANSC4

//-- module DS18X20 ------------------------------------------------------------
#define DS18X20_DIR     TRISCbits.TRISC4
#define DS18X20_ANS     ANSELCbits.ANSC4
#define DS18X20_IN      PORTCbits.RC4
#define DS18X20_OUT     LATCbits.LATC4


//#ifdef __DEBUG
//    #define mDEBUG_STOP()    if(mGET_ENC_BTN()){_asm TRAP _endasm}
//#else
//    #define mDEBUG_STOP()
//#endif

//##############################################################################

#endif //GLOBAL_DEF_H
