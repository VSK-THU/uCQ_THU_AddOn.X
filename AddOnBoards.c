//##############################################################################
//    	filename:        	AddOnBoards.c
//                          (AddOn uC-Quick-2018 boards)
//
//##############################################################################
//
//      Author:             V.SchK
//      Company:            HS-Ulm
//
//      Revision:           4.0  (for board uCQ_2018)
//      Date:               July 2024
//      Assembled using
//
//      todo    - add comments ;-)
//             	-
//
//##############################################################################
#pragma warning disable 520 // function never called
//#pragma warning disable 373 // implicit signed to unsigned conversion

#include "AddOnBoards.h"
#include <stdint.h>

union DemoFlags flags;

//################################################################### ACC_CIRCLE
#define MAX_STATE 8
void testEncCircle(void)
{
#if defined(ADDON_ENC_CIRCLE)
    signed char state;
//--------------------------------------------------------------------- __init()
  #if _XTAL_FREQ == 1000000
    OSCCONbits.IRCF = IRCF_1MHZ;  OSCTUNEbits.PLLEN = 0;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;
    ENC_INT_TRI = INPUT_PIN; ENC_INT_ANS = DIGITAL_PIN;
    ENC_DIR_TRI = INPUT_PIN; ENC_DIR_ANS = DIGITAL_PIN;
    mENC_IR_RST();
    mENC_IR_EN();

    GLCD_Init();
    GLCD_Text2Out(0,1,"< turn >");
    GLCD_Text2Out(1,1,"? push ?");
    GLCD_Text2Out(2,5,"5"); __delay_ms(1000);   // wait 5s because LEDs will
    GLCD_Text2Out(2,5,"4"); __delay_ms(1000);   //  reset the Grafik Display
    GLCD_Text2Out(2,5,"3"); __delay_ms(1000);
    GLCD_Text2Out(2,5,"2"); __delay_ms(1000);
    GLCD_Text2Out(2,5,"1"); __delay_ms(1000);

//    while(mGET_ENC_BTN()){;}    // wait because LEDs will reset the Display
    initC_LEDs();

    flags.all = 0;
    state = 0;

    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(mGET_ENC_BTN()){                 // encoder button
            if(state <= MAX_STATE){
                setC_LEDs();
                state = MAX_STATE + 1;
            }
            else{
                clrC_LEDs();
                state = 0;
            }
            while(mGET_ENC_BTN()){;}
        }
        if(flags.encUp || flags.encDown){   // encoder (rotate)
            if (flags.encUp){
                flags.encUp = 0;
                if (++state > MAX_STATE) {
                    state = 0;
                }
            }
            if (flags.encDown) {
                flags.encDown = 0;
                if (--state < 0) {
                    state = MAX_STATE;
                }
            }
            clrC_LEDs();
            switch(state){
                case 0: break;
                case 1: LED_N = 1; break;
                case 2: LED_NO = 1; break;
                case 3: LED_O = 1; break;
                case 4: LED_SO = 1; break;
                case 5: LED_S = 1; break;
                case 6: LED_SW = 1;  break;
                case 7: LED_W = 1; break;
                case 8: LED_NW = 1; break;
                case 9: break;
                default:    break;
            }
        }//if(flags.encUp || flags.encDown){
    }//while(1){
#endif //defined(ADDON_ENC_CIRCLE)
}

//##############################################################################

uint16_t timeSRF;
void testSRF0X(void)
{
//--------------------------------------------------------------------- __init()
#if defined(ADDON_SRF05)||defined(ADDON_SRF04)
  #if _XTAL_FREQ == 4000000
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    mINIT_SRFPIN();

    // SRF timer and capture ... -----------------------------------------------
    OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_FOSC_4
                & T1_PS_1_1 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF,
                TIMER_GATE_OFF);

    CCPTMRS0bits.C1TSEL = 0b00;                 // use timer 1
    CCP1CONbits.CCP1M = CAPTURE_MODULE_OFF;
    mSRF_IR_CLR(); mSRF_IR_EN();

    GLCD_Init();
  #if defined(ADDON_SRF05)
    GLCD_Text2Out(0,1," SRF-05 ");
  #else
    GLCD_Text2Out(0,1," SRF-04 ");
    
  #endif
    GLCD_Text2Out(1,1,"     cm ");

    flags.all = 0;

    //------------------------------------- setup global interrupt system ------
    RCONbits.IPEN = 0;      //  disable interrupt priority
    INTCONbits.PEIE = 1;    //  enable peripheral interrupts
    INTCONbits.GIE = 1;     //  enable global interrupts

//----------------------------------------------------------------------- main()
    while(1){
        if(flags.newSRF){
            GLCD_Value2Out_00(1, 2, timeSRF/58 ,3);
            flags.newSRF = 0;
        }
    }
#endif //defined(ADDON_SRF05)||defined(ADDON_SRF04)
}

//##############################################################################
void testMPU6000(void)
{
#if defined(ADDON_MPU6000)
    signed short value;
    signed char state;
    unsigned short data[3];
    unsigned short accOffset[3] = {0,0,0};

//--------------------------------------------------------------------- __init()
  #if _XTAL_FREQ == 1000000
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;
    ENC_INT_TRI = INPUT_PIN; ENC_INT_ANS = DIGITAL_PIN;
    ENC_DIR_TRI = INPUT_PIN; ENC_DIR_ANS = DIGITAL_PIN;
    mENC_IR_RST();
    mENC_IR_EN();

    GLCD_Init();
    GLCD_Text2Out(0,0,"ACC in X");

    initMPU6000(ACCpm2G, GYROpm1000, SMPLRT_10, LP_5_HZ);
    enMPU6000ir(NO_IR_EN, INT_LEVEL_LOW | INT_OPEN_DRAIN | INT_ALL_RD_CLR);

    flags.all = 0;
    state = 0;

    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(flags.encUp || flags.encDown) {
            if (flags.encUp){
                flags.encUp = 0;
                if (++state > 3) {
                    state = 3;
                }
            }
            if (flags.encDown) {
                flags.encDown = 0;
                if (--state < 0) {
                    state = 0;
                }
            }
            if(state < 3){
                SSP1CON1bits.SSPEN = 0;     // SCL conflict
                GLCD_Text2Out(0,0,"ACC in ");
                GLCD_Char2Out(0,2+7*8,('X'+ state));
                SSP1CON1bits.SSPEN = 1;
            }
            else{
                SSP1CON1bits.SSPEN = 0;     // SCL conflict
                GLCD_Text2Out(0,0," Clear  ");
                GLCD_Text2Out(1,0," Offset ");
                SSP1CON1bits.SSPEN = 1;
            }
        }

// read acc values
        readACC_LE((unsigned char *)data);
// new offset ?
        if(mGET_ENC_BTN()){
            if(state < 3){
//                accOffset[0] = data[0];
//                accOffset[1] = data[1];
//                accOffset[2] = data[2];
                accOffset[state] = data[state];
            }
            else{
                accOffset[0] = accOffset[1] = accOffset[2] = 0;
            }
        }

        if(state < 3){
            value  = data[state];
            value -= accOffset[state];
            SSP1CON1bits.SSPEN = 0;     // SCL conflict
            GLCD_Text2Out(1,0,"        ");
            GLCD_Value2Out(1,0,value);
            SSP1CON1bits.SSPEN = 1;
        }
        __delay_ms(188);         // ~200ms loop
    }
#endif //defined(ADDON_MPU6000)
}

//################################################################### ACC_CIRCLE
//#define mROTATE_LEDs()  asm(" rrncf _circleLEDs, 1, 0")
// TODO abuse a SFR for this???

#define mROTATE_LEDs() if(!(circleLEDs & 0x01)){circleLEDs = circleLEDs >> 1;} \
                       else{circleLEDs = circleLEDs >> 1; circleLEDs |= 0x80;}
#define PI  3.14159265

void testAccCircle(void)
{
#if defined(ADDON_ACC_CIRCLE)
    uint8_t circleLEDs;
    uint8_t data[6];
    uint16_t accOffset_X = 0;
    uint16_t accOffset_Y = 0;
    int16_t value_X, value_Y;
    unsigned long inclination2;
    float rotation;

//--------------------------------------------------------------------- __init()
  #if _XTAL_FREQ == 8000000
    OSCCONbits.IRCF = IRCF_8MHZ; OSCTUNEbits.PLLEN = 0;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    ENC_BTN_TRI = INPUT_PIN;
    ENC_BTN_ANS = DIGITAL_PIN;
    initC_LEDs();

    initMPU6000(ACCpm8G, GYROpm1000, SMPLRT_10, LP_5_HZ);
    enMPU6000ir(NO_IR_EN, INT_LEVEL_LOW | INT_OPEN_DRAIN | INT_ALL_RD_CLR);

//----------------------------------------------------------------------- main()
    while(1){
// read acc sensor values ------------------------------------------------------
        readACC_LE(data);
// new offset ?
        if(mGET_ENC_BTN()){
            accOffset_X = *(int16_t *)&data[0];
            accOffset_Y = *(int16_t *)&data[2];
        }
        value_X  = *(int16_t *)&data[0] - accOffset_X;
        value_Y  = *(int16_t *)&data[2] - accOffset_Y;

// calc. inclination -----------------------------------------------------------
        inclination2 = ( (long)value_X * (long)value_X )
            + ( (long)value_Y * (long)value_Y );
// set pattern for inclination
        if ( inclination2 <=         400)   circleLEDs = 0b11111111;
        else if (inclination2 <=    1600)   circleLEDs = 0b11101111;
        else if (inclination2 <=    6400)   circleLEDs = 0b11100111;
        else if (inclination2 <=   25600)   circleLEDs = 0b11000111;
        else if (inclination2 <=  102400)   circleLEDs = 0b11000011;
        else if (inclination2 <=  409600)   circleLEDs = 0b10000011;
        else if (inclination2 <= 1638400)   circleLEDs = 0b10000001;
        else                                circleLEDs = 0b00000001;

// calc. rotation --------------------------------------------------------------
        rotation = atan2( (float)value_Y,(float)value_X );
        if (rotation < 0){
            rotation += 2*PI;
        }
// do rotation(s)
        if (rotation > (PI * 1 / 8) ) { mROTATE_LEDs(); }   // (2*PI * 1/2 / 8)
        if (rotation > (PI * 3 / 8) ) { mROTATE_LEDs(); }
        if (rotation > (PI * 5 / 8) ) { mROTATE_LEDs(); }
        if (rotation > (PI * 7 / 8) ) { mROTATE_LEDs(); }
        if (rotation > (PI * 9 / 8) ) { mROTATE_LEDs(); }
        if (rotation > (PI * 11 / 8) ){ mROTATE_LEDs(); }
        if (rotation > (PI * 13 / 8) ){ mROTATE_LEDs(); }
        if (rotation > (PI * 15 / 8) ){ mROTATE_LEDs(); }

// set LEDs --------------------------------------------------------------------
        if(circleLEDs & 0b00000001) LED_N   = 1; else LED_N     = 0;
        if(circleLEDs & 0b00000010) LED_NO  = 1; else LED_NO    = 0;
        if(circleLEDs & 0b00000100) LED_O   = 1; else LED_O     = 0;
        if(circleLEDs & 0b00001000) LED_SO  = 1; else LED_SO    = 0;
        if(circleLEDs & 0b00010000) LED_S   = 1; else LED_S     = 0;
        if(circleLEDs & 0b00100000) LED_SW  = 1; else LED_SW    = 0;
        if(circleLEDs & 0b01000000) LED_W   = 1; else LED_W     = 0;
        if(circleLEDs & 0b10000000) LED_NW  = 1; else LED_NW    = 0;

        __delay_ms(12); // loop ~20ms
    }
#endif //defined(ADDON_ACC_CIRCLE)
}
//##############################################################################
void testGPS_NMEA(void)
{
#if defined(ADDON_GPS_NMEA)
    char gpsBuffer[MAX_BUFFER_LENGTH];
    unsigned char newCharacter, iGpsBuffer;

//--------------------------------------------------------------------- __init()
  #if _XTAL_FREQ == 64000000
    OSCCONbits.IRCF = IRCF_16MHZ; OSCTUNEbits.PLLEN = 1;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    GLCD_Init();
    GLCD_Text2Out(0,1,"GPS SATL");
    GLCD_Text2Out(1,1,"00 found");

    ANSELCbits.ANSC7 = DIGITAL_PIN;

//#define SPBRG_VAL (((_XTAL_FREQ/BAUDRATE)+32)/64)-1   // BRG16=0, BGH=0 !
//#define SPBRG_VAL (((_XTAL_FREQ/BAUDRATE)+8)/16)-1    // BRG16=0, BGH=1 !
#define SPBRG_VAL (((_XTAL_FREQ/BAUDRATE)+2)/4)-1     // BRG16=1, BGH=1 !

    baud1USART(BAUD_IDLE_RX_PIN_STATE_HIGH &
                 BAUD_IDLE_TX_PIN_STATE_HIGH &
                 BAUD_16_BIT_RATE &
                 BAUD_WAKEUP_OFF &
                 BAUD_AUTO_OFF);

    Open1USART( USART_TX_INT_OFF & USART_RX_INT_OFF &
                USART_ASYNCH_MODE & USART_EIGHT_BIT &
                USART_CONT_RX &
                USART_BRGH_HIGH & USART_ADDEN_OFF,
                SPBRG_VAL );

    iGpsBuffer = 0;
//----------------------------------------------------------------------- main()
    while(1){
        if(RCSTA1bits.OERR){        // clear receiver and buffer
            RCSTA1bits.CREN = 0;
            RCSTA1bits.CREN = 1;
            newCharacter = RCREG;
            newCharacter = RCREG;
            iGpsBuffer = 0;
        }
        if(DataRdy1USART()){
            newCharacter = RCREG;

            if(iGpsBuffer == 0){                         // -> waiting for start
                if(newCharacter == START_OF_GPSSTR)
                    gpsBuffer[iGpsBuffer++] = newCharacter;
            }
            else if(iGpsBuffer < (MAX_BUFFER_LENGTH -1)){
                gpsBuffer[iGpsBuffer++] = newCharacter;
            }
            if (newCharacter == END_OF_GPSSTR){
                extract_gpsData(gpsBuffer,iGpsBuffer);
                iGpsBuffer = 0;

                TXREG = 0x55;

                if(gps.Status[0] != 'A'){
                    GLCD_Text2Out(0,1,"GPS SATL");
                    GLCD_Text2Out(1,1,gps.NrSatellites);
                    GLCD_Text2Out(1,3,  " found");
                    if(gps.NrSatellites[1] != '0')
                        Nop();
                }
                else{
                    GLCD_Text2Out(0,1,&gps.Latitude[2]);
                    GLCD_Text2Out(1,1,&gps.Longitude[3]);
                }
            }
        }
    }//while(1){
#endif //defined(ADDON_GPS_NMEA)
}

//###################################################################### BMP280
void testBMP280(void)
{
#if defined(ADDON_BMP280)
    unsigned char error;
//--------------------------------------------------------------------- __init()
  #if _XTAL_FREQ != 1000000
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    ENC_BTN_TRI = INPUT_PIN;
    ENC_INT_TRI = INPUT_PIN;
    ENC_DIR_TRI = INPUT_PIN;
    mENC_IR_RST();
    mENC_IR_EN();

    LCD_Init();
    LCD_ConstTextOut(0,0," BMP280 ");

    flags.all = 0;
//    state = 0;

    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){

        error = initBMP280();

        ###TODO###

        __delay_ms(500);
    }
#endif //defined(ADDON_BMP280)
}

//###################################################################### DS18B20
void testDS18B20(void)
{
#if defined(ADDON_DS18B20)
    //----------------------------------------------------------------- __init()

  #if _XTAL_FREQ == 4000000
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    DS18X20_ANS = DIGITAL_PIN;

    GLCD_Init();
    GLCD_Text2Out(0,2,"DS18B20");
    GLCD_Text2Out(1,2,"   .  C");
    GLCD_Char2Out(1,2+7*8,'\x27');  // '´'


//----------------------------------------------------------------------- main()
    while(ds18x20_res()){
        ds18x20_wr(SKIP_ROM);
        ds18x20_wr(CONV_T);
        __delay_ms(750);
        ds18x20_res();
        ds18x20_wr(SKIP_ROM);
        ds18x20_wr(RD_SCRATCH);
//        __delay_ms(1);
        ds18x20_rdTemp();
        if(ds18x20.t_int < 0)
            GLCD_Value2Out_00(1,2,ds18x20.t_int,2);
        else
            GLCD_Value2Out_00(1,2,ds18x20.t_int,3);
        GLCD_Value2Out_00(1,6,ds18x20.t_dec,1);
    }
    GLCD_Text2Out(1,1," ERROR  ");
    while(1){;}
#endif //defined(ADDON_DS18B20)
}

//################################################################## NEOPIXEL_24
void testNeopixel_24(void)
{
#if defined (ADDON_NEOPIXEL_24)
    #define NR_OF_LED 24              // 24 LEDs
    #define SIZE_LED_DATA NR_OF_LED*3
    uint8_t ledColors[SIZE_LED_DATA], i;
//----------------------------------------------------------------- __init()
  #if _XTAL_FREQ == 16000000
    char strFreq[] = "16 MHz";
    OSCCONbits.IRCF = IRCF_16MHZ; OSCTUNEbits.PLLEN = 0;
  #elif _XTAL_FREQ == 32000000
    char strFreq[] = "32 MHz";
    OSCCONbits.IRCF = IRCF_8MHZ; OSCTUNEbits.PLLEN = 1;
  #elif _XTAL_FREQ == 64000000
    char strFreq[] = "64 MHz";
    OSCCONbits.IRCF = IRCF_16MHZ; OSCTUNEbits.PLLEN = 1;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    WS2812_TRI = OUTPUT_PIN; WS2812_ANS = DIGITAL_PIN;

    GLCD_Init();
    GLCD_Value2Out(0,0,NR_OF_LED);
    GLCD_Text2Out(0,2," WS2812");
    GLCD_Text2Out(1,2,strFreq);

//----------------------------------------------------------------------- main()
    while(1){
//        INTCONbits.GIE ...
        for(i = 0; i<SIZE_LED_DATA; i++){ ledColors[i] = 0;}    // off
        WS2812_wr(&ledColors[0],SIZE_LED_DATA);
        __delay_ms(1000);
        for(i = 0; i<SIZE_LED_DATA; i+=3){ ledColors[i] = i*2;} // green
        ledColors[0] = 1;
        WS2812_wr(&ledColors[0],SIZE_LED_DATA);
        __delay_ms(500);
        for(i = 1; i<SIZE_LED_DATA; i+=3){ ledColors[i] = i*2;} // yellow (green + red))
        WS2812_wr(&ledColors[0],SIZE_LED_DATA);
        __delay_ms(500);
        for(i = 0; i<SIZE_LED_DATA; i+=3){ ledColors[i] = 0;}   // red  (yellow - green)
        WS2812_wr(&ledColors[0],SIZE_LED_DATA);
        __delay_ms(500);
        for(i = 2; i<SIZE_LED_DATA; i+=3){ ledColors[i] = i*2;} // purple (red+blue)
        WS2812_wr(&ledColors[0],SIZE_LED_DATA);
        __delay_ms(500);
        for(i = 1; i<SIZE_LED_DATA; i+=3){ ledColors[i] = 0;}   // blue (purple - red)
        WS2812_wr(&ledColors[0],SIZE_LED_DATA);
        __delay_ms(500);
        for(i = 0; i<SIZE_LED_DATA; i++){ ledColors[i] = i*2;}  // white
        WS2812_wr(&ledColors[0],SIZE_LED_DATA);
        __delay_ms(500);
        for(i = 0; i<SIZE_LED_DATA; i+=12){ ledColors[i] = 0;}  // some pink
        WS2812_wr(&ledColors[0],SIZE_LED_DATA);
        __delay_ms(500);
        for(i = 5; i<SIZE_LED_DATA; i+=12){ ledColors[i] = 0;}  // some yellow
        WS2812_wr(&ledColors[0],SIZE_LED_DATA);
        __delay_ms(500);
        for(i = 7; i<SIZE_LED_DATA; i+=12){ ledColors[i] = 0;}  // some blue
        WS2812_wr(&ledColors[0],SIZE_LED_DATA);
        __delay_ms(500);

        for(i = 0;i<255; i+=3){                                 // rotate random
            WS2812_wr(i,SIZE_LED_DATA);
            __delay_ms(200);
        }
    }
#endif //defined (ADDON_NEOPIXEL_24)
}
//####################################################################### APA102
void testAPA102(void)
{
#if defined (ADDON_APA102)
    #define NR_OF_LED 3              // 1 LED
    #define SIZE_LED_DATA NR_OF_LED*4
    uint8_t ledColors[SIZE_LED_DATA], i;
//----------------------------------------------------------------- __init()
  #if _XTAL_FREQ == 16000000
    char strFreq[] = "16 MHz";
    OSCCONbits.IRCF = IRCF_16MHZ; OSCTUNEbits.PLLEN = 0;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

//    WS2812_TRI = OUTPUT_PIN; WS2812_ANS = DIGITAL_PIN;

    GLCD_Init();
    GLCD_Value2Out(0,0,NR_OF_LED);
    GLCD_Text2Out(0,2," APA102");
    GLCD_Text2Out(1,2,strFreq);
    
    initAPA102();

//----------------------------------------------------------------------- main()
    while(1){
//        INTCONbits.GIE ...
        for(i = 0; i<SIZE_LED_DATA; i++){                           // off
            ledColors[i++] = 0xFF; ledColors[i++] = 0x00;
            ledColors[i++] = 0x00; ledColors[i] = 0x00; }
        APA102_wr(&ledColors[0],SIZE_LED_DATA); __delay_ms(1000);
        
        for(i = 3; i<SIZE_LED_DATA; i+=4){
            ledColors[i] = 0x55;}                                   // red
        APA102_wr(&ledColors[0],SIZE_LED_DATA); __delay_ms(1000);
        
        for(i = 2; i<SIZE_LED_DATA; i+=3){
            ledColors[i++] = 0x55; ledColors[i] = 0x00; }           // green
        APA102_wr(&ledColors[0],SIZE_LED_DATA); __delay_ms(1000);
        
        for(i = 1; i<SIZE_LED_DATA; i+=3){
            ledColors[i++] = 0x55; ledColors[i] = 0x00; }           // blue
        APA102_wr(&ledColors[0],SIZE_LED_DATA); __delay_ms(1000);
        
        for(i = 2; i<SIZE_LED_DATA; i+=4){
            ledColors[i] = 0x55; }                                  // +green
        APA102_wr(&ledColors[0],SIZE_LED_DATA); __delay_ms(1000);
        
        for(i = 3; i<SIZE_LED_DATA; i+=4){
            ledColors[i] = 0x55; }                                  // +red
        APA102_wr(&ledColors[0],SIZE_LED_DATA); __delay_ms(1000);
        
        for(i = 0; i<SIZE_LED_DATA; i+=4){
            ledColors[i] = 0xE9; }                                  // - brightn
        APA102_wr(&ledColors[0],SIZE_LED_DATA); __delay_ms(1000);
 
        for(i = 1; i<SIZE_LED_DATA; i+=4){
            ledColors[i] = 0x00; }                                  // - blue
        APA102_wr(&ledColors[0],SIZE_LED_DATA); __delay_ms(1000);   // >yellow
        
        for(i = 0; i<SIZE_LED_DATA; i+=4){
            ledColors[i] = 0xE3; }                                  // - brightn
        APA102_wr(&ledColors[0],SIZE_LED_DATA); __delay_ms(1000);
    }
#endif //defined (ADDON_APA102)
}


//###################################################################### RGB_LED
#if defined (ADDON_RGB_LED)
signed char rgbMenu;
unsigned char rgbFrequency, dutyR, dutyG, dutyB;
unsigned short rgbTime, startG, startB, endRGB;
__bit rgbSeq, rgbExp;

void menueCursor(void)
{
    switch (rgbMenu) {
        case 0: GLCD_Text2Out(0,3,"<"); GLCD_Text2Out(0,6," ");
                GLCD_Text2Out(1,3," "); GLCD_Text2Out(1,6," "); break;
        case 1: GLCD_Text2Out(0,3," "); GLCD_Text2Out(0,6,">");
                GLCD_Text2Out(1,3," "); GLCD_Text2Out(1,6," "); break;
        case 2: GLCD_Text2Out(0,3," "); GLCD_Text2Out(0,6," ");
                GLCD_Text2Out(1,3,"<"); GLCD_Text2Out(1,6," "); break;
        case 3: GLCD_Text2Out(0,3," "); GLCD_Text2Out(0,6," ");
                GLCD_Text2Out(1,3," "); GLCD_Text2Out(1,6,">"); break;
    }
}

void rgbTiming(void)
{
    if (flags.encUp) {
        flags.encUp = 0;
        switch(rgbMenu){
            case 0: if(rgbExp){                                     // red
                        if(!dutyR) dutyR = 1;
                        else
                            if(dutyR < 50) dutyR = (dutyR<<1);
                            else dutyR = 99;
                    }
                    else if(dutyR < 99) dutyR++;
                    GLCD_Value2Out_00(0,1,dutyR,2); break;
            case 1: if(rgbExp){                                     // green
                        if(!dutyG) dutyG = 1;
                        else
                            if(dutyG < 50) dutyG = (dutyG<<1);
                            else dutyG = 99;
                    }
                    else if(dutyG < 99) dutyG++;
                    GLCD_Value2Out_00(0,8,dutyG,2); break;
            case 2: if(rgbExp){                                     // blue
                        if(!dutyB) dutyB = 1;
                        else
                            if(dutyB < 50) dutyB = (dutyB<<1);
                            else dutyB = 99;
                    }
                    else if(dutyB < 99) dutyB++;
                    GLCD_Value2Out_00(1,1,dutyB,2); break;
            case 3: if(rgbExp){                                     // frequency
                      if(!rgbFrequency) rgbFrequency = 1;
                      else
                        if(rgbFrequency < 50) rgbFrequency = (rgbFrequency<<1);
                        else rgbFrequency = 99;
                    }
                    else if(rgbFrequency < 99) rgbFrequency++;
                    GLCD_Value2Out_00(1,8,rgbFrequency,2); break;
        }
    }
    if (flags.encDown) {
        flags.encDown = 0;
        switch(rgbMenu){
            case 0: if(rgbExp){                                     // red
                        if(dutyR <= 64) dutyR = dutyR >> 1;
                        else dutyR = 64;
                    }
                    else if(dutyR) dutyR--;
                    GLCD_Value2Out_00(0,1,dutyR,2); break;
            case 1: if(rgbExp){                                     // green
                        if(dutyG <= 64) dutyG = dutyG >> 1;
                        else dutyG = 64;
                    }
                    else if(dutyG) dutyG--;
                    GLCD_Value2Out_00(0,8,dutyG,2); break;
            case 2: if(rgbExp){                                     // blue
                        if(dutyB <= 64) dutyB = dutyB >> 1;
                        else dutyB = 64;
                    }
                    else if(dutyB) dutyB--;
                    GLCD_Value2Out_00(1,1,dutyB,2); break;
            case 3: if(rgbExp){                                     // frequency
                      if(rgbFrequency){
                        if(rgbFrequency <= 64) rgbFrequency = rgbFrequency >> 1;
                        else rgbFrequency = 64;
                      }
                    }
                    else if(rgbFrequency) rgbFrequency--;
                    GLCD_Value2Out_00(1,8,rgbFrequency,2); break;
        }
    }
    if(dutyR) {startG = 100;} else {startG = 0;}
    if(dutyG) {startB = startG + 100;} else {startB = startG;}
    if(dutyB) {endRGB = startB + 100;} else {endRGB = startB;}

    if(rgbSeq) {
        if((rgbFrequency == 0)||(endRGB <= 100)) CCPR3 = 10000;
        else if(endRGB == 200) CCPR3 = 6666;
        else if(endRGB == 300) CCPR3 = 3333;
    }
    else CCPR3 = 10000;
    if(rgbFrequency) CCPR3 = CCPR3/rgbFrequency;

    menueCursor();
}
#endif //defined (ADDON_RGB_LED)

void testRGB_LED(void)
{
#if defined (ADDON_RGB_LED)

    //----------------------------------------------------------------- __init()
  #if _XTAL_FREQ == 16000000
    OSCCONbits.IRCF = IRCF_16MHZ; OSCTUNEbits.PLLEN = 0;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    LED_R_VAL = LED_G_VAL = LED_B_VAL = rgbLED_OFF;
    LED_R_TRI = LED_G_TRI = LED_B_TRI = OUTPUT_PIN;
    BTN_L_TRI = BTN_R_TRI = INPUT_PIN; BTN_L_ANS = BTN_R_ANS = DIGITAL_PIN;
    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;
    ENC_INT_TRI = INPUT_PIN; ENC_INT_ANS = DIGITAL_PIN;
    ENC_DIR_TRI = INPUT_PIN; ENC_DIR_ANS = DIGITAL_PIN;
    mENC_IR_RST();
    mENC_IR_EN();

    // use TMR1 and CCP3 for rgbTiming (1MHz base freq)
    OpenTimer1(TIMER_INT_OFF & T1_16BIT_RW & T1_SOURCE_FOSC_4 &
            T1_PS_1_4 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF,
            TIMER_GATE_OFF);
    OpenECompare3(COM_INT_ON & ECOM_TRIG_SEVNT & ECCP_3_SEL_TMR12, 10000);

    GLCD_Init();
    GLCD_Text2Out(0,1,"RGB_LEDs");
    GLCD_Text2Out(1,1,"   GO   ");
    while(!mGET_ENC_BTN()){};
    while(mGET_ENC_BTN()){};
    __delay_ms(200);

    rgbSeq = true; rgbExp = false;
    rgbFrequency = 0; dutyR = 8; dutyG = 0; dutyB = 0;
    GLCD_Text2Out(0,0,"R      G"); GLCD_Text2Out(1,0,"B      F");
    if(rgbSeq) {GLCD_Text2Out(2,0,"nach.");} else {GLCD_Text2Out(2,0,"zus. ");}
    if(rgbExp) {GLCD_Text2Out(2,6,"grob");} else {GLCD_Text2Out(2,6,"fein");}
    GLCD_Value2Out_00(0,1,dutyR,2); GLCD_Value2Out_00(0,8,dutyG,2);
    GLCD_Value2Out_00(1,1,dutyB,2); GLCD_Value2Out_00(1,8,rgbFrequency,2);

    flags.all = 0;  rgbMenu = 3;
    rgbTiming();

//    mRGB_TMR_IR_EN();
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(mGET_ENC_BTN()){                         // encoder button
            BTN_L_TRI = BTN_R_TRI = INPUT_PIN;
            __delay_ms(10);
            if(mGET_BTN_L()){
                    rgbSeq = !rgbSeq;
            }
            else if(mGET_BTN_R()){
                    rgbExp = !rgbExp;
            }
            else if(++rgbMenu > 3){
                rgbMenu = 0;
            }
            if(rgbSeq) {GLCD_Text2Out(2,0,"nach.");} else {GLCD_Text2Out(2,0,"zus. ");}
            if(rgbExp) {GLCD_Text2Out(2,6,"grob");} else {GLCD_Text2Out(2,6,"fein");}
            //menueCursor();
            rgbTiming();
            while(mGET_ENC_BTN()){;}
        }
        if (flags.encUp || flags.encDown) {         // encoder (rotate+-)
            mRGB_TMR_IR_DIS();
            rgbTiming();
            mRGB_TMR_IR_EN();
        }//if (flags.encUp || flags.
    }
#endif //defined (ADDON_RGB_LED)
}

//######################################################################## DCF77
void testDCF77(void)
{
#if defined (ADDON_DCF77)
    unsigned short period, duty, riseCCPR1;
    unsigned char goodBits = 0;
    char detected = 'E';
    unsigned char time[] = {'0','0',':','0','0',0};
    //----------------------------------------------------------------- __init()
  #if _XTAL_FREQ == 500000
    OSCCONbits.IRCF = IRCF_500KHZ; OSCTUNEbits.PLLEN = 0;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    ANSELCbits.ANSC2 = DIGITAL_PIN;
    TRISCbits.TRISC2 = INPUT_PIN;

    GLCD_Init();
    GLCD_Text2Out(0,2,"DCF 77");
    GLCD_Text2Out(2,2,time);

        // SRF timer and capture ... -----------------------------------------------
    OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_FOSC_4
                & T1_PS_1_8 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF,
                TIMER_GATE_OFF);

    CCPTMRS0bits.C1TSEL = 0b00;     // use timer 1
    CCP1CONbits.CCP1M = 0b0101;     // Capture mode: every rising edge
    PIR1bits.CCP1IF = 0;
    riseCCPR1 = 0;

//----------------------------------------------------------------------- main()
    while(1){
        if(PIR1bits.CCP1IF){
            PIR1bits.CCP1IF = 0;            // clear flag
            CCP1CONbits.CCP1M0 ^= 1;        // toggle sensitive edge

            if(CCP1CONbits.CCP1M0 != 1){    // --- rising edge (before toggling)
                period = CCPR1 - riseCCPR1;
                riseCCPR1 = CCPR1;
                if(period > MAX_SYNC){
                    detected = 'E';
                }else if(period >= MIN_SYNC){
                    detected = 'S';                             // ---------sync
                    if(goodBits == 58){
                        ERROR_NONE();                           //   stream ok
                        GLCD_Value2Out(1,3,goodBits);
                        GLCD_Text2Out(2,2,time);
                        time[0] = time[1] = time[3] = time[4] = '0';
                    }
                    else{
                        ERROR_SYNC();
                        GLCD_Text2Out(2,2,"00:00");
                    }
                    goodBits = 0;
                }else if(period > MAX_PERIOD){                  // >> 1s error
                    detected = 'E';
                }else if(period >= MIN_PERIOD){                 // ~1s period
                    detected = 'P';
                    goodBits++;
                    GLCD_Text2Out(1,3,"   ");
                    GLCD_Value2Out(1,3,goodBits);
                }else{ detected = 'E';}                         // << 1s error
                GLCD_Char2Out(1,2+6*8,detected);
                if(detected == 'E'){ERROR_PERIOD();}
            }
            else{                           // ---------------------falling edge
                duty = CCPR1 - riseCCPR1;
                if(duty > MAX_1){           detected = 'E';}    // error
                else if(duty >= MIN_1){     detected = '1';}    // a '1'
                else if(duty > MAX_0){      detected = 'E';}    // error
                else if(duty >= MIN_0){     detected = '0';}    // a '0'
                else{                       detected = 'E';}    // error
                GLCD_Char2Out(1,2+8*8,detected);
                if(detected == 'E'){
                    goodBits--; ERROR_DUTY();}
                else if(detected == '1'){
                    if(goodBits <= 58){
                      switch(goodBits){
                        case 21: time[4]+=1; break;
                        case 22: time[4]+=2; break;
                        case 23: time[4]+=4; break;
                        case 24: time[4]+=8; break;
                        case 25: time[3]+=1; break;
                        case 26: time[3]+=2; break;
                        case 27: time[3]+=4; break;
                        case 29: time[1]+=1; break;
                        case 30: time[1]+=2; break;
                        case 31: time[1]+=4; break;
                        case 32: time[1]+=8; break;
                        case 33: time[0]+=1; break;
                        case 34: time[0]+=2; break;
                        default: break;
                      }
                    }
                    else{ goodBits = 0; ERROR_SYNC();}
                }// if(detected == '1'){
            }// falling edge
            __delay_ms(50);                 // debounce :-(
//            CCP1CONbits.CCP1M0 ^= 1;        // toggle sensitive edge
//            PIR1bits.CCP1IF = 0;            // clear flag
        }
    }
#endif //defined (ADDON_DCF77)
}

//######################################################################## SHT21
void testSHT21(void)
{
#if defined(ADDON_SHT21)
    int8_t temp_int, temp_dec;
    uint16_t valT, valRH;
    float temperature;

//--------------------------------------------------------------------- __init()
  #if _XTAL_FREQ == 16000000
    OSCCONbits.IRCF = IRCF_16MHZ; OSCTUNEbits.PLLEN = 0;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    GLCD_Init();
    GLCD_Text2Out(0,0," SHT_21 ");
    GLCD_Text2Out(1,0," ??.?? C");

//    flags.all = 0;

    ANSELCbits.ANSC3 = ANSELCbits.ANSC4 = 0;    // I2C pins
    TRISCbits.TRISC3 = TRISCbits.TRISC4 = 1;

    SSP1ADD = 39;                               // I2C baudrate 100kHz
    initSHT21(MODE_RH12_T14_BIT | DIS_OC_HEATER | DIS_OTP_RELOAD);
//----------------------------------------------------------------------- main()
    wrSHT21(CMD_TRIG_T);
    __delay_ms(TMAX_14BIT);         // replace with wakeUpTimer and sleep()
//        setWakeUpTimer(TMAX_14BIT);
//        sleep();
    while(1){
        rdSHT21((uint8_t *)(&valT));
        wrSHT21(CMD_TRIG_T);

//        setWakeUpTimer(TMAX_14BIT);
        temperature = -46.85 + (175.72 * valT)/65536;
        if(temperature > 0){
            temp_int = (int)temperature;
            temp_dec = (int)((temperature - temp_int)*100);
            SSP1CON1bits.SSPEN = 0; TRISCbits.TRISC3 = 0;   // SCL conflict
            GLCD_Value2Out_00(1,1,temp_int,2);
            GLCD_Value2Out_00(1,4,temp_dec,2);
            TRISCbits.TRISC3 = 1; SSP1CON1bits.SSPEN = 1;
        }
        __delay_ms(TMAX_14BIT);     // replace with wakeUpTimer and sleep()
//        sleep();
    }
#endif //defined(ADDON_SHT21)
}

void testIRC_NEC(void)
{
#if defined(ADDON_IRC_NEC)
    int8_t temp_int, temp_dec;
    uint16_t valT, valRH;
    float temperature;

//--------------------------------------------------------------------- __init()
  #if _XTAL_FREQ == 16000000
    OSCCONbits.IRCF = IRCF_16MHZ; OSCTUNEbits.PLLEN = 0;
  #else
    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
  #endif

    
    ###TODO###
    
    LCD_Init();
    LCD_ConstTextOut(0,0,"IR CTRL ");
    LCD_ConstTextOut(1,0,"????????");
//
    ircNECdecoder_init();
    SAEFWQircNECdecoder_L();
//    ircNECdecoder_H();

    while(1){;}
#endif //defined(ADDON_IRC_NEC)
}


/*
//######################################################################
void testRadar(void)
{
//--------------------------------------------------------------------- __init()
//----------------------------------------------------------------------- main()
    while(1){
        ;
    }
}
//#######################################################################

//unsigned char arraywr[] = {1,2,3,4,5,6,7,8,0};
//unsigned char arrayrd[20];

void testAddOn(void)    //TestEEi2c(void)
{
////--------------------------------------------------------------------- __init()
//#if _XTAL_FREQ != 16000000
//    #error "Please define in _XTAL_FREQ in AddOnBoards.h"
//#endif
//    OSCCONbits.IRCF = IRCF_16MHZ;
//    OpenI2C(MASTER, SLEW_ON);   // Initialize I2C module
//    SSPADD = 9;                 //400kHz Baud clock(9) @16MHz
//                                //100kHz Baud clock(39) @16MHz
//
//
////----------------------------------------------------------------------- main()
//    while(1){
//        EEByteWrite(0xAE, 0x30, 0xA5);  // AE is the bus address
//
//        EEAckPolling(0xAE);
//
//        EECurrentAddRead(0xAE);
//
//        EEPageWrite(0xA0, 0x70, arraywr);
//
//        EEAckPolling(0xA0);
//
//        EESequentialRead(0xA0, 0x70, arrayrd, 20);
//
//        EERandomRead(0xA0, 0x30);
//    }
}
*/
