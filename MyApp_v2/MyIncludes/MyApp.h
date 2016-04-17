/*******************************************************************************
* Header file for MyApp                                                        *
*******************************************************************************/
//#define MINIBOT

#ifndef MyAPP_H_
#define MyAPP_H_

// Adds support for PIC32 Peripheral library functions and macros
#include <plib.h>
#include <math.h>
// My Includes
#include "MySPI.h"
#include "MyLED.h"
#include "MyCyclone.h"
#include "MyConsole.h"
#include "MyCAN.h"
#include "MyRTCC.h"
#include "MyIO.h"
#include "MyFlash.h"
#include "MyMIWI.h"
#include "MyWIFI.h"
#include "MyTemperature.h"
#include "MyMDDFS.h"
#include "MyCamera.h"
#include "MyMiniProjet.h"

/*******************************************************************************
* Constants                                                                    *
*******************************************************************************/

// Application defines
#define SYS_FREQ		(80000000)
#define SYS_FREQ_DOUBLE (80000000.0)
// Board defines
#define  CS_FPGA    BIT_4   // Port E
#define  CS_WIFI    BIT_3
#define  CS_MIWI    BIT_2
#define  CS_IO      BIT_1
#define  CS_FLASH   BIT_0

#define  RST_IO     BIT_5   // Port E
#define  RST_MIWI   BIT_6
#define  RST_WIFI   BIT_7

#define  RST_FPGA   BIT_7   // Port D
#define  GPIO_211   BIT_6
#define  GPIO_212   BIT_5
#define  GPIO_2IN0  BIT_4

#define  USD_CD     BIT_15  // Port B
#define  CS_USD     BIT_13
#define  CS_TEMP    BIT_12

#define  MIWI_WAKE  BIT_5   // Port B
#define  WIFI_WP    BIT_4
#define  WIFI_HIB   BIT_3
#define  SDN_5V     BIT_2
#define  FLASH_WP   BIT_6
#define  FLASH_HOLD BIT_7

/* The motors */
#define  SECURITYDC 25

/*******************************************************************************
* My Priorities for Interrupts                                                 *
*******************************************************************************/

#define My_INT_CAN_1_PRIORITY           INT_PRIORITY_LEVEL_2
#define My_INT_CAN_1_SUB_PRIORITY       INT_SUB_PRIORITY_LEVEL_0
#define My_INT_CAN_1_IPL                ipl2

#define My_INT_RTCC_PRIORITY            INT_PRIORITY_LEVEL_2
#define My_INT_RTCC_SUB_PRIORITY        INT_SUB_PRIORITY_LEVEL_0
#define My_INT_RTCC_IPL                 ipl2

#define My_INT_UART_PRIORITY            INT_PRIORITY_LEVEL_3
#define My_INT_UART_SUB_PRIORITY        INT_SUB_PRIORITY_LEVEL_0
#define My_INT_UART_IPL                 ipl3

#define My_INT_EXTERNAL_0_PRIORITY      INT_PRIORITY_LEVEL_5
#define My_INT_EXTERNAL_0_SUB_PRIORITY  INT_SUB_PRIORITY_LEVEL_0
#define My_INT_EXTERNAL_0_IPL           ipl5

#define My_INT_EXTERNAL_1_PRIORITY      INT_PRIORITY_LEVEL_6
#define My_INT_EXTERNAL_1_SUB_PRIORITY  INT_SUB_PRIORITY_LEVEL_0
#define My_INT_EXTERNAL_1_IPL           ipl6

#define My_INT_EXTERNAL_2_PRIORITY      INT_PRIORITY_LEVEL_1
#define My_INT_EXTERNAL_2_SUB_PRIORITY  INT_SUB_PRIORITY_LEVEL_0
#define My_INT_EXTERNAL_2_IPL           ipl1

#define My_INT_EXTERNAL_3_PRIORITY      INT_PRIORITY_LEVEL_4        // Not Modify ! (MIWI)
#define My_INT_EXTERNAL_3_SUB_PRIORITY  INT_SUB_PRIORITY_LEVEL_0
#define My_INT_EXTERNAL_3_IPL           ipl4

#define My_INT_EXTERNAL_4_PRIORITY      INT_PRIORITY_LEVEL_7        // Not Modify ! (WIFI)
#define My_INT_EXTERNAL_4_SUB_PRIORITY  INT_SUB_PRIORITY_LEVEL_0
#define My_INT_EXTERNAL_4_IPL           ipl7

/*******************************************************************************
* Global Variables                                                             *
*******************************************************************************/

#ifdef   MyAPP
#define  MyAPP_EXT
#define  MyAPP_INIT =FALSE
#else
#define  MyAPP_EXT  extern
#define  MyAPP_INIT
#endif

#define distanceRef_default         0.2

MyAPP_EXT  int      MyTime;
MyAPP_EXT  int      pbClk;
MyAPP_EXT  int      MyPing_Flag MyAPP_INIT;
MyAPP_EXT  int      MyMail_Flag MyAPP_INIT;

MyAPP_EXT  int      MyCyclone_LED;
MyAPP_EXT  int      MyCyclone_Switch;
MyAPP_EXT  int      MyVideoCounter MyAPP_INIT;
MyAPP_EXT  int      TourelleStatus MyAPP_INIT;
MyAPP_EXT  int      RightMotorStatus MyAPP_INIT;
MyAPP_EXT  int      LeftMotorStatus MyAPP_INIT;

MyAPP_EXT   double     var1Status MyAPP_INIT;
MyAPP_EXT   double     var2Status MyAPP_INIT;
MyAPP_EXT   double     var3Status MyAPP_INIT;
MyAPP_EXT   double     var4Status MyAPP_INIT;
MyAPP_EXT   double     var5Status MyAPP_INIT;
MyAPP_EXT   double     var6Status MyAPP_INIT;
MyAPP_EXT   double     var7Status MyAPP_INIT;
MyAPP_EXT   double     var8Status MyAPP_INIT;
MyAPP_EXT   double     var9Status MyAPP_INIT;
MyAPP_EXT   double     var10Status MyAPP_INIT;
MyAPP_EXT   double     var11Status MyAPP_INIT;
MyAPP_EXT   double     var12Status MyAPP_INIT;
MyAPP_EXT   double     var13Status MyAPP_INIT;
MyAPP_EXT   double     var14Status MyAPP_INIT;
MyAPP_EXT   double     var15Status MyAPP_INIT;
MyAPP_EXT   double     var16Status MyAPP_INIT;
MyAPP_EXT   double     var17Status MyAPP_INIT;
MyAPP_EXT   double     var18Status MyAPP_INIT;
MyAPP_EXT   double     var19Status MyAPP_INIT;
MyAPP_EXT   double     var20Status MyAPP_INIT;
MyAPP_EXT   double     var21Status MyAPP_INIT;
MyAPP_EXT   double     var22Status MyAPP_INIT;
MyAPP_EXT   double     var23Status MyAPP_INIT;
MyAPP_EXT   double     var24Status MyAPP_INIT;
MyAPP_EXT   double     var25Status MyAPP_INIT;
MyAPP_EXT   double     var26Status MyAPP_INIT;
MyAPP_EXT   double     var27Status MyAPP_INIT;
MyAPP_EXT   double     var28Status MyAPP_INIT;
MyAPP_EXT   double     var29Status MyAPP_INIT;
MyAPP_EXT   double     var30Status MyAPP_INIT;
MyAPP_EXT   double     var31Status MyAPP_INIT;
MyAPP_EXT   double     var32Status MyAPP_INIT;
MyAPP_EXT   double     var33Status MyAPP_INIT;
MyAPP_EXT   double     var34Status MyAPP_INIT;
MyAPP_EXT   double     var35Status MyAPP_INIT;
MyAPP_EXT   double     var36Status MyAPP_INIT;
MyAPP_EXT   double     var37Status MyAPP_INIT;
MyAPP_EXT   double     var38Status MyAPP_INIT;
MyAPP_EXT   double     var39Status MyAPP_INIT;
MyAPP_EXT   double     var40Status MyAPP_INIT;

/*****************************
 * Website variable
 ****************************/
int        LeftMotorDC;
int        RightMotorDC;
int        TourelleDC;
int        RateauLDC;
int        RateauRDC;
int        PinceDC;
bool CommandMotorByHand;
int        LeftMotorDCWeb;
int        RightMotorDCWeb;
int        TourelleDCWeb;
double     var1;
double     var2;
double     var3;
double     var4;
double     var5;
double     var6;
double     var7;
double     var8;
double     var9;
double     var10;
double     var11;
double     var12;
double     var13;
double     var14;
double     var15;
double     var16;
double     var17;
double     var18;
double     var19;
double     var20;
double     var21;
double     var22;
double     var23;
double     var24;
double     var25;
double     var26;
double     var27;
double     var28;
double     var29;
double     var30;
double     var31;
double     var32;
double     var33;
double     var34;
double     var35;
double     var36;
double     var37;
double     var38;
double     var39;
double     var40;
/*
#ifdef MINIBOT
    #define clicNumberDefaultR 20100
    #define clicNumberDefaultL 11500

    #define KpDefault          0.3
    #define KiDefault          5
#else
    #define clicNumberDefaultR 10000
    #define clicNumberDefaultL 10000
    #define KpDefault          0.3
    #define KiDefault          5
#endif /// MINIBOT
*/

/*******************************************************************************
* System Macros                                                                *
*******************************************************************************/

#define	GetSystemClock() 		(80000000ul)
#define	GetPeripheralClock()		(GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(GetSystemClock())

/*******************************************************************************
* Functions Prototypes                                                         *
*******************************************************************************/


#endif /* MyAPP_H_ */
