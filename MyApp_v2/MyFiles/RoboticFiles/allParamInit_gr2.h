#ifndef _allParamInit_GR2_H_
#define _allParamInit_GR2_H_

#include "CtrlStruct_gr2.h"

#ifndef REALBOT
#include "namespace_ctrl.h"
NAMESPACE_INIT(ctrlGr2);
#endif // !REALBOT

/****************** 
 ***** MACROS *****
 ******************/

#ifdef REALBOT
#ifdef MINIBOT
    #define clicNumberR_DEFAULT 20100*0.9*30/26 //1.1
    #define clicNumberL_DEFAULT 11500*0.9*30/26 //1.1
    #define clicNumberTower_DEFAULT 1800
    #define wheelRRadius_DEFAULT 0.03
    #define wheelLRadius_DEFAULT 0.03
	#define width_DEFAULT 0.225
	#define MaxAcceleration_DEFAULT	10
#else
    #define clicNumberR_DEFAULT     10000
    #define clicNumberL_DEFAULT     10000
    #define clicNumberTower_DEFAULT 930
    #define wheelRRadius_DEFAULT    0.022749
    #define wheelLRadius_DEFAULT    0.022725
	#define width_DEFAULT           0.2625
	#define MaxAcceleration_DEFAULT	2
#endif // MINIBOT
    #define clicNumberRatL_DEFAULT  10000
    #define clicNumberRatR_DEFAULT  10000
    #define clicNumberPince_DEFAULT 10000
#else
    #define wheelRRadius_DEFAULT 0.03
    #define wheelLRadius_DEFAULT 0.03
	#define width_DEFAULT 0.225
	#define MaxAcceleration_DEFAULT	100
#endif //REALBOT

#define Kp_DEFAULT 0.3
#define Ki_DEFAULT 5

/************************************
 * MOTORS ***************************
 ***********************************/
//LeftMotor
#define KpL_INIT				Kp_DEFAULT //0.3
#define KiL_INIT				Ki_DEFAULT //5
#define KpPosL_INIT				1
#define KphiL_INIT				0.3177
#define RL_INIT					10
#define speedL_INIT				0
#define positionL_INIT			0
#define reverseRotationL_INIT	1
#define totalErrorL_INIT		0
#define dutyCycleL_INIT			0
#ifdef REALBOT
    #define clicNumberL_INIT        clicNumberL_DEFAULT  
    #define PWMRegL_INIT            0x25
    #define timerRegL_INIT          0x21
    #define perRegL_INIT            0x23
    #define brakesMaskL_INIT        0x10
    #define SIDL_INIT               0x708 //708
#endif // REALBOT

// RightMotor
#define KpR_INIT				Kp_DEFAULT //0.3
#define KiR_INIT				Ki_DEFAULT //5
#define KpPosR_INIT				1
#define KphiR_INIT				0.3177
#define RR_INIT					10
#define speedR_INIT				0
#define positionR_INIT			0
#define reverseRotationR_INIT	1
#define totalErrorR_INIT		0
#define dutyCycleR_INIT			0
#ifdef REALBOT
    #define clicNumberR_INIT        clicNumberR_DEFAULT
    #define PWMRegR_INIT            0x26
    #define timerRegR_INIT          0x22
    #define perRegR_INIT            0x24
    #define brakesMaskR_INIT        0x20
    #define SIDR_INIT               0x708 //708
#endif // REALBOT

// TowerMotor
#define KpTower_INIT				0.3
#define KiTower_INIT				5
#define KpPosTower_INIT				1
#define KphiTower_INIT				0.3021
#define RTower_INIT					10
#define speedTower_INIT				1
#define positionTower_INIT			0
#define reverseRotationTower_INIT	1
#define totalErrorTower_INIT		0
#define dutyCycleTower_INIT			0
#ifdef REALBOT
#define clicNumberTower_INIT        clicNumberTower_DEFAULT
#define PWMRegTower_INIT            0x25
#define timerRegTower_INIT          0x21
#define perRegTower_INIT            0x23
#define brakesMaskTower_INIT        0x30
#define SIDTower_INIT               0x508
#endif // REALBOT

#ifdef REALBOT
    //MotorRatL
    #define KpRatL_INIT                 0.3//0.5
    #define KiRatL_INIT                 5//7
	#define KpPosRatL_INIT				1
    #define KphiRatL_INIT				0.3177
	#define RRatL_INIT					10
    #define speedRatL_INIT				0
	#define positionRatL_INIT			0
    #define reverseRotationRatL_INIT	1
    #define totalErrorRatL_INIT         0
    #define dutyCycleRatL_INIT			0
    #define clicNumberRatL_INIT         clicNumberRatL_DEFAULT  
    #define PWMRegRatL_INIT             0x25
    #define timerRegRatL_INIT           0x21
    #define perRegRatL_INIT             0x23
    #define brakesMaskRatL_INIT         0x10
    #define SIDRatL_INIT                0x408//608

    //MotorRatR
    #define KpRatR_INIT                 0.3//0.5
    #define KiRatR_INIT                 5//7
	#define KpPosRatR_INIT				1
    #define KphiRatR_INIT				0.3177
	#define RRatR_INIT					10
    #define speedRatR_INIT				0
	#define positionRatR_INIT			0
    #define reverseRotationRatR_INIT	1
    #define totalErrorRatR_INIT         0
    #define dutyCycleRatR_INIT			0
    #define clicNumberRatR_INIT         clicNumberRatR_DEFAULT  
    #define PWMRegRatR_INIT             0x26
    #define timerRegRatR_INIT           0x22
    #define perRegRatR_INIT             0x24
    #define brakesMaskRatR_INIT         0x20
    #define SIDRatR_INIT                0x408//608

    //MotorPince
    #define KpPince_INIT				0.3//0.5
    #define KiPince_INIT				1//7
	#define KpPosPince_INIT				1
    #define KphiPince_INIT				0.3177
	#define RPince_INIT					10
    #define speedPince_INIT				0
	#define positionPince_INIT			0
    #define reverseRotationPince_INIT	1
    #define totalErrorPince_INIT		0
    #define dutyCyclePince_INIT			0
    #define clicNumberPince_INIT        clicNumberPince_DEFAULT
    #define PWMRegPince_INIT            0x25
    #define timerRegPince_INIT          0x21
    #define perRegPince_INIT            0x23
    #define brakesMaskPince_INIT        0x10
    #define SIDPince_INIT               0x608//408
#endif // REALBOT

/************************************
 * PARAMS ***************************
 ***********************************/
#define radiusBot_INIT			0.17//0.2 //0.015 ?
#define width_INIT				width_DEFAULT //0.2625
#define lengthTower_INIT		0.0833
#define wheelLRadius_INIT		wheelLRadius_DEFAULT
#define wheelRRadius_INIT		wheelRRadius_DEFAULT
#define wheelRadius_INIT		0.03
#define KpRot_INIT				0.06//0.1
#define KiRot_INIT				0.03//0.2
#define totalErrorRot_INIT		0
#define speedDifThreshold_INIT	1
#define KiAngleThreshold_INIT	10
#define distanceRef_default_tower 3
# define PasFiletVisRat_INIT       3.0
# define PasFiletVisPince_INIT       2.0

#ifdef MINIBOT
#define rayonBeacon_INIT		0.04
#else
#define rayonBeacon_INIT		0.03
#endif
#define maxAcceleration_INIT    MaxAcceleration_DEFAULT
/************************************
 * POTO *****************************
 ***********************************/
#define katt_INIT				10//1.0
#define krep_INIT				0.0015//0.001
#define kFV_INIT				30
#define FXRob_INIT				0
#define FyRob_INIT				0
#define kw_INIT					200//100
#define minDistance_INIT		0.3//0.5
#define thresholdAligned_INIT   0.75//10

/************************************
 * ODOMETRY  ************************
 ***********************************/
#ifdef REALBOT
    #define clicNumberOdo_INIT     2048;
#endif // REALBOT

/************************************
 * OBSTACLES ************************
 ***********************************/
#define NumberOfQuarterOfCircle_INIT 4
#define NumberOfRectangles_INIT 10
#define NumberOfCircles_INIT	2 //2Bots + 1 
#define radiusEnnemyBot			0.43
 /************************************
 * GOALS ************************
 ***********************************/
#define maxtimewait_INIT 10.0
#define firstgoal_INIT 0
#define goalprecision_INIT 0.1

/************************************
 * TOWER ***************************
 ***********************************/
#define tower_pos_INIT 0
#define last_rising_INIT 0
#define last_falling_INIT 0
#define rising_index_INIT 0
#define nb_rising_INIT 0
#define nb_falling_INIT 0
#define nb_opponents_INIT 0
#define last_rising_fixed_INIT 0
#define last_falling_fixed_INIT 0
#define rising_index_fixed_INIT 0
#define nb_rising_fixed_INIT 0
#define nb_falling_fixed_INIT 0
#define falling_index_INIT 0
#define rising_index_INIT 0
#define angle_INIT 0
#define distance_INIT 3

/************************************
 * Dyna ***************************
 ***********************************/
// Id rateau gauche = 5, Id rateau droit = 6, Id parasol = 3
/******************
**** PROTOTYPES ***
******************/
void InitMotor(CtrlStruct *cvs);
void InitParam(CtrlStruct *cvs);
void InitSensors(CtrlStruct *cvs);
void InitTower(CtrlStruct *cvs);
void InitPotential(CtrlStruct *cvs);
void InitOdometry(CtrlStruct *cvs);
void InitObstacles(CtrlStruct *cvs);
void InitDyna(CtrlStruct *cvs);
void InitTowerFilters(CtrlStruct *cvs);
void InitTimer(CtrlStruct *cvs);

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // !REALBOT

#endif