#include "UpdateEcran_gr2.h"
#include "RoboticFiles/CtrlStruct_gr2.h"



/*********************
*** FUNCTIONS ********
**********************/
void getRobotID(CtrlStruct *cvs)
{
    unsigned int M = MyCyclone_Read(CYCLONE_IO_M_Data);
    switch(M){
        case(4) :
        {
                cvs->robotID = GREEN;
                cvs->colorIsSet = true;
                break;
        }
        case(5) :
        {
                cvs->robotID = PINK;
                cvs->colorIsSet = true;
                break;
        }
        default: break;
    }
}

void ChooseStratDuneOrNot(CtrlStruct *cvs)
{
    int color = cvs->robotID;
    unsigned int M = MyCyclone_Read(CYCLONE_IO_M_Data);
    switch(M){
        case(2) :
        {
            cvs->Tower->StrategyWithRushDunes   = true;
            break;
        }
        case(3) :
        {
            cvs->Tower->StrategyWithFish        = true;
            break;
        }
        default: break;
    }
}

void getStrategy(CtrlStruct *cvs)
{
    unsigned int M = MyCyclone_Read(CYCLONE_IO_M_Data);
    switch(M){
        case(14) :
        {   cvs->Param->ChooseToMatch = true;
            cvs->Param->maxSpeed = M_PI;
            cvs->Param->maxSpeedRot = 30*cvs->Param->maxSpeed;
            PointHomologation(cvs);
            break;
        }
        case(15) :
        {   cvs->Param->ChooseToMatch = true;
            cvs->Param->maxSpeed = 2*M_PI * 1.5;
            cvs->Param->maxSpeedRot = 30*cvs->Param->maxSpeed;
            MyStrategy(cvs, 1);
            break;
        }
        default: break;
    }
    
}
void ChooseBetweenMatchOrTest(CtrlStruct *cvs)
{
    /*unsigned int M = MyCyclone_Read(CYCLONE_IO_M_Data);
    switch(M){
        case(14) :
        {
            cvs->Param->ChooseToMatch = true;
            break;
        }
        case(15) :
        {
            cvs->Param->ChooseToMatch = true;
            break;
        }
        default: break;*/
    unsigned int M = MyCyclone_Read(CYCLONE_IO_M_Data);
    switch(M){
        case(2) :
        {
            cvs->Param->ChooseToMatch = true;
            break;
        }
        default: break;
    }
    
}
void getTests(CtrlStruct *cvs)
{
    int color = cvs->robotID;
    unsigned int M = MyCyclone_Read(CYCLONE_IO_M_Data);
    switch(M){
        case(6) :
        {
        cvs->MotorTower->dutyCycle = 0;
        cvs->MotorPince->dutyCycle = 0;    
        cvs->MotorL->dutyCycle = 0;
        cvs->MotorR->dutyCycle = 0;  
            (color == GREEN) ? RatGoTop(cvs, cvs->MotorRatL):RatGoTop(cvs, cvs->MotorRatR);
            break;
        }
        case(7) :
        {
        cvs->MotorTower->dutyCycle = 0;
        cvs->MotorPince->dutyCycle = 0;    
        cvs->MotorL->dutyCycle = 0;
        cvs->MotorR->dutyCycle = 0;
            (color == GREEN) ?  RatGoBottom(cvs, cvs->MotorRatL):RatGoBottom(cvs, cvs->MotorRatR);
            break;
        }
        case(8) :
        {
        cvs->MotorTower->dutyCycle = 0;    
        cvs->MotorL->dutyCycle = 0;
        cvs->MotorR->dutyCycle = 0; 
        cvs->MotorRatL->dutyCycle = 0;
        cvs->MotorRatR->dutyCycle = 0;  
        PinceCalibration(cvs);
            break;
        }
        case(9) :
        {
        cvs->MotorTower->dutyCycle = 0;  
        cvs->MotorL->dutyCycle = 0;
        cvs->MotorR->dutyCycle = 0;  
        cvs->MotorRatL->dutyCycle = 0;
        cvs->MotorRatR->dutyCycle = 0;  
            ClosePince(cvs, 40);
            break;
        }
        case(10) :
        {
        cvs->MotorTower->dutyCycle = 0;
        cvs->MotorPince->dutyCycle = 0; 
        cvs->MotorRatL->dutyCycle = 0;
        cvs->MotorRatR->dutyCycle = 0;  
            bool Direction = true;
            TestRoues(cvs, Direction);
            break;
        }
        case(11) :
        {
        cvs->MotorTower->dutyCycle = 0;
        cvs->MotorPince->dutyCycle = 0; 
        cvs->MotorRatL->dutyCycle = 0;
        cvs->MotorRatR->dutyCycle = 0;  
            bool Direction = false;
            TestRoues(cvs, Direction);
            break;
        }
        default: break;
    }
}

void getActions(CtrlStruct *cvs)
{
    unsigned int M = MyCyclone_Read(CYCLONE_IO_M_Data);
    switch(M){
        case(16) :
        {
            Action2(cvs);
            break;
        }
        case(17) :
        {
            Action1(cvs);
            break;
        }
        case(18) :
        {
            Action3(cvs);
            break;
        }
        case(19) :
        {
            Action4(cvs);
            break;
        }
        case(20) :
        {
            ActionParasol(cvs);
            break;
        }
   default: break;
    }
}

void TestRoues(CtrlStruct *cvs, bool Direction)
{
     cvs->MotorL->dutyCycle = (Direction) ? 25 : -25 ;
     cvs->MotorR->dutyCycle = (Direction) ?  25 : -25 ;
    if(!cvs->TimerCalibration->isSet)
    {
        SetTimer(cvs, cvs->TimerCalibration, 2);
    }
     if(IsTimerTimout(cvs,cvs->TimerCalibration) )
    {
        SpeedRefToDC(cvs, cvs->MotorL, 0);
        SpeedRefToDC(cvs, cvs->MotorR, 0);
        ResetTimer(cvs->TimerCalibration);

     }
}
