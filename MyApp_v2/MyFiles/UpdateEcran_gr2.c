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

void getTests(CtrlStruct *cvs)
{
    unsigned int M = MyCyclone_Read(CYCLONE_IO_M_Data);
    switch(M){
        case(6) :
        {
            RatGoTop(cvs, cvs->MotorRatL);
            break;
        }
        case(7) :
        {
            RatGoBottom(cvs, cvs->MotorRatL);
            break;
        }
        case(8) :
        {
            PinceCalibration(cvs);
            break;
        }
        case(9) :
        {
            ClosePince(cvs);
            break;
        }
        case(10) :
        {
            bool Direction = true;
            TestRoues(cvs, Direction);
            break;
        }
        case(11) :
        {
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
            Action4(cvs);
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
