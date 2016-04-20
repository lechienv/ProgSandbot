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
             MyConsole_SendMsg("GREEN");
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

void getActions(CtrlStruct *cvs)
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
        default: break;
    }
}

