#include "UpdateEcran.h"
#include "MyApp.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

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


#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT