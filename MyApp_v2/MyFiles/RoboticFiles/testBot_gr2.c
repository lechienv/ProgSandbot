#include "testBot_gr2.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT


/*********************
*** FUNCTIONS ********
**********************/
#ifdef REALBOT

void TorqueControl(Motor *Motor, double torqueRef) {
	double currentValue = torqueRef / Motor->Kphi;
	double voltage = Motor->R*currentValue + Motor->Kphi * Motor->speed;
	Motor->dutyCycle += voltage*VOLTtoDC;
}

void Action1Test(CtrlStruct *cvs){
    SpeedRefToDC(cvs, cvs->MotorL, var3);
    SpeedRefToDC(cvs, cvs->MotorR, var4);
}

void Action2Test(CtrlStruct *cvs){
  PinceCalibration(cvs);
}
void Action3Test(CtrlStruct *cvs){
    bool result = ReachPointPotential(cvs,var3, var4, var5);
}

void Action4Test(CtrlStruct *cvs){
    cvs->MotorL->dutyCycle = var1;//RightMotorDC;
    cvs->MotorR->dutyCycle = var2;// RightMotorDC;
    cvs->MotorPince->dutyCycle = var3;
    cvs->MotorRatL->dutyCycle = var4; //RightMotorDC;//RightMotorDC;
    cvs->MotorRatR->dutyCycle = var5; //RightMotorDC;//RightMotorDC;
}

void Action5Test(CtrlStruct *cvs){
    PointHomologation(cvs);
}
void Action6Test(CtrlStruct *cvs){
    ClosePince(cvs);
}
void Action7Test(CtrlStruct *cvs){
    cvs->MotorPince->dutyCycle = var39;
}
void Action8Test(CtrlStruct *cvs){
    cvs->MotorRatL->dutyCycle = var39;
}
void Action9Test(CtrlStruct *cvs){
    if(cvs->Odo->x < var3){
        cvs->MotorL->dutyCycle = var4;
        cvs->MotorR->dutyCycle = var5;
    }
}
void Action10Test(CtrlStruct *cvs){
    cvs->MotorTower->dutyCycle = var39;
}

void Action11Test(CtrlStruct *cvs){
    char s[64];
    switch(cvs->DynaLeft->stateDyna){
        case(grap) :
            if(!cvs->DynaLeft->enable){
                cvs->DynaLeft->timer == cvs->time;
                cvs->DynaLeft->enable = true;
            }
            else if(cvs->time - cvs->DynaLeft->timer < 4){
                TurnCCW(20);
                sprintf(s,"GoTurn\n");
                MyConsole_SendMsg(s);
            }
            else{
                sprintf(s,"StopTurn\n");
                MyConsole_SendMsg(s);
                StopTurn(1);
                cvs->DynaLeft->stateDyna = release;
                cvs->DynaLeft->enable = false;
            }
            break;
        case(release) :
            if(!cvs->DynaLeft->enable){
                cvs->DynaLeft->timer == cvs->time;
                cvs->DynaLeft->enable = true;
            }
            else if(cvs->time - cvs->DynaLeft->timer < 4){
                TurnCW(20);
                sprintf(s,"GoTurn\n");
                MyConsole_SendMsg(s);
            }
            else{
                sprintf(s,"StopTurn\n");
                MyConsole_SendMsg(s);
                StopTurn(0);
                cvs->DynaLeft->stateDyna = grap;
            }
    }
}

void Action12Test(CtrlStruct *cvs){
    IsAlignedWithTheta(cvs,var3,var4);
}

void Action13Test(CtrlStruct *cvs){
    if(cvs->Odo->x < var3){
        SpeedRefToDC(cvs,cvs->MotorL,var4);
        SpeedRefToDC(cvs,cvs->MotorR,var5);
    }
}

void Action14Test(CtrlStruct *cvs){
    cvs->Odo->x = var3;
    cvs->Odo->y = var4;
    cvs->Odo->theta = var5;
}

void StrategyTest(CtrlStruct *cvs){
    if(var1 == 1){
        Action1Test(cvs);
    }
    else if(var1 == 2){
        Action2Test(cvs);
    }
    else if(var1 == 3){
        Action3Test(cvs);
    }
    else if(var1 == 4){
        Action4Test(cvs);
    }
    else if(var1 == 5){
        Action5Test(cvs);
    }
    else if(var1 == 6){
        Action6Test(cvs);
    }
    else if(var1 == 7){
        Action7Test(cvs);
    }
    else if(var1 == 8){
        Action8Test(cvs);
    }
    else if(var1 == 9){
        Action9Test(cvs);
    }
    else if(var1 == 10){
        Action10Test(cvs);
    }
    else if(var1 == 11){
        Action11Test(cvs);
    }
    else if(var1 == 12){
        Action12Test(cvs);
    }
    else if(var1 == 13){
        Action13Test(cvs);
    }
    else if(var1 == 14){
        Action14Test(cvs);
    }
}

//ok
bool PinceCalibration(CtrlStruct *cvs){
    if(cvs->Sensors->uSwitchPinceIn){
        SpeedRefToDC(cvs, cvs->MotorPince, 1);
        return false;
    }
    else{
        cvs->MotorPince->position = 0;
        return true;
    }
}

bool ClosePince(CtrlStruct *cvs){
    if(cvs->MotorPince->position < -400 && cvs->MotorPince->position > 0){
        cvs->MotorPince->dutyCycle = 0;
        return true;
    }
    else
        cvs->MotorPince->dutyCycle = -40;

    if((cvs->MotorPince->speed == 0) && (!cvs->Sensors->uSwitchPinceIn) && (cvs->MotorPince->position < -100)){
        return true;
    }
    return false;
}

bool DeposeBlock(CtrlStruct *cvs){
    bool isOpen;
    double x = cvs->Odo->x;
    double y = cvs->Odo->y;
    if((fabs(cvs->Odo->bufferPosition) > fabs(x*x+y*y)-0.05) && cvs->Odo->flagBufferPosition == 0){
        cvs->MotorL->dutyCycle = 10;
        cvs->MotorR->dutyCycle = 10;
    }
    if((fabs(cvs->Odo->bufferPosition) >=fabs(x*x+y*y)-0.05)){
        cvs->Odo->flagBufferPosition == 1;
        isOpen = PinceCalibration(cvs);
    }
    if(isOpen){
        cvs->MotorL->dutyCycle = -15;
        cvs->MotorR->dutyCycle = -15;
    }
    if((fabs(x*x+y*y) <= fabs(cvs->Odo->bufferPosition)) && cvs->Odo->flagBufferPosition == 1){
        cvs->Odo->flagBufferPosition = 0;
        cvs->Odo->bufferPosition = -100000;
        return true;
    }
}


#endif // REALBOT

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT
