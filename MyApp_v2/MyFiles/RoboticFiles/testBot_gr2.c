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

void Action1Test(CtrlStruct *cvs, double speedRef){
    SpeedRefToDC(cvs, cvs->MotorL, speedRef);
    SpeedRefToDC(cvs, cvs->MotorR, speedRef);
}

void Action2Test(CtrlStruct *cvs, double torqueRef){
    if (!cvs->Sensors->uSwitchPinceIn) {
		TorqueControl(cvs->MotorPince,torqueRef);
    }
    if(cvs->Sensors->uSwitchPinceOut){
        cvs->MotorPince->dutyCycle = 0;
    }
}
void Action3Test(CtrlStruct *cvs){
    bool result = ReachPointPotential(cvs,-0.8, 0.8, 0.1);
}

void Action4Test(CtrlStruct *cvs){
    cvs->MotorL->dutyCycle = RightMotorDC;
    cvs->MotorR->dutyCycle = RightMotorDC;
    cvs->MotorPince->dutyCycle = RightMotorDC;
    cvs->MotorRatL->dutyCycle = RightMotorDC;
    cvs->MotorRatR->dutyCycle = RightMotorDC;
    cvs->MotorTower->dutyCycle = RightMotorDC;
}

void Action5Test(CtrlStruct *cvs){
    cvs->MotorL->dutyCycle = var39;
}
void Action6Test(CtrlStruct *cvs){
    cvs->MotorR->dutyCycle = var39;
} 
void Action7Test(CtrlStruct *cvs){
    cvs->MotorPince->dutyCycle = var39;
}
void Action8Test(CtrlStruct *cvs){
    cvs->MotorRatL->dutyCycle = var39;
}
void Action9Test(CtrlStruct *cvs){
    cvs->MotorRatR->dutyCycle = var39;
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
    ReachPointPotential(cvs,var38, var39, 0.005);
}

void Action13Test(CtrlStruct *cvs){
    IsAlignedWithTheta(cvs,var39,0.1);
}

void StrategyTest(CtrlStruct *cvs){
    char theStr[256];
    if(var40 == 1){
        Action1Test(cvs, var38);
    }
    else if(var40 == 2){
        Action2Test(cvs,var39);
    }
    else if(var40 == 3){
        Action3Test(cvs);
    }
    else if(var40 == 4){
        Action4Test(cvs);
    }
    else if(var40 == 5){
        Action5Test(cvs);
    }
    else if(var40 == 6){
        Action6Test(cvs);
    }
    else if(var40 == 7){
        Action7Test(cvs);
    }
    else if(var40 == 8){
        Action8Test(cvs);
    }
    else if(var40 == 9){
        Action9Test(cvs);
    }
    else if(var40 == 10){
        Action10Test(cvs);
    }
    else if(var40 == 11){
        Action11Test(cvs);
    }
    else if(var40 == 12){
        Action12Test(cvs);
    }
    else if(var40 == 13){
        Action13Test(cvs);
    }
}
#endif // REALBOT

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT
