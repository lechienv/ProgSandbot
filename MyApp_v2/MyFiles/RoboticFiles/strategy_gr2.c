/*!
* \file strategy_gr2.cc
* \
*/

#include "strategy_gr2.h"
#include "pathplanning_gr2.h"
#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT
void MyStrategy(CtrlStruct *cvs)
{
   int color = cvs->robotID;
   switch(cvs->stateStrategy){
        case(GoCalibration) :{ 
                bool succeed = Calibration(cvs);
                    if(!cvs->TimerAction->isSet)
                    {
                    SetTimer(cvs, cvs->TimerAction, 30);
                    }
                    if(IsTimerTimout(cvs,cvs->TimerAction))
                    {
                    ResetTimer(cvs->TimerAction);
                    cvs->stateStrategy = GoAction2;
                    }
                    if(succeed){
                    ResetTimer(cvs->TimerAction);
                    cvs->stateStrategy = GoAction2;
                    }
                break;
        }
       case(GoAction1) :{ 
                bool succeed = Action1(cvs);
                    if(!cvs->TimerAction->isSet)
                    {
                    SetTimer(cvs, cvs->TimerAction, 25); //20 pour une porte
                    }
                    if(IsTimerTimout(cvs,cvs->TimerAction))
                    {
                    ResetTimer(cvs->TimerAction);
                    cvs->stateStrategy = GoAction3;
                    }
                    if(succeed){
                    ResetTimer(cvs->TimerAction);
                    cvs->stateStrategy = GoAction3;
                     }
                break;
        }
        case(GoAction2) :{
               bool succeed = Action2(cvs);
                    if(!cvs->TimerAction->isSet)
                    {
                    SetTimer(cvs, cvs->TimerAction, 20);
                    }
                    if(IsTimerTimout(cvs,cvs->TimerAction))
                    {
                    ResetTimer(cvs->TimerReleaseBlocksRecule);
                    ResetTimer(cvs->TimerReleaseBlocksAvance);
                    ResetTimer(cvs->TimerAction);
                    cvs->stateStrategy = GoAction1;
                    }
                    if(succeed){
                    ResetTimer(cvs->TimerAction);
                    cvs->stateStrategy = GoAction1;
                     }
                break;
        }
        case(GoAction3) :{
                      bool succeed = Action3(cvs);
                    if(!cvs->TimerAction->isSet)
                    {
                    SetTimer(cvs, cvs->TimerAction, 45); // 35 on prend pas bien les blocs
                    }
                    if(IsTimerTimout(cvs,cvs->TimerAction))
                    {
                    ResetTimer(cvs->TimerAction);
                    cvs->stateStrategy = GoAction4;
                    }
                    if(succeed){
                    ResetTimer(cvs->TimerAction);
                    cvs->stateStrategy = GoAction4;
                     }
                break;
        }
         case(GoAction4) :{
                    bool succeed = Action4(cvs);
                    if(!cvs->TimerAction->isSet)
                    {
                    SetTimer(cvs, cvs->TimerAction, 60); // ??
                    }
                    if(IsTimerTimout(cvs,cvs->TimerAction))
                    {
                    ResetTimer(cvs->TimerAction);
                    cvs->stateStrategy = GoBase;
                    }
                    if(succeed){
                    ResetTimer(cvs->TimerAction);
                    cvs->stateStrategy = GoBase;
                     }
                break;
        }
         case(GoActionParasol) :{
                    bool succeed = ActionParasol(cvs);
                    cvs->stateStrategy = GoBase;
                break;
        }
        case(GoBase) :{
            ActionBase(cvs);
                break;
        }
    default: break;
    }
}

void ActivateBase(CtrlStruct *cvs) {
	int color = cvs->robotID;
	if (color == BLUE || color == RED) {
		cvs->Obstacles->QuarterOfCircleList[1].isActive = false;
	}
	else {
		cvs->Obstacles->QuarterOfCircleList[0].isActive = false;
	}
}

void DisactivateBase(CtrlStruct *cvs) {
	cvs->Obstacles->QuarterOfCircleList[0].isActive = true;
	cvs->Obstacles->QuarterOfCircleList[1].isActive = true;
}


bool Calibration(CtrlStruct *cvs) {
	/*cvs->Obstacles->CircleList[0].isActive = 0;
	cvs->Obstacles->CircleList[1].isActive = 0;
	cvs->Obstacles->CircleList[2].isActive = 0;
enum StateCalib {Cal_y_arr, GoToPoint, AlignAngle, Cal_x_arr, GoToBlocOne, AlignBlocOne, TakeBlocOne, BringBlocOne};*/
    
	double time = cvs->time;
	double x = cvs->Odo->x;
	double y = cvs->Odo->y;
	double theta = (cvs->Odo->theta);
	int color = cvs->robotID;
switch (cvs->stateCalib) {
	case(Cal_y_arr) : {
       bool isCalibrate = YCalibration(cvs, (color == GREEN) ? (1.5-0.1322) : -(1.5-0.1322), (color == GREEN) ? -90 : 90);
       if(isCalibrate){
           cvs->stateCalib = GoToPoint;
       }
       return false;
       break;
    }
	case(GoToPoint) :{
        PinceCalibration(cvs);
        bool reached =  (color == GREEN) ? ReachPointPotential(cvs, 0.7, 0.85, 0.03) : ReachPointPotential(cvs, 0.7, -0.85, 0.03);
        if(reached){
            cvs->stateCalib = AlignAngle;
        }
        return false;
        break;
    }

	case(AlignAngle) :{
            PinceCalibration(cvs);
            bool isAligned = IsAlignedWithTheta(cvs,180,3);
            if(isAligned){
                cvs->stateCalib = Cal_x_arr;
            }
            return false;
            break;
        }

	case(Cal_x_arr) :{
            PinceCalibration(cvs);
            bool isCalibrate = XCalibration(cvs, (1-0.1322), 180);
            if(isCalibrate){
                cvs->stateCalib = ReturnToBase;
            }
            return false;
            break;
    }

    case(ReturnToBase):{
        bool isopen = PinceCalibration(cvs);
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.14, 1.2, 0.03) : ReachPointPotential(cvs, -0.14, -1.2, 0.03);
        if(reached && isopen){
            cvs->stateCalib = AlignForBase;
        }
        return false;
        break;
    }
    case(AlignForBase):{
        
         bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs,-90,1) : IsAlignedWithTheta(cvs,90,1);
         if(isAligned){
            cvs->stateCalib = ReturnInIt;
         }
         return false;
         break;
    }
    case(ReturnInIt):{
        bool isCalibrate = YCalibration(cvs, (color == GREEN) ? (1.5-0.1322) : -(1.5-0.1322), (color == GREEN) ? -90 : 90);
        if(isCalibrate){
            cvs->stateCalib = Wait;
        }
     return false;
     break;
    }
    case(Wait):{
        //cvs->stateStrategy = GoAction1;
       return true;
        break;
    }
  }
}

void DynaTestFunction(CtrlStruct *cvs){
    //AllumeLed();
    if(cvs->time < 3)
        SetAngle(0x06, 10);
    else if(cvs->time < 5)
        SetAngle(0x06, 90);
    else if(cvs->time < 7)
        SetAngle(0x05, 200);
    else if(cvs->time < 9)
        SetAngle(0x05, 200);
    //ReadDyna();
    //ReadDyna();
    //WhichPosition(0x03);
    //WhichPosition(0x06);
}

	
//////////////////////////////////////////////////////////// homologation ///////////////////////////////////////////////////////////
void PointHomologation(CtrlStruct *cvs){     
    /*double distanceX = (cvs->Odo->x - cvs->Obstacles->CircleList[0].x )*(cvs->Odo->x - cvs->Obstacles->CircleList[0].x );
    double distanceY = (cvs->Odo->y - cvs->Obstacles->CircleList[0].y )*(cvs->Odo->y - cvs->Obstacles->CircleList[0].y );
    double distance = sqrt(distanceX+distanceY);*/
    
    MyStrategy(cvs);
   /* char s[128];
    sprintf(s,"distance= %f \t angle = %f \t\n", cvs->Tower->distance, cvs->Tower->angle);
    MyConsole_SendMsg(s);*/

    if(cvs->Tower->TooClose)
    {
        cvs->MotorL->dutyCycle = 0;//RightMotorDC;
        cvs->MotorR->dutyCycle = 0;// RightMotorDC;
        cvs->MotorL->totalError = 0;
        cvs->MotorR->totalError = 0;
    }
  
}

void SetTimer(CtrlStruct *cvs, MyTimer *Timer, double time){
    Timer->beginTime = cvs->time;
    Timer->endTime = cvs->time + time;
    Timer->isSet = true;
}

void ResetTimer(MyTimer *Timer){
    Timer->beginTime = 0;
    Timer->endTime = 0;
    Timer->isSet = false;
}

bool IsTimerTimout(CtrlStruct *cvs, MyTimer *Timer){
    return (cvs->time > Timer->endTime);
}


//ok
bool PinceCalibration(CtrlStruct *cvs){
    if(!cvs->Sensors->uSwitchPinceOut){
        SpeedRefToDC(cvs, cvs->MotorPince, 50);
        return false;
    }
    else{
        cvs->MotorPince->dutyCycle=  0;
        cvs->MotorPince->position = 0;
        return true;
    }
}

bool ClosePince(CtrlStruct *cvs, int duty){
    if(duty < 0){
        duty = -duty;
    }
    if(cvs->MotorPince->position <= -250){
        duty = 20;
    }
    cvs->MotorPince->dutyCycle = -duty;
    if((cvs->MotorPince->speed == 0.0) && (!cvs->Sensors->uSwitchPinceOut) && (cvs->MotorPince->position < -100)){
        return true;
    }
    return false;
}

bool DeposeBlock(CtrlStruct *cvs){
        if(!cvs->TimerReleaseBlocksAvance->isSet)
        {
            SetTimer(cvs, cvs->TimerReleaseBlocksAvance, 1);
        }
        if(IsTimerTimout(cvs,cvs->TimerReleaseBlocksAvance))
        {
                if(!cvs->TimerReleaseBlocksRecule->isSet)
                    {
                        SetTimer(cvs, cvs->TimerReleaseBlocksRecule, 1);
                    }            
                 if(IsTimerTimout(cvs,cvs->TimerReleaseBlocksRecule)){
                 ResetTimer(cvs->TimerReleaseBlocksRecule);
                 ResetTimer(cvs->TimerReleaseBlocksAvance);
                 SpeedRefToDC(cvs, cvs->MotorL, 0);
                 SpeedRefToDC(cvs, cvs->MotorR, 0);
                 return true;
            }
            else{
                SpeedRefToDC(cvs, cvs->MotorL, -4);
                SpeedRefToDC(cvs, cvs->MotorR, -4);
                return false;
            }
        }
        else{
            PinceCalibration(cvs);
            SpeedRefToDC(cvs, cvs->MotorL, 4);
            SpeedRefToDC(cvs, cvs->MotorR, 4);
            return false;
        }
        return false;
}
bool YCalibration(CtrlStruct *cvs, double Y, double Theta){
    int color = cvs->robotID;
    if (!cvs->Sensors->uSwitchLeft && !cvs->Sensors->uSwitchRight) {
            SpeedRefToDC(cvs, cvs->MotorL, -5);
            SpeedRefToDC(cvs, cvs->MotorR, -5);
        if(!cvs->TimerCalibration->isSet)
        {
            SetTimer(cvs, cvs->TimerCalibration, 4);
        }
        if(IsTimerTimout(cvs,cvs->TimerCalibration))
        {
            ResetTimer(cvs->TimerCalibration);
            return true;
        }
        return false;
    }   
    else  
    {
        ResetTimer(cvs->TimerCalibration);
        cvs->Odo->y = Y;
        cvs->Odo->theta = Theta;
        return true;
    }
}

bool XCalibration(CtrlStruct *cvs, double X, double Theta){
    int color = cvs->robotID;
    if (!cvs->Sensors->uSwitchLeft && !cvs->Sensors->uSwitchRight) {
            SpeedRefToDC(cvs, cvs->MotorL, -5);
            SpeedRefToDC(cvs, cvs->MotorR, -5);
        if(!cvs->TimerCalibration->isSet)
        {
            SetTimer(cvs, cvs->TimerCalibration,4);
        }
        if(IsTimerTimout(cvs,cvs->TimerCalibration))
        {
            ResetTimer(cvs->TimerCalibration);
            return true;
        }
        return false;
    }
    else{
        ResetTimer(cvs->TimerCalibration);
        cvs->Odo->x = X;
        cvs->Odo->theta = Theta;
        return true;
    }
}
bool Creneau(CtrlStruct *cvs){
    int color = cvs->robotID;
    if(color == GREEN){
        cvs->MotorL->dutyCycle = -10;
        cvs->MotorR->dutyCycle = -7;
        if(cvs->Odo->theta > -90){
            cvs->MotorL->dutyCycle = 0;
            cvs->MotorR->dutyCycle = 0;
            return true;
        }
    }
    else{
        cvs->MotorL->dutyCycle = -7;
        cvs->MotorR->dutyCycle = -10;
        if(cvs->Odo->theta < 90){
            cvs->MotorL->dutyCycle = 0;
            cvs->MotorR->dutyCycle = 0;
            return true;
        }
    }
    return false;
}


#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT




