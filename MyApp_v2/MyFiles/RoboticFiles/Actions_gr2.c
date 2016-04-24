#include "Actions_gr2.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

/*********************
*** FUNCTIONS ********
**********************/


////////////////////////////////////////////////////////////ACTION 1 /////////////////////////////////////////////////////////////////////:
bool Action1(CtrlStruct *cvs){
   int color = cvs->robotID;
//enum StateAction1{GoToHouses, AlignedWithHouses, PushHouses, FreeHouses};
   switch(cvs->stateAction1){
    case(GoToHouse1) :{
        cvs->Obstacles->RectangleList[6].isActive = false;
        cvs->Obstacles->RectangleList[8].isActive = false;
        cvs->Obstacles->RectangleList[9].isActive = false;
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6 , 1.2, 0.1) : ReachPointPotential(cvs, -0.6 , -1.2 , 0.1) ;
            if(reached){
                cvs->stateAction1 = GoToHouse1Precision;
            }
            return false;
            break;
        }
    case(GoToHouse1Precision) :{
        cvs->Param->maxSpeed = M_PI;
        cvs->Obstacles->RectangleList[6].isActive = false;
        cvs->Obstacles->RectangleList[8].isActive = false;
        cvs->Obstacles->RectangleList[9].isActive = false;
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6 , 1.2, 0.05) : ReachPointPotential(cvs, -0.6 , -1.2 , 0.05) ;
            if(reached){
                cvs->stateAction1 = AlignedWithHouse1;
                cvs->Param->maxSpeed = 2*M_PI*1.5;
            }
            return false;
            break;
        }
       case(AlignedWithHouse1) :{
            bool aligned =  (color == GREEN) ? IsAlignedWithTheta(cvs, 10, 4) : IsAlignedWithTheta(cvs, -10, 4);
            if(aligned){
                cvs->stateAction1 = PushHouse1;
                cvs->Obstacles->RectangleList[6].isActive = true;
                cvs->Obstacles->RectangleList[8].isActive = true;
                cvs->Obstacles->RectangleList[9].isActive = true;
            }
            return false;
            break;
        }
         case(PushHouse1) :{
            //SpeedRefToDC(cvs,cvs->MotorL,-5);
            //SpeedRefToDC(cvs,cvs->MotorR,-5);
             PinceCalibration(cvs);
             if(cvs->Odo->x <= -0.8){
                cvs->MotorL->dutyCycle = -30;
                cvs->MotorR->dutyCycle = -30;
             }
             else{
                cvs->MotorL->dutyCycle = -45;
                cvs->MotorR->dutyCycle = -45;
             }
            if(cvs->Sensors->uSwitchLeft || cvs->Sensors->uSwitchRight || cvs->Odo->speedL == 0 || cvs->Odo->speedR == 0){
                    if(!cvs->TimerCalibration->isSet)
                    {
                        SetTimer(cvs, cvs->TimerCalibration, 1);
                    }
                    if(IsTimerTimout(cvs,cvs->TimerCalibration))
                    {
                        ResetTimer(cvs->TimerCalibration);
                        cvs->stateAction1 = FreeHouse1;
                        return false;
                    }
                    return false;
            }
            return false;
            break;
        }
         case(FreeHouse1) :{
            cvs->Obstacles->RectangleList[5].isActive = false;
            cvs->Obstacles->RectangleList[4].isActive = false;
            cvs->Obstacles->RectangleList[6].isActive = false;
            PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6, 1, 0.06) : ReachPointPotential(cvs, -0.6, -1, 0.06);
            if(reached){
                cvs->stateAction1 = GoToHouse2Precision;
            }
            return false;
            break;
        }
         case(GoToHouse2Precision) :{
            cvs->Param->maxSpeed = M_PI;
            cvs->Obstacles->RectangleList[5].isActive = false;
            cvs->Obstacles->RectangleList[4].isActive = false;
            cvs->Obstacles->RectangleList[6].isActive = false;
            PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6, 1, 0.06) : ReachPointPotential(cvs, -0.6, -1, 0.06);
            if(reached){
                cvs->stateAction1 = AlignedWithHouse2;
                cvs->Param->maxSpeed = 2*M_PI*1.5;
            }
            return false;
            break;
        }
       case(AlignedWithHouse2) :{
            bool aligned =  (color == GREEN) ? IsAlignedWithTheta(cvs, 10, 4) : IsAlignedWithTheta(cvs, -10, 4);
            if(aligned){
                cvs->stateAction1 = PushHouse2;
                cvs->Obstacles->RectangleList[5].isActive = true;
                cvs->Obstacles->RectangleList[4].isActive = true;
                cvs->Obstacles->RectangleList[6].isActive = true;
            }
            return false;
            break;
        }
         case(PushHouse2) :{
            //SpeedRefToDC(cvs,cvs->MotorL,-5);
            //SpeedRefToDC(cvs,cvs->MotorR,-5);
             PinceCalibration(cvs);
             if(cvs->Odo->x <= -0.8){
                cvs->MotorL->dutyCycle = -30;
                cvs->MotorR->dutyCycle = -30;
             }
             else{
                cvs->MotorL->dutyCycle = -45;
                cvs->MotorR->dutyCycle = -45;
             }
            if(cvs->Sensors->uSwitchLeft || cvs->Sensors->uSwitchRight || cvs->Odo->speedL == 0 || cvs->Odo->speedR == 0){
                    if(!cvs->TimerCalibration->isSet)
                    {
                     SetTimer(cvs, cvs->TimerCalibration, 1);
                    }
                    if(IsTimerTimout(cvs,cvs->TimerCalibration))
                    {
                    ResetTimer(cvs->TimerCalibration);
                     cvs->stateAction1 = FreeHouse2;
                    return true;
                    }
            }
            return false;
            break;
        }
         case(FreeHouse2) :{
            PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.4, 1.3, 0.05) : ReachPointPotential(cvs, -0.4, -1.3, 0.05);
            return reached;
            break;
        }
    default: break;
    }
}


////////////////////////////////////////////////////////////ACTION 2 /////////////////////////////////////////////////////////////////////:
bool Action2(CtrlStruct *cvs){
    double time = cvs->time;
    double x = cvs->Odo->x;
    double y = cvs->Odo->y;
    double theta = (cvs->Odo->theta);
    int color = cvs->robotID;

    switch(cvs->stateAction2){
    case(GoToBlocOne) :{
        cvs->Obstacles->RectangleList[8].isActive = false;
        PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.1 , (1+0.075), 0.05) : ReachPointPotential(cvs, -0.1 , -(1+0.075), 0.05);
            if(reached){
                cvs->stateAction2 = AlignForBlocOne;
            }
            return false;
            break;
        }
    case(AlignForBlocOne):{
        cvs->Obstacles->RectangleList[8].isActive = true;
            bool isOpen = PinceCalibration(cvs);
            bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs,-90,3) : IsAlignedWithTheta(cvs,90,3);
            if(isAligned && isOpen){
                cvs->stateAction2 = TakeBlocOne;
            }
            return false;
            break;
        }
    case(TakeBlocOne):{
            bool close = false;
            close = ClosePince(cvs, 60);
            //SpeedRefToDC(cvs,cvs->MotorL,1);
            //SpeedRefToDC(cvs,cvs->MotorR,1);
            //if(close){
               cvs->stateAction2 = BringBlockOne;
            //}
        return false;
        break;
    }
    case(BringBlockOne):{
        ClosePince(cvs, 60);
        cvs->Tower->ActivateTooClose = false;
        cvs->Obstacles->RectangleList[0].isActive = false;
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0.1 , 0.6, 0.08) : ReachPointPotential(cvs, 0.1 , -0.6, 0.08);
        if(reached){
                cvs->stateAction2 = AlignForBlockOne;
                cvs->Obstacles->RectangleList[0].isActive = true;
        }
        return false;
        break;
    }
    case(AlignForBlockOne):{
        ClosePince(cvs, 60);
        bool isAligned = (color == GREEN)? IsAlignedWithTheta(cvs,-100,10) : IsAlignedWithTheta(cvs,100,10);
            if(isAligned)
            {
                cvs->stateAction2 = ReleaseBlockOne;
            }
           return false;
           break;
    }
    case(ReleaseBlockOne):{
        bool isDeposed = DeposeBlock(cvs);
        if(isDeposed)
        {
            cvs->stateAction2 = EndBlocOneViaPoint;
        }
        return false;
        break;
    }
     case(EndBlocOneViaPoint):{
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.2 , 1, 0.1) : ReachPointPotential(cvs, -0.2 , -1, 0.1);
        cvs->Tower->ActivateTooClose = true;
        return reached;
        break;
    }
   
    default: break;
    }
}

////////////////////////////////////////////////////////////ACTION 3 /////////////////////////////////////////////////////////////////////:
bool Action3(CtrlStruct *cvs){
    double time = cvs->time;
    double x = cvs->Odo->x;
    double y = cvs->Odo->y;
    double theta = (cvs->Odo->theta);
    int color = cvs->robotID;

   switch(cvs->stateAction3){
        case(CalibY):{
            PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.5 , 1.2, 0.07) : ReachPointPotential(cvs, -0.5 , -1.2, 0.07);
            if(reached){
             cvs->stateAction3 = AlignForCalibYblocTwo;
            }
            return false; 
            break;
        }
        case(AlignForCalibYblocTwo):{
            PinceCalibration(cvs);
            bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs, -90, 5) : IsAlignedWithTheta(cvs, 90, 5);
            if(isAligned){
                cvs->stateAction3 = Calib_yBlocTwo;
            }
            return false; 
            break;
        }
        case(Calib_yBlocTwo):{
            PinceCalibration(cvs);
            bool isCalibrate = YCalibration(cvs, (color == GREEN) ? (1.5-0.1322) : -(1.5-0.1322), (color == GREEN) ? -90 : 90);
            if(isCalibrate)
            {
                //cvs->stateAction3 = GoToBlocTwoCalib;
                cvs->stateAction3 = GoToBlocTwo;
            }
            return false; 
            break;
        }
        case(GoToBlocTwoCalib) :{
            PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.5 , (0.55 - 0.1375 - 0.039), 0.05) : ReachPointPotential(cvs, -0.5 , -(0.55 - 0.1375 - 0.039), 0.05);
            if(reached){
                cvs->stateAction3 = AlignForCalibAction3;
            } 
           // cvs->stateAction3 = AlignForCalibAction3;
            return false; 
            break;
        }
        case(AlignForCalibAction3) :{
            bool isOpen = PinceCalibration(cvs);
                bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs,180,5) : IsAlignedWithTheta(cvs,180, 5);
                if(isAligned)
                {
                    cvs->stateAction3 = Calib_x;
                }
                return false;
                break;
         }
        case(Calib_x) :{
            bool isOpen = PinceCalibration(cvs);
            bool isCalibrate = XCalibration(cvs, (-0.25 - 0.1322), 180);
            if(isCalibrate){
                cvs->stateAction3 = GoToBlocTwo;
            }
            return false; 
            break;
         }
        case(GoToBlocTwo) :{
            if(color == GREEN){
                cvs->Obstacles->RectangleList[6].isActive = false;
                cvs->Obstacles->RectangleList[5].isActive = false;
                cvs->Obstacles->RectangleList[0].isActive = false;
            }
            else{
                cvs->Obstacles->RectangleList[6].isActive = false;
                cvs->Obstacles->RectangleList[4].isActive = false;
                cvs->Obstacles->RectangleList[0].isActive = false;
            }
            bool isOpen = PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.52 , (0.7), 0.04) : ReachPointPotential(cvs, -0.52 , -(0.75), 0.04); // 0.7 - 0.1375 - 0.02
            if(reached){
                cvs->stateAction3 = GoToBlocTwoPrecision;
            } 
            return false; 
            break;
     }
        case(GoToBlocTwoPrecision) :{
            cvs->Param->maxSpeed = M_PI;
            bool isOpen = PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6 , (0.7 - 0.1375 - 0.022 -0.02 ), 0.01) : ReachPointPotential(cvs, -0.6 , -(0.7 - 0.1375 - 0.022-0.02), 0.01); // 0.7 - 0.1375 - 0.02
            if(reached){
                cvs->stateAction3 = AlignForBlocTwo;
                cvs->Param->maxSpeed = 2*M_PI*1.5;
            } 
            return false; 
            break;
     }
        
    case(AlignForBlocTwo):{
            bool isOpen = PinceCalibration(cvs);
            bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs,-178,5) : IsAlignedWithTheta(cvs,-182, 5);
            if(isAligned && isOpen)
            {
                cvs->stateAction3 = AlignForBlocTwoPrecision;
            }
            return false;
            break;
        }    
       case(AlignForBlocTwoPrecision):{
            bool isOpen = PinceCalibration(cvs);
            bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs,-178,1) : IsAlignedWithTheta(cvs,-182, 1);
            if(isAligned && isOpen)
            {
                cvs->stateAction3 = AvanceForBlockTwo;
            }
            return false;
            break;
        }
    case(AvanceForBlockTwo):{
            if(color == GREEN){
                cvs->Obstacles->RectangleList[6].isActive = true;
                cvs->Obstacles->RectangleList[5].isActive = true;
                cvs->Obstacles->RectangleList[0].isActive = true;
            }
            else{
                cvs->Obstacles->RectangleList[6].isActive = true;
                cvs->Obstacles->RectangleList[4].isActive = true;
                cvs->Obstacles->RectangleList[0].isActive = true;
            }
            bool isClosed=false;
            cvs->MotorL->dutyCycle = 25;
            cvs->MotorR->dutyCycle = 25;
           if(!cvs->TimerCalibration->isSet)
           {
               SetTimer(cvs, cvs->TimerCalibration, 2);
           }
            if(cvs->Odo->x < -0.8 || IsTimerTimout(cvs,cvs->TimerCalibration) )
           {
           isClosed = ClosePince(cvs,35);
            if(isClosed){
                      SpeedRefToDC(cvs, cvs->MotorL, 0);
                      SpeedRefToDC(cvs, cvs->MotorR, 0);
                      ResetTimer(cvs->TimerCalibration);
                      cvs->stateAction3 = ReculeForBlockTwo;
                  }
     }
         return false;
         break;
    }
    case(ReculeForBlockTwo):{
        bool isClosed = false;
        ClosePince(cvs, 30);
        cvs->MotorL->dutyCycle = -15;
        cvs->MotorR->dutyCycle = -15;
        if(cvs->Odo->x > -0.6)
        {
            isClosed = ClosePince(cvs, 35);
            if(isClosed){
                if(cvs->MotorPince->position <= -340){
                    SpeedRefToDC(cvs, cvs->MotorL, 0);
                    SpeedRefToDC(cvs, cvs->MotorR, 0);
                    //cvs->stateAction3 = BringBlockTwoViaPoint;
                    return true;
                }
                else{
                    SpeedRefToDC(cvs, cvs->MotorL, 0);
                    SpeedRefToDC(cvs, cvs->MotorR, 0);
                    cvs->stateAction3 = BringBlockTwoViaPoint;
                }
            }
        }
         return false;
         break;
    }
    case(BringBlockTwoViaPoint):{
         ClosePince(cvs, 25);
        cvs->Param->maxSpeed = 2*M_PI * 1.8;
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0 , 1, 0.08) : ReachPointPotential(cvs, 0 , -1, 0.08);
        if(reached){
            cvs->stateAction3 = BringBlockTwo;
        }
        return false;
        break;
    }
    case(BringBlockTwo):{
        ClosePince(cvs, 25);
        cvs->Tower->ActivateTooClose = false;
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0.1 , 0.6, 0.08): ReachPointPotential(cvs, 0.1 , -0.6, 0.08);
        if(reached){
            cvs->stateAction3 = AlignForBlockTwo;
        }
        return false;
        break;
    }
    case(AlignForBlockTwo):{
        ClosePince(cvs, 25);
        bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs,-90,8) : IsAlignedWithTheta(cvs,90,8);
        if(isAligned){ 
            cvs->stateAction3 = ReleaseBlockTwo;
        }
        return false;
        break;
    }
    case(ReleaseBlockTwo):{
        bool reached = false;
        bool isDeposed = DeposeBlock(cvs);
        /*if(isDeposed){
            reached = (color == GREEN) ? ReachPointPotential(cvs, 0 , 1, 0.03) : ReachPointPotential(cvs, 0 , -1, 0.03);
        }*/
        cvs->Tower->ActivateTooClose = true;
        cvs->Param->maxSpeed = 2*M_PI * 1.5;
        return isDeposed;
        break;
    }
    default:  break;
    }
}


////////////////////////////////////////////////////////////ACTION 4 /////////////////////////////////////////////////////////////////////:
bool Action4(CtrlStruct *cvs)
{
   int color = cvs->robotID;
   int angleRatR;
   switch(cvs->stateAction4){
    case(GoToFish) :{
        PinceCalibration(cvs);
        RatGoBottom(cvs, (color == GREEN) ? cvs->MotorRatL : cvs->MotorRatR);
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0.75 , 0.9, 0.02) : ReachPointPotential(cvs, 0.75 , -0.9 , 0.02) ;
        if(reached){
            cvs->stateAction4 = AlignForCalibFishes;
        }
        return false;
        break;
    }
    case(AlignForCalibFishes) :{
        PinceCalibration(cvs);
        RatGoBottom(cvs, (color == GREEN) ? cvs->MotorRatL : cvs->MotorRatR);
        bool isAligned = IsAlignedWithTheta(cvs, 180, 2);
        if(isAligned){
            cvs->stateAction4 = CalibFishes;
        }
        return false;
        break;
    }
    case(CalibFishes) :{
        ClosePince(cvs, -30);
        (color == GREEN) ? RatGoTop(cvs, cvs->MotorRatL) : RatGoTop(cvs, cvs->MotorRatR) ;
        bool isCalibrate = XCalibration(cvs, 1-0.1322, 180) ;
        if(isCalibrate){
            cvs->stateAction4 = DecaleBordFishes;
        }
           return false;
           break;
    }
    case(DecaleBordFishes) :{
        ClosePince(cvs, -30);
        cvs->Param->maxSpeed = 2*M_PI;
        (color == GREEN) ? RatGoTop(cvs, cvs->MotorRatL) : RatGoTop(cvs, cvs->MotorRatR) ;
        bool isReached = (color == GREEN) ? ReachPointPotential(cvs, 0.83 , 0.65, 0.1) : ReachPointPotential(cvs, 0.83 , -0.65 , 0.1); // y 0.5
        if(isReached){
            cvs->stateAction4 = AlignForCreneau;
        }
        return false;
        break;
    }
    case(AlignForCreneau) :{
        (color == GREEN) ? RatGoTop(cvs, cvs->MotorRatL) : RatGoTop(cvs, cvs->MotorRatR) ;
        bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs, -90, 1) : IsAlignedWithTheta(cvs, 90, 1);
        if(isAligned){
            cvs->stateAction4 = DoTheCreneau;
        }
        return false;
        break;
    }
    case(DoTheCreneau) :{
        (color == GREEN) ? RatGoTop(cvs, cvs->MotorRatL) : RatGoTop(cvs, cvs->MotorRatR) ;
         cvs->Obstacles->RectangleList[7].isActive = false;
         bool creneauDone = (color == GREEN) ? ReachPointPotential(cvs, 0.84 ,0.9, 0.02) : ReachPointPotential(cvs, 0.84 , -0.9, 0.02);
         if(creneauDone)//|| (cvs->Odo->speedL == 0 && cvs->Odo->speedR == 0))
         {
            cvs->stateAction4 = AlignedWithFishes;
         }
         return false;
         break;
    }
    case(AlignedWithFishes) :{
        cvs->Obstacles->RectangleList[7].isActive = false;
        (color == GREEN) ? RatGoTop(cvs, cvs->MotorRatL) : RatGoTop(cvs, cvs->MotorRatR) ;
        bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs, -89, 1) : IsAlignedWithTheta(cvs, 89, 1);
        if(isAligned){
            cvs->stateAction4 = RatGoTopStartFish;
            cvs->Obstacles->RectangleList[7].isActive = true;
        }
        return false;
        break;
    }
    case(RatGoTopStartFish) :{
        bool isTop = (color == GREEN) ? RatGoTop(cvs, cvs->MotorRatL) : RatGoTop(cvs, cvs->MotorRatR) ;
        if(isTop){
         cvs->stateAction4 = DyntakeFish1;
        }
        return false;
        break;
    }
    case(DyntakeFish1) :{
        (color == GREEN) ? SetAngle(DynaRatL, 60) : SetAngle(DynaRatR, 260);
        MyDelayMs(500);
        cvs->stateAction4 = RatDescend;
        return false;
        break;
    }
    case(RatDescend) :{
        bool descendu = RateauReachPoint(cvs, 130);
        if(descendu){
            cvs->stateAction4 = DyntakeFish2;
        }
        return false;
        break;
    }
    case(DyntakeFish2) :{
        (color == GREEN) ? SetAngle(DynaRatL, 80) : SetAngle(DynaRatR, 240);
        cvs->stateAction4 = Avance;
        MyDelayMs(500);
        return false;
        break;
    } 
    case(Avance) :{
        SpeedRefToDC(cvs,cvs->MotorL,5);
        SpeedRefToDC(cvs,cvs->MotorR,5);
        bool goodY = (color == GREEN) ? cvs->Odo->y < 0.75 :  cvs->Odo->y > -0.75;
        if(goodY)
        {
            SpeedRefToDC(cvs,cvs->MotorL,0);
            SpeedRefToDC(cvs,cvs->MotorR,0);
            cvs->stateAction4 = DyntakeFish3;
        }
        return false;
        break;
    }
    case(DyntakeFish3) :{
        cvs->Obstacles->RectangleList[7].isActive = true;
        (color == GREEN) ? SetAngle(DynaRatL, 160) : SetAngle(DynaRatR, 150);
        MyDelayMs(500);
        cvs->stateAction4 = RatGoUp;
        return false;
        break;
    } 
    case(RatGoUp) :{
        (color == GREEN) ? SetAngle(DynaRatL, 160) : SetAngle(DynaRatR, angleRatR);
        bool reachedPoint = (color == GREEN) ? RateauReachPoint(cvs, 160) : RateauReachPoint(cvs, 160) ;
        if(reachedPoint){
            cvs->stateAction4 = Recule;
        }
        return false;
        break;
    }
     case(Recule) :{
        RateauReachPoint(cvs, 200);
        (color == GREEN) ? SetAngle(DynaRatL, 160) : SetAngle(DynaRatR, angleRatR);
        SpeedRefToDC(cvs,cvs->MotorL,-5);
        SpeedRefToDC(cvs,cvs->MotorR,-5);
        bool goodY = (color == GREEN) ? cvs->Odo->y > 0.9 :  cvs->Odo->y < -0.9;
        if(goodY)
        {
            SpeedRefToDC(cvs,cvs->MotorL,0);
            SpeedRefToDC(cvs,cvs->MotorR,0);
            cvs->stateAction4 = DecaleWithFishes;
        }
        return false;
        break;
    }
    case(DecaleWithFishes) :{
        (color == GREEN) ? SetAngle(DynaRatL, 160) : SetAngle(DynaRatR, angleRatR);
        bool decale = (color == GREEN) ? ReachPointPotential(cvs, 0.75 , 0.6, 0.03) : ReachPointPotential(cvs, 0.75, -0.6, 0.03);
        if(decale)
        {
           cvs->stateAction4 = MoveWithFish;
        }
        return false;
        break;
    }
    case(MoveWithFish) :{
        (color == GREEN) ? SetAngle(DynaRatL, 160) : SetAngle(DynaRatR, angleRatR);
        cvs->Obstacles->RectangleList[7].isActive = false;
        bool decale = (color == GREEN) ? ReachPointPotential(cvs, 0.83 , 0.3, 0.04) : ReachPointPotential(cvs, 0.83 , -0.3, 0.04);
        if(decale)
        {
           cvs->stateAction4 = AlignedWithNet;
        }
        return false;
        break;
     }
     case(AlignedWithNet) :{
        (color == GREEN) ? SetAngle(DynaRatL, 160) : SetAngle(DynaRatR, angleRatR);
        bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs, -91, 2) : IsAlignedWithTheta(cvs, 91, 2);
        if(isAligned){
            cvs->stateAction4 = ReleaseFish;
        }
        return false;
        break;
    }
    case(ReleaseFish) :{
        (color == GREEN) ? SetAngle(DynaRatL, 300) : SetAngle(DynaRatR, 0);
        MyDelayMs(500);
        (color == GREEN) ? SetAngle(DynaRatL, 170) : SetAngle(DynaRatR, 170);
        MyDelayMs(500);
        return RatGoBottom(cvs,(color == GREEN) ? (cvs->MotorRatL) : (cvs->MotorRatR));
        break;
    }
    default: break;
    }
}
////////////////////////////////////////////////////////////ACTION 5 /////////////////////////////////////////////////////////////////////:
bool Action5(CtrlStruct *cvs){ // Dune
   int color = cvs->robotID;
   cvs->Param->maxSpeed = 2 * M_PI * 1.5;
   switch(cvs->stateAction5){
    case(GotoDuneViaPoint) :{
        cvs->Obstacles->RectangleList[0].isActive = false;
        if(color == GREEN){
            //cvs->Obstacles->RectangleList[5].isActive = false;
        }
        else{
            //cvs->Obstacles->RectangleList[4].isActive = false;
        }
            PinceCalibration(cvs);
            cvs->Param->maxSpeed = 2 * M_PI * 1.5;
            cvs->Param->radiusBot = 0.14;
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.5 , 0.7, 0.04) : ReachPointPotential(cvs, -0.5 , -0.7 , 0.04) ;
            if(reached){
                cvs->stateAction5 = GotoDune;
            }
            return false;
            break;
        }
    case(GotoDune) :{
        //cvs->Obstacles->RectangleList[5].isActive = false;
        //cvs->Obstacles->RectangleList[4].isActive = false;
            if(!cvs->TimerCalibration->isSet)
               {
                   SetTimer(cvs, cvs->TimerCalibration, 6);
               }
                if(IsTimerTimout(cvs,cvs->TimerCalibration) )
               {
                    ResetTimer(cvs->TimerCalibration);
                     //cvs->Obstacles->RectangleList[5].isActive = true;
                     //cvs->Obstacles->RectangleList[4].isActive = true;
                    return true;
             }
            PinceCalibration(cvs);
            cvs->Param->maxSpeed = 2 * M_PI * 0.5;
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.53 , 0.02, 0.05) : ReachPointPotential(cvs, -0.53 , -0.02 , 0.05) ;
            if(reached){
                ResetTimer(cvs->TimerCalibration);
                //cvs->Obstacles->RectangleList[5].isActive = true;
                //cvs->Obstacles->RectangleList[4].isActive = true;
                cvs->stateAction5 = GotoDunePrecision;
            }
            return false;
            break;
        }
        case(GotoDunePrecision) :{
            if(!cvs->TimerCalibration->isSet)
               {
                   SetTimer(cvs, cvs->TimerCalibration, 5);
               }
                if(IsTimerTimout(cvs,cvs->TimerCalibration) )
               {
                    ResetTimer(cvs->TimerCalibration);
                    return true;
             }
            PinceCalibration(cvs);
            cvs->Param->maxSpeed = 2 * M_PI ;
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.53 , 0.0, 0.03) : ReachPointPotential(cvs, -0.53 , 0.0 , 0.03) ;
            if(reached){
                ResetTimer(cvs->TimerCalibration);
                cvs->stateAction5 = AlignedForDune;
                cvs->Param->radiusBot = radiusBot_INIT;
            }
            return false;
            break;
        }
    case(AlignedForDune) :{
        cvs->Obstacles->RectangleList[0].isActive = true;
        if(color == GREEN){
            cvs->Obstacles->RectangleList[5].isActive = true;
        }
        else{
            cvs->Obstacles->RectangleList[4].isActive = true;
        }
            bool isOpen = PinceCalibration(cvs);
            bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs, 180, 5) : IsAlignedWithTheta(cvs, -180, 5);
            if(isAligned && isOpen){
                cvs->stateAction5 = AvanceForBlocs;
            }
            return false;
            break;
        }
    case(AvanceForBlocs) :{
           cvs->MotorL->dutyCycle = 25;
           cvs->MotorR->dutyCycle = 25;
           if(!cvs->TimerCalibration->isSet)
           {
               SetTimer(cvs, cvs->TimerCalibration, 2);
           }
            if(cvs->Odo->x < -0.9 || IsTimerTimout(cvs,cvs->TimerCalibration) )
           {
                  SpeedRefToDC(cvs, cvs->MotorL, 0);
                  SpeedRefToDC(cvs, cvs->MotorR, 0);
                  ResetTimer(cvs->TimerCalibration);
                  cvs->stateAction5 = CloseForDune;
            return false;
            break;
            }
        }
    case(CloseForDune):{
        bool closed = ClosePince(cvs, 30);
        if(closed){
            cvs->stateAction5 = DesAlignedForDune;
        }
        return false;
        break;
    }
    case(DesAlignedForDune) :{
           /* bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs, 90, 2) : IsAlignedWithTheta(cvs, -90, 2);
            if(isAligned){
                cvs->stateAction5 = GoToViaPoint;
            }
            return false;
            break;*/
        ClosePince(cvs, 30);
           cvs->MotorL->dutyCycle = -25;
           cvs->MotorR->dutyCycle = -25;
            if(cvs->Odo->x < -0.6 )
           {
                  SpeedRefToDC(cvs, cvs->MotorL, 0);
                  SpeedRefToDC(cvs, cvs->MotorR, 0);
                  cvs->stateAction5 = GoToViaPoint;
            return false;
            break;
            }
      }
    case(GoToViaPoint) :{
        cvs->Param->maxSpeed = 2 * M_PI * 1.5;
        ClosePince(cvs, 25);
        cvs->Obstacles->RectangleList[0].isActive = false;
        if(color == GREEN){
            cvs->Obstacles->RectangleList[5].isActive = false;
        }
        else{
            cvs->Obstacles->RectangleList[4].isActive = false;
        }
            bool isReached = (color == GREEN) ? ReachPointPotential(cvs, -0.5, 0.8 , 0.1) : ReachPointPotential(cvs, -0.5, -0.8 , 0.1);
            if(isReached){
                cvs->stateAction5 = GoToViaPoint2;
                cvs->Param->maxSpeed = 2 * M_PI * 2.0;
            }
            return false;
            break;
            }
    case(GoToViaPoint2) :{
        ClosePince(cvs, 25);
        cvs->Obstacles->RectangleList[0].isActive = true;
        if(color == GREEN){
            cvs->Obstacles->RectangleList[5].isActive = true;
        }
        else{
            cvs->Obstacles->RectangleList[4].isActive = true;
        }
            bool isReached = (color == GREEN) ? ReachPointPotential(cvs, 0.1, 0.5 , 0.05) : ReachPointPotential(cvs, 0.1, -0.5 , 0.05);
            if(isReached){
                cvs->stateAction5 = AlignForDune;
            }
            return false;
            break;
            }
    case(AlignForDune):{
        bool isAligned = (color == GREEN)? IsAlignedWithTheta(cvs,-100,5) : IsAlignedWithTheta(cvs,100,5);
            if(isAligned)
            {
                cvs->stateAction5 = DeposeDune;
            }
           return false;
           break;
    }
    case(DeposeDune) :{ 
            bool isDeposed = DeposeBlock(cvs);
            return isDeposed;
            break;
            }
     default: break;
        }
}


void ActionBase(CtrlStruct *cvs)
{   int color = cvs->robotID;
    bool reached;
    if(color == GREEN){
        reached = ReachPointPotential(cvs, 0.1, 1.3, 0.05);
    }
    else{
         reached=  ReachPointPotential(cvs, 0.1, -1.3, 0.05);
    }
}
bool ActionParasol(CtrlStruct *cvs){
    MyDelayMs(2000);
    SetAngle(DynaPara, 100);
    MyDelayMs(100);
    SetAngle(DynaPara, 170);
}
#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT