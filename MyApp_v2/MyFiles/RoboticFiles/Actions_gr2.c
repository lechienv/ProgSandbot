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
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6 , 1.2, 0.05) : ReachPointPotential(cvs, -0.6 , -1.2 , 0.05) ;
            if(reached){
                cvs->stateAction1 = AlignedWithHouse1;
            }
            return false;
            break;
        }
       case(AlignedWithHouse1) :{
            bool aligned =  (color == GREEN) ? IsAlignedWithTheta(cvs, 10, 3) : IsAlignedWithTheta(cvs, -10, 3);
            if(aligned){
                cvs->stateAction1 = PushHouse1;
            }
            return false;
            break;
        }
         case(PushHouse1) :{
            //SpeedRefToDC(cvs,cvs->MotorL,-5);
            //SpeedRefToDC(cvs,cvs->MotorR,-5);
             PinceCalibration(cvs);
             cvs->MotorL->dutyCycle = -30;
             cvs->MotorR->dutyCycle = -30;
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
            PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6, 1.0, 0.05) : ReachPointPotential(cvs, -0.6, -1.0, 0.05);
            if(reached){
                cvs->stateAction1 = AlignedWithHouse2;
            }
            return false;
            break;
        }
       case(AlignedWithHouse2) :{
            bool aligned =  (color == GREEN) ? IsAlignedWithTheta(cvs, 10, 4) : IsAlignedWithTheta(cvs, -10, 4);
            if(aligned){
                cvs->stateAction1 = PushHouse2;
            }
            return false;
            break;
        }
         case(PushHouse2) :{
            //SpeedRefToDC(cvs,cvs->MotorL,-5);
            //SpeedRefToDC(cvs,cvs->MotorR,-5);
             PinceCalibration(cvs);
             cvs->MotorL->dutyCycle = -30;
             cvs->MotorR->dutyCycle = -30;
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
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.4, 1.3, 0.04) : ReachPointPotential(cvs, -0.4, -1.3, 0.04);
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
            close = ClosePince(cvs, 50);
            SpeedRefToDC(cvs,cvs->MotorL,1);
            SpeedRefToDC(cvs,cvs->MotorR,1);
            if(close){
                cvs->stateAction2 = BringBlockOne;
            }
        return false;
        break;
    }
    case(BringBlockOne):{
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0.1 , 0.6, 0.08) : ReachPointPotential(cvs, 0.1 , -0.6, 0.08);
        if(reached){
                cvs->stateAction2 = AlignForBlockOne;
        }
        return false;
        break;
    }
    case(AlignForBlockOne):{
        bool isAligned = (color == GREEN)? IsAlignedWithTheta(cvs,-100,5) : IsAlignedWithTheta(cvs,100,5);
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
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6 , (0.7 - 0.1375 - 0.022 ), 0.01) : ReachPointPotential(cvs, -0.6 , -(0.7 - 0.1375 - 0.02), 0.01); // 0.7 - 0.1375 - 0.02
            if(reached){
                cvs->stateAction3 = AlignForBlocTwo;
            } 
            return false; 
            break;
     }
    case(AlignForBlocTwo):{
            bool isOpen = PinceCalibration(cvs);
            bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs,-178,0.5) : IsAlignedWithTheta(cvs,-182, 0.5);
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
           isClosed = ClosePince(cvs, 30);
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
            isClosed = ClosePince(cvs, 30);
            if(isClosed){
                if(cvs->MotorPince->position <= -340){
                    SpeedRefToDC(cvs, cvs->MotorL, 0);
                    SpeedRefToDC(cvs, cvs->MotorR, 0);
                    cvs->stateAction3 = BringBlockTwoViaPoint;
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
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0 , 1, 0.08) : ReachPointPotential(cvs, 0 , -1, 0.08);
        if(reached){
            cvs->stateAction3 = BringBlockTwo;
        }
        return false;
        break;
    }
    case(BringBlockTwo):{
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0.1 , 0.6, 0.08): ReachPointPotential(cvs, 0.1 , -0.6, 0.08);
        if(reached){
            cvs->stateAction3 = AlignForBlockTwo;
        }
        return false;
        break;
    }
    case(AlignForBlockTwo):{
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
   switch(cvs->stateAction4){
    case(GoToFish) :{
        PinceReachPosition(cvs, -300);
        RatGoBottom(cvs, (color == GREEN) ? cvs->MotorRatL : cvs->MotorRatR);
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0.76 , 1.0, 0.02) : ReachPointPotential(cvs, 0.76 , -1.0 , 0.02) ;
        if(reached){
            cvs->stateAction4 = AlignForCalibFishes;
        }
        return false;
        break;
    }
    case(AlignForCalibFishes) :{
        PinceReachPosition(cvs, -300);
        RatGoBottom(cvs, (color == GREEN) ? cvs->MotorRatL : cvs->MotorRatR);
        bool isAligned = IsAlignedWithTheta(cvs, 180, 2);
        if(isAligned){
            cvs->stateAction4 = CalibFishes;
        }
        return false;
        break;
    }
    case(CalibFishes) :{
        PinceReachPosition(cvs, -300);
        (color == GREEN) ? RatGoTop(cvs, cvs->MotorRatL) : RatGoTop(cvs, cvs->MotorRatR) ;
        bool isCalibrate = XCalibration(cvs, 1-0.1322, 180) ;
        if(isCalibrate){
            cvs->stateAction4 = DecaleBordFishes;
        }
           return false;
           break;
    }
    case(DecaleBordFishes) :{
        (color == GREEN) ? RatGoTop(cvs, cvs->MotorRatL) : RatGoTop(cvs, cvs->MotorRatR) ;
        bool isReached = (color == GREEN) ? ReachPointPotential(cvs, 0.83 , 0.6, 0.1) : ReachPointPotential(cvs, 0.83 , -0.6 , 0.1); // y 0.5
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
         bool creneauDone = (color == GREEN) ? ReachPointPotential(cvs, 0.84 , 0.9, 0.02) : ReachPointPotential(cvs, 0.84 , -0.9, 0.02);
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
        (color == GREEN) ? SetAngle(DynaRatL, 60) : SetAngle(DynaRatR, 60);
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
        (color == GREEN) ? SetAngle(DynaRatL, 80) : SetAngle(DynaRatR, 80);
        cvs->stateAction4 = Avance;
        MyDelayMs(500);
        return false;
        break;
    } 
    case(Avance) :{
        SpeedRefToDC(cvs,cvs->MotorL,5);
        SpeedRefToDC(cvs,cvs->MotorR,5);
        bool goodY = (color == GREEN) ? cvs->Odo->y < 0.87 :  cvs->Odo->y > -0.87;
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
        (color == GREEN) ? SetAngle(DynaRatL, 160) : SetAngle(DynaRatR, 160);
        MyDelayMs(500);
        cvs->stateAction4 = RatGoUp;
        return false;
        break;
    } 
    case(RatGoUp) :{
        (color == GREEN) ? SetAngle(DynaRatL, 160) : SetAngle(DynaRatR, 160);
        bool reachedPoint = (color == GREEN) ? RateauReachPoint(cvs, 160) : RateauReachPoint(cvs, 160) ;
        if(reachedPoint){
            cvs->stateAction4 = Recule;
        }
        return false;
        break;
    }
     case(Recule) :{
        SpeedRefToDC(cvs,cvs->MotorL,-5);
        SpeedRefToDC(cvs,cvs->MotorR,-5);
        bool goodY = (color == GREEN) ? cvs->Odo->y > 0.80 :  cvs->Odo->y < -0.80;
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
        (color == GREEN) ? SetAngle(DynaRatL, 160) : SetAngle(DynaRatR, 160);
        bool decale = (color == GREEN) ? ReachPointPotential(cvs, 0.75 , 0.6, 0.03) : ReachPointPotential(cvs, 0.75, -0.6, 0.03);
        if(decale)
        {
           cvs->stateAction4 = MoveWithFish;
        }
        return false;
        break;
    }
    case(MoveWithFish) :{
        cvs->Obstacles->RectangleList[7].isActive = false;
        bool decale = (color == GREEN) ? ReachPointPotential(cvs, 0.87 , 0.3, 0.04) : ReachPointPotential(cvs, 0.87 , -0.3, 0.04);
        if(decale)
        {
           cvs->stateAction4 = AlignedWithNet;
        }
        return false;
        break;
     }
     case(AlignedWithNet) :{
        bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs, -91, 2) : IsAlignedWithTheta(cvs, 91, 2);
        if(isAligned){
            cvs->stateAction4 = ReleaseFish;
        }
        return false;
        break;
    }
    case(ReleaseFish) :{
        (color == GREEN) ? SetAngle(DynaRatL, 0) : SetAngle(DynaRatR, 0);
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
bool Action5(CtrlStruct *cvs){
   int color = cvs->robotID;
   switch(cvs->stateAction5){
    case(GotoDune) :{
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6 , 0.3, 0.05) : ReachPointPotential(cvs, -0.6 , -0.3 , 0.05) ;
            if(reached){
                cvs->stateAction5 = AlignedForDune;
            }
            return false;
            break;
        }
    default: break;
    }
}


void ActionBase(CtrlStruct *cvs)
{   int color = cvs->robotID;
    bool reached;
    if(color == GREEN){
        reached = ReachPointPotential(cvs, 0.1, 0.575, 0.05);
    }
    else{
         reached=  ReachPointPotential(cvs, 0.1, -0.575, 0.05);
    }
}
bool ActionParasol(CtrlStruct *cvs){
    MyDelayMs(2000);
    SetAngle(DynaPara, 100);
    MyDelayMs(1000);
    SetAngle(DynaPara, 170);
}
#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT