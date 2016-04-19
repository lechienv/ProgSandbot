#include "Actions_gr2.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

/*********************
*** FUNCTIONS ********
**********************/


bool Action1(CtrlStruct *cvs){
   int color = cvs->robotID;
//enum StateAction1{GoToHouses, AlignedWithHouses, PushHouses, FreeHouses};
   switch(cvs->stateAction1){
    case(GoToHouses) :{
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6 , 1.17, 0.02) : ReachPointPotential(cvs, -0.6 , -1.17 , 0.02) ;
            if(reached){
                cvs->stateAction1 = AlignedWithHouses;
            }
            return false;
            break;
        }
       case(AlignedWithHouses) :{
            bool aligned =  (color == GREEN) ? IsAlignedWithTheta(cvs, 10, 5) : IsAlignedWithTheta(cvs, -10, 5);
            if(aligned){
                cvs->stateAction1 = PushHouses;
            }
            return false;
            break;
        }
         case(PushHouses) :{
            //SpeedRefToDC(cvs,cvs->MotorL,-5);
            //SpeedRefToDC(cvs,cvs->MotorR,-5);
             cvs->MotorL->dutyCycle = -40;
             cvs->MotorR->dutyCycle = -40;
            if(cvs->Sensors->uSwitchLeft && cvs->Sensors->uSwitchRight){
                    if(!cvs->TimerCalibration->isSet)
                    {
                    SetTimer(cvs, cvs->TimerCalibration, 2);
                    }
                    if(IsTimerTimout(cvs,cvs->TimerCalibration))
                    {
                    ResetTimer(cvs->TimerCalibration);
                     cvs->stateAction1 = FreeHouses;
                    return true;
                    }
            }
            return false;
            break;
        }
         case(FreeHouses) :{
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.14, 1.2, 0.03) : ReachPointPotential(cvs, -0.14, -1.2, 0.03);
            if(reached){
                cvs->stateStrategy = GoAction2;
            }
            return reached;
            break;
        }
    default: break;
    }
}

bool Action2(CtrlStruct *cvs){
    double time = cvs->time;
    double x = cvs->Odo->x;
    double y = cvs->Odo->y;
    double theta = (cvs->Odo->theta);
    int color = cvs->robotID;

    switch(cvs->stateAction2){
    case(GoToBlocOne) :{
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.1 , (1+0.075), 0.03) : ReachPointPotential(cvs, -0.1 , -(1+0.075), 0.03);
            if(reached){
                cvs->stateAction2 = AlignForBlocOne;
            }
            return false;
            break;
        }
    case(AlignForBlocOne):{
            bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs,-90,1) : IsAlignedWithTheta(cvs,90,1);
            if(isAligned){
                cvs->stateAction2 = TakeBlocOne;
            }
            return false;
            break;
        }
    case(TakeBlocOne):{
        bool close;

            close = ClosePince(cvs);
            SpeedRefToDC(cvs,cvs->MotorL,1);
            SpeedRefToDC(cvs,cvs->MotorR,1);
            if(close){
                cvs->stateAction2 = BringBlockOne;
            }
        return false;
        break;
    }
    case(BringBlockOne):{
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0.1 , 0.55, 0.05) : ReachPointPotential(cvs, 0.1 , -0.55, 0.05);
        if(reached){
                cvs->stateAction2 = AlignForBlockOne;
        }
        return false;
        break;
    }
    case(AlignForBlockOne):{
        bool isAligned = (color == GREEN)? IsAlignedWithTheta(cvs,-90,5) : IsAlignedWithTheta(cvs,90,5);
            if(isAligned)
                cvs->stateAction2 = ReleaseBlockOne;
           return false;
           break;
    }
    case(ReleaseBlockOne):{
        bool reached = false;
        bool isDeposed = DeposeBlock(cvs);
        if(isDeposed){
            reached = (color == GREEN) ? ReachPointPotential(cvs, 0 , 1.2, 0.03) : ReachPointPotential(cvs, 0 , -1.2, 0.03);
        }
        if(reached){
           cvs->stateAction2 = AlignForCalibAction2;
        }
        return false; 
        break;
    }
    case(AlignForCalibAction2):{
        bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs, -90, 1) : IsAlignedWithTheta(cvs, 90, 1);
        if(isAligned){
            cvs->stateAction2 = Calib_y;
        }
        return false; 
        break;
    }
    case(Calib_y):{
        bool isCalibrate = YCalibration(cvs, (color == GREEN) ? (1.5-0.1322) : -(1.5-0.1322), (color == GREEN) ? -90 : 90);
        return isCalibrate; 
        break;
    }

    default: break;
    }
}

bool Action3(CtrlStruct *cvs){
    double time = cvs->time;
    double x = cvs->Odo->x;
    double y = cvs->Odo->y;
    double theta = (cvs->Odo->theta);
    int color = cvs->robotID;

   switch(cvs->stateAction3){
    case(GoToBlocTwoCalib) :{
            PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6 , (0.55 - 0.1375 - 0.039), 0.04) : ReachPointPotential(cvs, -0.6 , -(0.55 - 0.1375 - 0.039), 0.04);
            if(reached){
                cvs->stateAction3 = AlignForCalibAction3;
            } 
            return false; 
            break;
        }
    case(AlignForCalibAction3) :{
        bool isOpen = PinceCalibration(cvs);
            bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs,180,1) : IsAlignedWithTheta(cvs,180, 1);
            if(isAligned && isOpen)
                cvs->stateAction3 = Calib_x;
            return false;
            break;
     }
    case(Calib_x) :{
        bool isCalibrate = XCalibration(cvs, (-0.25 - 0.1322), 180);
        if(isCalibrate){
            cvs->stateAction3 = GoToBlocTwo;
        }
         return false; 
         break;
     }
    case(GoToBlocTwo) :{
        PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6 , (0.7 - 0.1375 - 0.021), 0.02) : ReachPointPotential(cvs, -0.6 , -(0.7 - 0.1375 - 0.021), 0.02);
            if(reached){
                cvs->stateAction3 = AlignForBlocTwo;
            } 
            return false; 
            break;
     }
    case(AlignForBlocTwo):{
            bool isOpen = PinceCalibration(cvs);
            bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs,-177,1) : IsAlignedWithTheta(cvs,-183, 1);
            if(isAligned && isOpen)
                cvs->stateAction3 = AvanceForBlockTwo;
            return false;
            break;
        }
    case(AvanceForBlockTwo):{
     bool isAlign = false;
     bool isClosed=false;
     cvs->MotorL->dutyCycle = 25;
     cvs->MotorR->dutyCycle = 25;
     if(cvs->Odo->x < -0.8)
     {
         isClosed = ClosePince(cvs);
          if(isClosed){
                    SpeedRefToDC(cvs, cvs->MotorL, 0);
                    SpeedRefToDC(cvs, cvs->MotorR, 0);
                    cvs->stateAction3 = ReculeForBlockTwo;
                }
     }
         return false;
         break;
    }
    case(ReculeForBlockTwo):{
        bool isClosed = false;
        cvs->MotorL->dutyCycle = -15;
        cvs->MotorR->dutyCycle = -15;
        if(cvs->Odo->x > -0.5)
        {
            isClosed = ClosePince(cvs);
            if(isClosed){
                SpeedRefToDC(cvs, cvs->MotorL, 0);
                SpeedRefToDC(cvs, cvs->MotorR, 0);
                cvs->stateAction3 = BringBlockTwoViaPoint;
            }
        }
         return false;
         break;
    }
    case(BringBlockTwoViaPoint):{
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0 , 1, 0.05) : ReachPointPotential(cvs, 0 , -1, 0.05);
        if(reached){
            cvs->stateAction3 = BringBlockTwo;
        }
        return false;
        break;
    }
    case(BringBlockTwo):{
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0 , 0.55, 0.05): ReachPointPotential(cvs, 0 , -0.55, 0.05);
        if(reached){
            cvs->stateAction3 = AlignForBlockTwo;
        }
        return false;
        break;
    }
    case(AlignForBlockTwo):{
        bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs,-90,5) : IsAlignedWithTheta(cvs,90,5);
        if(isAligned)
            cvs->stateAction3 = ReleaseBlockTwo;
        return false;
        break;
    }
    case(ReleaseBlockTwo):{
        bool reached = false;
        bool isDeposed = DeposeBlock(cvs);
        if(isDeposed){
            reached = (color == GREEN) ? ReachPointPotential(cvs, 0 , 1, 0.03) : ReachPointPotential(cvs, 0 , -1, 0.03);
        }
        return reached;
        break;
    }
    default:  break;
    }
}

bool Action4(CtrlStruct *cvs)
{
   int color = cvs->robotID;
   switch(cvs->stateAction4){
    case(GoToFish) :{
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0.75 , 1.0, 0.02) : ReachPointPotential(cvs, 0.75 , -1.0 , 0.02) ;
            if(reached){
                cvs->stateAction4 = AlignForCalibFishes;
            }
            return false;
            break;
        }
     case(AlignForCalibFishes) :{
         bool isAligned = IsAlignedWithTheta(cvs, 180, 1);
         if(isAligned){
             cvs->stateAction4 = CalibFishes;
         }
            return false;
            break;
     }
     case(CalibFishes) :{
         bool isCalibrate = XCalibration(cvs, 1-0.1322, 180) ;
         if(isCalibrate){
             cvs->stateAction4 = DecaleBordFishes;
         }
            return false;
            break;
     }
       case(DecaleBordFishes) :{
           cvs->MotorL->dutyCycle = 15;
            cvs->MotorR->dutyCycle = 15;
         if(cvs->Odo->x <(1-0.22-0.1322+0.05) )
         {
            cvs->stateAction4 = DoTheCreneau; 
         }
         return false;
         break;
       }
       case(DoTheCreneau) :{
           bool creneauDone = Creneau(cvs);
         if(creneauDone)
         {
            cvs->stateAction4 = AlignedWithFishes; 
         }
         return false;
         break;
       }
     case(AlignedWithFishes) :{
         bool isAligned = (color == GREEN) ? IsAlignedWithTheta(cvs, -90, 1) : IsAlignedWithTheta(cvs, 90, 1);
         if(isAligned){
             cvs->stateAction4 = RatGoTopStartFish;
         }
            return false;
            break;
     }
          case(RatGoTopStartFish) :{
         bool isTop = (color == GREEN) ? RatGoTop(cvs, cvs->MotorRatL) : RatGoTop(cvs, cvs->MotorRatR) ;
         if(isTop){
             cvs->stateAction4 = DyntakeFish;
         }
            return false;
            break;
     }
     case(DyntakeFish) :{
           bool AreTaken = true;//(color == GREEN) ? RatGoTop(cvs, cvs->MotorRatL) : RatGoTop(cvs, cvs->MotorRatR) ;
         if(AreTaken){
             cvs->stateAction4 = MoveWithFish;
         }
         return false;
            break;
     }
        case(MoveWithFish) :{
            cvs->MotorL->dutyCycle = 5;
            cvs->MotorR->dutyCycle = 5;
            bool my_positionY = (color == GREEN) ? cvs->Odo->y < 0.4 : cvs->Odo->y > -0.4 ;
         if(my_positionY)
         {
            cvs->stateAction4 = ReleaseFish; 
         }
         return false;
            break;
     }
     case(ReleaseFish) :{

         return  RatGoBottom(cvs, cvs->MotorRatL);;
            break;
     }
    default: break;
    }
}


void ActionBase(CtrlStruct *cvs)
{     int color = cvs->robotID;
            bool reached;
            if(color == GREEN){
             reached = ReachPointPotential(cvs, 0.1, 0.575, 0.05);
            }
            else{
             reached=  ReachPointPotential(cvs, 0.1, -0.575, 0.05);
            }
     
}
#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT