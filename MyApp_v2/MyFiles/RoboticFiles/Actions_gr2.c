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
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6 , 1.05, 0.02) : ReachPointPotential(cvs, -0.6 , -1.05 , 0.02) ;
            if(reached){
                cvs->stateAction1 = AlignedWithHouses;
            }
            return false;
            break;
        }
       case(AlignedWithHouses) :{
            bool aligned =   IsAlignedWithTheta(cvs, 0,5);
            if(aligned){
                cvs->stateAction1 = PushHouses;
            }
            return false;
            break;
        }
         case(PushHouses) :{
            SpeedRefToDC(cvs,cvs->MotorL,-5);
            SpeedRefToDC(cvs,cvs->MotorR,-5);
            if(cvs->Sensors->uSwitchLeft && cvs->Sensors->uSwitchRight){
                cvs->stateAction1 = FreeHouses;
            }
            return false;
            break;
        }
         case(FreeHouses) :{
            bool reached =(color == GREEN) ?  ReachPointPotential(cvs, -0.4 ,  1.05 , 0.02) :  ReachPointPotential(cvs, -0.4 ,  -1.05 , 0.02);
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
            bool isAligned = IsAlignedWithTheta(cvs,-90,1);
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
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0.1 , 0.55, 0.03) : ReachPointPotential(cvs, 0.1 , -0.55, 0.03);
        if(reached){
                cvs->stateAction2 = AlignForBlockOne;
        }
        return false;
        break;
    }
    case(AlignForBlockOne):{
        bool isAligned = IsAlignedWithTheta(cvs,-90,1);
            if(isAligned)
                cvs->stateAction2 = ReleaseBlockOne;
           return false;
           break;
    }
    case(ReleaseBlockOne):{
        bool reached;
        if(cvs->Odo->bufferTime == -100000)
            cvs->Odo->bufferTime = cvs->time;
        bool isDeposed = DeposeBlock(cvs);
        if(isDeposed){
            reached = (color == GREEN) ? ReachPointPotential(cvs, 0 , 1.2, 0.03) : ReachPointPotential(cvs, 0 , -1.2, 0.03);
        }
        if(reached){
           //cvs->stateStrategy = GoAction3;
          
        }
        return reached; 
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
    case(GoToBlocTwo) :{
            PinceCalibration(cvs);
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, -0.6 , (0.7 - 0.1375 - 0.039 - 0.08), 0.02) : ReachPointPotential(cvs, -0.6 , -(0.7 - 0.1375 - 0.039 - 0.08), 0.02);
            if(reached){
                cvs->stateAction3 = AlignForBlocTwo;
            } 
            return false; 
            break;
        }
    case(AlignForBlocTwo):{
            bool isOpen = PinceCalibration(cvs);
            bool isAligned = IsAlignedWithTheta(cvs,145,1);
            if(isAligned && isOpen)
                cvs->stateAction3 = AvanceForBlockTwo;
            return false;
            break;
        }
    case(AvanceForBlockTwo):{
        bool isClosed;
        bool isAlign;
            if((cvs->Odo->bufferTime < 0))
                cvs->Odo->bufferTime = cvs->time;

            if(cvs->Odo->bufferTime > cvs->time - 2){
                cvs->MotorL->dutyCycle = 25;
                cvs->MotorR->dutyCycle = 25;
                cvs->Odo->flagBufferPosition = 1;
            }
             else{
                //isClosed = ClosePince(cvs);
                isClosed = ClosePince(cvs);
                isAlign = IsAlignedWithTheta(cvs,180,1);
                if(isClosed){
                    SpeedRefToDC(cvs, cvs->MotorL, 0);
                    SpeedRefToDC(cvs, cvs->MotorR, 0);
                    cvs->Odo->bufferTime = -100000;
                    cvs->stateAction3 = ReculeForBlockTwo;
                }
              else{
                    cvs->MotorL->dutyCycle = 10;
                    cvs->MotorR->dutyCycle = 10;
                    cvs->Odo->flagBufferPosition = 0;
                }
             }
         return false;
         break;
    }
    case(ReculeForBlockTwo):{
        bool isClosed;
            if((cvs->Odo->bufferTime < 0))
                cvs->Odo->bufferTime = cvs->time;

            if(cvs->Odo->bufferTime > cvs->time - 1){
                cvs->MotorL->dutyCycle = -25;
                cvs->MotorR->dutyCycle = -25;
                cvs->Odo->flagBufferPosition = 1;
            }
             else{
                cvs->MotorL->dutyCycle = 0;
                cvs->MotorR->dutyCycle = 0;
                cvs->Odo->flagBufferPosition = 0;
                isClosed = ClosePince(cvs);
                if(isClosed){
                    cvs->Odo->bufferTime = -100000;
                    cvs->stateAction3 = BringBlockTwoViaPoint;
                }
             }
         return false;
         break;
    }
    case(BringBlockTwoViaPoint):{
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0 , 1, 0.03) : ReachPointPotential(cvs, 0 , -1, 0.03);
        if(reached){
            cvs->stateAction3 = BringBlockTwo;
        }
        return false;
        break;
    }
    case(BringBlockTwo):{
        bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0 , 0.55, 0.03): ReachPointPotential(cvs, 0 , -0.55, 0.03);
        if(reached){
            cvs->stateAction3 = AlignForBlockTwo;
        }
        return false;
        break;
    }
    case(AlignForBlockTwo):{
        bool isAligned = IsAlignedWithTheta(cvs,-90,1);
            if(isAligned)
                cvs->stateAction3 = ReleaseBlockTwo;
            return false;
            break;
    }
    case(ReleaseBlockTwo):{
        bool reached;
        if(cvs->Odo->bufferTime == -100000)
            cvs->Odo->bufferTime = cvs->time;
        bool isDeposed = DeposeBlock(cvs);
        if(isDeposed){
            reached = (color == GREEN) ? ReachPointPotential(cvs, 0 , 1, 0.03) : ReachPointPotential(cvs, 0 , -1, 0.03);
        }
        if(reached){
            cvs->stateStrategy = GoBase;
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
            bool reached = (color == GREEN) ? ReachPointPotential(cvs, 0.8 , 0.8, 0.02) : ReachPointPotential(cvs, 0.8 , -0.8 , 0.02) ;
            if(reached){
                cvs->stateAction4 = AlignedWithFishes;
            }
            return false;
            break;
        }
     case(AlignedWithFishes) :{
                ActionBase(cvs);
            return true;
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