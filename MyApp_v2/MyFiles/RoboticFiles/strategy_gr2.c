/*!
* \file strategy_gr2.cc
* \
*/

#include "strategy_gr2.h"
#include "pathplanning_gr2.h"
#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

#ifdef REALBOT
void PointHomologation(CtrlStruct *cvs){
    //enum StateHomologation {PinceCalib, reachViaPoint, AlignWithTheta, ReachBlocs, ClosingPince, GoViaZone, GoInZone, OpeningPince};
    int color = cvs->robotID;
    switch (cvs->stateHomologation){
        case PinceCalib:{
            bool calibred = PinceCalibration(cvs);
            if(calibred)
                cvs->stateHomologation = reachViaPoint;
            break;
        }
        case reachViaPoint:{
            bool isReached;
            if(color == GREEN){
                isReached = ReachPointPotential(cvs, -0.1, 1.2, 0.03);
            }
            else{
                isReached = ReachPointPotential(cvs, -0.1, -1.2, 0.03);
            }
             
            cvs->Obstacles->RectangleList[8].isActive = false;
            if(isReached){
                cvs->Obstacles->RectangleList[8].isActive = true;
                cvs->stateHomologation = AlignWithTheta;
            }
            break;
        }
        case AlignWithTheta:{
            bool isAligned;
            if(color == GREEN){
                isAligned = IsAlignedWithTheta(cvs,-90,1);
            }
            else{
                isAligned = IsAlignedWithTheta(cvs,90,1);
            }
            if(isAligned)
                cvs->stateHomologation = ReachBlocs;
            break;
        }
        case ReachBlocs:
                if(fabs(cvs->Odo->y) > 0.85 + 0.075 + 0.058){
                    SpeedRefToDC(cvs,cvs->MotorL,3);
                    SpeedRefToDC(cvs,cvs->MotorR,3);
                }
                else
                    cvs->stateHomologation = ClosingPince;
                break;
        case ClosingPince:{
            bool closed = ClosePince(cvs);
            if(closed)
                cvs->stateHomologation = GoViaZone;
            break;
        }
        case GoViaZone:{
            bool isReached;
            if(color == GREEN){
                isReached = ReachPointPotential(cvs, 0.1, 0.575, 0.05);
            }
            else{
                isReached = ReachPointPotential(cvs, 0.1, -0.575, 0.05);
            }
            if(isReached)
                cvs->stateHomologation = AlignZone; //AlignZone
            break;
        }
        case AlignZone:{
            bool isAligned;
            if(color == GREEN){
                isAligned = IsAlignedWithTheta(cvs,-95,1);
            }
            else{
                isAligned = IsAlignedWithTheta(cvs,95,1);
            }
            if(isAligned)
                cvs->stateHomologation = GoInZone;
            break;
        }
        case GoInZone:
            if(fabs(cvs->Odo->y) > 0.4){
                SpeedRefToDC(cvs,cvs->MotorL,3);
                SpeedRefToDC(cvs,cvs->MotorR,3);
            }
            else
                cvs->stateHomologation = OpeningPince;
            break;
        case OpeningPince:{
            bool calibred = PinceCalibration(cvs);               
            break;
        }
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

void Calibration(CtrlStruct *cvs) {    
	double time = cvs->time;
	double x = cvs->Odo->x;
	double y = cvs->Odo->y;
	double theta = (cvs->Odo->theta);
	int color = cvs->robotID;
    
switch (cvs->stateCalib) {
	case(Cal_y_arr) :
		if (!cvs->Sensors->uSwitchLeft && !cvs->Sensors->uSwitchRight) {
			SpeedRefToDC(cvs, cvs->MotorL, -5);
			SpeedRefToDC(cvs, cvs->MotorR, -5);
		}
		else {
			cvs->Odo->timein = (cvs->Odo->timeDelay == 0) ? cvs->time : cvs->Odo->timein;
			cvs->Odo->timeDelay += 1;
			if (fabs(cvs->Odo->timein - cvs->time) < 0.5) {
				SpeedRefToDC(cvs, cvs->MotorL, -1);
				SpeedRefToDC(cvs, cvs->MotorR, -1);
			}
			else {
				cvs->Odo->y = (1.5-0.1322);
				cvs->Odo->theta = -90;
				cvs->stateCalib = GoToPoint;
				cvs->Odo->timein = 0;
				cvs->Odo->timeDelay = 0;
			}
		}
		break;

	case(GoToPoint) :{
        PinceCalibration(cvs);
        bool reached = ReachPointPotential(cvs, 0.7, 0.85, 0.03);
        if(reached){
            cvs->stateCalib = AlignAngle;
        }
        break;
    }

	case(AlignAngle) :{
            PinceCalibration(cvs);
            bool isAligned = IsAlignedWithTheta(cvs,180,3);
            if(isAligned)
                cvs->stateCalib = Cal_x_arr;
            break;
        }

	case(Cal_x_arr) :
            PinceCalibration(cvs);
		if (!cvs->Sensors->uSwitchLeft && !cvs->Sensors->uSwitchRight) {
			SpeedRefToDC(cvs, cvs->MotorL, -5);
			SpeedRefToDC(cvs, cvs->MotorR, -5);
		}
		else {
			cvs->Odo->timein = (cvs->Odo->timeDelay == 0) ? cvs->time : cvs->Odo->timein;
			cvs->Odo->timeDelay += 1;
			if (fabs(cvs->Odo->timein - cvs->time) < 0.5) {
				SpeedRefToDC(cvs, cvs->MotorL, -1);
				SpeedRefToDC(cvs, cvs->MotorR, -1);
			}
			else {
				cvs->Odo->x = (1-0.1322);
				cvs->Odo->theta = 180;
				cvs->stateCalib = ReturnToBase;
				cvs->Odo->timein = 0;
				cvs->Odo->timeDelay = 0;
			}
                    break;
		}

    case(ReturnToBase):{
        bool isopen = PinceCalibration(cvs);
        bool reached = ReachPointPotential(cvs, -0.14, 1.2, 0.03);
        if(reached && isopen){
            cvs->stateCalib = AlignForBaseAndReturnInIt;
        }
        break;
    }
    case(AlignForBaseAndReturnInIt):{
        
         bool isAligned = IsAlignedWithTheta(cvs,-90,1);
         if(isAligned){
             if (!cvs->Sensors->uSwitchLeft && !cvs->Sensors->uSwitchRight) {
			SpeedRefToDC(cvs, cvs->MotorL, -5);
			SpeedRefToDC(cvs, cvs->MotorR, -5);
		}
		else {
			cvs->Odo->timein = (cvs->Odo->timeDelay == 0) ? cvs->time : cvs->Odo->timein;
			cvs->Odo->timeDelay += 1;
			if (fabs(cvs->Odo->timein - cvs->time) < 0.5) {
				SpeedRefToDC(cvs, cvs->MotorL, -1);
				SpeedRefToDC(cvs, cvs->MotorR, -1);
                }
                else {
                        cvs->Odo->y = (1.5-0.1322);
                        cvs->Odo->theta = -90;
                        cvs->Odo->timein = 0;
                        cvs->Odo->timeDelay = 0;
                        cvs->stateCalib = Wait;
                }
            }
         }
         break;
    }
    case(Wait):{
        Action2(cvs);
        break;
    }
  }
}
	

void Action1(CtrlStruct *cvs){
    double time = cvs->time;
    double x = cvs->Odo->x;
    double y = cvs->Odo->y;
    double theta = (cvs->Odo->theta);
    int color = cvs->robotID;

    switch(cvs->stateAction1){
    case(GoToBlocOne) :{
            bool reached = ReachPointPotential(cvs, -0.1 , 1+0.075, 0.03);
            if(reached){
                cvs->stateAction1 = AlignForBlocOne;
            }
            break;
        }
    case(AlignForBlocOne):{
            bool isAligned = IsAlignedWithTheta(cvs,-90,1);
            if(isAligned)
                cvs->stateAction1 = TakeBlocOne;
            break;
        }
    case(TakeBlocOne):{
        bool close;

            close = ClosePince(cvs);
            SpeedRefToDC(cvs,cvs->MotorL,1);
            SpeedRefToDC(cvs,cvs->MotorR,1);
            if(close){
                cvs->stateAction1 = BringBlockOne;
            }
         break;
    }
    case(BringBlockOne):{
        bool reached = ReachPointPotential(cvs, 0.1 , 0.55, 0.03);
        if(reached){
                cvs->stateAction1 = AlignForBlockOne;
        }
        break;
    }
    case(AlignForBlockOne):{
        bool isAligned = IsAlignedWithTheta(cvs,-90,1);
            if(isAligned)
                cvs->stateAction1 = ReleaseBlockOne;
            break;
    }
    case(ReleaseBlockOne):{
        bool reached;
        if(cvs->Odo->bufferTime == -100000)
            cvs->Odo->bufferTime = cvs->time;
        bool isDeposed = DeposeBlock(cvs);
        if(isDeposed){
            reached = ReachPointPotential(cvs, 0 , 1.2, 0.03);
        }
        if(reached){
            Action2(cvs);
        }
        break;
    }

    default: break;
    }
}
void Action2(CtrlStruct *cvs){
    double time = cvs->time;
    double x = cvs->Odo->x;
    double y = cvs->Odo->y;
    double theta = (cvs->Odo->theta);
    int color = cvs->robotID;

   switch(cvs->stateAction2){
    case(GoToBlocTwo) :{
            PinceCalibration(cvs);
            bool reached = ReachPointPotential(cvs, -0.6 , 0.7 - 0.1375 - 0.039 - 0.08, 0.02);
            if(reached){
                cvs->stateAction2 = AlignForBlocTwo;
            }
            break;
        }
    case(AlignForBlocTwo):{
            bool isOpen = PinceCalibration(cvs);
            bool isAligned = IsAlignedWithTheta(cvs,145,1);
            if(isAligned && isOpen)
                cvs->stateAction2 = AvanceForBlockTwo;
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
                    cvs->stateAction2 = ReculeForBlockTwo;
                }
              else{
                    cvs->MotorL->dutyCycle = 10;
                    cvs->MotorR->dutyCycle = 10;
                    cvs->Odo->flagBufferPosition = 0;
                }
             }
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
                    cvs->stateAction2 = BringBlockTwoViaPoint;
                }
             }
         break;
    }
    case(BringBlockTwoViaPoint):{
        bool reached = ReachPointPotential(cvs, 0 , 1, 0.03);
        if(reached){
            cvs->stateAction2 = BringBlockTwo;
        }
        break;
    }
    case(BringBlockTwo):{
        bool reached = ReachPointPotential(cvs, 0 , 0.55, 0.03);
        if(reached){
            cvs->stateAction2 = AlignForBlockTwo;
        }
        break;
    }
    case(AlignForBlockTwo):{
        bool isAligned = IsAlignedWithTheta(cvs,-90,1);
            if(isAligned)
                cvs->stateAction2 = ReleaseBlockTwo;
            break;
    }
    case(ReleaseBlockTwo):{
        bool reached;
        if(cvs->Odo->bufferTime == -100000)
            cvs->Odo->bufferTime = cvs->time;
        bool isDeposed = DeposeBlock(cvs);
        if(isDeposed){
            reached = ReachPointPotential(cvs, 0 , 1, 0.03);
        }
        if(reached){
            //Action2(cvs);
        }
        break;
    }

    default: break;
    }
}

void DynaTestFunction(CtrlStruct *cvs){
    //AllumeLed();
    //MyCyclone_Write(CYCLONE_IO_O_Data, 0x1111);
}

#endif


#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT




