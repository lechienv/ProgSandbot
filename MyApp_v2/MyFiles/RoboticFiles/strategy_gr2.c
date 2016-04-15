/*!
* \file strategy_gr2.cc
* \
*/

#include "strategy_gr2.h"
#include "pathplanning_gr2.h"
#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT
void TheGoals(CtrlStruct *cvs)
{
#ifndef REALBOT
	int color = cvs->robotID;
	bool my_bool =false;
	bool releaseConstr = false;
	double x = cvs->Odo->x;
	double y = cvs->Odo->y;
	bool inConstArea = (x < 0.35) && (x > -0.25) && ((y * y) < (0.85*0.85 - (x - 0.25)*(x - 0.25)));//0.6*0.6
	bool recal = false;
	bool constr = (cvs->inputs->nb_targets == 2);

	/* ----- Carry 2 targets ----- */
	if (constr) {
		ActivateBase(cvs);
		UpdateTarget(cvs);
		recal = true;

		/// Via point to avoid local min
		if (!cvs->Goals->via && !cvs->Goals->inConstr) {
			cvs->Goals->via = ReachPointPotential(cvs, 0, (color == BLUE || color == RED) ? -0.7 : 0.7, cvs->Goals->precision);
		}
		/// Go in constr area 
		else if (cvs->Goals->via && !cvs->Goals->inConstr) {
			cvs->Goals->inConstr = ReachPointPotential(cvs, 0, (color == BLUE || color == RED) ? -0.25 : 0.25, cvs->Goals->precision);
		}
		/// Release target in constr area
		else if (cvs->Goals->inConstr) {
			cvs->outputs->flag_release = 1;
			cvs->Goals->nbr_target_prev = 0;
			cvs->Goals->CurrentGoal = (cvs->Goals->previousGoal == 6) ? 0 : cvs->Goals->previousGoal + 1;
			cvs->stateReCalib = ReCal_rot1;
			cvs->Goals->via = false;
			cvs->Goals->inConstr = false;
		}

		DisactivateBase(cvs);
	}

	/* ----- Recalibration ----- */
	else if (inConstArea && (cvs->stateReCalib != ReCal_nextStrat)) {
		recal = true;
		ReCalibration(cvs);
		cvs->Goals->timeIN = cvs->time;
	}

	/* ----- End of match: go in construction area ----- */
	else if (cvs->Goals->backHome) {
		recal = false;
		ActivateBase(cvs);

		/// Via point to avoid local min
		if (!cvs->Goals->via && !cvs->Goals->endConstr) {
			cvs->Goals->via = ReachPointPotential(cvs, 0, (color == BLUE || color == RED) ? -0.85 : 0.85, cvs->Goals->precision);
		}

		/// Go in constr area 
		else if (cvs->Goals->via && !cvs->Goals->endConstr) {
			cvs->Goals->endConstr = ReachPointPotential(cvs, 0, (color == BLUE || color == RED) ? -0.25 : 0.25, cvs->Goals->precision);
		}

		/// Release target in constr area
		else if (cvs->Goals->endConstr) {
			cvs->outputs->flag_release = 1;
			cvs->Goals->nbr_target_prev = 0;
			cvs->Goals->CurrentGoal = (cvs->Goals->previousGoal == 6) ? 0 : cvs->Goals->previousGoal + 1;
			cvs->stateReCalib = ReCal_rot1;
			cvs->Goals->via = false;
			cvs->Goals->endConstr = false;
		}
		DisactivateBase(cvs);
	}

	/* ----- Reach next goal ----- */
	else {
		recal = false;
		cvs->outputs->flag_release = 0;
		my_bool = ReachPointPotential(cvs, cvs->Goals->ListOfGoals[cvs->Goals->CurrentGoal].X, cvs->Goals->ListOfGoals[cvs->Goals->CurrentGoal].Y, cvs->Goals->precision);
		if (my_bool) {
			cvs->Goals->previousGoal = cvs->Goals->CurrentGoal;
		}
	}

	/* ---- Are all the tragets taken? ---- */
	int nbrTaken = 0;
	int i;
	for (i = 0; i < 7; i++) {
		nbrTaken = (cvs->Goals->ListOfGoals[i].taken == true) ? (nbrTaken + 1) : nbrTaken;
	}
	if (nbrTaken == 7) {
		cvs->Goals->backHome = true;
	}
	
	/* ---- Determine if bot is stuck ---- */
	if (fabs(cvs->time - cvs->Goals->timeIN) > cvs->Goals->maxtimewait) {
		cvs->Goals->isblocked = true;
		cvs->Goals->lockState = cvs->Goals->CurrentGoal;
	}
	if (!my_bool) {
		cvs->Goals->goalIN = 0;
	}

	/* ----- Find next goal if not recalibrating ----- */
	if (!recal) {
		cvs->Goals->isblocked = (cvs->Goals->CurrentGoal != cvs->Goals->lockState) ? 0 : 1;
		UpdateTarget(cvs);
		if (my_bool || cvs->Goals->isblocked) {
			goto_nextstate(cvs, my_bool);
		}
	}
#endif // !REALBOT
}

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

void goto_nextstate(CtrlStruct *cvs, bool my_bool)
{		
	if (cvs->Goals->goalIN == 0) {
		cvs->Goals->timeIN = cvs->time;
	}
	cvs->Goals->goalIN += 1;

	if (fabs(cvs->Goals->timeIN - cvs->time) > 2 || cvs->Goals->isblocked) {
		bool nextFound = false;
		bool boucle = false;
		while (!nextFound) {
			if ((cvs->Goals->CurrentGoal < 6) && !boucle) {
				cvs->Goals->CurrentGoal = cvs->Goals->CurrentGoal + 1;
				if (cvs->Goals->CurrentGoal == cvs->Goals->previousGoal) {
					boucle = true;
				}
			}
			else if (!boucle)
			{
				cvs->Goals->CurrentGoal = 0;
			}

			if (!(cvs->Goals->ListOfGoals[cvs->Goals->CurrentGoal].taken) && !boucle) {
				nextFound = true;
			}
			else if (boucle) {
				cvs->Goals->backHome = true;
				break;
			}
		}
	}
}

void UpdateTarget(CtrlStruct *cvs) {
#ifndef REALBOT
	if (cvs->Goals->nbr_target_prev != cvs->inputs->nb_targets) {
		cvs->Goals->ListOfGoals[cvs->Goals->CurrentGoal].taken = true;
		cvs->Goals->nbr_target_prev = cvs->inputs->nb_targets;
	}
#endif
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
            bool reached = ReachPointPotential(cvs, -0.6 , 0.7 - 0.1375 - 0.039, 0.02);
            if(reached){
                cvs->stateAction2 = AlignForBlocTwo;
            }
            break;
        }
    case(AlignForBlocTwo):{
            bool isOpen = PinceCalibration(cvs);
            bool isAligned = IsAlignedWithTheta(cvs,180,1);
            if(isAligned && isOpen)
                cvs->stateAction2 = AvanceForBlockTwo;
            break;
        }
    case(AvanceForBlockTwo):{
        bool isClosed;
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















#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT




