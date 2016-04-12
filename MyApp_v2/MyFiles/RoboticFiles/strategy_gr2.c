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
	cvs->Obstacles->CircleList[0].isActive = 0;
	cvs->Obstacles->CircleList[1].isActive = 0;
	cvs->Obstacles->CircleList[2].isActive = 0;
	double time = cvs->time;
	double x = cvs->Odo->x;
	double y = cvs->Odo->y;
	double theta = (cvs->Odo->theta);
	int color = cvs->robotID;
	bool TwoBots_bool = color == BLUE || color == YELLOW;
	switch (cvs->stateCalib) {
	case(Cal_y_av1) :
		if (fabs(y) > 1.1) {
			SpeedRefToDC(cvs, cvs->MotorL, 10);
			SpeedRefToDC(cvs, cvs->MotorR, 10);
		}
		else {
			cvs->stateCalib = (color == BLUE) ? Cal_rot_neg : Cal_rot_pos;
		}
		break;

	case(Cal_y_arr2) :
		if (fabs(y) < 1.37) {
			SpeedRefToDC(cvs, cvs->MotorL, -10);
			SpeedRefToDC(cvs, cvs->MotorR, -10);
		}
		else {
			cvs->stateCalib = Action1;
		}
		break;

	case(Cal_y_arr) :
		if (!cvs->Sensors->uSwitchLeft && !cvs->Sensors->uSwitchRight) {
			SpeedRefToDC(cvs, cvs->MotorL, ((color == YELLOW || color == BLUE) ? -12 : -6));
			SpeedRefToDC(cvs, cvs->MotorR, ((color == YELLOW || color == BLUE) ? -12 : -6));
		}
		else {
			cvs->Odo->timein = (cvs->Odo->timeDelay == 0) ? cvs->time : cvs->Odo->timein;
			cvs->Odo->timeDelay += 1;
			if (fabs(cvs->Odo->timein - cvs->time) < 0.5) {
				SpeedRefToDC(cvs, cvs->MotorL, ((color == YELLOW || color == BLUE) ? -12 : -6));
				SpeedRefToDC(cvs, cvs->MotorR, ((color == YELLOW || color == BLUE) ? -12 : -6));
			}
			else {
				cvs->Odo->y = (color == RED || color == BLUE) ? -1.44 : 1.44;
				cvs->Odo->theta = (color == RED || color == BLUE) ? 90 : -90;
				cvs->stateCalib = Cal_y_av;
				cvs->Odo->timein = 0;
				cvs->Odo->timeDelay = 0;
			}
		}
		break;

	case(Cal_y_av) :
		if (fabs(y) > ((color == YELLOW || color == BLUE) ? 1.2 : 1.42)) {
			SpeedRefToDC(cvs, cvs->MotorL, ((color == YELLOW || color == BLUE) ? 12 : 6));
			SpeedRefToDC(cvs, cvs->MotorR, ((color == YELLOW || color == BLUE) ? 12 : 6));
		}
		else {
			cvs->stateCalib = (color == YELLOW || color == BLUE) ? Action1 : ((color == WHITE) ? Cal_rot_pos : Cal_rot_neg);
		}
		break;

	case(Cal_rot_neg) : {
		double angleRef = (color == RED || color == BLUE) ? 0 : -90;
		bool isAligned = IsAlignedWithTheta(cvs, angleRef, 5);
		if (isAligned) {
			cvs->stateCalib = (color == RED || color == BLUE) ? Cal_x_arr : ((color == WHITE) ? Cal_y_arr2 : Cal_y_arr);
		}
		break;
	}

	case(Cal_x_arr) :
		if (!cvs->Sensors->uSwitchLeft && !cvs->Sensors->uSwitchRight) {
			SpeedRefToDC(cvs, cvs->MotorL, ((color == YELLOW || color == BLUE) ? -12 : -6));
			SpeedRefToDC(cvs, cvs->MotorR, ((color == YELLOW || color == BLUE) ? -12 : -6));
		}
		else {
			cvs->Odo->timein = (cvs->Odo->timeDelay == 0) ? cvs->time : cvs->Odo->timein;
			cvs->Odo->timeDelay += 1;
			if (fabs(cvs->Odo->timein - cvs->time) < 0.5) {
				SpeedRefToDC(cvs, cvs->MotorL, ((color == YELLOW || color == BLUE) ? -12 : -6));
				SpeedRefToDC(cvs, cvs->MotorR, ((color == YELLOW || color == BLUE) ? -12 : -6));
			}
			else {
				cvs->Odo->x = -0.94;
				cvs->Odo->theta = 0;
				cvs->stateCalib = Cal_x_av;
				cvs->Odo->timein = 0;
				cvs->Odo->timeDelay = 0;
			}
		}
		break;

	case(Cal_x_av) :
		if (x < ((color == YELLOW || color == BLUE) ? -0.25 : -0.15)) {
			SpeedRefToDC(cvs, cvs->MotorL, ((color == YELLOW || color == BLUE) ? 12 : 6));
			SpeedRefToDC(cvs, cvs->MotorR, ((color == YELLOW || color == BLUE) ? 12 : 6));
		}
		else {
			cvs->stateCalib = (color == YELLOW || color == WHITE) ? Cal_rot_neg : Cal_rot_pos;
		}
		break;

	case(Cal_rot_pos) : {
		double angleRef = (color == RED || color == BLUE) ? 90 : 0;
		bool isAligned = IsAlignedWithTheta(cvs, angleRef, 5);
		if (isAligned) {
			cvs->stateCalib = (color == YELLOW || color == WHITE) ? Cal_x_arr : ((color == RED) ? Cal_y_arr2 : Cal_y_arr);
		}
		break;
	}
	default: break;
	}
	cvs->Obstacles->CircleList[0].isActive = 1;
	cvs->Obstacles->CircleList[1].isActive = 1;
	cvs->Obstacles->CircleList[2].isActive = 1;
}

void ReCalibration(CtrlStruct *cvs) {
	cvs->Obstacles->CircleList[0].isActive = 0;
	cvs->Obstacles->CircleList[1].isActive = 0;
	cvs->Obstacles->CircleList[2].isActive = 0;
	double time = cvs->time;
	double x = cvs->Odo->x;
	double y = cvs->Odo->y;
	double theta = (cvs->Odo->theta);
	int color = cvs->robotID;
	bool TwoBots_bool = color == BLUE || color == YELLOW;
	switch (cvs->stateReCalib) {

	case(ReCal_rot1) : {
		double angleRef = 0;
		bool isAligned = IsAlignedWithTheta(cvs, angleRef, 5);
		if (isAligned) {
			cvs->stateReCalib = ReCal_x_arr;
		}
		break;
	}

	case(ReCal_x_arr) : {
		if (!cvs->Sensors->uSwitchLeft && !cvs->Sensors->uSwitchRight) {
			SpeedRefToDC(cvs, cvs->MotorL,-MAXSPEED);
			SpeedRefToDC(cvs, cvs->MotorR,-MAXSPEED);
		}
		else {
			cvs->Odo->timein = (cvs->Odo->timeDelay == 0) ? cvs->time : cvs->Odo->timein;
			cvs->Odo->timeDelay += 1;
			if (fabs(cvs->Odo->timein - cvs->time) < 0.5) {
				SpeedRefToDC(cvs, cvs->MotorL, -MAXSPEED);
				SpeedRefToDC(cvs, cvs->MotorR, -MAXSPEED);
			}
			else {
				cvs->Odo->x = -0.168;
				cvs->Odo->theta = 0;
				cvs->stateReCalib = ReCal_x_av;
				cvs->Odo->timein = 0;
				cvs->Odo->timeDelay = 0;
			}
		}
		break;
	}

	case(ReCal_x_av) : {
		if (x < 0) {
			SpeedRefToDC(cvs, cvs->MotorL, MAXSPEED / 2);
			SpeedRefToDC(cvs, cvs->MotorR, MAXSPEED / 2);
		}
		else {
			cvs->stateReCalib = ReCal_rot2;
		}
		break;
	}

	case(ReCal_rot2) : {
		double angleRef = ((color == BLUE || color == RED) ? -90 : 90);
		bool isAligned = IsAlignedWithTheta(cvs, angleRef, 5);
		if (isAligned) {
			cvs->stateReCalib = ReCal_y_arr;
		}
		break;
	}

	case(ReCal_y_arr) : {
		if (!cvs->Sensors->uSwitchLeft && !cvs->Sensors->uSwitchRight) {
			SpeedRefToDC(cvs, cvs->MotorL,-MAXSPEED);
			SpeedRefToDC(cvs, cvs->MotorR,-MAXSPEED);
		}
		else {
			cvs->Odo->timein = (cvs->Odo->timeDelay == 0) ? cvs->time : cvs->Odo->timein;
			cvs->Odo->timeDelay += 1;
			if (fabs(cvs->Odo->timein - cvs->time) < 0.5) {
				SpeedRefToDC(cvs, cvs->MotorL, -MAXSPEED);
				SpeedRefToDC(cvs, cvs->MotorR, -MAXSPEED);
			}
			else{
				cvs->Odo->y = (color == RED || color == BLUE) ? -0.071 : 0.071;
				cvs->Odo->theta = (color == RED || color == BLUE) ? -90 : 90;
				cvs->stateReCalib = ReCal_y_av;
				cvs->Odo->timein = 0;
				cvs->Odo->timeDelay = 0;
			}
		}
		break;
	}

	case(ReCal_y_av) : {
		if (fabs(y) < 0.6) {//0.9
			SpeedRefToDC(cvs, cvs->MotorL, MAXSPEED / 2);
			SpeedRefToDC(cvs, cvs->MotorR, MAXSPEED / 2);
		}
		else {
			cvs->stateReCalib = ReCal_nextStrat;
		}
		break;
	}

	default: break;
	}
	cvs->Obstacles->CircleList[0].isActive = 1;
	cvs->Obstacles->CircleList[1].isActive = 1;
	cvs->Obstacles->CircleList[2].isActive = 1;
}

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT