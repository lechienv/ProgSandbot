#include "allParamInit_gr2.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT


void InitMotor(CtrlStruct *cvs)
{
    /* Left motor */
	cvs->MotorL = (Motor*)malloc(sizeof(Motor));
	cvs->MotorL->Kp					= KpL_INIT;
	cvs->MotorL->Ki					= KiL_INIT;
	cvs->MotorL->KpPos				= KpPosL_INIT;
	cvs->MotorL->Kphi				= KphiL_INIT;
	cvs->MotorL->R					= RL_INIT;
	cvs->MotorL->speed				= speedL_INIT;
	cvs->MotorL->position			= positionL_INIT;
	cvs->MotorL->reverseRotation	= reverseRotationL_INIT;
	cvs->MotorL->totalError			= totalErrorL_INIT;
	cvs->MotorL->dutyCycle			= dutyCycleL_INIT;
    cvs->MotorL->previousDutyCycle  = cvs->MotorL->dutyCycle;
    cvs->MotorL->areBrakesEnabled   = true;
	cvs->MotorL->ID					= LEFTMOTOR;
#ifdef REALBOT
    cvs->MotorL->PWMReg             = PWMRegL_INIT;
    cvs->MotorL->timerReg           = timerRegL_INIT;
    cvs->MotorL->perReg             = perRegL_INIT;
    cvs->MotorL->brakesMask         = brakesMaskL_INIT;
    cvs->MotorL->SID                = SIDL_INIT;
    cvs->MotorL->clicNumber         = clicNumberL_INIT;
#endif // REALBOT
    
    /* Right motor */
	cvs->MotorR = (Motor*)malloc(sizeof(Motor));
	cvs->MotorR->Kp					= KpR_INIT;
	cvs->MotorR->Ki					= KiR_INIT;
	cvs->MotorR->KpPos				= KpPosR_INIT;
	cvs->MotorR->Kphi				= KphiR_INIT;
	cvs->MotorR->R					= RR_INIT;
	cvs->MotorR->speed				= speedR_INIT;
	cvs->MotorR->position			= positionR_INIT;
	cvs->MotorR->reverseRotation	= reverseRotationR_INIT;
	cvs->MotorR->totalError			= totalErrorR_INIT;
	cvs->MotorR->dutyCycle			= dutyCycleR_INIT;
    cvs->MotorR->previousDutyCycle  = cvs->MotorR->dutyCycle;
    cvs->MotorR->areBrakesEnabled   = true;
	cvs->MotorR->ID					= RIGHTMOTOR;
#ifdef REALBOT
    cvs->MotorR->PWMReg             = PWMRegR_INIT;
    cvs->MotorR->timerReg           = timerRegR_INIT;
    cvs->MotorR->perReg             = perRegR_INIT;
    cvs->MotorR->brakesMask         = brakesMaskR_INIT;
    cvs->MotorR->SID                = SIDR_INIT;
    cvs->MotorR->clicNumber         = clicNumberR_INIT;
#endif // REALBOT
    
	/* Tower motor */
	cvs->MotorTower = (Motor*)malloc(sizeof(Motor));
	cvs->MotorTower->Kp             = KpTower_INIT;
	cvs->MotorTower->Ki             = KiTower_INIT;
	cvs->MotorTower->KpPos			= KpPosTower_INIT;
	cvs->MotorTower->Kphi           = KphiTower_INIT;
	cvs->MotorTower->R				= RTower_INIT;
	cvs->MotorTower->speed          = speedTower_INIT;
	cvs->MotorTower->position		= positionTower_INIT;
	cvs->MotorTower->reverseRotation = reverseRotationTower_INIT;
	cvs->MotorTower->totalError     = totalErrorTower_INIT;
	cvs->MotorTower->dutyCycle      = dutyCycleTower_INIT;
    cvs->MotorTower->previousDutyCycle  = cvs->MotorTower->dutyCycle;
    cvs->MotorTower->areBrakesEnabled   = true;
	cvs->MotorTower->ID             = TOWERMOTOR;
#ifdef REALBOT
	cvs->MotorTower->PWMReg         = PWMRegTower_INIT;
	cvs->MotorTower->timerReg       = timerRegTower_INIT;
	cvs->MotorTower->perReg         = perRegTower_INIT;
	cvs->MotorTower->brakesMask     = brakesMaskTower_INIT;
	cvs->MotorTower->SID            = SIDTower_INIT;
	cvs->MotorTower->clicNumber     = clicNumberTower_INIT;
#endif // REALBOT

#ifdef REALBOT
    /* Right rateau motor */
    cvs->MotorRatR = (Motor*)malloc(sizeof(Motor));
	cvs->MotorRatR->Kp					= KpRatR_INIT;
	cvs->MotorRatR->Ki					= KiRatR_INIT;
	cvs->MotorRatR->KpPos				= KpPosRatR_INIT;
	cvs->MotorRatR->Kphi				= KphiRatR_INIT;
	cvs->MotorRatR->R					= RRatR_INIT;
	cvs->MotorRatR->speed				= speedRatR_INIT;
	cvs->MotorRatR->position			= positionRatR_INIT;
	cvs->MotorRatR->reverseRotation     = reverseRotationRatR_INIT;
	cvs->MotorRatR->totalError			= totalErrorRatR_INIT;
	cvs->MotorRatR->dutyCycle			= dutyCycleRatR_INIT;
    cvs->MotorRatR->previousDutyCycle   = cvs->MotorRatR->dutyCycle;
    cvs->MotorRatR->areBrakesEnabled    = true;
	cvs->MotorRatR->ID					= RATRMOTOR;
    cvs->MotorRatR->PWMReg              = PWMRegRatR_INIT;
    cvs->MotorRatR->timerReg            = timerRegRatR_INIT;
    cvs->MotorRatR->perReg              = perRegRatR_INIT;
    cvs->MotorRatR->brakesMask          = brakesMaskRatR_INIT;
    cvs->MotorRatR->SID                 = SIDRatR_INIT;
    cvs->MotorRatR->clicNumber          = clicNumberRatR_INIT;
    
    /* Left rateau motor */
    cvs->MotorRatL = (Motor*)malloc(sizeof(Motor));
	cvs->MotorRatL->Kp					= KpRatL_INIT;
	cvs->MotorRatL->Ki					= KiRatL_INIT;
	cvs->MotorRatL->KpPos				= KpPosRatL_INIT;
	cvs->MotorRatL->Kphi				= KphiRatL_INIT;
	cvs->MotorRatL->R					= RRatL_INIT;
	cvs->MotorRatL->speed				= speedRatL_INIT;
	cvs->MotorRatL->position			= positionRatL_INIT;
	cvs->MotorRatL->reverseRotation     = reverseRotationRatL_INIT;
	cvs->MotorRatL->totalError			= totalErrorRatL_INIT;
	cvs->MotorRatL->dutyCycle			= dutyCycleRatL_INIT;
    cvs->MotorRatL->previousDutyCycle  = cvs->MotorRatL->dutyCycle;
    cvs->MotorRatL->areBrakesEnabled    = true;
	cvs->MotorRatL->ID					= RATLMOTOR;
    cvs->MotorRatL->PWMReg              = PWMRegRatL_INIT;
    cvs->MotorRatL->timerReg            = timerRegRatL_INIT;
    cvs->MotorRatL->perReg              = perRegRatL_INIT;
    cvs->MotorRatL->brakesMask          = brakesMaskRatL_INIT;
    cvs->MotorRatL->SID                 = SIDRatL_INIT;
    cvs->MotorRatL->clicNumber          = clicNumberRatL_INIT;
    
    /* Pince motor */
    cvs->MotorPince = (Motor*)malloc(sizeof(Motor));
	cvs->MotorPince->Kp					= KpPince_INIT;
	cvs->MotorPince->Ki					= KiPince_INIT;
	cvs->MotorPince->KpPos				= KpPosPince_INIT;
	cvs->MotorPince->Kphi				= KphiPince_INIT;
	cvs->MotorPince->R					= RPince_INIT;
	cvs->MotorPince->speed				= speedPince_INIT;
	cvs->MotorPince->speed				= positionPince_INIT;
	cvs->MotorPince->reverseRotation    = reverseRotationPince_INIT;
	cvs->MotorPince->totalError			= totalErrorPince_INIT;
	cvs->MotorPince->dutyCycle			= dutyCyclePince_INIT;
    cvs->MotorPince->previousDutyCycle  = cvs->MotorPince->dutyCycle;
    cvs->MotorPince->areBrakesEnabled   = true;
	cvs->MotorPince->ID					= PINCEMOTOR;
    cvs->MotorPince->PWMReg             = PWMRegPince_INIT;
    cvs->MotorPince->timerReg           = timerRegPince_INIT;
    cvs->MotorPince->perReg             = perRegPince_INIT;
    cvs->MotorPince->brakesMask         = brakesMaskPince_INIT;
    cvs->MotorPince->SID                = SIDPince_INIT;
    cvs->MotorPince->clicNumber         = clicNumberPince_INIT;
#endif // REALBOT

}

void InitParam(CtrlStruct *cvs) {
	cvs->Param = (Parametres*)malloc(sizeof(Parametres));
	cvs->Param->radiusBot			= radiusBot_INIT;
	cvs->Param->width				= width_INIT;
	cvs->Param->lengthTower			= lengthTower_INIT;
	cvs->Param->rayonBeacon			= rayonBeacon_INIT;
    cvs->Param->wheelLRadius        = wheelLRadius_INIT;
    cvs->Param->wheelRRadius        = wheelRRadius_INIT;
	cvs->Param->wheelRadius			= wheelRadius_INIT;
	cvs->Param->KpRot				= KpRot_INIT;
	cvs->Param->KiRot				= KiRot_INIT;
	cvs->Param->totalErrorRot		= totalErrorRot_INIT;
	cvs->Param->speedDifThreshold	= speedDifThreshold_INIT; //Max speed difference before considering the robot "at rest"
	cvs->Param->KiAngleThreshold	= KiAngleThreshold_INIT; //Angle control: threshold to activate Ki
    cvs->Param->maxAcceleration     = maxAcceleration_INIT;
}

void InitSensors(CtrlStruct *cvs) {
	cvs->Sensors = (Sensors*)malloc(sizeof(Sensors));
	cvs->Sensors->uSwitchLeft		= false;
	cvs->Sensors->uSwitchRight		= false;
#ifdef REALBOT
    cvs->Sensors->uSwitchPinceIn    = false;
    cvs->Sensors->uSwitchPinceOut   = false;
#endif // REALBOT
}
void InitTower(CtrlStruct * cvs)
{
	cvs->Tower = (Tower*)malloc(sizeof(Tower));
	cvs->Tower->falling_index						= falling_index_INIT;
	cvs->Tower->falling_index_fixed					= last_falling_fixed_INIT;
	int i; 
	for (i = 0; i <  NB_STORE_EDGE ; i++) {
		cvs->Tower->last_falling[i]			= last_falling_INIT;
		cvs->Tower->last_falling_fixed[i]	= last_falling_fixed_INIT;
		cvs->Tower->last_rising[i]			= last_rising_INIT;
		cvs->Tower->last_rising_fixed[i]	= rising_index_INIT;
	}
	cvs->Tower->nb_falling							= nb_falling_fixed_INIT;
	cvs->Tower->nb_falling_fixed					= nb_falling_fixed_INIT;
	cvs->Tower->nb_opponents						= nb_opponents_INIT;
	cvs->Tower->nb_rising							=  nb_rising_INIT;
	cvs->Tower->nb_rising_fixed						= nb_rising_fixed_INIT;
	cvs->Tower->rising_index						= rising_index_fixed_INIT;
	cvs->Tower->rising_index_fixed					= rising_index_fixed_INIT;
	cvs->Tower->tower_pos							= tower_pos_INIT;
	cvs->Tower->angle								= angle_INIT;
	cvs->Tower->distance							= distance_INIT;
}

void InitPotential(CtrlStruct *cvs) {
	cvs->Poto = (Potential*)malloc(sizeof(Potential));
	cvs->Poto->katt					= katt_INIT;
	cvs->Poto->krep					= krep_INIT;
	cvs->Poto->kFV					= kFV_INIT;
	cvs->Poto->FXRob				= FXRob_INIT;
	cvs->Poto->FYRob				= FyRob_INIT;
	cvs->Poto->kw					= kw_INIT;
	cvs->Poto->minDistance			= minDistance_INIT;
    cvs->Poto->thresholdAligned     = thresholdAligned_INIT;
}


void InitOdometry(CtrlStruct *cvs) {
	cvs->Odo = (Odometry*)malloc(sizeof(Odometry));
	cvs->Odo->timein = 0;
	cvs->Odo->timeDelay = 0.0;
    cvs->Odo->speedL = 0;
    cvs->Odo->speedR = 0;
	int color = cvs->robotID;
#ifdef REALBOT
    cvs->Odo->clicNumber = clicNumberOdo_INIT;
    cvs->Odo->x		= -0.1425;
    cvs->Odo->y		= 1.3678;
    cvs->Odo->theta	= -90.0;
#else

	if (color == RED) {
		cvs->Odo->x		= -0.075;
		cvs->Odo->y		= -1.4;
		cvs->Odo->theta	= 90.0;
	}
	else if (color == BLUE) {
		cvs->Odo->x		= -0.225;
		cvs->Odo->y		= -1.15;
		cvs->Odo->theta = 90;
	}
	else if (color == YELLOW) {
		cvs->Odo->x		= -0.225;
		cvs->Odo->y		= 1.15;
		cvs->Odo->theta = -90.0;
	}
	else if (color == WHITE) {
		cvs->Odo->x		= -0.075;
		cvs->Odo->y		= 1.4;
		cvs->Odo->theta = -90.0;
	}
	else {
		cvs->Odo->x		= 0;
		cvs->Odo->y		= 0;
		cvs->Odo->theta = 0;
	}
#endif // REALBOT
}

void InitGoals(CtrlStruct *cvs) {
	cvs->Goals = (Goals*)malloc(sizeof(Goals));
	cvs->Goals->NumberOfGoals = MaxGoals;
	cvs->Goals->ListOfGoals = (Points*)malloc(sizeof(Points)*MaxGoals);
	int color = cvs->robotID;
	cvs->stateVia = normalPoint;

	//3 pts
	cvs->Goals->ListOfGoals[0].X = -0.6;
	cvs->Goals->ListOfGoals[0].Y = (color == BLUE || color == RED) ? 0.05 : -0.05;
	cvs->Goals->ListOfGoals[0].taken = false;
	//1 pts
	cvs->Goals->ListOfGoals[1].X = -0.65;
	cvs->Goals->ListOfGoals[1].Y = (color == BLUE || color == RED) ? -1.25 : 1.25;
	cvs->Goals->ListOfGoals[1].taken = false;
	//1 pts
	cvs->Goals->ListOfGoals[2].X = 0.4;
	cvs->Goals->ListOfGoals[2].Y = (color == BLUE || color == RED) ? -1.1 : 1.1;
	cvs->Goals->ListOfGoals[2].taken = false;
	//2 pts
	cvs->Goals->ListOfGoals[3].X = 0.85;
	cvs->Goals->ListOfGoals[3].Y = (color == BLUE || color == RED) ? -0.8 : 0.8;
	cvs->Goals->ListOfGoals[3].taken = false;
	//2 pts
	cvs->Goals->ListOfGoals[4].X = 0.85;
	cvs->Goals->ListOfGoals[4].Y = (color == BLUE || color == RED) ? 0.9 : -0.9;
	cvs->Goals->ListOfGoals[4].taken = false;
	//1 pts
	cvs->Goals->ListOfGoals[5].X = 0.38;
	cvs->Goals->ListOfGoals[5].Y = (color == BLUE || color == RED) ? 1.15 : -1.15;
	cvs->Goals->ListOfGoals[5].taken = false;
	//1 pts
	cvs->Goals->ListOfGoals[6].X = -0.65;
	cvs->Goals->ListOfGoals[6].Y = (color == BLUE || color == RED) ? 1.25 : -1.25;
	cvs->Goals->ListOfGoals[6].taken = false;

	cvs->Goals->precision = goalprecision_INIT;
	cvs->Goals->CurrentGoal = firstgoal_INIT;
	cvs->Goals->previousGoal = firstgoal_INIT;
	cvs->Goals->via = false;
	cvs->Goals->endConstr = false;
	cvs->Goals->inConstr = false;
	cvs->Goals->lockState = -1;
	cvs->Goals->endParcourt = false;
	cvs->Goals->backHome = false;
	cvs->Goals->timeIN = 0.0;
	cvs->Goals->maxtimewait =  maxtimewait_INIT;
	cvs->Goals->isblocked = false;
	cvs->Goals->goalIN = 0;
	cvs->Goals->over = false;
	cvs->Goals->nbr_target_prev = 0;
}


void InitObstacles(CtrlStruct *cvs) {
	cvs->Obstacles = (Obstacles*)malloc(sizeof(Obstacles));
	cvs->Obstacles->NumberOfCircles = NumberOfCircles_INIT;
	cvs->Obstacles->NumberOfRectangles = NumberOfRectangles_INIT;
	cvs->Obstacles->NumberOfQuarterOfCircle = NumberOfQuarterOfCircle_INIT;

	cvs->Obstacles->CircleList = (Circle*)malloc(sizeof(Circle)*NumberOfCircles_INIT);
	cvs->Obstacles->RectangleList = (Rectangle*)malloc(sizeof(Rectangle)*NumberOfRectangles_INIT);
	cvs->Obstacles->QuarterOfCircleList = (QuarterOfCircle*)malloc(sizeof(QuarterOfCircle)*NumberOfQuarterOfCircle_INIT);
	//EnnemyBot
	cvs->Obstacles->CircleList[0].isActive = true;
	cvs->Obstacles->CircleList[0].radius = 0.2;
	cvs->Obstacles->CircleList[0].x = -2;
	cvs->Obstacles->CircleList[0].y = -2;

	cvs->Obstacles->CircleList[1].isActive = true;
	cvs->Obstacles->CircleList[1].radius = 0.2;
	cvs->Obstacles->CircleList[1].x = -2;
	cvs->Obstacles->CircleList[1].y = -2;

	cvs->Obstacles->CircleList[2].isActive = true;
	cvs->Obstacles->CircleList[2].radius = 0.2;
	cvs->Obstacles->CircleList[2].x = -2;
	cvs->Obstacles->CircleList[2].y = -2;

	int color = cvs->robotID;
	//MidAreaGreenSide
	cvs->Obstacles->QuarterOfCircleList[0].isActive = ((color == WHITE) || (color == YELLOW)) ? false : true;
	cvs->Obstacles->QuarterOfCircleList[0].quadrant = 1;
	cvs->Obstacles->QuarterOfCircleList[0].radius = 0.6;
	cvs->Obstacles->QuarterOfCircleList[0].x = -0.25;
	cvs->Obstacles->QuarterOfCircleList[0].y = 0;
	//MidAreaBlueSide
	cvs->Obstacles->QuarterOfCircleList[1].isActive = ((color == BLUE) || (color == RED)) ? false : true;
	cvs->Obstacles->QuarterOfCircleList[1].quadrant = 4;
	cvs->Obstacles->QuarterOfCircleList[1].radius = 0.6;
	cvs->Obstacles->QuarterOfCircleList[1].x = -0.25;
	cvs->Obstacles->QuarterOfCircleList[1].y = 0;

	//MidWall
	cvs->Obstacles->RectangleList[0].isActive = true;
	cvs->Obstacles->RectangleList[0].x = -0.2369;
	cvs->Obstacles->RectangleList[0].y = 0;
	cvs->Obstacles->RectangleList[0].width = 0.022;
	cvs->Obstacles->RectangleList[0].height = 1.2;
	//MidEdge1
	cvs->Obstacles->RectangleList[1].isActive = true;
	cvs->Obstacles->RectangleList[1].x = 0.05;
	cvs->Obstacles->RectangleList[1].y = 0.0;
	cvs->Obstacles->RectangleList[1].width = 0.6;
	cvs->Obstacles->RectangleList[1].height = 0.022;
	//MidEdge2
	cvs->Obstacles->RectangleList[2].isActive = true;
	cvs->Obstacles->RectangleList[2].x = -0.228;
	cvs->Obstacles->RectangleList[2].y = 0.0;
	cvs->Obstacles->RectangleList[2].width = 0.044;
	cvs->Obstacles->RectangleList[2].height = 0.048;
	//MidEdge3
	cvs->Obstacles->RectangleList[3].isActive = true;
	cvs->Obstacles->RectangleList[3].x = 0.328;
	cvs->Obstacles->RectangleList[3].y = 0.0;
	cvs->Obstacles->RectangleList[3].width = 0.044;
	cvs->Obstacles->RectangleList[3].height = 0.048;
	//SandEdge1
	cvs->Obstacles->RectangleList[4].isActive = true;
	cvs->Obstacles->RectangleList[4].x = -0.9;
	cvs->Obstacles->RectangleList[4].y = -0.689;
	cvs->Obstacles->RectangleList[4].width = 0.2;
	cvs->Obstacles->RectangleList[4].height = 0.022;
	//SandEdge2
	cvs->Obstacles->RectangleList[5].isActive = true;
	cvs->Obstacles->RectangleList[5].x = -0.9;
	cvs->Obstacles->RectangleList[5].y = 0.689;
	cvs->Obstacles->RectangleList[5].width = 0.2;
	cvs->Obstacles->RectangleList[5].height = 0.022;

	double offset = 0.1;
	//LeftEdge
	cvs->Obstacles->RectangleList[6].isActive = true;
	cvs->Obstacles->RectangleList[6].x = -1.0 - offset;
	cvs->Obstacles->RectangleList[6].y = 0.0;
	cvs->Obstacles->RectangleList[6].width = 0.01;
	cvs->Obstacles->RectangleList[6].height = 3.0;
	//RightEdge
	cvs->Obstacles->RectangleList[7].isActive = true;
	cvs->Obstacles->RectangleList[7].x = 1.0 + offset;
	cvs->Obstacles->RectangleList[7].y = 0.0;
	cvs->Obstacles->RectangleList[7].width = 0.01;
	cvs->Obstacles->RectangleList[7].height = 3.0;
	//TopEdge
	cvs->Obstacles->RectangleList[8].isActive = true;
	cvs->Obstacles->RectangleList[8].x = 0.0;
	cvs->Obstacles->RectangleList[8].y = 1.5 + offset;
	cvs->Obstacles->RectangleList[8].width = 2.0;
	cvs->Obstacles->RectangleList[8].height = 0.01;
	//BottomEdge
	cvs->Obstacles->RectangleList[9].isActive = true;
	cvs->Obstacles->RectangleList[9].x = 0.0;
	cvs->Obstacles->RectangleList[9].y = -1.5 - offset;
	cvs->Obstacles->RectangleList[9].width = 2.0;
	cvs->Obstacles->RectangleList[9].height = 0.01;
}

void InitDyna(CtrlStruct *cvs){
#ifdef REALBOT
    cvs->DynaLeft = (Dyna*)malloc(sizeof(Dyna));
    cvs->DynaLeft->timer = 0;
    cvs->DynaLeft->enable = false;
    cvs->DynaLeft->stateDyna = grap;
    
    cvs->DynaRight = (Dyna*)malloc(sizeof(Dyna));
    cvs->DynaRight->timer = 0;
    cvs->DynaRight->enable = false;
    cvs->DynaRight->stateDyna = grap;
    
    SendMessageDyna(0x06,0x0005,0x0008,0x0000);
    SendMessageDyna(0x06,0x0005,0x0019,0x1);
#endif
}


#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT