/*! 
 * \file ctrl_main_gr2.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "ctrl_main_gr2.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

/*! \brief initialize controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
    
void controller_init(CtrlStruct *cvs){	
    cvs->previousTimeCAN = 0;
    cvs->timeOffset = 0;
#ifdef REALBOT
    cvs->timeStep = TIMESTEP_REALBOT;
#else
    cvs->robotID = cvs->inputs->robot_id;
    cvs->timeStep = 0.001;
#endif // REALBOT
	InitParam(cvs);
	InitMotor(cvs);
	InitPotential(cvs);
	InitOdometry(cvs);
	InitSensors(cvs);
	InitObstacles(cvs);
	InitTower(cvs);
    InitDyna();
    InitTowerFilters(cvs);
    InitTimer(cvs);

	int color = cvs->robotID;
	cvs->stateCalib = Cal_y_arr;
    cvs->stateHomologation = PinceCalib;
    cvs->stateAction1 = GoToHouse1;// FreeHouse1;//
    cvs->stateAction2 = GoToBlocOne;
    cvs->stateAction3 = CalibY;
    cvs->stateAction4 = GoCalibY_Action4;
    cvs->stateAction5 = GotoDuneViaPoint;
    //cvs->stateStrategy =  (cvs->Tower->StrategyWithRushDunes) ? GoAction5 : GoAction2; //GoAction4;// 
    cvs->stateStrategy = GoAction2;
#ifdef REALBOT
    InitRegMotor(cvs->MotorL);
    InitRegMotor(cvs->MotorR);
    InitRegMotor(cvs->MotorRatL);
    InitRegMotor(cvs->MotorRatR);
    InitRegMotor(cvs->MotorPince);
    InitRegMotor(cvs->MotorTower);
#endif // REALBOT
    AlwaysInController(cvs);
}

/*! \brief controller loop (called every timestep)
 * 
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs){
	AlwaysInController(cvs);
    int endtime = 120;
    int starttime = 0;
//#define WEBSITETEST
#ifndef WEBSITETEST
    cvs->Param->MotorCommandByHand = CommandMotorByHand;
    if(cvs->Param->MotorCommandByHand){
        cvs->MotorL->dutyCycle = LeftMotorDC;
        cvs->MotorR->dutyCycle = RightMotorDC;
        cvs->MotorTower->dutyCycle = TourelleDC;
        cvs->MotorRatL->dutyCycle = RateauLDC; 
        cvs->MotorRatR->dutyCycle = RateauRDC; 
        cvs->MotorPince->dutyCycle = PinceDC;
    }
   /* else if(cvs->time <= starttime)
    {
        cvs->MotorL->dutyCycle = 0;
        cvs->MotorR->dutyCycle = 0;
        cvs->MotorTower->dutyCycle = 0;
        cvs->MotorRatL->dutyCycle = 0; 
        cvs->MotorRatR->dutyCycle = 0; 
        cvs->MotorPince->dutyCycle = 0;
    }*/
    else if( cvs->time >= endtime +0.5 && cvs->time < endtime + 3.5){
        cvs->MotorL->dutyCycle = 0;
        cvs->MotorR->dutyCycle = 0;
        cvs->MotorTower->dutyCycle = 0;
        cvs->MotorRatL->dutyCycle = 0; 
        cvs->MotorRatR->dutyCycle = 0; 
        cvs->MotorPince->dutyCycle = 0;
        ActionParasol(cvs);   
    }
    else if(cvs->time >= endtime){
        cvs->MotorL->dutyCycle = 0;
        cvs->MotorR->dutyCycle = 0;
        cvs->MotorTower->dutyCycle = 0;
        cvs->MotorRatL->dutyCycle = 0; 
        cvs->MotorRatR->dutyCycle = 0; 
        cvs->MotorPince->dutyCycle = 0;
    }
    else{ //During match
        //ChooseStratDuneOrNot(cvs);
        ChooseBetweenMatchOrTest(cvs);
        if(!cvs->Param->ChooseToMatch)
        {
            
            getTests(cvs);
            cvs->timeOffset = getTime();
            cvs->previousTime = 0;
            cvs->time = 0;
        }
        else
        {          
                getStrategy(cvs);

        }
        
        
        //SpeedRefToDC(cvs,cvs->MotorR,5);
        
        //SpeedRefToDC(cvs,cvs->MotorL,5);
        //ChooseBetweenMatchOrTest(cvs);
        //cvs->Param->ChooseToMatch
        //MyStrategy(cvs, 1);
        //Action4(cvs);
        //MyConsole_SendMsg("demarrage\n");
      //  MyStrategy(cvs, 1);
      
        /*if(cvs->time>=5){
        SpeedRefToDC(cvs,cvs->MotorR,8);
        SpeedRefToDC(cvs,cvs->MotorL,8);
        }
        else{
          SpeedRefToDC(cvs,cvs->MotorR,4);
        SpeedRefToDC(cvs,cvs->MotorL,4);           
        }*/
        
   /*unsigned int A = MyCyclone_Read(CYCLONE_IO_A_Data);
     char theStr[256];
        sprintf(theStr, "speedG = %f \t speedDroite = %f \t mystart = %d  \n", cvs->MotorL->speed, cvs->MotorR->speed, (bool) extractBits(A,13,13));
    MyConsole_SendMsg(theStr);*/
    }
       
#else
    StrategyTest(cvs);
    cvs->Tower->ActivateTooClose = true;
    if(cvs->Tower->ActivateTooClose){
        int i;
        for(i = 0; i < cvs->AllFiltersTower->numberOfEnnemy; i++){
            // Case "Dection Ahead"
            if(cvs->AllFiltersTower->FilterTowerList[i].tooCloseAhead){
                if(((cvs->MotorL->dutyCycle > 0) || (cvs->MotorR->dutyCycle > 0)) && (cvs->MotorR->dutyCycle != -cvs->MotorL->dutyCycle)){ //Robot tends to go ahead and does not move on itself
                    cvs->MotorL->dutyCycle = 0;
                    cvs->MotorR->dutyCycle = 0;
                    cvs->MotorL->totalError = 0;
                    cvs->MotorR->totalError = 0;
                }
            }
            // Case "Dection Behind"
            else if(cvs->AllFiltersTower->FilterTowerList[i].tooCloseBehind){
                if(((cvs->MotorL->dutyCycle < 0) || (cvs->MotorR->dutyCycle < 0)) && (cvs->MotorR->dutyCycle != -cvs->MotorL->dutyCycle)){ //Robot tends to go behind and not move on itself
                    cvs->MotorL->dutyCycle = 0;
                    cvs->MotorR->dutyCycle = 0;
                    cvs->MotorL->totalError = 0;
                    cvs->MotorR->totalError = 0;
                }
            }
        }
    }
#endif

	AlwaysEndController(cvs, endtime);
}


/*! \brief last controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{
	free(cvs->Odo);
	free(cvs->Poto);
	free(cvs->Param);
	free(cvs->Sensors);
	free(cvs->Tower);
	free(cvs->MotorL);
	free(cvs->MotorR);
	free(cvs->MotorTower);
	free(cvs->Obstacles->CircleList);
	free(cvs->Obstacles->RectangleList);
	free(cvs->Obstacles->QuarterOfCircleList);
	free(cvs->Obstacles);
	free(cvs->Goals->ListOfGoals);
	free(cvs->Goals);
    free(cvs->AllFiltersTower->FilterTowerList);
	free(cvs->AllFiltersTower);
    free(cvs->TimerAction);
    free(cvs->TimerCalibration);
#ifdef REALBOT
    free(cvs->MotorRatL);
    free(cvs->MotorRatR);
    free(cvs->MotorPince);
	free(cvs->DynaLeft);
	free(cvs->DynaRight);
#endif
}

void UpdateFromFPGA(CtrlStruct *cvs) {
    #ifndef REALBOT
    cvs->time = cvs->inputs->t;
	cvs->MotorL->speed = cvs->inputs->l_wheel_speed;
	cvs->MotorR->speed = cvs->inputs->r_wheel_speed;
    cvs->Odo->speedR = cvs->MotorR->speed;
    cvs->Odo->speedL = cvs->MotorL->speed;
	cvs->Tower->falling_index = cvs->inputs->falling_index;
	cvs->Tower->falling_index_fixed = cvs->inputs->falling_index_fixed;
	
	int i;
	for (i = 0; i <  NB_STORE_EDGE ; i++) {
		cvs->Tower->last_falling[i] = cvs->inputs->last_falling[i];
		cvs->Tower->last_falling_fixed[i] = cvs->inputs->last_falling[i];
		cvs->Tower->last_rising[i] = cvs->inputs->last_rising[i];
		cvs->Tower->last_rising_fixed[i] = cvs->inputs->last_rising_fixed[i];
	}

	cvs->Tower->nb_falling = cvs->inputs->nb_falling;
	cvs->Tower->nb_falling_fixed = cvs->inputs->nb_falling_fixed;
	cvs->Tower->nb_opponents = cvs->inputs->nb_opponents;
	cvs->Tower->nb_rising = cvs->inputs->nb_rising;
	cvs->Tower->nb_rising_fixed = cvs->inputs->nb_rising_fixed;
	cvs->Tower->tower_pos = cvs->inputs->tower_pos;
	cvs->MotorTower->speed = fabs(cvs->Tower->tower_pos - cvs->Tower->tower_prevPos) / cvs->timeStep;
	cvs->Tower->rising_index = cvs->inputs->rising_index;
	cvs->Tower->rising_index_fixed = cvs->inputs->rising_index_fixed;
	cvs->Sensors->uSwitchLeft = cvs->inputs->u_switch[L_ID];
	cvs->Sensors->uSwitchRight = cvs->inputs->u_switch[R_ID];
    #endif // ! REALBOT
}


void AlwaysInController(CtrlStruct *cvs) {

#ifndef REALBOT
	UpdateFromFPGA(cvs);
  //  OpponentDetection(cvs); //NEED TO BE IN TWO
#else 
    UpdateFromFPGARealBot(cvs);
#endif // ! REALBOT
    cvs->timeStep = cvs->time - cvs->previousTime;
    cvs->previousTime = cvs->time;
    OdometryLoop(cvs);
	cvs->MotorL->dutyCycle	= 0; //to be sure that the motor is not starting
	cvs->MotorR->dutyCycle	= 0;
	cvs->MotorTower->dutyCycle = 0;

	cvs->MotorL->position += cvs->MotorL->speed*cvs->timeStep;
	cvs->MotorR->position += cvs->MotorR->speed*cvs->timeStep;
	cvs->MotorTower->position += cvs->MotorTower->speed*cvs->timeStep;
    
#ifdef REALBOT
    cvs->MotorRatL->dutyCycle = 0;
    cvs->MotorRatR->dutyCycle = 0;
    cvs->MotorPince->dutyCycle = 0;

	cvs->MotorPince->position += cvs->MotorPince->speed*cvs->timeStep; // *cvs->Param->PasFiletVisPince/ (2*M_PI); // en mm
	cvs->MotorRatL->position += cvs->MotorRatL->speed*cvs->timeStep *cvs->Param->PasFiletVisRat / (2*M_PI); // en mm
	cvs->MotorRatR->position += cvs->MotorRatR->speed*cvs->timeStep *cvs->Param->PasFiletVisRat / (2*M_PI); // en mm
#endif //REALBOT
	OpponentDetection(cvs);
}

void AlwaysEndController(CtrlStruct *cvs, double endtime) {
#ifndef WEBSITETEST
	if(cvs->time >= endtime){
        cvs->MotorL->dutyCycle = 0;//RightMotorDC;
        cvs->MotorR->dutyCycle = 0;// RightMotorDC;
        cvs->MotorTower->dutyCycle = 0;
        cvs->MotorRatL->dutyCycle = 0; //RightMotorDC;//RightMotorDC;
        cvs->MotorRatR->dutyCycle = 0; //RightMotorDC;//RightMotorDC;
        cvs->MotorPince->dutyCycle = 0;//RightMotorDC;
    }
#endif
    SendMotorCommand(cvs);
}

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // !REALBOT