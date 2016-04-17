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
    cvs->robotID = PINK; //getRobotID();
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
	InitGoals(cvs);
    InitDyna(cvs);
	int color = cvs->robotID;
	cvs->stateCalib = Cal_y_arr;
	cvs->stateReCalib = ReCal_rot1;
	cvs->stateStrat = reachPointA;
    cvs->stateHomologation = PinceCalib;
    cvs->stateAction1 = GoToHouses;
    cvs->stateAction2 = GoToBlocOne;
    cvs->stateAction3 = GoToBlocTwoCalib;
    cvs->stateAction4 = GoToFish;
    cvs->stateStrategy = GoCalibration;
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
    /*
    if(!var1Ok){
        bool var1 = ReachPointPotential(cvs,-0.6,0.0,0.1);
        var1Ok = var1;
    }
    else{
        if(!var2Ok){
            bool var2 = ReachPointPotential(cvs,-0.6,-1.2,0.1);
            var2Ok = var2;
        }
        else{
            if(!var3Ok){
                bool var3 = ReachPointPotential(cvs,0.0,-1.0,0.1);
                var3Ok = var3;
            }
            else{
                if(!var4Ok){
                    bool var4 = ReachPointPotential(cvs,0.0,-0.25,0.1);
                    var4Ok = var4;
                }
                else{
                    if(!var5Ok){
                    bool var5 = ReachPointPotential(cvs,0.4,-1.1,0.1);
                        var5Ok = var5;
                    }
                    else{
                        if(!var6Ok){
                            bool var6 = ReachPointPotential(cvs,0.8,-0.8,0.1);
                            var6Ok = var6;
                        }
                        else{
                            if(!var7Ok){
                                bool var7 = ReachPointPotential(cvs,0.0,-0.25,0.1);
                                var7Ok = var7;
                            }
                        }
                    }
                }
            }
        }
    }*/
    //StrategyTest(cvs);
    //PointHomologation(cvs);

    
   cvs->Param->MotorCommandByHand = MotorCommandByHand;
   if(cvs->Param->MotorCommandByHand)
   {
    cvs->MotorL->dutyCycle = LeftMotorDC;//RightMotorDC;
    cvs->MotorR->dutyCycle = RightMotorDC;// RightMotorDC;
    cvs->MotorTower->dutyCycle = TourelleDC;
    cvs->MotorRatL->dutyCycle = RateauLDC; //RightMotorDC;//RightMotorDC;
    cvs->MotorRatR->dutyCycle = RateauRDC; //RightMotorDC;//RightMotorDC;
<<<<<<< HEAD
    cvs->MotorPince->dutyCycle = PinceDC;//RightMotorDC;*/   
   }
   else if(cvs->time > 200){
       cvs->MotorL->dutyCycle = 0;//RightMotorDC;
        cvs->MotorR->dutyCycle = 0;// RightMotorDC;
        cvs->MotorTower->dutyCycle = 0;
        cvs->MotorRatL->dutyCycle = 0; //RightMotorDC;//RightMotorDC;
        cvs->MotorRatR->dutyCycle = 0; //RightMotorDC;//RightMotorDC;
        cvs->MotorPince->dutyCycle = 0;//RightMotorDC;*/
   }
   else{

          //   StartMyRat(cvs);

                   /*char s[659];
       sprintf(s,"time = %f \t speedL = %f \t speedR = %f \t x = %f  \t y = %f \theta = %f \n", cvs->time, cvs->MotorL->speed, cvs->MotorR->speed, cvs->Odo->x, cvs->Odo->y, cvs->Odo->theta);
        MyConsole_SendMsg(s);*/
     //  Calibration(cvs);

    //   ReachPointPotential(cvs, 0.8, 0.8, 0.03);
        DynaTestFunction(cvs);
     // MyStrategy(cvs);
       
   }
    //PinceCalibration(cvs);

=======
    cvs->MotorPince->dutyCycle = PinceDC;//RightMotorDC;
    

    SpeedRefToDC(cvs,cvs->MotorR, 3*M_PI);
    SpeedRefToDC(cvs,cvs->MotorL, 3*M_PI);*/
    SpeedRefToDC(cvs, cvs->MotorTower, TourelleDC);
    
>>>>>>> master
	AlwaysEndController(cvs);
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

void AlwaysEndController(CtrlStruct *cvs) {
	SendMotorCommand(cvs);
}

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // !REALBOT