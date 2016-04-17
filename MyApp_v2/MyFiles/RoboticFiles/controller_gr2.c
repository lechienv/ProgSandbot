#include "controller_gr2.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

double limitDC(double DC) {
	if (DC > 100) return 100;
	else if (DC < -100) return -100;
	else return DC;
}

//Return a speed;
double limitXAcceleration(CtrlStruct *cvs, double speedRef){
    double xSpeed = cvs->Param->wheelRadius*(cvs->MotorL->speed + cvs->MotorR->speed)/2;
    double currentAcceleration = (cvs->Param->wheelRadius*speedRef - xSpeed) / cvs->timeStep;
    double limit = cvs->Param->maxAcceleration;
	if (currentAcceleration < limit)
		if (currentAcceleration > -limit)
			return speedRef;
		else
			return ((-limit*cvs->timeStep) + xSpeed)/cvs->Param->wheelRadius; 
	else
		return (limit*cvs->timeStep + xSpeed)/cvs->Param->wheelRadius;
}

void SpeedRefToDC(CtrlStruct *cvs, Motor *Motor, double speedRef)
{    
	double errorSpeed = speedRef - Motor->speed;
	if (Motor->totalError > KIFLUSHLIMIT)
		Motor->totalError = 0;
	double voltage = (Motor->Kp*errorSpeed + (Motor->Ki)*Motor->totalError + Motor->Kphi*Motor->speed)*Motor->reverseRotation;
    
	Motor->dutyCycle += voltage*VOLTtoDC;
	Motor->totalError += errorSpeed*cvs->timeStep;
    /*
    char theStr[128];
    sprintf(theStr, "errorSpeed = %f \t dutyCYcleCommand = %f \t totalError = %f \t timestep = %f \t \n",errorSpeed, voltage, Motor->totalError, cvs->timeStep);
    MyConsole_SendMsg(theStr);*/
}

bool IsAlignedWithTheta(CtrlStruct *cvs, double thetaRef, double anglePrecision) {
	//printf("speed = %f, theta = %f thetaref = %f \n", fabs(cvs->MotorL->speed - cvs->MotorR->speed), cvs->Odo->theta, thetaRef);
	// To stop the function. Condition on position and condition on rotational speed (Ki might have overshoot)
	while (thetaRef > 180)
		thetaRef = thetaRef - 360;
	while (thetaRef < -180)
		thetaRef = thetaRef + 360;
	if ((fabs(cvs->Odo->theta - thetaRef) < anglePrecision) && (fabs(cvs->MotorL->speed - cvs->MotorR->speed) < cvs->Param->speedDifThreshold)) {
		cvs->Param->totalErrorRot = 0;
		return true;
	}
	else {
		// To change rotation side
		if (thetaRef - cvs->Odo->theta >= 180)
			thetaRef = thetaRef - 360;
		else if (thetaRef - cvs->Odo->theta < -180)
			thetaRef = thetaRef + 360;
		double errorAngle = thetaRef - cvs->Odo->theta;
		if (cvs->Param->totalErrorRot > KIFLUSHLIMIT)
			cvs->Param->totalErrorRot = 0;
		double speedRefW = cvs->Param->KpRot * errorAngle + cvs->Param->KiRot * cvs->Param->totalErrorRot;
        double speedRefX = 0;
        if(LIMITACCELERATION){
            speedRefX = limitXAcceleration(cvs,0);
        }
        SpeedRefToDC(cvs,cvs->MotorL,speedRefX - speedRefW);
        SpeedRefToDC(cvs,cvs->MotorR,speedRefX + speedRefW);
        
		cvs->Param->totalErrorRot += (fabs(errorAngle) <  cvs->Param->KiAngleThreshold) ? errorAngle * (cvs->timeStep) : 0;
		return false;
	}
}

void SendMotorCommand(CtrlStruct *cvs) {
#ifdef REALBOT
#ifndef MINIBOT
// PINCES
   if((cvs->MotorPince->dutyCycle == 0)){
        if((!cvs->MotorPince->areBrakesEnabled)){
            EnableBrakes(cvs->MotorPince);
        }
    }
    else{
        DisableBrakes(cvs->MotorPince);
        if((cvs->MotorPince->previousDutyCycle != cvs->MotorPince->dutyCycle)){
            if(cvs->MotorPince->dutyCycle > 50 )
            {
                cvs->MotorPince->dutyCycle = 50;
            }
            if(cvs->MotorPince->dutyCycle < -50 )
            {
                cvs->MotorPince->dutyCycle = -50;
            }
            cvs->MotorPince->previousDutyCycle = cvs->MotorPince->dutyCycle;       
            ActivateMotor_RealBot(cvs->MotorPince);
        }
    }
// RATEAULeft
    if((cvs->MotorRatL->dutyCycle == 0)){
        if((!cvs->MotorRatL->areBrakesEnabled)){
            EnableBrakes(cvs->MotorRatL);
        }
    }
    else{
        DisableBrakes(cvs->MotorRatL);
            if(cvs->MotorRatL->dutyCycle > 50 )
            {
                cvs->MotorRatL->dutyCycle = 50;
            }
            if(cvs->MotorRatL->dutyCycle < -50 )
            {
                cvs->MotorRatL->dutyCycle = -50;
            }
        if((cvs->MotorRatL->previousDutyCycle != cvs->MotorRatL->dutyCycle)){
            cvs->MotorRatL->previousDutyCycle = cvs->MotorRatL->dutyCycle;       
            ActivateMotor_RealBot(cvs->MotorRatL);
        }
    }
// RATEAUR
    if((cvs->MotorRatR->dutyCycle == 0)){
        if((!cvs->MotorRatR->areBrakesEnabled)){
            EnableBrakes(cvs->MotorRatR);
        }
    }
    else{
        DisableBrakes(cvs->MotorRatR);
            if(cvs->MotorRatR->dutyCycle > 50 )
            {
                cvs->MotorRatR->dutyCycle = 50;
            }
            if(cvs->MotorRatR->dutyCycle < -50 )
            {
                cvs->MotorRatR->dutyCycle = -50;
            }
        if((cvs->MotorRatR->previousDutyCycle != cvs->MotorRatR->dutyCycle)){
            cvs->MotorRatR->previousDutyCycle = cvs->MotorRatR->dutyCycle;       
            ActivateMotor_RealBot(cvs->MotorRatR);
        }
    }
#endif
   
// Motor LEFT
    if((cvs->MotorL->dutyCycle == 0)){
        if((!cvs->MotorL->areBrakesEnabled)){
            EnableBrakes(cvs->MotorL);
        }
    }
    else{
        DisableBrakes(cvs->MotorL);
        if((cvs->MotorL->previousDutyCycle != cvs->MotorL->dutyCycle)){
            cvs->MotorL->previousDutyCycle = cvs->MotorL->dutyCycle;       
            ActivateMotor_RealBot(cvs->MotorL);
        }
    }
   
//  Motor RIGHT
    if((cvs->MotorR->dutyCycle == 0)){
        if((!cvs->MotorR->areBrakesEnabled)){
            EnableBrakes(cvs->MotorR);
        }
    }
    else{
        DisableBrakes(cvs->MotorR);
        if((cvs->MotorR->previousDutyCycle != cvs->MotorR->dutyCycle)){
            cvs->MotorR->previousDutyCycle = cvs->MotorR->dutyCycle;       
            ActivateMotor_RealBot(cvs->MotorR);
        }
    }
    
// TOWER
    if((cvs->MotorTower->dutyCycle == 0)){
        if((!cvs->MotorTower->areBrakesEnabled)){
            EnableBrakes(cvs->MotorTower);
        }
    }
    else{
        DisableBrakes(cvs->MotorTower);
        if((cvs->MotorTower->previousDutyCycle != cvs->MotorTower->dutyCycle)){
            cvs->MotorTower->previousDutyCycle = cvs->MotorTower->dutyCycle;       
            ActivateMotor_RealBot(cvs->MotorTower);
        }
    }
    
#else
    cvs->outputs->wheel_commands[cvs->MotorL->ID] = limitDC(cvs->MotorL->dutyCycle);
	cvs->outputs->wheel_commands[cvs->MotorR->ID] = limitDC(cvs->MotorR->dutyCycle);
	cvs->outputs->tower_command = limitDC(cvs->MotorTower->dutyCycle);
#endif // REALBOT
}


#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT