/*!
* \file odometry_gr2.cc
* \
*/

#include "odometry_gr2.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

void OdometryLoop(CtrlStruct *cvs) {
	double dbaseline = cvs->Param->width;
	double timeStep = cvs->timeStep;

	double dl = cvs->Odo->speedL*cvs->Param->wheelLRadius*timeStep;
	double dr = cvs->Odo->speedR*cvs->Param->wheelRRadius*timeStep;
	double dc = (dl + dr) / 2;
	double phi = (dr - dl) / dbaseline;

	cvs->Odo->x += dc * cos(DEGtoRAD*cvs->Odo->theta);
	cvs->Odo->y += dc * sin(DEGtoRAD*cvs->Odo->theta);
	cvs->Odo->theta += RADtoDEG*phi;


	if (cvs->Odo->theta > 180) {
		cvs->Odo->theta = cvs->Odo->theta - 360;
	}
	else if (cvs->Odo->theta < -180) {
		cvs->Odo->theta = cvs->Odo->theta + 360;
	}

    /*char theStr[64];
    sprintf(theStr,"speedL =  %f \t speedR = %f \t x = %f \t y = %f \t", cvs->Odo->speedL, cvs->Odo->speedR, cvs->Odo->x, cvs->Odo->y);
    MyConsole_SendMsg(theStr);*/

	//printf("omega = %f \t v = %f \t OdoX = %f\n", omega, v, cvs->Odo->x);
	//printf("theta = %f \t x = %f \t y = %f \n", cvs->Odo->theta, cvs->Odo->x, cvs->Odo->y);
	//set_plot(cvs->Odo->theta, "Theta");

}

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT