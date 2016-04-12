#include "tower_gr2.h"


#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT


/*********************
*** FUNCTIONS ********
**********************/
void OpponentDetection(CtrlStruct *cvs)
{
    cvs->MotorTower->dutyCycle = -80;
	UpdateDetectedBotPosition(cvs);
}


void UpdateDetectedBotPosition(CtrlStruct *cvs) {
	int numberDetected = cvs->Tower->nb_rising;//fmin(cvs->Tower->nb_rising, cvs->Tower->nb_falling);
	int i;
	int currentBot = 0;
	for (i = 0; i < numberDetected; i++) { //Loop to add coordinates of each bot
		int risingIndex = cvs->Tower->rising_index - i;
		if (risingIndex < 0) 
			risingIndex = risingIndex + NB_STORE_EDGE;

		int fallingIndex = cvs->Tower->falling_index - i;
		if (fallingIndex < 0)
			fallingIndex = fallingIndex + NB_STORE_EDGE;

		double angleFromTower = ComputeAngle(cvs, risingIndex, fallingIndex);
		double distanceFromTower = ComputeDistance(cvs, risingIndex, fallingIndex);

		double distance = distanceFromTower;// - cvs->Param->radiusBot; // sqrt(distanceFromTower*distanceFromTower + cvs->Param->lengthTower*cvs->Param->lengthTower - 2 * distanceFromTower*cvs->Param->lengthTower*cos(DEGtoRAD*(180 - angleFromTower))); //Al Kaishi
		double angle = angleFromTower; // RADtoDEG*asin(distanceFromTower*sin(M_PI / 2 - angleFromTower) / distance);
		cvs->Tower->distance = distance;
        cvs->Tower->angle = angle;
        double x = cvs->Odo->x + distance*cos(DEGtoRAD * (cvs->Odo->theta + angle));
		double y = cvs->Odo->y + distance*sin(DEGtoRAD * (cvs->Odo->theta + angle));
  
		//if (IsBot(x, y)) { //TODECOMMENT
			if (currentBot < NumberOfCircles_INIT) { //If glitch or too much bots
				cvs->Obstacles->CircleList[currentBot].x = x;
				cvs->Obstacles->CircleList[currentBot].y = y;
				cvs->Obstacles->CircleList[currentBot].isActive = true;
                
				currentBot++;
			}
		//}
	}
	for (i = currentBot; i < NumberOfCircles_INIT; i++) { //Deactivate the bots that have not been detected
		cvs->Obstacles->CircleList[i].isActive = false;
	}
}

//Not useful here since the external beacons are not detected, but might be really useful for the cup (glitch).
bool IsBot(double x, double y) {
	if ((x + BEACON_POSITION_TOLERANCE > 1) || (x - BEACON_POSITION_TOLERANCE < -1))
			return false;//Outside map
	else if ((y + BEACON_POSITION_TOLERANCE > 1.5) || (y - BEACON_POSITION_TOLERANCE < -1.5))
			return false;//Outside map

	else if ((x + BEACON_POSITION_TOLERANCE > 0) && (x - BEACON_POSITION_TOLERANCE < 0)) {
		if ((y + BEACON_POSITION_TOLERANCE > -1.562) && (y - BEACON_POSITION_TOLERANCE < -1.562))
			return false;//Top Mid Beacon
		else if ((y + BEACON_POSITION_TOLERANCE > 1.562) && (y - BEACON_POSITION_TOLERANCE < 1.562))
			return false;//Bottom Mid Beacon
	}

	else if ((x + BEACON_POSITION_TOLERANCE > 1.062) && (x - BEACON_POSITION_TOLERANCE < 1.062)) {
		if ((y + BEACON_POSITION_TOLERANCE > -1.562) && (y - BEACON_POSITION_TOLERANCE < -1.562))
			return false;//Top Right Beacon
		else if ((y + BEACON_POSITION_TOLERANCE > 1.562) && (y - BEACON_POSITION_TOLERANCE < 1.562))
			return false;//Bottom Right Beacon
	}

	else if ((x + BEACON_POSITION_TOLERANCE > -1.062) && (x - BEACON_POSITION_TOLERANCE < -1.062)) {
		if ((y + BEACON_POSITION_TOLERANCE > -1.562) && (y - BEACON_POSITION_TOLERANCE < -1.562))
			return false;//Top Left Beacon
		else if ((y + BEACON_POSITION_TOLERANCE > 1.562) && (y - BEACON_POSITION_TOLERANCE < 1.562))
			return false;//Bottom Left Beacon
	}

	else
		return true; //This is a bot
}

double ComputeAngle(CtrlStruct *cvs, int risingIndex, int fallingIndex) {
	double firstAngle = cvs->Tower->last_rising[risingIndex];
	double lastAngle = cvs->Tower->last_falling[fallingIndex];
    double offset = 18.0;
    
	bool changeFlag = false;
	if (firstAngle > M_PI / 2 && lastAngle < -M_PI / 2) {
		lastAngle = lastAngle + 2 * M_PI;
		changeFlag = true;
	}
	else if (firstAngle < -M_PI/2 && lastAngle > M_PI / 2) {
		firstAngle = firstAngle + 2 * M_PI;
		changeFlag = true;
	}
	double angleFromTower = (firstAngle + lastAngle) / 2;
	if (changeFlag) angleFromTower = angleFromTower - 2 * M_PI;
	return  RADtoDEG*angleFromTower - offset;
}

double ComputeDistance(CtrlStruct *cvs, int risingIndex, int fallingIndex) {
	double firstAngle = cvs->Tower->last_rising[risingIndex];
	double lastAngle = cvs->Tower->last_falling[fallingIndex];
    double offset = 0.15;// rayonbeacon + distance entre la tourelle et le bord du robot?
	if (firstAngle > 0 && lastAngle < 0)
		lastAngle = lastAngle + 2 * M_PI;
	else if (firstAngle < 0 && lastAngle > 0)
		firstAngle = firstAngle + 2 * M_PI;
	double distanceFromTower = cvs->Param->rayonBeacon / sin(fabs(lastAngle - firstAngle) / 2);
              /*    
        char theStr[1024];
        sprintf(theStr,"distanceFromTower = %f \t \n", distanceFromTower);
        MyConsole_SendMsg(theStr);
        */
    if(distanceFromTower > 3)
        distanceFromTower = 3;
    else if(distanceFromTower < 0)
        distanceFromTower = EPSILON;
	return distanceFromTower;
}

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT