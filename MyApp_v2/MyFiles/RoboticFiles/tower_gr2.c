#include "tower_gr2.h"


#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT


/*********************
*** FUNCTIONS ********
**********************/
void OpponentDetection(CtrlStruct *cvs)
{
#ifndef REALBOT
    cvs->MotorTower->dutyCycle = 15;
#else
	cvs->MotorTower->dutyCycle = -100;
#endif
	UpdateDetectedBotPosition(cvs);
}

void UpdateDetectedBotPosition(CtrlStruct *cvs) {
#ifndef REALBOT
	if (cvs->Tower->tower_pos > 0 && cvs->Tower->tower_prevPos < 0) {
		cvs->Tower->newTurn = true;
		cvs->Tower->tower_prevPos = cvs->Tower->tower_pos;
	}
	else {
		cvs->Tower->tower_prevPos = cvs->Tower->tower_pos;
	}
#endif
	if(cvs->Tower->newTurn) {
		int j;
		for (j = 0; j < cvs->AllFiltersTower->numberOfEnnemy; j++) {
			cvs->Obstacles->CircleList[j].hasBeenUpdated = false;
		}
		cvs->Tower->newTurn = false;
		int numberDetected = cvs->Tower->nb_rising;
		int i;
		int numberUpdated = 0;
		for (i = 0; i < numberDetected; i++) { //Loop to add coordinates of each bot
			int risingIndex = cvs->Tower->rising_index - i;
			if (risingIndex < 0)
				risingIndex = risingIndex + NB_STORE_EDGE;

			int fallingIndex = cvs->Tower->falling_index - i;
			if (fallingIndex < 0)
				fallingIndex = fallingIndex + NB_STORE_EDGE;

			double angleFromTower = ComputeAngle(cvs, risingIndex, fallingIndex);
			double distanceFromTower = ComputeDistance(cvs, risingIndex, fallingIndex);
			double distance = distanceFromTower;
			double angle = angleFromTower;
            bool tooClose = (distance < MINDISTANCE_TOWER);
            
            cvs->Tower->distance = distance ;
            cvs->Tower->angle    = angle;
            
			double x = cvs->Odo->x + distance*cos(DEGtoRAD * (cvs->Odo->theta + angle));
			double y = cvs->Odo->y + distance*sin(DEGtoRAD * (cvs->Odo->theta + angle));
			if(IsBot(x, y)) { //!!! TO UNCOMMENT
				if (numberUpdated < NumberOfCircles_INIT) { //If all bots have not been updated
					FilterTowerBot(cvs, x, y, tooClose);
					numberUpdated++;
				}
			}
		}
		for (i = 0; i < NumberOfCircles_INIT; i++) { //Deactivate the bots that have not been detected
			if(!cvs->Obstacles->CircleList[i].hasBeenUpdated){
				if(cvs->AllFiltersTower->FilterTowerList[i].numberWithoutDetection >= NUMBER_WITHOUT_DETECTION_MAX){
                    cvs->Obstacles->CircleList[i].isActive = false;
                    cvs->AllFiltersTower->FilterTowerList[i].firstInit = true;
                    cvs->AllFiltersTower->FilterTowerList[i].detectedVeryClose = false;
                    cvs->AllFiltersTower->FilterTowerList[i].currentCountOutliers = 0;
                    cvs->AllFiltersTower->FilterTowerList[i].currentIndex = 0;
                    cvs->AllFiltersTower->FilterTowerList[i].numberWithoutDetection = 0;
                    int j;
                    for(j = 0; j < TOWER_AVERAGING_NUMBER; j++){
                        cvs->AllFiltersTower->FilterTowerList[i].xList[j] = - 10;
                        cvs->AllFiltersTower->FilterTowerList[i].yList[j] = - 10;
                    }
                } //Deactivate the beacon
                else {
                    cvs->AllFiltersTower->FilterTowerList[i].numberWithoutDetection++;
                } //Do other turn before deactivating the beacon
            }
		}
	}
}

void FilterTowerBot(CtrlStruct *cvs, double x, double y, bool tooClose) {
		int botNumber = FindCorrespondingBot(cvs,x,y);
        cvs->AllFiltersTower->FilterTowerList[botNumber].detectedVeryClose = tooClose;
		int currentIndex = cvs->AllFiltersTower->FilterTowerList[botNumber].currentIndex;
		if (currentIndex >= TOWER_AVERAGING_NUMBER) {
			AverageAndAddPosition(cvs, botNumber);
			currentIndex = 0;
		} //Update bot position
        
        if(cvs->AllFiltersTower->FilterTowerList[botNumber].firstInit){
            cvs->AllFiltersTower->FilterTowerList[botNumber].firstInit = false;
            cvs->Obstacles->CircleList[botNumber].x = x;
            cvs->Obstacles->CircleList[botNumber].y = y;
            cvs->Obstacles->CircleList[botNumber].isActive = true;
        } //Update bot position for first detection
        
		if (!IsPositionOutliers(cvs, botNumber, x, y)) {
			cvs->AllFiltersTower->FilterTowerList[botNumber].xList[currentIndex] = x;
			cvs->AllFiltersTower->FilterTowerList[botNumber].yList[currentIndex] = y;
			currentIndex++;
		}
		cvs->AllFiltersTower->FilterTowerList[botNumber].currentIndex = currentIndex;
}


int FindCorrespondingBot(CtrlStruct *cvs, double x, double y) {
	int index = 0;
	double previousMinError = 100000;
	double error;
	int i;
	for (i = 0; i < cvs->AllFiltersTower->numberOfEnnemy; i++) {
		if (!cvs->Obstacles->CircleList[i].hasBeenUpdated) {
			error = pow(cvs->Obstacles->CircleList[i].x - x, 2) + pow(cvs->Obstacles->CircleList[i].y - y, 2);
			if (error < previousMinError) {
				previousMinError = error;
				index = i;
			}
		}
	}
	cvs->Obstacles->CircleList[index].hasBeenUpdated = true;
	return index;
}

void AverageAndAddPosition(CtrlStruct *cvs, int botNumber) {
	double xAvg = 0;
	double yAvg = 0;
	int i;
	for (i = 0; i < TOWER_AVERAGING_NUMBER; i++) {
		xAvg += cvs->AllFiltersTower->FilterTowerList[botNumber].xList[i];
		yAvg += cvs->AllFiltersTower->FilterTowerList[botNumber].yList[i];
	}
	xAvg = xAvg / TOWER_AVERAGING_NUMBER;
	yAvg = yAvg / TOWER_AVERAGING_NUMBER;
	cvs->Obstacles->CircleList[botNumber].x = xAvg;
	cvs->Obstacles->CircleList[botNumber].y = yAvg;
	cvs->Obstacles->CircleList[botNumber].isActive = true;
}

bool IsPositionOutliers(CtrlStruct *cvs, int botNumber, double x, double y) {
	double time = 2 * M_PI / cvs->MotorTower->speed;
	double ennemySpeed = sqrt(pow(cvs->Obstacles->CircleList[botNumber].x - x, 2) + pow(cvs->Obstacles->CircleList[botNumber].y - y, 2))/time;
	if (ennemySpeed < MAXSPEED_ENNEMYBOT) {
		cvs->AllFiltersTower->FilterTowerList[botNumber].currentCountOutliers = 0;
		return false;
	}
	else { // (ennemySpeed > MAXSPEED_ENNEMYBOT)
		if (cvs->AllFiltersTower->FilterTowerList[botNumber].currentCountOutliers >= TOWER_OUTLIERS_COMPARE) {
			cvs->AllFiltersTower->FilterTowerList[botNumber].currentCountOutliers = 0;
			return false;
		}
		else {
			cvs->AllFiltersTower->FilterTowerList[botNumber].currentCountOutliers++;
			return true;
		}
	}
}

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
		else
			return true;
	}

	else if ((x + BEACON_POSITION_TOLERANCE > 1.062) && (x - BEACON_POSITION_TOLERANCE < 1.062)) {
		if ((y + BEACON_POSITION_TOLERANCE > -1.562) && (y - BEACON_POSITION_TOLERANCE < -1.562))
			return false;//Top Right Beacon
		else if ((y + BEACON_POSITION_TOLERANCE > 1.562) && (y - BEACON_POSITION_TOLERANCE < 1.562))
			return false;//Bottom Right Beacon
		else
			return true;
	}

	else if ((x + BEACON_POSITION_TOLERANCE > -1.062) && (x - BEACON_POSITION_TOLERANCE < -1.062)) {
		if ((y + BEACON_POSITION_TOLERANCE > -1.562) && (y - BEACON_POSITION_TOLERANCE < -1.562))
			return false;//Top Left Beacon
		else if ((y + BEACON_POSITION_TOLERANCE > 1.562) && (y - BEACON_POSITION_TOLERANCE < 1.562))
			return false;//Bottom Left Beacon
		else
			return true;
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
	if (firstAngle > 0 && lastAngle < 0)
		lastAngle = lastAngle + 2 * M_PI;
	else if (firstAngle < 0 && lastAngle > 0)
		firstAngle = firstAngle + 2 * M_PI;
	double distanceFromTower = cvs->Param->rayonBeacon / sin(fabs(lastAngle - firstAngle) / 2);
    if(distanceFromTower > 3)
        distanceFromTower = 3;
    else if(distanceFromTower < 0)
        distanceFromTower = EPSILON;
	return distanceFromTower;
}

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT