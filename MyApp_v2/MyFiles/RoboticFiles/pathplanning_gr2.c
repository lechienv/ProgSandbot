/*! 
 * \file pathplanning.cc
 * \
 */

#include "pathplanning_gr2.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT


bool ReachPointPotential(CtrlStruct *cvs, double xGoal, double yGoal, double precisionRadius) {
	double speedRefL;
	double speedRefR;
	double hasReached = false;
	if ((cvs->Odo->x - xGoal)*(cvs->Odo->x - xGoal) + (cvs->Odo->y - yGoal)*(cvs->Odo->y - yGoal) < precisionRadius*precisionRadius) {
		speedRefL = 0;
		speedRefR = 0;
		SpeedRefToDC(cvs, cvs->MotorL, speedRefL);
		SpeedRefToDC(cvs, cvs->MotorR, speedRefR);
		return true;
	} //Objective is reached
	else {
		AttractiveForce(cvs, xGoal, yGoal);
		RepulsiveForce(cvs);

		// Taking into account non-holonmy
		double angle = atan2(cvs->Poto->FYRob, cvs->Poto->FXRob);
		if (fabs(cvs->Poto->FYRob) > cvs->Poto->thresholdAligned * fabs(cvs->Poto->FXRob)) {
			bool result = IsAlignedWithTheta(cvs, RADtoDEG*angle + cvs->Odo->theta, 5);
		}
		else {
			double speedRefW = cvs->Poto->kw * angle;
			double speedRefX = cvs->Poto->kFV*cvs->Poto->FXRob;
			speedRefX = limitSpeed(speedRefX, MAXSPEED);
            if(LIMITACCELERATION){
                speedRefX = limitXAcceleration(cvs,speedRefX);
            }
			speedRefW = limitSpeed(speedRefW, MAXSPEEDROT);
			speedRefL = speedRefX - cvs->Param->wheelRadius*speedRefW;
			speedRefR = speedRefX + cvs->Param->wheelRadius*speedRefW;
			SpeedRefToDC(cvs, cvs->MotorL, speedRefL);
			SpeedRefToDC(cvs, cvs->MotorR, speedRefR);
		}
		return false;
	} //Objective is not reached
}


double limitSpeed(double speed, double limit) {
	if (speed < limit)
		if (speed > -limit)
			return speed;
		else
			return -limit;
	else
		return limit;
}

void AttractiveForce(CtrlStruct *cvs, double xGoal, double yGoal){
	double FXInertial = -1 * (cvs->Poto->katt)*(cvs->Odo->x - xGoal);
	double FYInertial = -1 * (cvs->Poto->katt)*(cvs->Odo->y - yGoal);
	cvs->Poto->FXRob = cos(DEGtoRAD*cvs->Odo->theta)*FXInertial + sin(DEGtoRAD*cvs->Odo->theta)*FYInertial;
	cvs->Poto->FYRob = -sin(DEGtoRAD*cvs->Odo->theta)*FXInertial + cos(DEGtoRAD*cvs->Odo->theta)*FYInertial;
}

void RepulsiveForce(CtrlStruct *cvs) {
	double FXInertial = 0;
	double FYInertial = 0;
	int i;
	for (i = 0; i < cvs->Obstacles->NumberOfCircles; i++) {
		if (cvs->Obstacles->CircleList[i].isActive)
			ComputeFrepCircle(cvs, &(cvs->Obstacles->CircleList[i]), &FXInertial, &FYInertial);	
	}
	for (i = 0; i < cvs->Obstacles->NumberOfRectangles; i++) {
		if(cvs->Obstacles->RectangleList[i].isActive)
			ComputeFrepRectangle(cvs, &(cvs->Obstacles->RectangleList[i]), &FXInertial, &FYInertial);
	}
	for (i = 0; i < cvs->Obstacles->NumberOfQuarterOfCircle; i++) {
		if (cvs->Obstacles->QuarterOfCircleList[i].isActive)
			ComputeFrepQuarterOfCircle(cvs, &(cvs->Obstacles->QuarterOfCircleList[i]), &FXInertial, &FYInertial);
	}
	cvs->Poto->FXRob += cos(DEGtoRAD*cvs->Odo->theta)*FXInertial + sin(DEGtoRAD*cvs->Odo->theta)*FYInertial;
	cvs->Poto->FYRob += -sin(DEGtoRAD*cvs->Odo->theta)*FXInertial + cos(DEGtoRAD*cvs->Odo->theta)*FYInertial;
}

void FromClosestPointToRepForces(CtrlStruct *cvs, double x0, double y0, double xp, double yp, double *FXInertial, double *FYInertial) {
	double distance = sqrt(pow(x0 - xp, 2) + pow(y0 - yp, 2)) - cvs->Param->radiusBot;
	if (distance < EPSILON)
		distance = EPSILON;
	if (distance < cvs->Poto->minDistance) {
		double distanceSquared = distance*distance;
		double difY = y0 - yp;
		double difX = x0 - xp;
		*FXInertial += (cvs->Poto->krep * ((1 / distance - 1 / cvs->Poto->minDistance) / distanceSquared) * (difX / distance));
		*FYInertial += (cvs->Poto->krep * ((1 / distance - 1 / cvs->Poto->minDistance) / distanceSquared) * (difY / distance));
	}
	else {
		FXInertial += 0;
		FYInertial += 0;
	}
}

void ComputeFrepCircle(CtrlStruct *cvs, Circle *circle, double *FXInertial, double *FYInertial) {
	double x0 = cvs->Odo->x;
	double y0 = cvs->Odo->y;
	double xc = circle->x;
	double yc = circle->y;
	double radius = circle->radius;
	double xp = 0;
	double yp = 0;

	if (pow(x0 - xc, 2) + pow(y0 - yc, 2) < radius*radius) {
		xp = x0;
		yp = y0;
	}
	else {
		double angle = atan2((y0 - yc), (x0 - xc));
		xp = xc + radius*cos(angle);
		yp = yc + radius*sin(angle);
	}
	FromClosestPointToRepForces(cvs, x0, y0, xp, yp, FXInertial, FYInertial);
}

void ComputeFrepRectangle(CtrlStruct *cvs, Rectangle *Rectangle, double *FXInertial, double *FYInertial) {
	/* Coordinate of nearest rectangle's point*/
	double x0 = cvs->Odo->x;
	double y0 = cvs->Odo->y;
	double leftEdgeX = Rectangle->x - Rectangle->width / 2;
	double rightEdgeX = Rectangle->x + Rectangle->width / 2;
	double bottomEdgeY = Rectangle->y - Rectangle->height / 2;
	double topEdgeY = Rectangle->y + Rectangle->height / 2;
	int zone = DetermineRectangleZone(Rectangle, x0,y0);
	double xp;
	double yp;
	switch (zone) {
	case (1) :
		xp = leftEdgeX;
		yp = bottomEdgeY;
		break;
	case(2) :
		xp = leftEdgeX;
		yp = y0;
		break;
	case(3) :
		xp = leftEdgeX;
		yp = topEdgeY;
		break;
	case(4) :
		xp = x0;
		yp = topEdgeY;
		break;
	case(5) :
		xp = rightEdgeX;
		yp = topEdgeY;
		break;
	case(6) :
		xp = rightEdgeX;
		yp = y0;
		break;
	case(7) :
		xp = rightEdgeX;
		yp = bottomEdgeY;
		break;
	case(8) :
		xp = x0;
		yp = bottomEdgeY;
		break;
	case(9) :
		xp = x0;
		yp = y0;
		break;
	}
	FromClosestPointToRepForces(cvs, x0, y0, xp, yp, FXInertial, FYInertial);
}

int DetermineRectangleZone(Rectangle *Rectangle, double x0, double y0) {
	double leftEdgeX = Rectangle->x - Rectangle->width / 2;
	double rightEdgeX = Rectangle->x + Rectangle->width / 2;
	double bottomEdgeY = Rectangle->y - Rectangle->height / 2;
	double topEdgeY = Rectangle->y + Rectangle->height / 2;
	int zone = 0;
	if (x0 < leftEdgeX) {
		if (y0 < bottomEdgeY)
			zone = 1;
		else if (y0 < topEdgeY)
			zone = 2;
		else
			zone = 3;
	}

	else if (x0 > rightEdgeX) {
		if (y0 < bottomEdgeY)
			zone = 7;
		else if (y0 < topEdgeY)
			zone = 6;
		else
			zone = 5;
	}
	else {
		if (y0 < bottomEdgeY)
			zone = 8;
		else if (y0 < topEdgeY)
			zone = 9;
		else
			zone = 4;
	}
	return zone;
}

void ComputeFrepQuarterOfCircle(CtrlStruct *cvs, QuarterOfCircle *QuarterOfCircle, double *FXInertial, double *FYInertial) {
	double x0 = cvs->Odo->x;
	double y0 = cvs->Odo->y;
	double xc = QuarterOfCircle->x;
	double yc = QuarterOfCircle->y;
	//Align point with first quadrant frame (to define zone)
	double angleRot = -M_PI / 2 * (QuarterOfCircle->quadrant - 1);
	double newX0 = (cos(angleRot)*(x0 - xc) - sin(angleRot)*(y0 - yc)) + xc;
	double newY0 = (sin(angleRot)*(x0 - xc) + cos(angleRot)*(y0 - yc)) + yc;
	double radius = QuarterOfCircle->radius;
	int zone = DetermineQuarterOfCircleZone(QuarterOfCircle, newX0, newY0);
	double xp;
	double yp;
	switch (zone) {
		case (1) :
			xp = xc;
			yp = yc;
			break;
		case (2) :
			xp = xc;
			yp = newY0;
			break;
		case (3) :
			xp = xc;
			yp = yc + radius;
			break;
		case (4) : {
			double angle = atan2((newY0 - yc), (newX0 - xc));
			xp = xc + radius*cos(angle);
			yp = yc + radius*sin(angle);
			break;
		}
		case (5) :
			xp = xc + radius;
			yp = yc;
			break;
		case (6) :
			xp = newX0;
			yp = yc;
			break;
		case (7) :
			xp = newX0;
			yp = newY0;
			break;
	}

	double angleRotInverse = -angleRot;
	double xpInverse = (cos(angleRotInverse)*(xp - xc) - sin(angleRotInverse)*(yp - yc)) + xc; //InertialCoordinates
	double ypInverse = (sin(angleRotInverse)*(xp - xc) + cos(angleRotInverse)*(yp - yc)) + yc; //InertialCoordinates

	FromClosestPointToRepForces(cvs, x0, y0, xpInverse, ypInverse, FXInertial, FYInertial);
}

int DetermineQuarterOfCircleZone(QuarterOfCircle *QuarterOfCircle, double newX0, double newY0) {
	double xc = QuarterOfCircle->x;
	double yc = QuarterOfCircle->y;
	double radius = QuarterOfCircle->radius;
	int zone = 0;
	if (newX0 < xc) {
		if (newY0 < yc)
			zone = 1;
		else if (newY0 < yc + radius)
			zone = 2;
		else
			zone = 3;
	}
	else if (newY0 < yc) {
		if (newX0 > xc + radius)
			zone = 5;
		else
			zone = 6;
	}
	else {
		if (pow((newX0 - xc),2) + pow((newY0 - yc),2) < pow(radius,2))
			zone = 7;
		else
			zone = 4;
	}
	return zone;
}



#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT