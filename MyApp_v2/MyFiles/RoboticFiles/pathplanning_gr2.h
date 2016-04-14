/*!
 * \file path_planning.h
 * \brief main header of the pathplanning
 */

#ifndef _PATH_PLANNING_GR2_H_
#define _PATH_PLANNING_GR2_H_

#include "CtrlStruct_gr2.h"
#include <math.h>
#include "controller_gr2.h"
#ifndef REALBOT
#include "namespace_ctrl.h"
#include <stdlib.h>
#include "user_realtime.h"
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT
bool	ReachPointPotential(CtrlStruct *cvs, double xGoal, double yGoal, double precisionRadius);
double	limitSpeed(double speed, double limit);
void	AttractiveForce(CtrlStruct *cvs, double xGoal, double yGoal);
void	RepulsiveForce(CtrlStruct *cvs);
void	FromClosestPointToRepForces(CtrlStruct *cvs, double x0, double y0, double xp, double yp, double *FXInertial, double *FYInertial);
void	ComputeFrepCircle(CtrlStruct *cvs, Circle *circle, double *FXInertial, double *FYInertial);
void	ComputeFrepRectangle(CtrlStruct *cvs, Rectangle *Rectangle, double *FXInertial, double *FYInertial);
int		DetermineRectangleZone(Rectangle *Rectangle, double x0, double y0);
void	ComputeFrepQuarterOfCircle(CtrlStruct *cvs, QuarterOfCircle *QuarterOfCircle, double *FXInertial, double *FYInertial);
int		DetermineQuarterOfCircleZone(QuarterOfCircle *QuarterOfCircle, double newX0, double newY0);


#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif
