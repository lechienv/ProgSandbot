/*!
* \file odometry_gr2.h
* \brief main header of the strategy
*/

#ifndef _TOWER_GR2_H_
#define _TOWER_GR2_H_
#include <math.h>
#include "CtrlStruct_gr2.h"
#include "ctrl_main_gr2.h"

#ifndef REALBOT
#include "namespace_ctrl.h"
#include <stdlib.h>
#include <cstdio>
NAMESPACE_INIT(ctrlGr2);
#endif // !REALBOT

void	OpponentDetection(CtrlStruct *cvs);
void	UpdateDetectedBotPosition(CtrlStruct *cvs);
bool	IsBot(double x, double y);
double	ComputeAngle(CtrlStruct *cvs, int risingIndex, int fallingIndex);
double	ComputeDistance(CtrlStruct *cvs, int risingIndex, int fallingIndex);




#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif
