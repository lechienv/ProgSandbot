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
int     FindCorrespondingBot(CtrlStruct *cvs, double x, double y);
void    AverageAndAddPosition(CtrlStruct *cvs, int botNumber);
bool    IsPositionOutliers(CtrlStruct *cvs, int botNumber, double x, double y);
void    FilterTowerBot(CtrlStruct *cvs, double x, double y, bool tooClose);
#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif
