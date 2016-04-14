/*!
* \file Strategy_gr2.h
* \brief main header of the strategy
*/

#ifndef _STRATEGY_GR2_H_
#define _STRATEGY_GR2_H_

#include "CtrlStruct_gr2.h"
#include <math.h>
#include "controller_gr2.h"
#include "ctrl_main_gr2.h"
#ifndef REALBOT
#include "namespace_ctrl.h"
#include <stdlib.h>
#include <cstdio>
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

void	TheGoals(CtrlStruct *cvs);
void	goto_nextstate(CtrlStruct *cvs, bool my_bool);
void	UpdateTarget(CtrlStruct *cvs);
void	ActivateBase(CtrlStruct *cvs);
void	DisactivateBase(CtrlStruct *cvs);
void	Calibration(CtrlStruct *cvs);
void	ReCalibration(CtrlStruct *cvs);
void    PointHomologation(CtrlStruct *cvs);
void    Action1(CtrlStruct *cvs);
void    Action2(CtrlStruct *cvs);
#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif