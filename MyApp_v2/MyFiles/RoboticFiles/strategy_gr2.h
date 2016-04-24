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
#include "dynamixel_gr2.h"
#include "testBot_gr2.h"
#include "Actions_gr2.h"
#ifndef REALBOT
#include "namespace_ctrl.h"
#include <stdlib.h>
#include <cstdio>
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

void    MyStrategy(CtrlStruct *cvs, int dune);
void	ActivateBase(CtrlStruct *cvs);
void	DisactivateBase(CtrlStruct *cvs);
bool	Calibration(CtrlStruct *cvs);
void	ReCalibration(CtrlStruct *cvs);
void    PointHomologation(CtrlStruct *cvs);
void    DynaTestFunction(CtrlStruct *cvs);
void    SetTimer(CtrlStruct *cvs, MyTimer *Timer, double time);
void    ResetTimer(MyTimer *Timer);
bool    IsTimerTimout(CtrlStruct *cvs, MyTimer *Timer);
bool    PinceCalibration(CtrlStruct *cvs);
bool    ClosePince(CtrlStruct *cvs, int duty);
bool    PinceReachPosition(CtrlStruct *cvs, int pos);
bool    DeposeBlock(CtrlStruct *cvs);
bool    YCalibration(CtrlStruct *cvs, double Y, double Theta);
bool    XCalibration(CtrlStruct *cvs, double X, double Theta);
bool    Creneau(CtrlStruct *cvs);
#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif