/*! 
 * \file ctrl_main_gr2.h
 * \brief main header of the controller
 */

#ifndef _CTRL_MAIN_GR2_H_
#define _CTRL_MAIN_GR2_H_

#include "CtrlStruct_gr2.h"

#ifndef REALBOT
#include "namespace_ctrl.h"
#include <stdlib.h>
#include <cstdio>
#include "user_realtime.h"
#else
#include "testBot_gr2.h"
#endif // !REALBOT

#include "pathplanning_gr2.h"
#include "strategy_gr2.h"
#include "odometry_gr2.h"
#include "allParamInit_gr2.h"
#include "controller_gr2.h"
#include "tower_gr2.h"
#include <math.h>

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // !REALBOT

void	controller_init(CtrlStruct *cvs);
void	controller_loop(CtrlStruct *cvs);
void	controller_finish(CtrlStruct *cvs);

void	UpdateFromFPGA(CtrlStruct *cvs);
void	AlwaysInController(CtrlStruct *cvs);
void	AlwaysEndController(CtrlStruct *cvs);


#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // !REALBOT

#endif
