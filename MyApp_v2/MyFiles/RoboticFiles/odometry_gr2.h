/*!
* \file odometry_gr2.h
* \brief main header of the strategy
*/

#ifndef _ODOMETRY_GR2_H_
#define _ODOMETRY_GR2_H_
#include <math.h>
#include "CtrlStruct_gr2.h"


#ifndef REALBOT
#include "namespace_ctrl.h"
#include <stdlib.h>
#include <cstdio>
NAMESPACE_INIT(ctrlGr2);
#endif // !REALBOT

void OdometryLoop(CtrlStruct *cvs);

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif