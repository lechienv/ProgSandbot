
#ifndef _UPDATEECRAN_GR2_H_
#define _UPDATEECRAN_GR2_H_

#include "RoboticFiles/CtrlStruct_gr2.h"

#ifndef REALBOT
#include "namespace_ctrl.h"
#include <stdlib.h>
#include <cstdio>
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

/************************
* FUNCTIONS PROTOYPE ****
*************************/
void getRobotID(CtrlStruct *cvs);

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif