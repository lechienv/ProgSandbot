
#ifndef _UPDATEECRAN_GR2_H_
#define _UPDATEECRAN_GR2_H_

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

/************************
* FUNCTIONS PROTOYPE ****
*************************/
void MyUpdateEcran(CtrlStruct *cvs, unsigned int M);

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif