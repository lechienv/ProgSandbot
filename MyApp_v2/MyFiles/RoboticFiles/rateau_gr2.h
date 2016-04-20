
#ifndef _RATEAU_GR2_H_
#define _RATEAU_GR2_H_

#include "CtrlStruct_gr2.h"
#include "strategy_gr2.h"
#ifndef REALBOT
#include "namespace_ctrl.h"
#include <stdlib.h>
#include <cstdio>
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

/************************
* FUNCTIONS PROTOYPE ****
*************************/
void StartMyRat(CtrlStruct *cvs);
bool RatGoBottom(CtrlStruct *cvs, Motor *Motor);
bool RatGoTop(CtrlStruct *cvs, Motor *Motor);
bool RateauReachPoint(CtrlStruct *cvs, double pos);

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif