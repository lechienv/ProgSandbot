
#ifndef _ACTIONS_GR2_H_
#define _ACTIONS_GR2_H_

#include "CtrlStruct_gr2.h"
#include <math.h>
#include "controller_gr2.h"
#include "ctrl_main_gr2.h"
#include "dynamixel_gr2.h"
#include "rateau_gr2.h"

#ifndef REALBOT
#include "namespace_ctrl.h"
#include <stdlib.h>
#include <cstdio>
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

/************************
* FUNCTIONS PROTOYPE ****
*************************/

bool Action1(CtrlStruct *cvs); // hisser les drapeaux
bool Action2(CtrlStruct *cvs); // prendre bloc milieu
bool Action3(CtrlStruct *cvs); // prendre bloc cote
bool Action4(CtrlStruct *cvs); // aller pres des poissons
void ActionBase(CtrlStruct *cvs); // go to Base
bool ActionParasol(CtrlStruct *cvs); //Open the parasol
#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif