#ifndef _TESTBOT_GR2_H_
#define _TESTBOT_GR2_H_

#include "CtrlStruct_gr2.h"
#include "strategy_gr2.h"
#include "controller_gr2.h"
#ifndef REALBOT
#include "namespace_ctrl.h"
#include <stdlib.h>
#include <cstdio>
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

/************************
* FUNCTIONS PROTOYPE ****
*************************/

#ifdef REALBOT
void StrategyTest(CtrlStruct *cvs);
void TorqueControl(Motor *Motor, double torqueRef);
void Action1Test(CtrlStruct *cvs);
void Action2Test(CtrlStruct *cvs);
void Action3Test(CtrlStruct *cvs);
void Action4Test(CtrlStruct *cvs);
void Action5Test(CtrlStruct *cvs);
void Action6Test(CtrlStruct *cvs);
void Action7Test(CtrlStruct *cvs);
void Action8Test(CtrlStruct *cvs);
void Action9Test(CtrlStruct *cvs);
void Action10Test(CtrlStruct *cvs);
void Action11Test(CtrlStruct *cvs);
void Action12Test(CtrlStruct *cvs);
void Action13Test(CtrlStruct *cvs);
void StrategyTest(CtrlStruct *cvs);
bool PinceCalibration(CtrlStruct *cvs);
bool ClosePince(CtrlStruct *cvs);
bool DeposeBlock(CtrlStruct *cvs);
bool YCalibration(CtrlStruct *cvs, double Y, double Theta);
bool XCalibration(CtrlStruct *cvs, double X, double Theta);
bool Creneau(CtrlStruct *cvs);
#endif //REALBOT


#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT
#endif