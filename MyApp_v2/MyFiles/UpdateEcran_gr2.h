
#ifndef _UPDATEECRAN_GR2_H_
#define _UPDATEECRAN_GR2_H_

#include "RoboticFiles/CtrlStruct_gr2.h"
#include "MyApp.h"
#include "RoboticFiles/rateau_gr2.h"
#include "RoboticFiles/ctrl_main_gr2.h"

/************************
* FUNCTIONS PROTOYPE ****
*************************/
void getRobotID(CtrlStruct *cvs);
void getActions(CtrlStruct *cvs);
void TestRoues(CtrlStruct *cvs, bool Direction);
void getTests(CtrlStruct *cvs);
void ChooseStratDuneOrNot(CtrlStruct *cvs);
void getStrategy(CtrlStruct *cvs);
void ChooseBetweenMatchOrTest(CtrlStruct *cvs);


#endif