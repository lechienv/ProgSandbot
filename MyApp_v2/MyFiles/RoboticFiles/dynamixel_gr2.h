
#ifndef _DYNAMIXEL_GR2_H_
#define _DYNAMIXEL_GR2_H_

#include "CtrlStruct_gr2.h"
#include <math.h>
#include "controller_gr2.h"
#include "ctrl_main_gr2.h"
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

#ifdef REALBOT
void SendMessageDyna(int id, int size, int address, double value);
void SendMessageDyna2(int id, int size, int address, double value);
void InitDyna();
void TurnCCW(int id, int value);
void TurnCW(int id, int value);
void ReadDyna();
void StopTurn(int id, int i);
void AllumeLed(int id, int value);
void WhichPosition(int id);


#endif

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif