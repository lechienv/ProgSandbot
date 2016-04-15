
#ifndef _DYNAMIXEL_GR2_H_
#define _DYNAMIXEL_GR2_H_

#include "CtrlStruct_gr2.h"

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
void TurnCCW(int value);
void TurnCW(int value);
void ReadDyna();
void StopTurn(int i);
void AllumeLed();

#endif

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif