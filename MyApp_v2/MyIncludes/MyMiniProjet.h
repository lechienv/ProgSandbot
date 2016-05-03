/*******************************************************************************
* Header file for MyMiniProjet                                                 *
*******************************************************************************/

#ifndef MyMINIPROJET_H_
#define MyMINIPROJET_H_
#include "../MyFiles/RoboticFiles/CtrlStruct_gr2.h"

#define TIMESTEP_REALBOT 0.002
#define TIME_DATAREFRESH 0.01
/*******************************************************************************
* Functions Prototypes                                                         *
*******************************************************************************/

void            MyMiniProjet_Task(void);
void            InitSPIChannel();
double fmin(double A, double B);
/*******************************************************************************
* Global Variables for MyMiniProjet Functions                                  *
*******************************************************************************/

#endif /* MyMINIPROJET_H_ */