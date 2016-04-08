#ifndef INTERFACEFPGA_H_
#define INTERFACEFPGA_H_
#include "MyApp.h"

#define PWMMASK             0xFF
#define TIMERMASK           0x03
#define PERMASK             0xFF
#define BRAKESREG           0x1E
#define BRAKESOFF           0x00
#define BRAKESON            0xFF
#define TIMEROFF            0x80
#define PEROFF              0x3D
#define REFRESH_TIME_SPEED  0.01
double ComputeSpeed(double clicNumber, unsigned int numberOfClic, int positiveSpeed);
void UpdateFromFPGARealBot(CtrlStruct *cvs);
int getRobotID();
unsigned int extractBits(unsigned int number, unsigned int b, unsigned int a);
char DutyCycle_RealBot(int dutyCyclePercent);
void MyDelayUs(unsigned int usec);
void InitWebVariables(CtrlStruct *cvs);
void RefreshWebVariables(CtrlStruct *cvs);
void InitRegMotor(Motor *Motor);
void ActivateMotor_RealBot(Motor *Motor);
void EnableBrakes(Motor *Motor);
void DisableBrakes(Motor *Motor);
double getTime();
#endif