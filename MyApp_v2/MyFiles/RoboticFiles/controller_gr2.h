#ifndef _CONTROLLER_GR2_H_
#define _CONTROLLER_GR2_H_

#include "CtrlStruct_gr2.h"
#include <math.h>

#ifndef REALBOT
#include "namespace_ctrl.h"
#include <cstdio>
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

double	limitDC(double DC);
void	SpeedRefToDC(CtrlStruct *cvs, Motor *Motor, double speedRef);
bool	IsAlignedWithTheta(CtrlStruct *cvs, double thetaRef, double anglePrecision);
void	SendMotorCommand(CtrlStruct *cvs);

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT

#endif