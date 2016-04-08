#ifndef Controller_H_
#define Controller_H_

double ComputeSpeed(MotorStruct *Motor, unsigned int numberOfClics, int positiveSpeed);

int     Limiteur(int dutyCycle);
double SpeedController(MotorStruct *Motor, double speedRef, double dt);


#endif

