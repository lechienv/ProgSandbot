#include "interfaceFPGA.h"
#include "RoboticFiles/CtrlStruct_gr2.h"
int previousTurn = 0;
int previousSomethingDetected = 0;
int rising_index = 0;
int falling_index = 0;
int nb_rising = 0;
int nb_falling = 0;
int nb_opponents = 3;

int previousValue1 = 0;
int previousValue2 = 0;

double timeGlitch = 0;
double timePrevious = 0;

void UpdateFromFPGARealBot(CtrlStruct *cvs){
    unsigned int A = MyCyclone_Read(CYCLONE_IO_A_Data);
    unsigned int B = MyCyclone_Read(CYCLONE_IO_B_Data);
    unsigned int C = MyCyclone_Read(CYCLONE_IO_C_Data);
    unsigned int D = MyCyclone_Read(CYCLONE_IO_D_Data);
    unsigned int E = MyCyclone_Read(CYCLONE_IO_E_Data);
    unsigned int F = MyCyclone_Read(CYCLONE_IO_F_Data);
    unsigned int G = MyCyclone_Read(CYCLONE_IO_G_Data);
    unsigned int H = MyCyclone_Read(CYCLONE_IO_H_Data);
    unsigned int I = MyCyclone_Read(CYCLONE_IO_I_Data);
    unsigned int J = MyCyclone_Read(CYCLONE_IO_J_Data);
    unsigned int K = MyCyclone_Read(CYCLONE_IO_K_Data);
    unsigned int L = MyCyclone_Read(CYCLONE_IO_L_Data);
    
#ifdef MINIBOT
    cvs->MotorR->speed = ComputeSpeed(cvs->MotorL->clicNumber,C,extractBits(A,1,1));
    cvs->MotorL->speed = ComputeSpeed(cvs->MotorR->clicNumber,B,extractBits(A,2,2));
    cvs->Odo->speedL = cvs->MotorL->speed;
    cvs->Odo->speedR = cvs->MotorR->speed;
#else
    cvs->MotorL->speed = ComputeSpeed(cvs->MotorL->clicNumber,C,extractBits(A,1,1));
    cvs->MotorR->speed = ComputeSpeed(cvs->MotorR->clicNumber,B,extractBits(A,2,2));
    cvs->Odo->speedL = ComputeSpeed(cvs->Odo->clicNumber,D,extractBits(A,6,6));
    cvs->Odo->speedR = ComputeSpeed(cvs->Odo->clicNumber,E,extractBits(A,7,7));
#endif

    cvs->MotorPince->speed = ComputeSpeed(cvs->MotorPince->clicNumber,F,extractBits(A,5,5));
    cvs->MotorRatL->speed = ComputeSpeed(cvs->MotorRatL->clicNumber,G,extractBits(A,3,3));
    cvs->MotorRatR->speed = ComputeSpeed(cvs->MotorRatR->clicNumber,H,extractBits(A,4,4));
    
#ifdef MINIBOT
   /* MyCAN_USwitch(&(cvs->Sensors->uSwitchLeft), &(cvs->Sensors->uSwitchRight));
    char theStr[64];
    sprintf(theStr,"uSL = %d \t uSR = %d \t \n", cvs->Sensors->uSwitchLeft, cvs->Sensors->uSwitchRight);
    MyConsole_SendMsg(theStr);*/
#else
    cvs->Sensors->uSwitchLeft = (bool) extractBits(A,8,8);
    cvs->Sensors->uSwitchRight = (bool) extractBits(A,9,9);
#endif // MINIBOT
    cvs->Sensors->uSwitchPinceIn = (bool) extractBits(A,10,10);
    cvs->Sensors->uSwitchPinceOut = (bool) extractBits(A,11,11);
    
    /* TOWER */
    int newTurn = extractBits(A,15,15);
    if(newTurn != previousTurn){        
        previousTurn = newTurn;
        cvs->Tower->nb_rising = nb_rising;
        cvs->Tower->nb_falling = nb_falling;
        cvs->Tower->nb_opponents = nb_opponents;
        nb_rising = 0;
        nb_falling = 0;
    }

    int newValue1 = K; 
    int newValue2 = L;
    /*
    char theStr[512];
    double angleRising2 = 2*M_PI*K/cvs->MotorTower->clicNumber;
    double angleFalling2 = 2*M_PI*L/cvs->MotorTower->clicNumber;
    sprintf(theStr, "AngleRising = %f \t AngleFalling = %f \t \n", angleRising2*RADtoDEG, angleFalling2*RADtoDEG);
    MyConsole_SendMsg(theStr);*/
    if(newValue1 != previousValue1 || newValue2 != previousValue2){
        previousValue1 = newValue1;
        previousValue2 = newValue2;
        double angleOffset = 5*M_PI/4;
        double angleRising = 2*M_PI*K/cvs->MotorTower->clicNumber + angleOffset;
        double angleFalling = 2*M_PI*L/cvs->MotorTower->clicNumber + angleOffset;
        while(angleRising > M_PI) angleRising = angleRising - 2*M_PI; //To work in [-pi; pi)
        while(angleFalling > M_PI) angleFalling = angleFalling - 2*M_PI; //To work in [-pi; pi)
        cvs->Tower->rising_index = rising_index;
        cvs->Tower->falling_index = falling_index;
        cvs->Tower->last_rising[rising_index] = angleRising;
        cvs->Tower->last_falling[falling_index] = angleFalling;
        
        rising_index++;
        falling_index++;
        if(rising_index >= NB_STORE_EDGE) rising_index = rising_index - NB_STORE_EDGE;
        if(falling_index >= NB_STORE_EDGE) falling_index = falling_index - NB_STORE_EDGE;
        nb_rising++;
        nb_falling++;
    }
    
    cvs->time = getTime() - cvs->timeOffset;

    /*
    char theStr[512];
    sprintf(theStr,"SpeedL = %f \t SpeedR = %f \t SpeedOdoL = %f \t SpeedOdoR = %f \t \n", cvs->MotorL->speed, cvs->MotorR->speed, cvs->Odo->speedL, cvs->Odo->speedR);
    MyConsole_SendMsg(theStr);
    */
    
    //char theStr[1024];
    //sprintf(theStr,"A %d \t B %d \t C %d \t D %d \t E %d \t F %d \t G %d \t H %d \t I %d \t J %d \t K %d \t \n \n ", A, B, C, D, E, F, G, H, I, J, K);
    /*char theStr[128];
    sprintf(theStr,"ULeft= %d \t URight = %d \t UPinceIn = %d \t UPinceOut = %d \t \n",A, cvs->Sensors->uSwitchLeft, cvs->Sensors->uSwitchRight, cvs->Sensors->uSwitchPinceIn, cvs->Sensors->uSwitchPinceOut);
    MyConsole_SendMsg(theStr);*/
}

double getTime(){
    unsigned int currentTime = ReadCoreTimer();
    double time = (currentTime/SYS_FREQ_DOUBLE)*2.0;
    if(timePrevious > time){
        timeGlitch = timeGlitch + timePrevious;
        timePrevious = 0;
    }
    else {
        timePrevious = time;
    }
    time = time + timeGlitch;
    return time;
}

int getRobotID(){
    unsigned int A = MyCyclone_Read(CYCLONE_IO_A_Data);
    return extractBits(A,0,0);
}

double ComputeSpeed(double clicNumber, unsigned int numberOfClic, int positiveSpeed){
    if(positiveSpeed) positiveSpeed = 1;
    else positiveSpeed = -1;
    return numberOfClic*(2*M_PI/clicNumber)*positiveSpeed/REFRESH_TIME_SPEED;
}

unsigned int extractBits(unsigned int number, unsigned int b, unsigned int a){//! least significant bit at position 0
   unsigned int r = 0;
   unsigned int i;
   for (i=a; i<=b; i++)
       r |= 1 << i;
   return (r & number);
}

char DutyCycle_RealBot(int dutyCyclePercent){
    double duty = (127.5 + 127.5*dutyCyclePercent/100);
    if(duty < 0){
        return 0;
    }
    else if (duty > 255){
        return 255;
    }
    else return duty;
}

void InitRegMotor(Motor *Motor){
    EnableBrakes(Motor); //Added

    char theData[3];
    /* TIMER */
    theData[0] = Motor->timerReg;
    theData[1] = TIMERMASK;
    theData[2] = TIMEROFF;
    MyCAN_TxMsg(Motor->SID,theData);
    MyCAN_Acknowledge();
    MyDelayUs(2);
    /* PERIOD */
    theData[0] = Motor->perReg;
    theData[1] = PERMASK;
    theData[2] = PEROFF;
    MyCAN_TxMsg(Motor->SID,theData);
    MyCAN_Acknowledge();
    Motor->totalError = 0;
    MyDelayUs(2);
   // Motor->dutyCycle = 0;
    //ActivateMotor_RealBot(Motor);
}

void ActivateMotor_RealBot(Motor *Motor){   
   // DisableBrakes(Motor);
    char theData[3];
    unsigned char DC = DutyCycle_RealBot((int) Motor->dutyCycle);
    /* PWM */
    theData[0] = Motor->PWMReg;
    theData[1] = PWMMASK;
    theData[2] = DC>>2;
    MyCAN_TxMsg(Motor->SID,theData);
    MyCAN_Acknowledge();
    MyDelayUs(2);
    /* TIMER */
    theData[0] = Motor->timerReg;
    theData[1] = TIMERMASK;
    theData[2] = 0xDC;
    MyCAN_TxMsg(Motor->SID,theData);
    MyCAN_Acknowledge();
    MyDelayUs(2);
}

void DisableBrakes(Motor *Motor){
    MyDelayUs(2);
    char theData[3];
    theData[0] = BRAKESREG;
    theData[1] = Motor->brakesMask;
    theData[2] = BRAKESOFF;
    Motor->areBrakesEnabled = false;
    MyCAN_TxMsg(Motor->SID,theData);
    MyCAN_Acknowledge();
    MyDelayUs(2);
}

void EnableBrakes(Motor *Motor){
    MyDelayUs(2);
    char theData[3];
    theData[0] = BRAKESREG;
    theData[1] = Motor->brakesMask;
    theData[2] = BRAKESON;
    Motor->areBrakesEnabled = true;
    MyCAN_TxMsg(Motor->SID,theData);
    MyCAN_Acknowledge();
    MyDelayUs(2);
}

void MyDelayUs(unsigned int usec){
	unsigned int tWait, tStart;
    tWait=(SYS_FREQ/2000000)*usec;
    tStart=ReadCoreTimer();
    while((ReadCoreTimer()-tStart)<tWait) ;		// wait for the time to pass
}

void InitWebVariables(CtrlStruct *cvs){
    var1 = cvs->MotorL->speed;
    var2 = cvs->MotorL->Kp;
    var3 = cvs->MotorL->Ki;
    var4 = cvs->MotorL->position;
    var5 = cvs->MotorL->clicNumber;
    
    var6 = cvs->Odo->x;
    var7 = cvs->Odo->y;
    var8 = cvs->Odo->theta;
    
    var9 = cvs->Poto->katt;
    var10 = cvs->Poto->krep;
    var11 = cvs->Poto->minDistance;
    var12 = cvs->Poto->kw;
    var13 = cvs->Poto->kFV;
    
    var14 = cvs->MotorPince->speed;
    var15 = cvs->MotorPince->KpPos;
    var16 = cvs->MotorPince->Ki;
    var17 = cvs->MotorPince->dutyCycle;
    var18 = cvs->MotorPince->clicNumber;
    
    var19 = cvs->MotorRatL->speed;
    var20 = cvs->MotorRatL->KpPos;
    var21 = cvs->MotorRatL->Ki;
    var22 = cvs->MotorRatL->dutyCycle;
    var23 = cvs->MotorRatL->clicNumber;
    
    var24 = cvs->Odo->speedL;
    var25 = cvs->Odo->speedR;
    var26 = cvs->Odo->clicNumber;
    var27 = cvs->Param->radiusBot;
    var28 = cvs->Param->width;
    var29 = cvs->Param->KpRot;
    var30 = cvs->Param->KiRot;
    //var31 = cvs->Param->speedDifThreshold;
    //var32 = cvs->Param->KiAngleThreshold;
    var31 = cvs->MotorR->clicNumber;
    var32 = cvs->MotorL->clicNumber;
    var33 = cvs->Sensors->uSwitchLeft;
    var34 = cvs->Sensors->uSwitchRight;
    var35 = cvs->Sensors->uSwitchPinceIn;
    var36 = cvs->Sensors->uSwitchPinceOut;
    var37 = cvs->time;
    var38 = 0;
    var39 = 0;
    var40 = 0;
    var1Status = var1;
    var2Status = var2;
    var3Status = var3;
    var4Status = var4;
    var5Status = var5;
    var6Status = var6;
    var7Status = var7;
    var8Status = var8;
    var9Status = var9;
    var10Status = var10;
    var11Status = var11;
    var12Status = var12;
    var13Status = var13;
    var14Status = var14;
    var15Status = var15;
    var16Status = var16;
    var17Status = var17;
    var18Status = var18;
    var19Status = var19;
    var20Status = var20;
    var21Status = var21;
    var22Status = var22;
    var23Status = var23;
    var24Status = var24;
    var25Status = var25;
    var26Status = var26; 
    var27Status = var27; 
    var28Status = var28; 
    var29Status = var29; 
    var30Status = var30; 
    var31Status = var31; 
    var32Status = var32; 
    var33Status = var33; 
    var34Status = var34; 
    var35Status = var35; 
    var36Status = var36; 
    var37Status = var37; 
    var38Status = var38; 
    var39Status = var39; 
    var40Status = var40; 
}
void RefreshWebVariables(CtrlStruct *cvs){
    //cvs->MotorL->speed          = var1;
    cvs->MotorL->Kp             = var2;
    cvs->MotorL->Ki             = var3;
    //cvs->MotorL->position       = var4;
    cvs->MotorL->clicNumber     = var5;
    
    //cvs->Odo->x          = var6;
    //cvs->Odo->y          = var7;
    //cvs->Odo->theta      = var8;
    
    cvs->Poto->katt          = var9;
    cvs->Poto->krep          = var10;
    cvs->Poto->minDistance   = var11;
    cvs->Poto->kw            = var12;
    cvs->Poto->kFV           = var13;
    
    //cvs->MotorPince->speed          = var14;
    cvs->MotorPince->KpPos          = var15;
    cvs->MotorPince->Ki             = var16;    
    //cvs->MotorPince->dutyCycle      = var17;
    cvs->MotorPince->clicNumber     = var18;
    
    //cvs->MotorRatL->speed          = var19;
    cvs->MotorRatL->KpPos          = var20;
    cvs->MotorRatL->Ki             = var21;
    //cvs->MotorRatL->dutyCycle      = var22;
    cvs->MotorRatL->clicNumber     = var23;
    
    /*
    cvs->Odo->speedL    = var24;
    cvs->Odo->speedR    = var25;
    */
    cvs->Odo->clicNumber= var26;
     
    cvs->Param->radiusBot = var27;
    cvs->Param->width = var28;
    cvs->Param->KpRot = var29;
    cvs->Param->KiRot = var30;
    //cvs->Param->speedDifThreshold = var31;
    cvs->MotorR->clicNumber= var31;
    //cvs->Param->KiAngleThreshold = var32;
    cvs->MotorL->clicNumber = var32;
    /*
    cvs->Sensors->uSwitchLeft = var33;
    cvs->Sensors->uSwitchRight = var34;
    cvs->Sensors->uSwitchPinceIn = var35;
    cvs->Sensors->uSwitchPinceOut = var36;
    cvs->robotID = var37;
    cvs->time = var38;
     */
    
    var1Status = cvs->MotorL->speed;
    var2Status = cvs->MotorL->Kp;
    var3Status = cvs->MotorL->Ki;
    var4Status = cvs->MotorL->position;
    var5Status = cvs->MotorL->clicNumber;
    
    var6Status = cvs->Odo->x;
    var7Status = cvs->Odo->y;
    var8Status = cvs->Odo->theta;
    
    var9Status = cvs->Poto->katt;
    var10Status = cvs->Poto->krep;
    var11Status = cvs->Poto->minDistance;
    var12Status = cvs->Poto->kw;
    var13Status = cvs->Poto->kFV;
    
    var14Status = cvs->MotorPince->speed;
    var15Status = cvs->MotorPince->KpPos;
    var16Status = cvs->MotorPince->Ki;
    var17Status = cvs->MotorPince->dutyCycle;
    var18Status = cvs->MotorPince->clicNumber;
    
    var19Status = cvs->MotorRatL->speed;
    var20Status = cvs->MotorRatL->KpPos;
    var21Status = cvs->MotorRatL->Ki;
    var22Status = cvs->MotorRatL->dutyCycle;
    var23Status = cvs->MotorRatL->clicNumber;
    
    var24Status = cvs->Odo->speedL;
    var25Status = cvs->Odo->speedR;
    var26Status = cvs->Odo->clicNumber;
    var27Status = cvs->Param->radiusBot;
    var28Status = cvs->Param->width;
    var29Status = cvs->Param->KpRot;
    var30Status = cvs->Param->KiRot;
    //var31Status = cvs->Param->speedDifThreshold;
    var31Status = cvs->MotorR->clicNumber;
    //var32Status = cvs->Param->KiAngleThreshold;
    var32Status = cvs->MotorL->clicNumber;
    var33Status = cvs->Sensors->uSwitchLeft;
    var34Status = cvs->Sensors->uSwitchRight;
    var35Status = cvs->Sensors->uSwitchPinceIn;
    var36Status = cvs->Sensors->uSwitchPinceOut;
    var37Status = cvs->time;
    var38Status = var38;
    var39Status = var39;
    var40Status = var40;
    
}