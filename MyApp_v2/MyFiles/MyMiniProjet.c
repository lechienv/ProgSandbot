/*******************************************************************************
* MyMiniProjet                                                                 *
********************************************************************************
* Description:                                                                 *
* Functions to control the Robot of the MiniProjet 2.0                         *
********************************************************************************
* Version : 1.00 - July 2011                                                   *
*******************************************************************************/
#include "MyApp.h"
#include "interfaceFPGA.h"
/*****************************************************************************
 * STRUCTURES
 *****************************************************************************/
/*****************************************************************************
 * MAIN
 *****************************************************************************/

#define WEB
void MyMiniProjet_Task(void)
{

    MyConsole_SendMsg("here we go ! \n");
    MyDelayMs(2000);

    /*********************************
     *DO NOT TOUCH THAT **************
     ********************************/
    InitSPIChannel();
    CtrlStruct *cvs;
	cvs = (CtrlStruct*) malloc(sizeof(CtrlStruct));
    unsigned long long currentTime = 0;
    unsigned long long previousTime = 0;
    unsigned long long previousTimeData = 0;
    controller_init(cvs);
    InitWebVariables(cvs);

    /*********************************
     * SD Memory *********************
     ********************************/
    unsigned int size = 8192;
    char timeSD[size];
    CreateBuffer(timeSD);

    char xSD[size];
    CreateBuffer(xSD);

    char ySD[size];
    CreateBuffer(ySD);

    char thetaSD[size];
    CreateBuffer(thetaSD);

    char speedLSD[size];
    CreateBuffer(speedLSD);

    char speedRSD[size];
    CreateBuffer(speedRSD);

    bool hasSaved = false;
    /*********************************
    * MAIN LOOP **********************
    *********************************/

      
    while(1){
          unsigned int A = MyCyclone_Read(CYCLONE_IO_A_Data);
          unsigned int I = MyCyclone_Read(CYCLONE_IO_I_Data);
          MyConsole_SendMsg("waiting for start \n");
          bool start = (bool) extractBits(A,13,13);
          if(start){
              MyConsole_SendMsg("Starting\n");
              cvs->timeOffset = getTime();
              cvs->previousTime = 0;
              cvs->time = 0;
               

              while((bool) extractBits(A,13,13)){
                               if(cvs->time <90)
                {
                                   MyConsole_Task();
                  //MyCAN_Task();
  #ifdef WEB
                  MyWIFI_Task();
  #endif
                  unsigned long long currentTime = ReadCoreTimer();
                  if((double)(currentTime - previousTime) > TIMESTEP_REALBOT*(SYS_FREQ/2000)*1000){//

                      previousTime = currentTime;

                          controller_loop(cvs);

                     /* char s[659];
                      sprintf(s,"x = %f \t y = %f \t stateCalib = %d \t color = %d\n", cvs->Odo->x, cvs->Odo->y, cvs->stateCalib, cvs->robotID);
                      MyConsole_SendMsg(s);*/

        //#define TESTS

          #ifndef TESTS
                      if((double)(currentTime - previousTimeData) > TIME_DATAREFRESH*(SYS_FREQ/2000)*1000){//
                          previousTimeData = currentTime;
  #ifdef WEB
                          /* Refresh Web Variables */
                          RefreshWebVariables(cvs);
                          /* Save on SD */
  #else
                          if(!hasSaved){
                              AddElement(cvs->MotorL->speed, speedLSD);
                              MyDelayMs(1);
                              AddElement(cvs->MotorR->speed, speedRSD);
                              MyDelayMs(1);
                              AddElement(cvs->timeStep,timeSD);
                              MyDelayMs(1);
                          }
  #endif
                      }
  #ifndef WEB
                      /* Stopping condition*/
                      //unsigned int A = MyCyclone_Read(CYCLONE_IO_A_Data);
                      //int newTurn = extractBits(A,15,15);
                      if(cvs->time > 20 && !hasSaved){
                      //if((newTurn != previousTurn) && !hasSaved){
                          //MyConsole_SendMsg("Here2 \n");
                          /* Save data on SD */
                          WriteSDMemory(timeSD, "time.txt", size);
                          WriteSDMemory(speedLSD, "speedL.txt", size);
                          WriteSDMemory(speedRSD, "speedR.txt", size);
                          //WriteSDMemory(xSD, "odox.txt", size);
                          //WriteSDMemory(ySD, "odoy.txt",size);
                          //WriteSDMemory(thetaSD, "theta.txt",size);

                          hasSaved = true;
                      }
          #endif
  #endif
                  }
              }
                             else
                 {
                                  MyConsole_SendMsg("end \n");
                            cvs->MotorL->dutyCycle =0;
                            cvs->MotorR->dutyCycle = 0;
                            cvs->MotorTower->dutyCycle =0;
                            cvs->MotorRatL->dutyCycle = 0;
                            cvs->MotorRatR->dutyCycle = 0;
                            cvs->MotorPince->dutyCycle = 0;
                 }
              }

          }
      }
  
 

    /* Finish the program */
    controller_finish(cvs);
    free(cvs);

}
/*****************************************************************************
 * MOTORS
 *****************************************************************************/

/*****************************************************************************
 * DATA
 *****************************************************************************/

void InitSPIChannel(){
    MyCyclone_Write(CYCLONE_IO_A_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_B_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_C_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_D_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_E_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_F_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_G_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_H_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_I_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_J_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_K_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_L_Enable_Out, 0x00);
    MyCyclone_Write(CYCLONE_IO_M_Enable_Out, 0x00);
    RightMotorDC = 0;
    LeftMotorDC = 0;
    PinceDC = 0;
    RateauRDC = 0;
    RateauLDC = 0;
    MotorCommandByHand = false;
}

double fmin(double A, double B){
    if(A < B) return A;
    return B;
}
