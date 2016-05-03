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
#include "UpdateEcran_gr2.h"
/*****************************************************************************
 * STRUCTURES
 *****************************************************************************/
/*****************************************************************************
 * MAIN
 *****************************************************************************/
#define SD_CARD
#define WEB


void MyMiniProjet_Task(void)
{

    //MyConsole_SendMsg("here \n");
    MyDelayMs(100);
    
    /*********************************
     *DO NOT TOUCH THAT **************
     ********************************/
    InitSPIChannel();
    CtrlStruct *cvs;
    
	cvs = (CtrlStruct*) malloc(sizeof(CtrlStruct));
    unsigned long long currentTime = 0;
    unsigned long long previousTime = 0;
    unsigned long long previousTimeData = 0;
    
    cvs->colorIsSet = false;
    while(!cvs->colorIsSet){
       getRobotID(cvs);
       // cvs->robotID = GREEN ; 
      //  cvs->colorIsSet = true;
        //MyConsole_SendMsg("robotID not set\n");
    }
    

    controller_init(cvs);
    InitWebVariables(cvs);
    /*********************************
     * SD Memory *********************
     ********************************/
    unsigned int size = 8192;    
    char TIME[size];
    CreateBuffer(TIME);
        
    char THETA[size];
    CreateBuffer(THETA);
  
     char DISTANCE[size];
    CreateBuffer(DISTANCE);
    
   /* char X[size];
    CreateBuffer(X);
    
        char Y[size];
    CreateBuffer(Y);*/

    
    bool hasSaved = false;
    /*********************************
    * MAIN LOOP **********************
    *********************************/
    while(1){
        unsigned int A = MyCyclone_Read(CYCLONE_IO_A_Data);
        unsigned int I = MyCyclone_Read(CYCLONE_IO_I_Data);
      //  MyConsole_SendMsg("wtf \n");
        bool start = (bool) !extractBits(A,13,13);

       // ChooseStratDuneOrNot(cvs);// if calib X pressed choooose to start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if(start){
            cvs->Param->start =true;
            //MyConsole_SendMsg("Starting\n");
            cvs->timeOffset = getTime();
            cvs->previousTime = 0;
            cvs->time = 0;
            while(start){ //(bool) extractBits(A,13,13)){ !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                MyConsole_Task();
                //MyCAN_Task();
#ifdef WEB
                MyWIFI_Task();
#endif
                unsigned long long currentTime = ReadCoreTimer();
                if((double)(currentTime - previousTime) > TIMESTEP_REALBOT*(SYS_FREQ/2000)*1000){//
                   
                    previousTime = currentTime;
                    controller_loop(cvs);    
                    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    
                    //getActions(cvs);
                    //getTests(cvs);
                    //SendMotorCommand(cvs);
                             
                             
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    if((double)(currentTime - previousTimeData) > TIME_DATAREFRESH*(SYS_FREQ/2000)*1000){//
                        previousTimeData = currentTime;                     
#ifdef WEB
                        /* Refresh Web Variables */
                        RefreshWebVariables(cvs);
#endif
                        
#ifdef SD_CARD
                        /* Save on SD */
                        if(!hasSaved){
                            AddElement(cvs->Tower->distance, DISTANCE);
                            MyDelayMs(1);
                            AddElement(cvs->Tower->angle, THETA);
                            MyDelayMs(1);
                            AddElement(cvs->time, TIME);
                        }
#endif
                    }
#ifdef SD_CARD
                    /* Stopping condition*/
                    //unsigned int A = MyCyclone_Read(CYCLONE_IO_A_Data);
                    //int newTurn = extractBits(A,15,15);
                    if(cvs->time > 15 && !hasSaved){  
                       // WriteSDMemory(X, "X.txt", size);  
                        WriteSDMemory(DISTANCE, "DISTANCE.txt", size);  
                        WriteSDMemory(THETA, "THETA.txt", size);
                        WriteSDMemory(TIME, "TIME.txt", size);                   
                        hasSaved = true;
                    }
#endif
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
    CommandMotorByHand = false;
}

double fmin(double A, double B){
    if(A < B) return A;
    return B;
}
