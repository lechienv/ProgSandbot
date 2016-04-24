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
//#define SD_CARD
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
    char X1[size];
    CreateBuffer(X1);
        
    char Y1[size];
    CreateBuffer(Y1);
    
    char X2[size];
    CreateBuffer(X2);
    
    char Y2[size];
    CreateBuffer(Y2);
    
    bool hasSaved = false;
    /*********************************
    * MAIN LOOP **********************
    *********************************/
    while(1){
        unsigned int A = MyCyclone_Read(CYCLONE_IO_A_Data);
        unsigned int I = MyCyclone_Read(CYCLONE_IO_I_Data);
      //  MyConsole_SendMsg("wtf \n");
        bool start = (bool) extractBits(A,13,13);
        if(start){
            //MyConsole_SendMsg("Starting\n");
            cvs->timeOffset = getTime();
            cvs->previousTime = 0;
            cvs->time = 0;
            /*char s1[128];
            sprintf(s1, "Avant = %f\n", extractBits(A,13,13));
            MyConsole_SendMsg(s1);
            */while((bool) extractBits(A,13,13)){
                /*char s2[128];
                sprintf(s2, "Après = %f\n", extractBits(A,13,13));
                MyConsole_SendMsg(s2);*/
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
                        //RefreshWebVariables(cvs);
#endif
                        
#ifdef SD_CARD
                        /* Save on SD */
                        if(!hasSaved){
                            AddElement(cvs->Obstacles->CircleList[0].x, X1);
                            MyDelayMs(1);
                            AddElement(cvs->Obstacles->CircleList[0].y, Y1);
                            MyDelayMs(1);
                            AddElement(cvs->Obstacles->CircleList[1].x, X2);
                            MyDelayMs(1);
                            AddElement(cvs->Obstacles->CircleList[1].y, Y2);
                        }
#endif
                    }
#ifdef SD_CARD
                    /* Stopping condition*/
                    //unsigned int A = MyCyclone_Read(CYCLONE_IO_A_Data);
                    //int newTurn = extractBits(A,15,15);
                    if(cvs->time > 30 && !hasSaved){  
                        WriteSDMemory(X1, "X1.txt", size);  
                        WriteSDMemory(Y1, "Y1.txt", size);  
                        WriteSDMemory(X2, "X2.txt", size);
                        WriteSDMemory(Y2, "Y2.txt",size);                      
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
