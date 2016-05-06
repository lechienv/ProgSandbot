#include "dynamixel_gr2.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

/*********************
*** FUNCTIONS ********
**********************/

#ifdef REALBOT



void SendMessageDyna(int id, int size, int address, double value){
    unsigned int Data1;
    unsigned int Data2;
    unsigned int Data3;
    unsigned int Data4;
    unsigned int Data5;
    unsigned int Data6;
    unsigned int Data7;
    unsigned int Data8;
    unsigned short CRC;
    double i = 0;

    //unsigned int val1 = value*1024/300;

    //while(1){
        //val1 = abs(1024*sin(i/100));
        //MyDelayMs(10);
    Data1 = id;  //id (communiquer avec tous)
    Data2 = size;   //Length_L
    Data3 = 0x0003;   //Instruction
    Data5 = address;//0x001e;   //P0  	Write data start address LSB (ou 0x32)
    //Data6 = (int) value;
    Data6 = (int) value & 0x00FF;   //P1        Write data start address MSB
    Data7 =  ((int) value >> 8) & 0x00FF;   //P2        Write data 1st byte
    Data4 =  ~ ( Data1 + Data2 + Data3 + Data5 + Data6 + Data7);


    /*char s[64];
    sprintf(s,"angle = %d\n", Data6 );
    MyConsole_SendMsg(s);*/



    // A_IO_dynDrive = CYCLONE_IO_B_Data [15:0]
    // A_IO_dynDataWrite_1 = CYCLONE_IO_B_Data [31:16]
    // A_IO_dynDataWrite_2 = CYCLONE_IO_C_Data [15:0]
    // A_IO_dynDataWrite_3 = CYCLONE_IO_C_Data [31:16]
    // A_IO_dynDataWrite_4 = CYCLONE_IO_D_Data [15:0]
    //

    MyCyclone_Write(CYCLONE_IO_S_Data,0x0000); // Mise à 0 des commandes d'écritures      A_IO_dynDrive
    MyCyclone_Write(CYCLONE_IO_S_Data, 0x0000); // Mise à 0 de TXD_Enable
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0014);
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0000); // Mise à 0 des commandes d'écritures

    MyCyclone_Write(CYCLONE_IO_S_Data,(Data1 << 8) & 0xFF00);
    MyCyclone_Write(CYCLONE_IO_O_Data, ((Data3 << 8) & 0xFF00) + (Data2&0x00FF));
    //MyCyclone_Write(A_IO_dynDataWrite_3,Data3);
    MyCyclone_Write(CYCLONE_IO_P_Data, (Data4) & 0x00FF);
    MyCyclone_Write(CYCLONE_IO_S_Data,((Data1 << 8) & 0xFF00) + (0x15 & 0x00FF));
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0000);
    MyCyclone_Write(CYCLONE_IO_S_Data,(Data5<<8)&0xFF00);
    MyCyclone_Write(CYCLONE_IO_O_Data,((Data7<<8)&0xFF00) + (Data6&0x00FF));
    //MyCyclone_Write(CYCLONE_IO_C_Data,Data6);
    //MyCyclone_Write(A_IO_dynDataWrite_3,Data7);
    MyCyclone_Write(CYCLONE_IO_S_Data,((Data5<<8)&0xFF00) + (0x16 & 0x00FF));
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0000);

    MyCyclone_Write(CYCLONE_IO_S_Data, (0x01<<8)&0xFF00);
    MyCyclone_Write(CYCLONE_IO_S_Data, ((0x01<<8)&0xFF00) + (0x14 & 0x00FF));
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0000); // Mise à 0 des commandes d'écritures
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0c);
    i = i+1;
}

void InitDyna(){
    SendMessageDyna(DynaAll,0x0005,0x0008,0x03ff);
    MyDelayMs(100);
    SendMessageDyna(DynaAll,0x0005,0x0006,0x0000);
    MyDelayMs(100);
    SendMessageDyna(DynaAll,0x0005,0x0019,0x1);
    MyDelayMs(100);
    SetAngle(DynaPara, 170);
    MyDelayMs(100);
    SetAngle(DynaRatL, 150);
    MyDelayMs(100);
    SetAngle(DynaRatR, 150);
    MyDelayMs(100);
    SendMessageDyna(DynaRatL,0x0005,0x0020,0x1F4);
    MyDelayMs(100);
    SendMessageDyna(DynaRatR,0x0005,0x0020,0x1F4);
    //SendMessageDyna(0xFE,0x0005,0x0004,0x34);
}
void AllumeLed(int id, int value){
    SendMessageDyna(id,0x0005,0x0019,value);
}
void TurnCCW(int id, int value){
    SendMessageDyna(id,0x0005,0x0008,0x0000);
    //SendMessageDyna(0x06,0x0005,0x0008,0x0000);
    if(value <0) value = 0;
    if(value > 100) value = 100;
    else value = value * 1023/100;
    SendMessageDyna(id, 0x0005, 0x0020, value);
}
void TurnCW(int id, int value){
    SendMessageDyna(id,0x0005,0x0008,0x0000);
    if(value <0) value = 0;
    if(value > 100) value = 100;
    else value = (value * 1023/100) + 1024;
    SendMessageDyna(id, 0x0005, 0x0020, value);
}
void  ReadDyna(){
    //MyCyclone_Write(CYCLONE_IO_B_Data,(0x01<<8));
    //MyCyclone_Write(CYCLONE_IO_B_Data,0x14);
    //MyCyclone_Write(CYCLONE_IO_B_Data,0x0c);

   int ID1 = MyCyclone_Read(CYCLONE_IO_Q_Data)&0x00FF;
    int ID3 = MyCyclone_Read(CYCLONE_IO_R_Data);
    //int ID2 = MyCyclone_Read(CYCLONE_IO_F_Data)&0xFF00;
   /* char s[64];
    sprintf(s, "ERROR = %d, Para1 = %d, Para2 = %d\n", ID1, ID3, ID3);
    MyConsole_SendMsg(s);*/
}
void StopTurn(int id, int i){
    if(i == 1){
        SendMessageDyna(id, 0x0005, 0x0020, 0);
    }
    else
        SendMessageDyna(id, 0x0005, 0x0020, 1024);
}
void SetAngle(int id, int angle){
    angle = angle *1023/300;
    if(angle < 0) angle = 0;
    if(angle > 1023) angle = 1023;
    SendMessageDyna(id, 0x0005, 0x001E, angle);
}
void WhichPosition(int id){
    SendMessageDyna2(id, 0x0004, 0x0024, 0x2);
    int ID = MyCyclone_Read(CYCLONE_IO_Q_Data)&0x00FF;
    int Error = MyCyclone_Read(CYCLONE_IO_Q_Data) & 0xFF00;
    int P1 = MyCyclone_Read(CYCLONE_IO_R_Data)&0x00FF;
    int P2 = MyCyclone_Read(CYCLONE_IO_R_Data) & 0xFF00;
    double allQ = MyCyclone_Read(CYCLONE_IO_Q_Data);
    double allR= MyCyclone_Read(CYCLONE_IO_R_Data);
    //int ID2 = MyCyclone_Read(CYCLONE_IO_F_Data)&0xFF00;
   /* char s[128];
    sprintf(s, "Id = %d, Error = %d, P1 = %d, P2 = %d, allQ = %f, allR = %f\n", ID, Error, P1, P2, allQ, allR);
    MyConsole_SendMsg(s);*/
}
void SendMessageDyna2(int id, int size, int address, double value){
    unsigned int Data1;
    unsigned int Data2;
    unsigned int Data3;
    unsigned int Data4;
    unsigned int Data5;
    unsigned int Data6;
    unsigned int Data7;
    unsigned int Data8;
    unsigned short CRC;
    double i = 0;

    //unsigned int val1 = value*1024/300;

    //while(1){
        //val1 = abs(1024*sin(i/100));
        //MyDelayMs(10);
    Data1 = id;  //id (communiquer avec tous)
    Data2 = 0x04;   //Length_L
    Data3 = 0x0002;   //Instruction
    Data5 = address;//0x001e;   //P0  	Write data start address LSB (ou 0x32)
    //Data6 = (int) value;
    Data6 = 0x02;   //P1        Write data start address MSB
    Data7 =  0x0;   //P2        Write data 1st byte
    Data4 =  ~ ( Data1 + Data2 + Data3 + Data5 + Data6 + Data7);


    /*char s[64];
    sprintf(s,"angle = %d\n", Data6 );
    MyConsole_SendMsg(s);*/



    // A_IO_dynDrive = CYCLONE_IO_B_Data [15:0]
    // A_IO_dynDataWrite_1 = CYCLONE_IO_B_Data [31:16]
    // A_IO_dynDataWrite_2 = CYCLONE_IO_C_Data [15:0]
    // A_IO_dynDataWrite_3 = CYCLONE_IO_C_Data [31:16]
    // A_IO_dynDataWrite_4 = CYCLONE_IO_D_Data [15:0]
    //

    MyCyclone_Write(CYCLONE_IO_S_Data,0x0000); // Mise à 0 des commandes d'écritures      A_IO_dynDrive
    MyCyclone_Write(CYCLONE_IO_S_Data, 0x0000); // Mise à 0 de TXD_Enable
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0014);
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0000); // Mise à 0 des commandes d'écritures

    MyCyclone_Write(CYCLONE_IO_S_Data,(Data1 << 8) & 0xFF00);
    MyCyclone_Write(CYCLONE_IO_O_Data, ((Data3 << 8) & 0xFF00) + (Data2&0x00FF));
    //MyCyclone_Write(A_IO_dynDataWrite_3,Data3);
    MyCyclone_Write(CYCLONE_IO_P_Data, (Data4) & 0x00FF);
    MyCyclone_Write(CYCLONE_IO_S_Data,((Data1 << 8) & 0xFF00) + (0x15 & 0x00FF));
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0000);
    MyCyclone_Write(CYCLONE_IO_S_Data,(Data5<<8)&0xFF00);
    MyCyclone_Write(CYCLONE_IO_O_Data,((Data7<<8)&0xFF00) + (Data6&0x00FF));
    //MyCyclone_Write(CYCLONE_IO_C_Data,Data6);
    //MyCyclone_Write(A_IO_dynDataWrite_3,Data7);
    MyCyclone_Write(CYCLONE_IO_S_Data,((Data5<<8)&0xFF00) + (0x16 & 0x00FF));
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0000);

    MyCyclone_Write(CYCLONE_IO_S_Data, (0x01<<8)&0xFF00);
    MyCyclone_Write(CYCLONE_IO_S_Data, ((0x01<<8)&0xFF00) + (0x14 & 0x00FF));
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0000); // Mise à 0 des commandes d'écritures
    MyCyclone_Write(CYCLONE_IO_S_Data,0x0c);
    i = i+1;
}

#endif // REALBOT


#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT