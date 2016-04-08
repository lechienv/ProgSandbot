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

    Data1 = id;  //id (communiquer avec tous)
    Data2 = size;   //Length_L
    Data3 = 0x0003;   //Instruction
    Data5 = address;//0x001e;   //P0  	Write data start address LSB (ou 0x32)
    Data6 = (int) value & 0x00FF;   //P1        Write data start address MSB
    Data7 =  ((int) value >> 8) & 0x00FF;   //P2        Write data 1st byte
    Data4 =  ~ ( Data1 + Data2 + Data3 + Data5 + Data6 + Data7);

    MyCyclone_Write(CYCLONE_IO_B_Data,0x0000); // Mise à 0 des commandes d'écritures      A_IO_dynDrive
    MyCyclone_Write(CYCLONE_IO_B_Data, 0x0000); // Mise à 0 de TXD_Enable
    MyCyclone_Write(CYCLONE_IO_B_Data,0x0014);
    MyCyclone_Write(CYCLONE_IO_B_Data,0x0000); // Mise à 0 des commandes d'écritures

    MyCyclone_Write(CYCLONE_IO_B_Data,(Data1 << 8) & 0xFF00);
    MyCyclone_Write(CYCLONE_IO_C_Data, ((Data3 << 8) & 0xFF00) + (Data2&0x00FF));
    MyCyclone_Write(CYCLONE_IO_D_Data, (Data4) & 0x00FF);
    MyCyclone_Write(CYCLONE_IO_B_Data,((Data1 << 8) & 0xFF00) + (0x15 & 0x00FF));
    MyCyclone_Write(CYCLONE_IO_B_Data,0x0000);
    MyCyclone_Write(CYCLONE_IO_B_Data,(Data5<<8)&0xFF00);
    MyCyclone_Write(CYCLONE_IO_C_Data,((Data7<<8)&0xFF00) + (Data6&0x00FF));
    MyCyclone_Write(CYCLONE_IO_B_Data,((Data5<<8)&0xFF00) + (0x16 & 0x00FF));
    MyCyclone_Write(CYCLONE_IO_B_Data,0x0000);

    MyCyclone_Write(CYCLONE_IO_B_Data, (0x01<<8)&0xFF00);
    MyCyclone_Write(CYCLONE_IO_B_Data, ((0x01<<8)&0xFF00) + (0x14 & 0x00FF));
    MyCyclone_Write(CYCLONE_IO_B_Data,0x0000); // Mise à 0 des commandes d'écritures
    MyCyclone_Write(CYCLONE_IO_B_Data,0x0c);
}

void TurnCCW(int value){
    if(value <0) value = 0;
    if(value > 100) value = 100;
    else value = value * 1023/100;
    SendMessageDyna(0x06, 0x0005, 0x0020, value);
}
void TurnCW(int value){
    if(value <0) value = 0;
    if(value > 100) value = 100;
    else value = (value * 1023/100) + 1024;
    SendMessageDyna(0x06, 0x0005, 0x0020, value);
}
void  ReadDyna(){
    int ID = MyCyclone_Read(CYCLONE_IO_E_Data);
}
void StopTurn(int i){
    if(i == 1){
        SendMessageDyna(0x06, 0x0005, 0x0020, 0);
    }
    else
        SendMessageDyna(0x06, 0x0005, 0x0020, 1024);
}
#endif // REALBOT


#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT