#include "SDInterface.h"
#include "RoboticFiles/CtrlStruct_gr2.h"
#include "MDD File System/FSIO.h"

void CreateBuffer(char *buffer){
    strcpy(buffer, "");
}
void AddElement(double value, char *buffer){;
    char temp[64]; //Double in 4 bytes;
    sprintf(temp,"%f \n",value);
    strcat(buffer,temp);
}

void WriteSDMemory(char *buffer, char *FileName, unsigned int size){
    MyConsole_SendMsg("here5");
    FSFILE *pointer;
    if (!MDD_MediaDetect()) {
       MyMDDFS_RestoreSPI();
       MyConsole_SendMsg("MyMDDFS - Error MDD_MediaDetect\n>");
       return;
   }

   // Initialize the library
   if (!FSInit()) {
       MyMDDFS_RestoreSPI();
       MyConsole_SendMsg("MyMDDFS - Error FSInit\n>");
       return;
   }
    pointer = FSfopen(FileName, "w");
    if (pointer == NULL)
      MyConsole_SendMsg("SD Interface - Error FSfopen\n>");
    FSfwrite(buffer, size, 1, pointer);
    if (FSfclose(pointer))
      MyConsole_SendMsg("SD Interface - Error FSfclose\n>");
    MyConsole_SendMsg("SD Interface finished \n>");
    //MyConsole_SendMsg(buffer);
    MyMDDFS_RestoreSPI();
}