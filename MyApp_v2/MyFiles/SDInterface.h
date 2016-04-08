#ifndef SDINTERFACE_H_
#define SDINTERFACE_H_
#include "MyApp.h"
void CreateBuffer(char *buffer);
void AddElement(double value, char *buffer);
void WriteSDMemory(char *buffer, char *FileName, unsigned int size);

#endif