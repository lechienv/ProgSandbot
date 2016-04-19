#include "UpdateEcran.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

/*********************
*** FUNCTIONS ********
**********************/
void MyUpdateEcran(CtrlStruct *cvs, unsigned int M)
{
  switch(M){
    case(3) :
    {
             cvs->robotID = GREEN;
    }
    case(4) :
    {
             cvs->robotID = PINK;
    }
            
  }
}


#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT