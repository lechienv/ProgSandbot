#include "rateau_gr2.h"

#ifndef REALBOT
NAMESPACE_INIT(ctrlGr2);
#endif // ! REALBOT

/*********************
*** FUNCTIONS ********
**********************/

 void StartMyRat(CtrlStruct *cvs){
      
    if(cvs->time >5 && cvs->time <20)
    {   
        RatGoTop(cvs, cvs->MotorRatL); 
        RatGoTop(cvs, cvs->MotorRatR); 
    }
    else if (cvs->time >30 && cvs->time <50)
    { RatGoBottom(cvs, cvs->MotorRatL);
    RatGoBottom(cvs, cvs->MotorRatR);
    }
    else
    { 
        cvs->MotorRatL->dutyCycle = 0;
    }
  
}
 
  /*void UpdateRatPosition(CtrlStruct *cvs){
      double my_cst_ratL =0.01;
           cvs->MotorRatL->position = cvs->MotorRatL->position + cvs->MotorRatL->dutyCycle * my_cst_ratL * cvs->timeStep;
  }*/
  
  bool RatGoTop(CtrlStruct *cvs, Motor *Motor){

               if( Motor->position >50 && Motor->position <150) //150
               {   
                 Motor->dutyCycle = +35;
                 return false;
               }
               else if( Motor->position <50)
               {
                  
                 Motor->dutyCycle = +25;
                 return false;
               }
               else
               { 
                   Motor->dutyCycle = 0;
                   return  true;
               }
  }
 bool RatGoBottom(CtrlStruct *cvs, Motor *Motor){
     // UpdateRatPosition(cvs);
     bool my_bool = (Motor->ID==RATLMOTOR ) ? !cvs->Sensors->uSwitchRatR : !cvs->Sensors->uSwitchRatL ;
     if(my_bool)
     {
        if( Motor->position <50)
        {
           Motor->dutyCycle = -10;
        }
        else
        {
           Motor->dutyCycle = -20;
        }
         return false;
     }
     else
     { 
       Motor->position =0;
       Motor->dutyCycle = 0;
        return true; 
     }
    
  }

 

#ifndef REALBOT
NAMESPACE_CLOSE();
#endif // ! REALBOT