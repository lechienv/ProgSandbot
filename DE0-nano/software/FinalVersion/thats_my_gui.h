/*
 * thats_my_gui.h
 *
 *  Created on: 5 avr. 2016
 *      Author: Vianney Lechien
 */

#ifndef THATS_MY_GUI_H_
#define THATS_MY_GUI_H_

/* Includes */
#include "terasic_lib/terasic_includes.h"
#include "graphic_lib/alt_video_display.h"
#include "terasic_lib/touch_spi.h"
#include "graphic_lib/simple_graphics.h"
#include "graphic_lib/geometry.h"

/* Typedef */
typedef struct{
    RECT rcUp;
    alt_u32 UpColor;
    alt_u32 UpEdgeColor;
    RECT rcDown;
    alt_u32 DownColor;
    alt_u32 DownEdgeColor;
}TwoRect;

typedef enum{StateFisrtMenu, StateCalibMenu, StateChooseColorMenu} StateMenu;

/* header */
void my_GUI_ShowWelcome(alt_video_display *pDisplay);
void Set_Menu(alt_video_display *pDisplay, TwoRect *Menu, alt_u32 *Color);
void Draw_Menu(alt_video_display *pDisplay, TwoRect *Menu, char *UpMessage, char *DownMessage);
void my_GUI(alt_video_display *pDisplay, TOUCH_HANDLE *pTouch);
void Finish(alt_video_display *pDisplay, char *Message, alt_u32 Color);


#endif /* THATS_MY_GUI_H_ */
