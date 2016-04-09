/*
 * thats_my_gui.c
 *
 *  Created on: 5 avr. 2016
 *      Author: Vianney Lechien
 */

/* Includes */
#include "thats_my_gui.h"


void my_GUI_ShowWelcome(alt_video_display *pDisplay){
	int x, y;


	x = 5;
	y = pDisplay->height / 2 - 10;

	vid_print_string_alpha(x, y, BLUE_24, BLACK_24, tahomabold_20, pDisplay, "Team Mr. Sandbot :");
	vid_print_string_alpha(x, y+22, BLUE_24, BLACK_24, tahomabold_20, pDisplay, "Welcome in the brain");
	vid_print_string_alpha(x, y+44, BLUE_24, BLACK_24, tahomabold_20, pDisplay, "of Mr. Sandbot");
}

//----------------------------------------------------------------------------------------------------

void Set_Menu(alt_video_display *pDisplay, TwoRect *Menu, alt_u32 *Color){

	int left = (pDisplay->width - 180) / 2;
	int right = left + 180;
	int top = (pDisplay->height / 4) - 40;
	int bottom = top + 80;
	int verticalOffset = pDisplay->height /2;

	RectSet(&Menu->rcUp, left, right, top, bottom);
	RectSet(&Menu->rcDown, left, right, top + verticalOffset , bottom + verticalOffset );

	Menu->UpColor = Color[0];
	Menu->UpEdgeColor = Color[1];
	Menu->DownColor = Color[2];
	Menu->DownEdgeColor = Color[3];
}

//----------------------------------------------------------------------------------------------------

void Finish(alt_video_display *pDisplay, char *Message, alt_u32 Color){
	RECT rc;
	int left = (pDisplay->width - 180) / 2;
	int right = left + 180;
	int top = (pDisplay->height / 2) - 40;
	int bottom = top + 80;

	RectSet(&rc, left, right, top, bottom);

	vid_draw_box (left, top, right, bottom, Color , DO_NOT_FILL, pDisplay);
	vid_print_string_alpha(left+50, top + (80 - 22) / 2, Color, WHITE_24, tahomabold_20, pDisplay, Message);
}

//----------------------------------------------------------------------------------------------------

void Draw_Menu(alt_video_display *pDisplay, TwoRect *Menu, char *UpMessage, char *DownMessage){
	vid_draw_box (Menu->rcUp.left  , Menu->rcUp.top  , Menu->rcUp.right  , Menu->rcUp.bottom  , Menu->UpEdgeColor  , DO_NOT_FILL, pDisplay);
	vid_draw_box (Menu->rcDown.left, Menu->rcDown.top, Menu->rcDown.right, Menu->rcDown.bottom, Menu->DownEdgeColor, DO_NOT_FILL, pDisplay);

	vid_print_string_alpha(Menu->rcUp.left+8  , Menu->rcUp.top+(RectHeight(&Menu->rcUp)-22)/2    , Menu->UpColor  , BLACK_24, tahomabold_20, pDisplay, UpMessage);
	vid_print_string_alpha(Menu->rcDown.left+8, Menu->rcDown.top+(RectHeight(&Menu->rcDown)-22)/2, Menu->DownColor, BLACK_24, tahomabold_20, pDisplay, DownMessage);
}

//----------------------------------------------------------------------------------------------------

void my_GUI(alt_video_display *pDisplay, TOUCH_HANDLE *pTouch){
    // video
    int X, Y, Xold, Yold;
    POINT Pt;
    TwoRect rectMenu;
    TwoRect CalibMenu;
    TwoRect ColorMenu;
    StateMenu State = StateFisrtMenu;

    alt_u32 ColorFisrt[4]  = {WHITE_24, WHITE_24, WHITE_24, WHITE_24};
    alt_u32 ColorCalib[4]  = {WHITE_24, WHITE_24, CYAN_24, CYAN_24};
    alt_u32 ColorChoose[4] = {GREEN_24, GREEN_24, VIOLET_24, VIOLET_24};

    volatile int * response = (int*) RESPONSETL24_BASE;

    // clean screen
    vid_clean_screen(pDisplay, BLACK_24);

    // Show Welcome Message
    my_GUI_ShowWelcome(pDisplay);
    usleep(1*1000*1000);
    vid_clean_screen(pDisplay, BLACK_24);

    // Set different menus
    Set_Menu(pDisplay, &rectMenu,  ColorFisrt);
    Set_Menu(pDisplay, &CalibMenu, ColorCalib);
    Set_Menu(pDisplay, &ColorMenu, ColorChoose);

    // Draw first menu
    vid_clean_screen(pDisplay, BLACK_24); // clean screen
    Draw_Menu(pDisplay, &rectMenu, "Calibration", "Choose Color");

    Xold = 0;
    Yold = 0;
    X = 0;
    Y = 0;

    while(1){
    	Touch_GetXY(pTouch, &X, &Y);
    	if(fabs(Xold - X) >= 5 && fabs(Yold - Y) >= 5){
    		Xold = X;
    		Yold = Y;
    		printf("x=%d, y=%d\r\n", X,Y);
    		PtSet(&Pt, X, Y);
    		switch(State){
    		case(StateFisrtMenu):
							if (IsPtInRect(&Pt, &rectMenu.rcUp)){
								State = StateCalibMenu;
								vid_clean_screen(pDisplay, BLACK_24); // clean screen
								Draw_Menu(pDisplay, &CalibMenu, "X et Theta", "Y et Theta");
							}
							else if(IsPtInRect(&Pt, &rectMenu.rcDown)){
								State = StateChooseColorMenu;
								vid_clean_screen(pDisplay, BLACK_24); // clean screen
								Draw_Menu(pDisplay, &ColorMenu, "Green", "Violet");
							}
    		break;

    		case(StateCalibMenu):
						if (IsPtInRect(&Pt, &CalibMenu.rcUp)){
							printf("X \n");
							vid_clean_screen(pDisplay, BLACK_24); // clean screen
							Finish(pDisplay, "X et Theta", RED_24);
							*response = 0;
							return;
						}
						else if(IsPtInRect(&Pt, &CalibMenu.rcDown)){
							printf("Y \n");
							vid_clean_screen(pDisplay, BLACK_24); // clean screen
							Finish(pDisplay, "Y et Theta", RED_24);
							*response = 1;
							return;
						}
    		break;

    		case(StateChooseColorMenu):
						if (IsPtInRect(&Pt, &ColorMenu.rcUp)){
							printf("Green \n");
							vid_clean_screen(pDisplay, BLACK_24); // clean screen
							Finish(pDisplay, "GREEN", GREEN_24);
							*response = 2;
							return;
						}
						else if(IsPtInRect(&Pt, &ColorMenu.rcDown)){
							printf("Violet \n");
							vid_clean_screen(pDisplay, BLACK_24); // clean screen
							Finish(pDisplay, "VIOLET", VIOLET_24);
							*response  = 3;
							return;
						}
    		break;

    		default:
    			*response  = -1;
    			return;
    			break;

    		} // switch
    	} // if touch
    } // while
}
