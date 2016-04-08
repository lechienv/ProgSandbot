/*
 * Author : L. Moreau, A. Willème 2016
 *
 * Based on Terasic LT24 Painter's demo
 *
 */

#include "terasic_lib/terasic_includes.h"
#include "terasic_lib/touch_spi.h"
#include "graphic_lib/alt_video_display.h"
#include "graphic_lib/ILI9341.h"
#include "thats_my_gui.h"

int main()
{
Clr_BUFFER_FLAG();

 alt_video_display Display;
 TOUCH_HANDLE *pTouch;

 printf("Hi There !\n");

 // Write 0x3C on LED[6:0] through the dedicated custom IP
 IOWR(LED_CTRL_BASE, 0x0, 0x3C);

 // TOUCH INITIALIZATION
 pTouch = Touch_Init(LT24_TOUCH_SPI_BASE,  LT24_TOUCH_PENIRQ_N_BASE, LT24_TOUCH_PENIRQ_N_IRQ);
 if (!pTouch){
	 printf("Failed to init touch\r\n");
 }else{
	 printf("Init touch successfully\r\n");
 }

 // LCD INITIALIZATION
 LCD_Init();

 // my_GUI
  Clr_BUFFER_FLAG();
  Display.interlace = 0;
  Display.bytes_per_pixel = 2;
  Display.color_depth = 16;
  Display.height = SCREEN_HEIGHT;
  Display.width = SCREEN_WIDTH;
  my_GUI(&Display, pTouch);

 return 0;
}
