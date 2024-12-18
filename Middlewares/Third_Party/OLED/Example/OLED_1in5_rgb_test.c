#include "test.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "OLED_1in5_rgb.h"
#include "ImageData.h"
#include <stdbool.h>
#include "cmsis_os2.h"
#include "string.h"
#include "lwip.h"
#include "lwip/netif.h"
#include "timers.h"


#ifdef DEBUG
extern UBaseType_t HighWaterMarkOLED;
#endif

// Queue OLED
extern osMessageQueueId_t QueueOLEDHandle;
extern void WriteReg(uint8_t Reg);
extern uint8_t BlackImage[32768];
extern uint8_t OLED;
extern struct netif gnetif;
extern uint16_t REGISTERS[3];
extern osEventFlagsId_t evt_id;
extern uint8_t IP_ADDRESS[4];
extern uint8_t NETMASK_ADDRESS[4];
extern uint8_t GATEWAY_ADDRESS[4];
extern uint16_t BACKGROUND_COLOR;
extern uint8_t TERGET_VALUE;
extern uint8_t OLED_RESET;
extern uint32_t TIME_RESET_OLED;
uint8_t theme = 1;
extern uint8_t RS485;
extern uint8_t SOFTWARE_VERSION[3];
extern uint32_t usart_speed;
extern char uartStopBits[];
extern char uartPARITY[];

void DrawCenteredSemiCircle();
void DrawCenteredSemiCircle2(UWORD percent);
void OLED_1in5_rgb_run();
void OLED_TimerCallback(TimerHandle_t xTimer);
#define HOLDING_TIME 3000
#define INIT_TIME 8000
uint8_t OLED_START = 1;
#define WIDTH 128   
#define HEIGHT 128


uint8_t OLED_RESET1 = 0;


void OLED_1in5_rgb_run() {


    TimerHandle_t myTimer = xTimerCreate
  (
    "OneShotTimer",                       
    pdMS_TO_TICKS(TIME_RESET_OLED),            
    pdFALSE,                              
    (void *) 0,                           
    OLED_TimerCallback                      
  );



  //
  char cStr[24] = {0};
  
  
  
  // display initialization
  OLED_1in5_rgb_Init();  
  osDelay(500);
  // clear screen
  OLED_1in5_rgb_Clear();
  // create image
  Paint_NewImage(BlackImage, OLED_1in5_RGB_WIDTH, OLED_1in5_RGB_HEIGHT, 0, BLACK);
  // set scale
  Paint_SetScale(65);
  // Select Image
  Paint_SelectImage(BlackImage);
  // draw image on the screen
  

  
  WriteReg(0xae);
  osDelay(1000);
  // display on
  WriteReg(0xaf);
  //

  
  OLED_1in5_rgb_Init();
  OLED_1in5_rgb_Clear();

  OLED_1in5_rgb_Display(BlackImage);
  osDelay(1500);
  OLED_1in5_rgb_Init();
  OLED_1in5_rgb_Clear();
  Paint_SelectImage(BlackImage);
      
      
  Fill_DarkGrayGradient();
  
  
  if(OLED_START)
  {


    /*
    Paint_DrawString_EN(25, 4, "MAC_ADDRESS", &Courier12R, WHITE, BACKGROUND_COLOR);
    sprintf(cStr, "%02X.%02X.%02X.%02X.%02X.%02X",  gnetif.hwaddr[0], gnetif.hwaddr[1], gnetif.hwaddr[2], gnetif.hwaddr[3], gnetif.hwaddr[4], gnetif.hwaddr[5]);
    Paint_DrawString_EN(5, 22, cStr, &Courier12R, WHITE, BACKGROUND_COLOR);
    Paint_DrawLine(0, 40, 128, 40, BLACK, 1, 0);   // 0 or LINE_STYLE_DOTTED
    */
    Paint_DrawString_EN(33, 4, "IP АДРЕС", &Courier12R, WHITE, BACKGROUND_COLOR);
    sprintf(cStr, "%d.%d.%d.%d",  IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
    Paint_DrawString_EN(15, 22, cStr, &Courier12R, WHITE, BACKGROUND_COLOR);
    Paint_DrawLine(0, 40, 128, 40, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

    
    Paint_DrawString_EN(45, 42, "МАСКА", &Courier12R, WHITE, BACKGROUND_COLOR);
    sprintf(cStr, "%d.%d.%d.%d",  NETMASK_ADDRESS[0], NETMASK_ADDRESS[1], NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
    Paint_DrawString_EN(15, 62, cStr, &Courier12R, WHITE, BACKGROUND_COLOR);
    Paint_DrawLine(0, 80, 128, 80, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    

      
    Paint_DrawString_EN(45, 82, "ШЛЮЗ", &Courier12R, WHITE, BACKGROUND_COLOR);
    sprintf(cStr, "%d.%d.%d.%d", GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);
    Paint_DrawString_EN(15, 102, cStr, &Courier12R, WHITE, BACKGROUND_COLOR);
    Paint_DrawLine(0, 120, 128, 120, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    
    OLED_1in5_rgb_Display(BlackImage);
    
    osDelay(4000);
    
    Fill_DarkGrayGradient();
    
    Paint_DrawString_EN(10, 4, "настройки RS-485", &Courier12R, WHITE, BACKGROUND_COLOR);
    Paint_DrawLine(0, 24, 128, 24, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);   // 0 or LINE_STYLE_DOTTED
    
    
    
    
    Paint_DrawString_EN(5, 42, "битрейт - ", &Courier12R, WHITE, BACKGROUND_COLOR);
    sprintf(cStr, "%d",  usart_speed);
    Paint_DrawString_EN(74, 42, cStr, &Courier12R, WHITE, BACKGROUND_COLOR);
    Paint_DrawString_EN(5, 62, "длина слова - 8b", &Courier12R, WHITE, BACKGROUND_COLOR);
    const char *string = "none";
    const char *string2 = "even";
    const char *string3 = "odd";
    
    if(strcmp(uartPARITY, string) == 0)
    {
      Paint_DrawString_EN(5, 82, "бит четности - No", &Courier12R, WHITE, BACKGROUND_COLOR);
    }
    else if(strcmp(uartPARITY, string2) == 0)
    {
      Paint_DrawString_EN(5, 82, "бит четности even", &Courier12R, WHITE, BACKGROUND_COLOR);
    }
    else if(strcmp(uartPARITY, string3) == 0)
    {
      Paint_DrawString_EN(5, 82, "бит четности - odd", &Courier12R, WHITE, BACKGROUND_COLOR);
    }
 
    sprintf(cStr, "стоп бит - %d",  uartStopBits[0]);
    
    Paint_DrawString_EN(5, 102, cStr, &Courier12R, WHITE, BACKGROUND_COLOR);
    
    Paint_DrawLine(0, 120, 128, 120, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    
    
    
    
    
    OLED_1in5_rgb_Display(BlackImage);
    
    osDelay(4000);
    
    Fill_DarkGrayGradient();
    
    OLED_START = 0;
  }
  
  
  while (OLED) 
    {
      
      int OLED_RESET2 = 0;
      
      if(OLED_RESET2) //OLED_RESET2 - OFF 
      {
        

                  

          OLED_1in5_rgb_Init();  
          //osDelay(500);
          // clear screen
          OLED_1in5_rgb_Clear();
          // create image
          Paint_NewImage(BlackImage, OLED_1in5_RGB_WIDTH, OLED_1in5_RGB_HEIGHT, 0, BLACK);
          // set scale
          Paint_SetScale(65);
          // Select Image
          Paint_SelectImage(BlackImage);
          // draw image on the screen
          
          

          
          WriteReg(0xae);
          osDelay(1000);
          // display on
          WriteReg(0xaf);

          
          OLED_1in5_rgb_Init();
          OLED_1in5_rgb_Clear();
          
          Fill_DarkGrayGradient();
          Paint_DrawPoint(40, 64, WHITE, DOT_PIXEL_5X5, DOT_FILL_AROUND);
          Paint_DrawPoint(64, 64, WHITE, DOT_PIXEL_2X2, DOT_FILL_AROUND);
          Paint_DrawPoint(88, 64, WHITE, DOT_PIXEL_2X2, DOT_FILL_AROUND);
          OLED_1in5_rgb_Display(BlackImage);
          osDelay(600);
          
          Fill_DarkGrayGradient();
          Paint_DrawPoint(40, 64, WHITE, DOT_PIXEL_2X2, DOT_FILL_AROUND);
          Paint_DrawPoint(64, 64, WHITE, DOT_PIXEL_5X5, DOT_FILL_AROUND);
          Paint_DrawPoint(88, 64, WHITE, DOT_PIXEL_2X2, DOT_FILL_AROUND);          
          OLED_1in5_rgb_Display(BlackImage);
          osDelay(600);
          
          Fill_DarkGrayGradient();
          Paint_DrawPoint(40, 64, WHITE, DOT_PIXEL_2X2, DOT_FILL_AROUND);
          Paint_DrawPoint(64, 64, WHITE, DOT_PIXEL_2X2, DOT_FILL_AROUND);
          Paint_DrawPoint(88, 64, WHITE, DOT_PIXEL_5X5, DOT_FILL_AROUND);          
          OLED_1in5_rgb_Display(BlackImage);
          osDelay(600);
          Fill_DarkGrayGradient();
          
          OLED_RESET = 0;
          xTimerStart(myTimer, 0);
      }



    
      
        /*
       for (UBYTE i = 0; i < 32; i++) {
            Paint_DrawRectangle(0, 4 * i, 127, 4 * (i + 1), i * 1047, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        }
        */
      if(theme == 1) //--------------------------------------------------------------------------------------------------------------
      {
        
      uint16_t mA = REGISTERS[1];
      char buffer[10] = {0};
      uint16_t percent = 0;
      
      if(mA >= TERGET_VALUE) 
      {
        percent = 0;
      }
      else 
      {
        percent = (100 - (mA * 4));
      }
      
      sprintf(buffer, "%u/25mA", mA);
       
      
       Paint_DrawString_EN(30, 75, "РЕЛЕ - ", &Courier12R, WHITE, BACKGROUND_COLOR);
       
       if(REGISTERS[2])
       {
         Fill_DarkGrayGradient();
         Paint_DrawString_EN(30, 75, "РЕЛЕ - ", &Courier12R, WHITE, BACKGROUND_COLOR);
         Paint_DrawString_EN(76, 75, "ВКЛ", &Courier12R, GREEN, BACKGROUND_COLOR);
       }
       else
       {
         Paint_DrawString_EN(76, 75, "ВЫКЛ", &Courier12R, RED, BACKGROUND_COLOR);
       }
       
       
       Paint_DrawLine(8, 90, 120, 90, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
       Paint_DrawLine(64, 90, 64, 120, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
       
       
       if(RS485)
       {
           Paint_DrawString_EN(20, 98, "RS485", &Courier12R, GREEN, BACKGROUND_COLOR);
       }
       else
       {
           Paint_DrawString_EN(20, 98, "RS485", &Courier12R, RED, BACKGROUND_COLOR);
       }
       Paint_DrawString_EN(37, 51, buffer, &Courier12R, WHITE, BACKGROUND_COLOR );
       sprintf(cStr, "V.%d.%d.%d",  SOFTWARE_VERSION[0], SOFTWARE_VERSION[1], SOFTWARE_VERSION[2]);
       Paint_DrawString_EN(67, 98, cStr, &Courier12R, WHITE, BACKGROUND_COLOR);
       



       
       
       DrawCenteredSemiCircle();
       DrawCenteredSemiCircle2(percent);
       
       
       Paint_DrawRectangle(120, 70, 8, 120, WHITE, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
       OLED_1in5_rgb_Display(BlackImage);
       //update = false;
       osDelay(200); //the image will be transmitted in ~188ms (for 1.4MBits/sec)
       
    }//----------------------------------------------------------------------------------------------------------------------------------
    
    osDelay(220);  
    
    if(theme == 2)
    {
      uint16_t mA = REGISTERS[1];
      char buffer[10] = {0};

      
      sprintf(buffer, "%u/25mA", mA);
       
      
     Paint_DrawString_EN(30, 70, "РЕЛЕ - ", &Courier12R, WHITE, BACKGROUND_COLOR);
     
     if(REGISTERS[2])
     {
       Fill_DarkGrayGradient();
       Paint_DrawString_EN(30, 70, "РЕЛЕ - ", &Courier12R, WHITE, BACKGROUND_COLOR);
       Paint_DrawString_EN(76, 70, "ВКЛ", &Courier12R, GREEN, BACKGROUND_COLOR);
     }
     else
     {
       Paint_DrawString_EN(76, 70, "ВЫКЛ", &Courier12R, RED, BACKGROUND_COLOR);
     }
     Paint_DrawString_EN(40, 40, buffer, &Courier12R, WHITE, BACKGROUND_COLOR );
     
     OLED_1in5_rgb_Display(BlackImage);
     osDelay(200); //the image will be transmitted in ~188ms (for 1.4MBits/sec)
    }
  
}


}



void DrawCenteredSemiCircle() 
{
  UWORD centerX = WIDTH / 2;
  UWORD centerY = HEIGHT / 2; 
  UWORD color = 0xA001; 
  DOT_PIXEL lineWidth = DOT_PIXEL_5X5;
  
  Paint_DrawSemiCircle(centerX, centerY, color, lineWidth, 100);
}
void DrawCenteredSemiCircle2(UWORD percent) 
{
  UWORD centerX = WIDTH / 2;
  UWORD centerY = HEIGHT / 2; 
  UWORD color = 0xFFFF; 
  DOT_PIXEL lineWidth = DOT_PIXEL_5X5;
 
  Paint_DrawSemiCircle(centerX, centerY, color, lineWidth, percent);
}

void Fill_DarkGrayGradient() 
{
UWORD X, Y;
UWORD Color = BACKGROUND_COLOR;
for (Y = 0; Y < 128; Y++) {
    for (X = 0; X < 128; X++) {
        Paint_SetPixel(X, Y, Color);
    }
}
  }


void OLED_TimerCallback(TimerHandle_t xTimer) 
{
      OLED_RESET = 1;
}
  

