/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "lwip.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "api.h"
#include "modbus.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "lwip/apps/fs.h"
#include "timers.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

  uint16_t adcBuffer[457] = {0};


  volatile uint16_t REGISTERS[9];
  uint8_t OLED = 1;
  uint8_t t = 0;
  uint8_t rxBuffer[8] = {0};
  extern uint8_t TIME_RESET_OLED;
    typedef struct 
    {
      uint8_t seconds;  
      uint8_t minutes;  
      uint8_t hours;    
      uint16_t days;     
      
    } TimeStruct;
    
        typedef enum 
    {
      MAC,
      IP,
      NETMASK,
      GATEWAY,
      SERIAL,
      RS485SPEED,
      RS485PARITIY,
      RS485STOPBIT
    } FlashDataType;
    
  extern uint16_t start;
  extern uint8_t fff;
  extern TimeStruct time;
  extern uint8_t USART_3_SPEED[10];
  extern uint8_t uartStopBits[10];
  extern uint8_t uartPARITY[10]; 
  uint32_t usart_speed = 0;
  
  #define START_ADDR 0x08020000
  
 uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] @ ".ccmram";
  
  TimeStruct time_raw = {0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void vTimerCallback(TimerHandle_t xTimer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

    
     
    

 
int main(void)
{

  /* USER CODE BEGIN 1 */
    __disable_irq();
    SCB->VTOR = START_ADDR;
    __enable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  REGISTERS[2] = 1;
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI4_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  
  uint8_t TEST[10] = {0};
  uint8_t test = 0;
  
  ReadFlash(RS485SPEED, TEST);
  for (int i = 0; i < 4; i++) 
  {
        if (TEST[i] != 0xFF) 
        {
          test = 1;
        }
  }
  if(test == 1)
  {
        ReadFlash(RS485SPEED, USART_3_SPEED);
        if((USART_3_SPEED[0] == 0x31) && (USART_3_SPEED[1] == 0x32))
        {
          usart_speed = 1200;
        }
        else if ((USART_3_SPEED[0] == 0x39) && (USART_3_SPEED[1] == 0x36))
        {
          usart_speed = 9600;
        }
        else if ((USART_3_SPEED[0] == 0x31) && (USART_3_SPEED[1] == 0x39))
        {
          usart_speed = 19200;
        }
        else if ((USART_3_SPEED[0] == 0x33) && (USART_3_SPEED[1] == 0x38))
        {
          usart_speed = 38400;
        }
        else if ((USART_3_SPEED[0] == 0x35) && (USART_3_SPEED[1] == 0x37))
        {
          usart_speed = 57600;
        }
        else if ((USART_3_SPEED[0] == 0x31) && (USART_3_SPEED[1] == 0x31))
        {
          usart_speed = 115200;
        }
        
        test = 0;
  }
  else
  {
    usart_speed = 115200;
  }
  
  
  ReadFlash(RS485PARITIY, TEST);
  for (int i = 0; i < 4; i++) 
  {
        if (TEST[i] != 0xFF) 
        {
          test = 1;
        }
  }
  if(test == 1)
  {
        ReadFlash(RS485PARITIY, uartPARITY);
        test = 0;
  }
  else
  {
    strcpy(uartPARITY, "none");
  }
  
  
  
  
  ReadFlash(RS485STOPBIT, TEST);
  for (int i = 0; i < 4; i++) 
  {
        if (TEST[i] != 0xFF) 
        {
          test = 1;
        }
  }
  if(test == 1)
  {
        ReadFlash(RS485STOPBIT, uartStopBits);
        test = 0;
  }
  else 
  {
    uartStopBits[0] = 1;
  }
  

  
  
  
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer, sizeof(rxBuffer));
  //HAL_UART_Receive_DMA(&huart1, rxBuffer, sizeof(rxBuffer));
  //__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();
  TimerHandle_t xTimer = xTimerCreate("Timer", pdMS_TO_TICKS(1000), pdTRUE, 0, vTimerCallback);
  
    if (xTimer != NULL)
    {
        xTimerStart(xTimer, 0);
    }

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  HAL_GPIO_WritePin(RST_PHYLAN_GPIO_Port, RST_PHYLAN_Pin, GPIO_PIN_RESET);
  osDelay(19);
  HAL_GPIO_WritePin(RST_PHYLAN_GPIO_Port, RST_PHYLAN_Pin, GPIO_PIN_SET);  // - PHY init
  osDelay(19);
  
  //HAL_GPIO_WritePin(UART2_RE_DE_GPIO_Port, UART2_RE_DE_Pin, GPIO_PIN_RESET); 
  
  
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
  //uint8_t rxBuffer[10];
  //HAL_UART_Receive_DMA(&huart1, rxBuffer, sizeof(rxBuffer));
  
  //__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  
  
 // MX_LWIP_Init();
  
  
  
  // - task priority should been normal
  
    //HAL_GPIO_WritePin(RST_PHYLAN_GPIO_Port, RST_PHYLAN_Pin, GPIO_PIN_RESET);
   //HAL_Delay(2);
   // HAL_GPIO_WritePin(RST_PHYLAN_GPIO_Port, RST_PHYLAN_Pin, GPIO_PIN_SET);
   // HAL_Delay(2);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //scale3 is normal on 90mhz  ------------------------------

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)  //latency 2 is normal on 90mhz
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */
void vTimerCallback(TimerHandle_t xTimer)
{
      time_raw.seconds++;
      
      
        if (time_raw.seconds >= 60)
          {
            
            
            time_raw.seconds = 0;
            time_raw.minutes++;
            REGISTERS[3] = (time_raw.minutes + (time_raw.hours * 60) + (time_raw.days * 24 * 60));
             
             if(t == TIME_RESET_OLED)
             {
               t = 0;
               OLED = 0;              
             }

            if (time_raw.minutes >= 60)
              {
                  time_raw.minutes = 0;
                  time_raw.hours++;
                  if (time_raw.hours >= 24)
                    {
                        time_raw.hours = 0;
                        time_raw.days++;
                    }
              }
          }
        
       time.seconds = time_raw.seconds;
       time.minutes = time_raw.minutes;
       time.hours = time_raw.hours;
       time.days = time_raw.days;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
