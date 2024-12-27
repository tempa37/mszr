/* USER CODE BEGIN Header */
/**
******************************************************************************
* File Name          : freertos.c
* Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus.h"
#include "lwip.h"
#include "semphr.h"
#include "lwip/netif.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"
#include "lwip/apps/httpd.h"
#include "cmsis_os.h"
#include "cJSON.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "adc.h"
#include "timers.h"
/* USER CODE END Includes */
#include "test.h"
#include "lwip/tcp.h"
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t packetReceived = 0;

osMutexId_t flashMutexHandle;
osMutexId_t RelayMutexHandle;


extern struct netif gnetif;
extern UART_HandleTypeDef huart3;
extern  uint8_t rxBuffer[50];
extern uint8_t IP_ADDRESS[4];
extern uint8_t NETMASK_ADDRESS[4];
extern uint8_t GATEWAY_ADDRESS[4];
extern uint16_t REGISTERS[4];
extern uint32_t usart_speed;
uint8_t SERIAL_ADDRESS[6] = {0};



uint8_t USART_3_SPEED[10];
uint8_t uartStopBits[10];
char uartPARITY[10];  
uint8_t output[200] = {0};


uint16_t ADC_BUFFER_SIZE = 2000;
extern uint16_t adcBuffer[2000];
uint8_t adc_ready = 0;



//-------------------------------------------------------------------
uint8_t SOFTWARE_VERSION[3] = {0x01, 0x00, 0x01};
uint16_t soft_ver_modbus = 101;
extern struct httpd_state *hs;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct netconn * nc;
struct netconn * in_nc;
struct netbuf * nb;
struct netconn *newconn = NULL;



const char *ssi_tags[] = {"MAC", "IP", "MASK", "GETAWEY", "AMP", "SEC", "MIN", "HOUR", "DAY", "PIN", "RELAY", "SERIAL", "SOFT", "RS485", "SPEED", "PARITY", "STOPB", "SOFTACCEPT"};
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

typedef struct 
{
  uint8_t seconds;  
  uint8_t minutes;  
  uint8_t hours;    
  uint16_t days;  
  
} TimeStruct;

TimeStruct time = {0}; 

//-------------------------------------SETINGS-OLED-------------------------------------------------

uint16_t BACKGROUND_COLOR = 0x0000;   //0x4A49;
uint8_t brightness = 0xFF;  // 0x00-0xFF
uint32_t TIME_RESET_OLED = 18000; // in miliseconds

//---------------------------------------FLASH-OS----------------------------------------------------
static uint16_t iteration = 0;
static uint32_t address = 0;
static uint32_t len = 0;
static uint32_t next_free_addr = 0;
uint8_t error_flash = 0;
//---------------------------------------------------------------------------------------------------


uint8_t TERGET_VALUE = 25;

//PA9_out
#define RELAY_CONTROL_PIN          GPIO_PIN_9
#define RELAY_CONTROL_PORT         GPIOA

//PA6_in
#define FIXING_THE_LEAK_PIN        GPIO_PIN_6
#define FIXING_THE_LEAK_PORT       GPIOA

//PA10_in
#define CHECKING_FOR_LEAKS_PIN     GPIO_PIN_10
#define CHECKING_FOR_LEAKS_PORT    GPIOA

//PA12_in
#define NO_NAME_PIN_PIN            GPIO_PIN_12
#define NO_NAME_PIN_PORT           GPIOA

#define MAC_SIZE 16
#define IP_SIZE 16
#define NETMASK_SIZE 16
#define GATEWAY_SIZE 16
#define SERIAL_SIZE 16 
uint8_t PIN[6] = {0x34, 0x37, 0x32, 0x32, 0x30, 0x31};   // pin in ASCII - 4-7-2-2-0-1
uint8_t pinaccept = 0;




#define FLASH_ADDRESS_MAC 0x0800C000
#define FLASH_ADDRESS_IP  0x0800C010
#define FLASH_ADDRESS_NETMASK 0x0800C020
#define FLASH_ADDRESS_GATEWAY 0x0800C030
#define FLASH_ADDRESS_SERIAL 0x0800C040

#define FLASH_ADDRESS_RS485_SPEED 0x0800C050
#define FLASH_ADDRESS_RS485_PARITIY 0x0800C060
#define FLASH_ADDRESS_RS485_STOPBIT 0x0800C070
#define TIMEOUT_MS 8000


#define BOOT_OS_OK_ADDRESS 0x0800C080
#define BOOT_CRC_ADDRESS 0x0800C200
#define BOOT_FLAG_NEW_ADDRESS 0x0800C084
#define SECTOR_1_ADDRESS 0x08020000 //end in 0x08080000 (sector 5/6/7) 384kb in total
#define SECTOR_2_ADDRESS 0x08080000 // end in 0x080E0000 (sector 8/9/10) 384kb in total
#define BOOTLOADER_ADDRESS 0x08010000 //end in 0x0801FFFF (sector 4)  64kb in total
#define SECTOR_ENABLED_ADDRESS 0x0800C088 //активный сектор
#define SECTOR_1_LEN 0x0800C090
#define SECTOR_2_LEN 0x0800C100

//--------------------------(critical flags)------------------------------------
//после изменения этих флагов нужно вызвать WriteFlash(0, 0); 
//для их автоматической запиши во флеш
uint8_t client_accepted = 0;
uint8_t boot_os_ok = 0;
uint8_t boot_flag_crc = 0;
uint8_t boot_flag_new = 0;
uint8_t sector_enabled = 0;
uint32_t sector_1_len = 0;
uint32_t sector_2_len = 0;
uint32_t crc_os = 0;
//------------------------------------------------------------------------------

uint8_t os_accepted = 0;
uint32_t adc_value = 0;
uint32_t adc_value_2 = 0;
uint8_t tim = 0;
uint8_t restart = 0;
uint8_t connectionFLAG = 0;
SemaphoreHandle_t xPacketSemaphore;
SemaphoreHandle_t xxMutex;
SemaphoreHandle_t xPacketSaved;
CRC_HandleTypeDef hcrc;
uint16_t start = 1;
uint8_t OLED_RESET = 1;
uint8_t RS485 = 0;
uint8_t RX_Flag = 0;
uint8_t i9 = 0;
uint8_t fff = 0;
uint8_t ch = 0;
volatile uint32_t er = 0;
uint8_t response_data[20] = {0};

//-------------------------------------------------------------------------------------------------------------------
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1248 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Relay_task */
osThreadId_t Relay_taskHandle;
const osThreadAttr_t Relay_task_attributes = {
  .name = "Relay_task",
  .stack_size = 1128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mobdus */
osThreadId_t mobdusHandle;
const osThreadAttr_t mobdus_attributes = {
  .name = "mobdus",
  .stack_size = 1500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Relay_control */
osThreadId_t Relay_controlHandle;
const osThreadAttr_t Relay_control_attributes = {
  .name = "Relay_control",
  .stack_size = 1128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for LWGL_control */
osThreadId_t LWGL_controlHandle;
const osThreadAttr_t LWGL_control_attributes = {
  .name = "LWGL_control",
  .stack_size = 1280 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for WDI */
osThreadId_t WDIHandle;
const osThreadAttr_t WDI_attributes = {
  .name = "WDI",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void httpd_ssi_init(void);
uint16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen);
void ReadFlash(FlashDataType type, uint8_t* buffer);
void WriteFlash(FlashDataType type, uint8_t* data);
void convert_str_to_uint8_array(const char* input, uint8_t* output, int is_mac);
void setrelay(uint16_t i);
void send_uart(const uint8_t *response, uint16_t len);
void DrawCenteredSemiCircle2(UWORD percent);
void DrawCenteredSemiCircle();
void startMyTimer_RESET(uint32_t timeout_ms);
void vMyTimerCallback(TimerHandle_t xTimer);
void convert_str_to_uint8_array_serial(const char* input, uint8_t* output);
void EXTI12_Init(void);
void EXTI15_10_IRQHandler(void);
void WriteToFlash(uint32_t startAddress, uint8_t* data, uint32_t length);
void UpdateSector3();
void load_flags_from_flash(void);
HAL_StatusTypeDef Flash_WritePacket(uint8_t *packet, uint16_t packet_size);
uint16_t get_body_length(uint8_t *packet, uint16_t packet_size);
char* extract_body(uint8_t *packet);
const char *handle_file_upload(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
void WriteToFlash(uint32_t startAddress, uint8_t* data, uint32_t length);
void send_ethernet(uint8_t *data, uint16_t len, struct netconn *newconn);
uint16_t adc_get_rms(uint16_t *arr, uint16_t length);
void CleanupResources(struct netconn *nc, struct netconn *newconn, struct netbuf *buf);

//crc32 func
uint32_t test();
uint32_t crc32_formula_normal_STM32( size_t len, void *data );
static uint8_t *reorder4 (uint8_t *src, uint32_t len);
static uint8_t reverse (uint8_t val8);
static uint32_t reflect32 (uint32_t val32);
uint32_t crc32_formula_normal( size_t len, const void *data );
uint32_t crc32_hw_equivalent(size_t len, const void *data);

uint32_t calculate_flash_crc(uint32_t start_address, uint32_t end_address);
void CRC_Config(void);
uint16_t parser_num(uint8_t *buf, uint16_t len, const char *str);
char *parser(uint8_t *buf, uint16_t len, const char *str);



err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd);
err_t httpd_post_receive_data(void *connection, struct pbuf *p);
void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len);





extern void OLED_1in5_rgb_run();


const char * SAVE_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const tCGI LEDS_CGI = {"/save", SAVE_CGI_Handler};
tCGI CGI_TAB[1];
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
* @brief  FreeRTOS initialization
* @param  None
* @retval None
*/
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */
  
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */
  
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  
  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  
  /* creation of Relay_task */
  Relay_taskHandle = osThreadNew(StartTask02, NULL, &Relay_task_attributes);
  
  /* creation of mobdus */
  mobdusHandle = osThreadNew(StartTask03, NULL, &mobdus_attributes);
  
  /* creation of Relay_control */
  Relay_controlHandle = osThreadNew(StartTask04, NULL, &Relay_control_attributes);
  
  /* creation of LWGL_control */
  LWGL_controlHandle = osThreadNew(StartTask05, NULL, &LWGL_control_attributes);
  
  /* creation of WDI */
  WDIHandle = osThreadNew(StartTask06, NULL, &WDI_attributes);
  
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  
  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
  
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief  Function implementing the defaultTask thread.
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  HAL_GPIO_WritePin(RST_PHYLAN_GPIO_Port, RST_PHYLAN_Pin, GPIO_PIN_RESET);
  osDelay(19);
  HAL_GPIO_WritePin(RST_PHYLAN_GPIO_Port, RST_PHYLAN_Pin, GPIO_PIN_SET);  // - PHY init
  osDelay(19);
  
  EXTI12_Init();
  MX_LWIP_Init();
  i9 =1;
  
  if(i9)
  {
    httpd_init();
    httpd_ssi_init();
  }
  
  __HAL_RCC_CRC_CLK_ENABLE();
  CRC_Config(); 
  
  /* USER CODE BEGIN StartDefaultTask */
  
  CGI_TAB[0] = LEDS_CGI;
  http_set_cgi_handlers(CGI_TAB, 1);
  xPacketSemaphore = xSemaphoreCreateBinary();
  xPacketSaved = xSemaphoreCreateBinary();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
  load_flags_from_flash();
  xSemaphoreGive(xPacketSaved);
  /* Infinite loop */

  
  
  for(;;)     //------------------------------------------------------modbus_RTU------------------------------
  {
    
    if (xSemaphoreTake(xPacketSemaphore, portMAX_DELAY) == pdTRUE) 
    {
      
      packetReceived = 0;
      uint8_t data[20] = {0};
      uint16_t len_ext = 0;
      
      uint16_t len = (sizeof(rxBuffer) - __HAL_DMA_GET_COUNTER(huart3.hdmarx));
      
      memcpy(data, rxBuffer, len);
      
      modbus(data, len, response_data, &len_ext, RTU); 
      
      send_uart(response_data, len_ext);   
      
    } 
    osDelay(80);
  }
  
  
  /* USER CODE END StartDefaultTask */
  
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Relay_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  
  REGISTERS[0] = soft_ver_modbus;
  HAL_GPIO_WritePin(UART1_RE_DE_GPIO_Port, UART1_RE_DE_Pin, GPIO_PIN_RESET);
  ReadFlash(SERIAL, SERIAL_ADDRESS);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);
  
  for(;;)
  {
    
    //---------------------------------ADC-and-Realay-logic-------------------------
    
    if(adc_ready == 1)
    {
      uint16_t rms = adc_get_rms(adcBuffer, ADC_BUFFER_SIZE);
      //adc_value = (((float)rms / 4096) * 3.3);
      //REGISTERS[1] = (uint16_t)((adc_value * 3) / (121.24) * 1000);
      //REGISTERS[1] = rms * 0.02;
      REGISTERS[1] = (uint16_t)(rms * 0.019922);  // REGISTERS[1] = ((((rms / 4096) * 3.3) * 3) / (121.1775) * 1000);
    }
    
    if(REGISTERS[1] >= TERGET_VALUE)
    {
      setrelay(0);
    }
    
    if (HAL_GPIO_ReadPin(Fixing_the_leak_GPIO_Port, Fixing_the_leak_Pin) == GPIO_PIN_SET)
    {
      setrelay(0);
    }   
    
    if(restart == 1)
    {
      startMyTimer_RESET(25000);
      restart = 0;
    } 
//-------------------------------------------------------------------------------------------------------CRC-TEST-AREA-----------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------CRC-TEST-AREA-----------------------------------------------------------------------------------------
     sector_enabled = 1;
    
     osDelay(100); 
     CRC->CR |= CRC_CR_RESET;
     osDelay(200); 
     uint32_t tet = test();
    
    
    if(boot_flag_new == 1)
    {
      
      uint32_t array[50] = {0};
      
      for (size_t i = 0; i < 50; i++) 
      {
        array[i] = *((uint32_t*)address + i);
      }
      
      
      
      
      
      uint32_t crc_app_arr =  crc32_formula_normal(50, array);
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);
      
      uint32_t crc_app_arr2 =  crc32_formula_normal(49, array);
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);
      
      uint32_t crc_stm_arr = calculate_flash_crc(address, address+(50 * 4));  
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);
      
      uint32_t crc_stm_arr2 = calculate_flash_crc(address, address+(49 * 4));  
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);

     
      uint32_t crc_stm_arr3 = HAL_CRC_Calculate(&hcrc, array, 50);  
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);      
      
      uint32_t crc_stm_arr4 = HAL_CRC_Calculate(&hcrc, array, 49);  
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);      
      
      
      uint32_t crc_app_2 = crc32_formula_normal_STM32(50, array);
      uint32_t crc_app_2_2 = crc32_formula_normal_STM32(49, array);
      uint32_t crc_app_2_reverce = ~crc32_formula_normal_STM32(50, array);
      
      uint32_t crc_chatGPT = crc32_hw_equivalent(50, array);
      uint32_t crc_chatGPT2 = crc32_hw_equivalent(49, array);
      uint32_t crc_chatGPT_reverce = ~crc32_hw_equivalent(50, array);
      
      
      
      
      
      uint32_t crc_stm = calculate_flash_crc(address, (next_free_addr-4));  
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);
      uint32_t crc_stm2 = calculate_flash_crc(address, (next_free_addr-5));
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);
      uint32_t crc_stm3 = calculate_flash_crc(address, (next_free_addr-3));
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);
      uint32_t crc_static = calculate_flash_crc(0x08080000, 0x080DFFFB);   //start address + 5FFFB (as in IAR opt)
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);   
      uint32_t crc_static2 = calculate_flash_crc(0x08080000, 0x080DFFFC);
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);
      uint32_t crc_static3 = calculate_flash_crc(0x08080000, 0x080DFFFA);
      CRC->CR |= CRC_CR_RESET;
      osDelay(100);
      uint32_t crc_static4 = calculate_flash_crc(0x08080000, 0x080DFFFD);
      uint32_t crc_reverce = ~calculate_flash_crc(address, (next_free_addr-4));
      CRC->CR |= CRC_CR_RESET;
      osDelay(100); 
      
      
      if(crc_os == crc_stm)
      {
        startMyTimer_RESET(2500);
      }
    }
    
//--------------------------------------------------------------------------------------------------------TEST-AREA-END------------------------------------------------------------------------------------
    osDelay(25);
    
    
    //--------------------------------------------------------------------
    
  }
  
  
  /* USER CODE END StartTask02 */
  
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the mobdus thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */

  /* Infinite loop */
  for (;;)
  {
    
    volatile err_t res = 0;
    ip_addr_t local_ip;
    struct netbuf *buf;
    void *data;
    uint16_t len;
    uint16_t len_ext;
    uint8_t buffer[20] = {0};
    local_ip = gnetif.ip_addr;
    err_t bind_result = 0;
    
    
    typedef enum 
    {
      STATE_INIT,
      STATE_CREATE_SOCKET,
      STATE_BIND,
      STATE_LISTEN,
      STATE_ACCEPT,
      STATE_RECEIVE,
      STATE_PROCESS,
      STATE_SEND,
      STATE_CLOSE_CONNECTION,
      STATE_ERROR
    } FSM_State;
    
    FSM_State current_state = STATE_INIT;
    
    
    while (1)
    {
      switch (current_state)
      {
      case STATE_INIT:
        nc = netconn_new(NETCONN_TCP);
        if (nc != NULL) 
        {
          current_state = STATE_CREATE_SOCKET;
        } 
        else 
        {
          current_state = STATE_ERROR;
        }
        break;
        
        //------------------------------------------------------------------------------
      case STATE_CREATE_SOCKET:
        local_ip = gnetif.ip_addr;
        
        nc->pcb.tcp->so_options |= SOF_REUSEADDR;
        
        bind_result = netconn_bind(nc, &local_ip, 502);
        
        if (bind_result == ERR_OK)
        {
          current_state = STATE_LISTEN;
        } 
        else if(bind_result == ERR_USE)
        {
          CleanupResources(nc, newconn, buf);
          current_state = STATE_INIT;
        }
        else
        {
          current_state = STATE_ERROR;
        }
        break;
        
        //------------------------------------------------------------------------------
      case STATE_LISTEN:
        netconn_listen(nc);
        current_state = STATE_ACCEPT;
        break;
        
        //------------------------------------------------------------------------------
      case STATE_ACCEPT:
        res = netconn_accept(nc, &newconn);
        if (res == ERR_OK) 
        {
          if(nc != NULL)
          {
            netconn_close(nc);
            netconn_delete(nc);
          }
          TickType_t last_activity_time = xTaskGetTickCount();
          netconn_set_recvtimeout(newconn, 5000);
          current_state = STATE_RECEIVE;
        } 
        else 
        {
          current_state = STATE_ERROR;
        }
        break;
        
        //------------------------------------------------------------------------------
      case STATE_RECEIVE:
        res = netconn_recv(newconn, &buf);
        if (res == ERR_OK) 
        {
          netbuf_data(buf, &data, &len);
          memcpy(buffer, data, len);
          current_state = STATE_PROCESS;
        } 
        else
        {
          if(buf != 0)
          {
            netbuf_delete(buf);
          }
          current_state = STATE_CLOSE_CONNECTION;
        }
        break;
        
        //------------------------------------------------------------------------------
      case STATE_PROCESS:
        modbus(buffer, len, response_data, &len_ext, TCP);
        current_state = STATE_SEND;
        break;
        
        //------------------------------------------------------------------------------
      case STATE_SEND:
        send_ethernet(response_data, len_ext, newconn);
        netbuf_delete(buf);
        current_state = STATE_RECEIVE;  
        break;
        
        //------------------------------------------------------------------------------
      case STATE_CLOSE_CONNECTION:
        if(newconn != NULL)
        {
          netconn_close(newconn);
          netconn_delete(newconn);
        }
        current_state = STATE_CREATE_SOCKET; 
        break;
        //-----------------------------------------------------------------------------             
      case STATE_ERROR:
        if(nc != NULL)
        {
          netconn_close(nc);
          netconn_delete(nc);
        }
        osDelay(1000);
        current_state = STATE_INIT;  
        break;
      }
    }
  }
  /* USER CODE END StartTask03 */
}



/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Relay_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  
  /* Infinite loop */
  for(;;)
  {
    
    switch(REGISTERS[2])
    {
    case 0:
      HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_RESET);
      //HAL_GPIO_WritePin(Checking_for_leaks_GPIO_Port, Checking_for_leaks_Pin, GPIO_PIN_SET);
      break;
      
    case 1:
      HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_SET);
      //HAL_GPIO_WritePin(Checking_for_leaks_GPIO_Port, Checking_for_leaks_Pin, GPIO_PIN_RESET);
      break;  
    }
    if(fff)
    {
      setrelay(0);
      osDelay(5000);
      start = 0;
      fff = 0;
    }
  

    osDelay(10);
  }
  /* USER CODE END StartTask04 */
  
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the LWGL_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
    if(start == 0)
    {
      OLED_1in5_rgb_run();
    }
    
    osDelay(10);
    
    /* USER CODE END StartTask05 */
  }
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the WDI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  
  /* Infinite loop */
  for(;;)
  {  
    HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
    osDelay(300);
    size_t minFreeHeapSize = xPortGetMinimumEverFreeHeapSize();
    if(HAL_GPIO_ReadPin(RS485_1_ON_GPIO_Port, RS485_1_ON_Pin) == GPIO_PIN_SET)
    {
      RS485 = 0;
    }
    else
    {
      RS485 = 1;
    }
    
          if(ch == 1)
          {
            HAL_GPIO_WritePin(Checking_for_leaks_GPIO_Port, Checking_for_leaks_Pin, GPIO_PIN_SET);
            setrelay(0);
            
            
            osDelay(1000);
            HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_SET);
            osDelay(100);
            HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
            osDelay(1000);
            HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_SET);
            osDelay(100);
            HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
            osDelay(1000);
            HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_SET);
            osDelay(100);
            HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
            osDelay(1000);
            HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_SET);
            osDelay(100);
            HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
            osDelay(1000);
            HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_SET);
            osDelay(100);
            HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);

            
            HAL_GPIO_WritePin(Checking_for_leaks_GPIO_Port, Checking_for_leaks_Pin, GPIO_PIN_RESET);
            setrelay(1);
            ch = 0;
          }
    
  }
  /* USER CODE END StartTask06 */
  
}

/* Private application code */
/* USER CODE BEGIN Application */

void setrelay(uint16_t i)
{
  osMutexWait(RelayMutexHandle, osWaitForever);
  REGISTERS[2] = i;
  osMutexRelease(RelayMutexHandle);
}

//---------------------------------------------------------------------------------HTTPD-SERVER-LOGICS-START---

uint16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen) 
{
  uint8_t buffer[50] = {0};
  uint8_t bufferSize = sizeof(buffer);
  
  if (iIndex == 0) 
  {
    snprintf((char*)buffer, bufferSize, "%02X.%02X.%02X.%02X.%02X.%02X",  gnetif.hwaddr[0], gnetif.hwaddr[1], gnetif.hwaddr[2], gnetif.hwaddr[3], gnetif.hwaddr[4], gnetif.hwaddr[5]);
  }
  else if(iIndex == 1)
  {
    snprintf((char*)buffer, bufferSize, "%d.%d.%d.%d",  IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  }
  else if(iIndex == 2)
  {
    snprintf((char*)buffer, bufferSize, "%d.%d.%d.%d", NETMASK_ADDRESS[0], NETMASK_ADDRESS[1], NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  }
  else if(iIndex == 3)
  {
    snprintf((char*)buffer, bufferSize, "%d.%d.%d.%d", GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);
  }
  else if(iIndex == 4)
  {
    snprintf((char*)buffer, bufferSize, "%u", REGISTERS[1]);
  }
  else if(iIndex == 5)
  {
    snprintf((char*)buffer, bufferSize, "%d", time.seconds);
  }
  else if(iIndex == 6)
  {
    snprintf((char*)buffer, bufferSize, "%d", time.minutes);
  }
  else if(iIndex == 7)
  {
    snprintf((char*)buffer, bufferSize, "%d", time.hours);
  }
  else if(iIndex == 8)
  {
    snprintf((char*)buffer, bufferSize, "%d", time.days);
  }
  else if(iIndex == 9)
  {
    snprintf((char*)buffer, bufferSize, "%d", pinaccept);
  }
  else if(iIndex == 10)
  {
    if(REGISTERS[2])
    {
      snprintf((char*)buffer, bufferSize, "РЕЛЕ В СОСТОЯНИИ - ВКЛ");
    }
    else
    {
      snprintf((char*)buffer, bufferSize, "РЕЛЕ В СОСТОЯНИИ - ВЫКЛ");
    }
  }
  else if(iIndex == 11)
  {
    snprintf((char*)buffer, bufferSize, "%d%d%d%d%d%d", SERIAL_ADDRESS[0], SERIAL_ADDRESS[1], SERIAL_ADDRESS[2], SERIAL_ADDRESS[3], SERIAL_ADDRESS[4], SERIAL_ADDRESS[5]);
  }
  else if(iIndex == 12)
  {
    snprintf((char*)buffer, bufferSize, "%d.%d.%d", SOFTWARE_VERSION[0], SOFTWARE_VERSION[1], SOFTWARE_VERSION[2]);
  }
  else if(iIndex == 13)
  {
    if(RS485)
    {
      snprintf((char*)buffer, bufferSize, "Соединение установлено");
    }
    else
    {
      snprintf((char*)buffer, bufferSize, "Соединение разорвано");
    } 
  }
  else if(iIndex == 14)
  {
    snprintf((char*)buffer, bufferSize, "%lu", (unsigned long)usart_speed);
  }
  else if(iIndex == 15)
  {
    const char *string = "none";
    const char *string2 = "even";
    const char *string3 = "odd";
    
    if(strcmp((const char *)uartPARITY, (string)) == 0)
    {
      snprintf((char*)buffer, bufferSize, "%s", string);
    }
    else if(strcmp((const char *)uartPARITY, string2) == 0)
    {
      snprintf((char*)buffer, bufferSize, "%s", string2);
    }
    else if(strcmp((const char *)uartPARITY, string3) == 0)
    {
      snprintf((char*)buffer, bufferSize, "%s", string3);
    }
  }
  else if(iIndex == 16)
  {
    snprintf((char*)buffer, bufferSize, "%d", uartStopBits[0]);
  }
  else if(iIndex == 17)
  {
    snprintf((char*)buffer, bufferSize, "%d", time.days);
  }
  
  snprintf(pcInsert, iInsertLen, "%s", buffer);
  return strlen(pcInsert);
}


void httpd_ssi_init(void) 
{
  http_set_ssi_handler(ssi_handler, ssi_tags, 18);
}


//Логика обработки /Save запросов от клиента в веб интерфейсе
const char * SAVE_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
  char ip[16] = {0};
  char mask[16] = {0};
  char gateway[16] = {0};
  char mac[18] = {0};
  char serial[18] = {0};
  char rs485speed[18] = {0};
  char rs485paritiy[18] = {0};
  char rs485stopbit[18] = {0};
  
  
  
  uint8_t ip_flag = 0;
  uint8_t mask_flag = 0;
  uint8_t gateway_flag = 0;
  uint8_t mac_flag = 0;
  uint8_t pinget = 0;
  uint8_t serial_flag = 0;
  uint8_t speed_flag = 0;
  uint8_t paritiy_flag = 0;
  uint8_t stopbit_flag = 0;
  
  for (int i = 0; i < iNumParams; i++) 
  {
    if (strcmp(pcParam[i], "ip") == 0) 
    {
      strncpy(ip, pcValue[i], sizeof(ip) - 1);
      ip_flag = 1;
    } 
    else if (strcmp(pcParam[i], "mask") == 0) 
    {
      strncpy(mask, pcValue[i], sizeof(mask) - 1);
      mask_flag = 1;
    } 
    else if (strcmp(pcParam[i], "gateway") == 0) 
    {
      strncpy(gateway, pcValue[i], sizeof(gateway) - 1);
      gateway_flag = 1;
    } 
    else if (strcmp(pcParam[i], "mac") == 0) 
    {
      strncpy(mac, pcValue[i], sizeof(mac) - 1);
      mac_flag = 1;
    }
    else if(strcmp(pcParam[i], "pin") == 0) 
    {
      strncpy(mac, pcValue[i], sizeof(mac) - 1);
      for (int i = 0; i < 6; i++) 
      {
        if(mac[i] == PIN[i])
        {
          pinget++;
        }
      }
      if (pinget == 6)
      {
        pinaccept = 1;
      }
      
      memset(mac, 0, sizeof(mac));
    }
    else if (strcmp(pcParam[i], "relay") == 0) 
    {
      if(REGISTERS[2])
      {
        REGISTERS[2] = 0;
      }
      else
      {
        REGISTERS[2] = 1;
      }
      
    }
    if (strcmp(pcParam[i], "serial") == 0) 
    {
      strncpy(serial, pcValue[i], sizeof(serial) - 1);
      serial_flag = 1;
    }
    else if (strcmp(pcParam[i], "rs485speed") == 0) 
    {
      strncpy(rs485speed, pcValue[i], sizeof(rs485speed) - 1);
      speed_flag = 1;
    } 
    else if (strcmp(pcParam[i], "rs485parity") == 0) 
    {
      strncpy(rs485paritiy, pcValue[i], sizeof(rs485paritiy) - 1);
      paritiy_flag = 1;
    } 
    else if (strcmp(pcParam[i], "rs485stopbit") == 0) 
    {
      strncpy(rs485stopbit, pcValue[i], sizeof(rs485stopbit) - 1);
      stopbit_flag = 1;
    }  
  }
  
  
  if(mac_flag != 0)
  {
    convert_str_to_uint8_array(mac, output, 1);
    WriteFlash(MAC, output);
    memset(output, 0, sizeof(output));
    mac_flag = 0;
    ReadFlash(MAC, gnetif.hwaddr);
    restart = 1;
    pinaccept = 1;
  }
  if(serial_flag != 0)
  {
    convert_str_to_uint8_array_serial(serial, output);
    WriteFlash(SERIAL, output);
    memset(output, 0, sizeof(output));
    mask_flag = 0;
    ReadFlash(SERIAL, SERIAL_ADDRESS);
    restart = 1;
    pinaccept = 1;
  }
  
  
  if(pinaccept)
  {
    if(ip_flag != 0)
    {
      convert_str_to_uint8_array(ip, output, 0);
      WriteFlash(IP, output);
      memset(output, 0, sizeof(output));
      ip_flag = 0; 
      ReadFlash(IP, IP_ADDRESS);
    }  
    if(mask_flag != 0)
    {
      convert_str_to_uint8_array(mask, output, 0);
      WriteFlash(NETMASK, output);
      memset(output, 0, sizeof(output));
      mask_flag = 0;
      ReadFlash(NETMASK, NETMASK_ADDRESS);
    } 
    if(gateway_flag != 0)
    {
      convert_str_to_uint8_array(gateway, output, 0);
      WriteFlash(GATEWAY, output);
      memset(output, 0, sizeof(output));
      gateway_flag = 0;   
      ReadFlash(GATEWAY, GATEWAY_ADDRESS);
    }      
    if(speed_flag != 0)
    {
      memcpy(output, rs485speed, sizeof(rs485speed)); 
      WriteFlash(RS485SPEED, output);
      memset(output, 0, sizeof(output));
      speed_flag = 0;   
      ReadFlash(RS485SPEED, USART_3_SPEED);
    }
    if(paritiy_flag != 0)
    {
      //convert_str_to_uint8_array(rs485paritiy, output, 2);
      WriteFlash(RS485PARITIY, (uint8_t*)rs485paritiy);
      memset(output, 0, sizeof(output));
      paritiy_flag = 0;   
      ReadFlash(RS485PARITIY, (uint8_t*)uartPARITY);
    }
    if(stopbit_flag != 0)
    {
      convert_str_to_uint8_array(rs485stopbit, output, 2);
      WriteFlash(RS485STOPBIT, output);
      memset(output, 0, sizeof(output));
      stopbit_flag = 0;   
      ReadFlash(RS485STOPBIT, uartStopBits);
    }
    
    
    restart = 1;
    return 0;
  }

  
  
  
  
  return 0;
}

//---------------------------------------------------------------------------------HTTPD-SERVER-LOGICS-END---




//---------------------------------------------------------------------------------FLASH-LOGICS-START--------

/*
void WriteFlash(FlashDataType type, uint8_t* data)
{
  uint32_t address = 0;
  uint32_t dataSize = 0;
  uint8_t flashBuffer[MAC_SIZE + IP_SIZE + NETMASK_SIZE + GATEWAY_SIZE + SERIAL_SIZE + 50] = {0xFF};
  
  
  switch (type) 
  {
  case MAC:
    address = FLASH_ADDRESS_MAC;
    dataSize = 6;
    break;
  case IP:
    address = FLASH_ADDRESS_IP;
    dataSize = 4;
    break;
  case NETMASK:
    address = FLASH_ADDRESS_NETMASK;
    dataSize = 4;
    break;
  case GATEWAY:
    address = FLASH_ADDRESS_GATEWAY;
    dataSize = 4;
    break;
  case SERIAL:
    address = FLASH_ADDRESS_SERIAL;
    dataSize = 6;
    break;
  case RS485SPEED:
    address = FLASH_ADDRESS_RS485_SPEED;
    dataSize = 3;
    break;
  case RS485PARITIY:
    address = FLASH_ADDRESS_RS485_PARITIY;
    dataSize = 10;
    break;
  case RS485STOPBIT:
    address = FLASH_ADDRESS_RS485_STOPBIT;
    dataSize = 1;
    break;
  default:
    return; 
  }
  
  osMutexWait(flashMutexHandle, osWaitForever);
  
  ReadFlash(MAC, flashBuffer);
  ReadFlash(IP, flashBuffer + MAC_SIZE);
  ReadFlash(NETMASK, flashBuffer + MAC_SIZE + IP_SIZE);
  ReadFlash(GATEWAY, flashBuffer + MAC_SIZE + IP_SIZE + NETMASK_SIZE);
  ReadFlash(SERIAL, flashBuffer + MAC_SIZE + IP_SIZE + NETMASK_SIZE + SERIAL_SIZE);
  
  ReadFlash(RS485SPEED, flashBuffer + MAC_SIZE + IP_SIZE + NETMASK_SIZE + SERIAL_SIZE + 16);
  ReadFlash(RS485PARITIY, flashBuffer + MAC_SIZE + IP_SIZE + NETMASK_SIZE + SERIAL_SIZE + 16 + 16);
  ReadFlash(SERIAL, flashBuffer + MAC_SIZE + IP_SIZE + NETMASK_SIZE + SERIAL_SIZE + 16 + 16 + 16);
  
  memcpy(flashBuffer + (address - FLASH_ADDRESS_MAC), data, dataSize);
  
  
  HAL_FLASH_Unlock();
  FLASH_Erase_Sector(FLASH_SECTOR_12, VOLTAGE_RANGE_3);
  
  for (uint16_t i = 0; i < sizeof(flashBuffer); i++) 
  {
    uint8_t word = flashBuffer[i];
    
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_MAC + i, word) != HAL_OK) 
    {
      while (1);  
    }
  }
  
  HAL_FLASH_Lock();
  
  osMutexRelease(flashMutexHandle);
  
}
*/


void ReadFlash(FlashDataType type, uint8_t* buffer) 
{
  uint32_t address = 0;
  uint32_t dataSize = 0;
  
  switch (type) 
  {
  case MAC:
    address = FLASH_ADDRESS_MAC;
    dataSize = 6;
    break;
  case IP:
    address = FLASH_ADDRESS_IP;
    dataSize = 4;
    break;
  case NETMASK:
    address = FLASH_ADDRESS_NETMASK;
    dataSize = 4;
    break;
  case GATEWAY:
    address = FLASH_ADDRESS_GATEWAY;
    dataSize = 4;
    break;
  case SERIAL:
    address = FLASH_ADDRESS_SERIAL;
    dataSize = 6;
    break;
  case RS485SPEED:
    address = FLASH_ADDRESS_RS485_SPEED;
    dataSize = 10;
    break;
  case RS485PARITIY:
    address = FLASH_ADDRESS_RS485_PARITIY;
    dataSize = 10;
    break;
  case RS485STOPBIT:
    address = FLASH_ADDRESS_RS485_STOPBIT;
    dataSize = 10;
    break;
  default:
    return;  
  }
  
  for (uint32_t i = 0; i < dataSize; i++) 
  {
    buffer[i] = *(uint8_t*)(address + i);
  }
}


//Удаляет лишние знаки из строки (АА:FF:SS:DD:HH -> AAFFSSDDHH)
void convert_str_to_uint8_array(const char* input, uint8_t* output, int is_mac) 
{
  int i = 0;
  if (is_mac == 1) 
  {
    while (*input) 
    {
      if (*input != ',' && *input != '.' && *input != ':' && *input != '-')
      {
        char hex[3] = {input[0], input[1], '\0'};
        output[i++] = (uint8_t) strtol(hex, NULL, 16);
        input += 2;
      } 
      else 
      {
        input++;
      }
    }
  }
  else if(is_mac == 2)
  {
    int i = 0;
    while(input[i]) 
    {  
      if (input[i] >= '0' && input[i] <= '9') 
      { 
        output[i] = input[i] - '0';
        i++;
      } 
      else 
      {
        output[i] = 0;   
        i++;
      }
    }
  }
  else
  {
    while (*input) 
    {
      if (*input != '.' && *input != ':' && *input != '-')
      {
        output[i++] = (uint8_t) strtol(input, (char **) &input, 10);
      } 
      else 
      {
        input++;
      }
    }
  }
}

void convert_str_to_uint8_array_serial(const char* input, uint8_t* output)
{
  for (int i = 0; i < 6; i++) 
  {
    output[i] = input[i] - '0';
  }
}

//---------------------------------------------------------------------------------FLASH-LOGICS-END--------

//---------------------------------------------------------------------------------MOODBUS-OUTPUT-START----
//функция для отправки ответа modbus TCP 
void send_ethernet(uint8_t *data, uint16_t len, struct netconn *newconn)
{
  struct netbuf *response_buf = netbuf_new();
  
  netbuf_ref(response_buf, data, len); 
  if (newconn != NULL && netconn_err(newconn) == ERR_OK) 
  {
    err_t xRes = netconn_write(newconn, data, len, NETCONN_COPY); 
    if (xRes != ERR_OK)
    {
      osDelay(10);
    }
  }
  netbuf_delete(response_buf); 
}

//функция вызывается когда передача по usart завершена
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_WritePin(UART1_RE_DE_GPIO_Port, UART1_RE_DE_Pin, GPIO_PIN_RESET);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
}

//функция для отправки ответа modbus RTU
void send_uart(const uint8_t *response, uint16_t len)
{
  HAL_GPIO_WritePin(UART1_RE_DE_GPIO_Port, UART1_RE_DE_Pin, GPIO_PIN_SET);
  HAL_StatusTypeDef ui;
  ui = HAL_UART_Transmit_DMA(&huart3, response, len); 
  if (ui != HAL_OK)
  {
    HAL_UART_ErrorCallback(&huart3);
  }
}

//функция для очистки флагов usart и его последующего перезапуска 
//(необходима для ситуаций, когда к usart подклются на неверной скорости)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_WritePin(UART1_RE_DE_GPIO_Port, UART1_RE_DE_Pin, GPIO_PIN_SET);
  HAL_UART_DMAStop(&huart3);
  osDelay(100);
  
  if (huart == &huart3)
  {
    uint32_t er = HAL_UART_GetError(&huart3);
    
    if (er & HAL_UART_ERROR_PE)
    {
      __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    if (er & HAL_UART_ERROR_NE)
    {
      __HAL_UART_CLEAR_NEFLAG(&huart3);
    }
    if (er & HAL_UART_ERROR_FE)
    {
      __HAL_UART_CLEAR_FEFLAG(&huart3);
    }
    if (er & HAL_UART_ERROR_ORE)
    {
      __HAL_UART_CLEAR_OREFLAG(&huart3);
    }
    if (er & HAL_UART_ERROR_DMA)
    {
      __HAL_UART_CLEAR_NEFLAG(&huart3); 
    }
    
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    er = HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, sizeof(rxBuffer));
    osDelay(100);
    HAL_GPIO_WritePin(UART1_RE_DE_GPIO_Port, UART1_RE_DE_Pin, GPIO_PIN_RESET);
  }
}

//---------------------------------------------------------------------------------MOODBUS-OUTPUT-END----


//---------------------------------------------------------------------------------ADC-LOGICS-START----
//Функция расчета U_rms
uint16_t adc_get_rms(uint16_t *arr, uint16_t length)
{
  uint16_t rms = 0;
  uint16_t drop1 = 0;
  uint16_t drop2 = 0;
  uint16_t drop3 = 0;
  uint16_t drop4 = 0;
  uint16_t drop5 = 0;
  
  uint16_t trigger = 50;
  uint16_t j = 0;
  uint16_t current_min = trigger;
  uint16_t min_index = 0;
  uint16_t stable_count = 0; 
  
  while (j < length) {
    
    if (arr[j] < current_min) 
    {
      current_min = arr[j];
      min_index = j;
      stable_count = 0;  
    } 
    else 
    {
      stable_count++;
    }
    
    j++;
    
    
    if (stable_count >= 70) 
    {
      //находим нижние точки синусоиды
      stable_count = 0;
      
      if(drop1 == 0)
      {
        drop1 = min_index;
        current_min = trigger;
      }
      else if(drop2 == 0)
      {
        drop2 = min_index;
        current_min = trigger;
      }
      else if(drop3 == 0)
      {
        drop3 = min_index;
        current_min = trigger;
      }
      else if(drop4 == 0)
      {
        drop4 = min_index;
        current_min = trigger;
      }
      else if(drop5 == 0)
      {
        drop5 = min_index;
        current_min = trigger;
      }
      else
      {
        break;
      }
    }
    
  }
  
  if(drop2 && drop3)
  {
    //между 2 и 4 'дропами' находится полный период синусоиды
    float sum = 0;
    for (int i = drop2; i < drop4; i++)
    {
      sum += arr[i] * arr[i];  
    }
    rms = (uint16_t)sqrt((double)sum/ (drop4 - drop2));
  }
  else 
  {
    //на случай если в сигнале нельзя выделить точки падения
    //расчитываем среднеквадратичное первых 1200 значений
    float sum = 0;
    for (int i = 0; i < 1200; i++)
    {
      sum += arr[i] * arr[i];  
    }
    rms = (uint16_t)sqrt((double)sum / 1200);
  }
  
  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);
  
  return rms;
}

//показывает что данные с ADC готовы к обработке
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) 
  {
    adc_ready = 1;
    HAL_ADC_Stop_DMA(&hadc1);
  }
}


void ADC_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
}
//---------------------------------------------------------------------------------ANOTHER-CODE-START---

//Таймер, который срабатывает если пользователь изменил настройки 
//в веб интерфейсе и необходима перезагрузка для их применения
void startMyTimer_RESET(uint32_t timeout_ms) 
{
  TimerHandle_t myTimer = xTimerCreate
    (
     "OneShotTimer",                       
     pdMS_TO_TICKS(timeout_ms),            
     pdFALSE,                              
     (void *) 0,                           
     vMyTimerCallback                      
       );
  
  if (myTimer != NULL) 
  {
    xTimerStart(myTimer, 0);  
  } 
  else 
  {
    while(1);
  }
}
//---↑
//---↑
void vMyTimerCallback(TimerHandle_t xTimer) 
{
  HAL_NVIC_SystemReset();
}


char stackOverflowTaskName[64];
//Переполнение стека
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  strncpy(stackOverflowTaskName, pcTaskName, 64 - 1);
  stackOverflowTaskName[64 - 1] = '\0';
  uint16_t TTT = 0;
  
  while(1)
  {
    TTT++;  //FREERTOS STACK ERROR
  }

}

//прерывание с кнопки PA12
void EXTI12_Init(void)   
{
  
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  
  
  SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI12);
  SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI12_PA);
  
  EXTI->IMR |= EXTI_IMR_MR12;
  
  EXTI->RTSR |= EXTI_RTSR_TR12;  
  EXTI->FTSR &= ~EXTI_FTSR_TR12; 
  
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  NVIC_SetPriority(EXTI15_10_IRQn, 1);
}
//---↑
//---↑
//---↑
void EXTI15_10_IRQHandler(void) 
{
  if (EXTI->PR & EXTI_PR_PR12) 
  { 
    EXTI->PR |= EXTI_PR_PR12; 
    //setrelay(0);
    if(start == 1)
    {
      fff = 1;
    }
    else
    {
      ch = 1;
    }
    for (int i = 0; i < 100; i++) 
    {
    }
    
  }
}

//Прерывание с пина PA6 (fixing the leak)
void EXTI6_Init(void)   
{
  
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  
  SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI6);
  SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI6_PA);
  
  EXTI->IMR |= EXTI_IMR_MR6;
  
  EXTI->RTSR |= EXTI_RTSR_TR6;
  
  EXTI->FTSR &= ~EXTI_FTSR_TR6;
  
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, 1);
}
//---↑
//---↑
//---↑
void EXTI9_5_IRQHandler(void)  
{
  if (EXTI->PR & EXTI_PR_PR6)
  {
    EXTI->PR |= EXTI_PR_PR6;
    setrelay(0);
  }
  
}

void CleanupResources(struct netconn *nc, struct netconn *newconn, struct netbuf *buf)
{
      if (newconn != NULL)
    {
        netconn_close(newconn);
        netconn_delete(newconn);
        newconn = NULL;
    }
      if (nc != NULL)
    {
        netconn_close(nc);
        netconn_delete(nc);
        nc = NULL;
    }
      if (buf != NULL)
    {
        netbuf_delete(buf);
        buf = NULL;
    } 
}


void load_flags_from_flash(void)
{
    boot_os_ok = *(volatile uint8_t *)BOOT_OS_OK_ADDRESS;
    //boot_flag_crc = *(volatile uint8_t *)BOOT_FLAG_CRC_ADDRESS;
    boot_flag_new = *(volatile uint8_t *)BOOT_FLAG_NEW_ADDRESS;
    sector_enabled = *(volatile uint8_t *)SECTOR_ENABLED_ADDRESS;
}

/*
void UpdateSector3()
{
    uint8_t sectorData[0x1000]; // Буфер для данных сектора (4 KB)

    // Чтение всего 3-го сектора во флеш
    for (uint32_t i = 0; i < 0x1000; i++) {
        sectorData[i] = *(volatile uint8_t*)(0x0800C000 + i);
    }

    // Изменение значений в буфере
    sectorData[BOOT_OS_OK_ADDRESS - 0x0800C000] = boot_os_ok;
    sectorData[BOOT_FLAG__ADDRESS - 0x0800C000] = boot_flag_;
    sectorData[BOOT_FLAG_NEW_ADDRESS - 0x0800C000] = boot_flag_new;
    sectorData[SECTOR_ENABLED_ADDRESS - 0x0800C000] = sector_enabled;

    // Запись обратно в 3-й сектор
    WriteToFlash(0x0800C000, sectorData, 0x1000);
}
*/

        
HAL_StatusTypeDef Flash_WritePacket(uint8_t *packet, uint16_t packet_size)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    if(next_free_addr == 0)
    {
    switch(sector_enabled)
        {
          case 1:
            address = SECTOR_2_ADDRESS;
            len = 393216;
            next_free_addr = SECTOR_2_ADDRESS;
            FLASH_Erase_Sector(FLASH_SECTOR_8, VOLTAGE_RANGE_3);
            FLASH_Erase_Sector(FLASH_SECTOR_9, VOLTAGE_RANGE_3);
            FLASH_Erase_Sector(FLASH_SECTOR_10, VOLTAGE_RANGE_3);
            break;
          
          case 2:
            address = SECTOR_1_ADDRESS;
            len = 393216;
            next_free_addr = SECTOR_1_ADDRESS;
            FLASH_Erase_Sector(FLASH_SECTOR_5, VOLTAGE_RANGE_3);
            FLASH_Erase_Sector(FLASH_SECTOR_6, VOLTAGE_RANGE_3);
            FLASH_Erase_Sector(FLASH_SECTOR_7, VOLTAGE_RANGE_3);
            break;
        }
    }
    
    
    
    
    // Проверка на переполнение области
    if ((next_free_addr + packet_size) == (address + len + 1))
    {
        return HAL_ERROR; // Флеш память переполнена
    }
    taskENTER_CRITICAL();
    // Разблокировка флеш памяти для записи
    HAL_FLASH_Unlock();

    // Запись пакета во флеш по байтам (или словам, в зависимости от требований)
    for (uint16_t i = 0; i < packet_size; i += 4)
    {
        uint32_t data_word = 0xFFFFFFFF;

        // Копируем данные пакета в 32-битное слово
        memcpy(&data_word, &packet[i], (packet_size - i >= 4) ? 4 : (packet_size - i));

        // Записываем слово во флеш
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, next_free_addr + i, data_word);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return status; // Ошибка при записи
        }
    }

    // Сохраняем указатель на свободную ячейку для следующего пакета ------------------------------------------установить в 0 после обработки всех пакетов
    next_free_addr += packet_size;  // ---invalid
    // Блокировка флеш памяти после записи
    iteration++;
    HAL_FLASH_Lock();
    /*
    uint32_t _stm = calculate_flash_(address, (next_free_addr));
    uint32_t _stm_reverce = ~(calculate_flash_(address, (next_free_addr)));
    uint32_t _stm_down = calculate_flash_(address, (next_free_addr-1));
    */
    taskEXIT_CRITICAL();
    //xSemaphoreGive(xPacketSaved);
    return status;
}


/*
// Функция для извлечения тела HTTP POST-запроса
char* extract_body(uint8_t *packet) 
{

    int total_read = 0;

    // Проверка на NULL и наличие \r\n\r\n
    char* header_end = strstr(packet, "\r\n\r\n");
    if (header_end == NULL) {
        return NULL; // Вернуть NULL, если конец заголовка не найден
    }

    // Указатель на тело запроса
    char* body = header_end + 4;

    return body;
}


uint16_t get_body_length(uint8_t *packet, uint16_t packet_size) 
{

    // Проверка на NULL и наличие \r\n\r\n
    char* header_end = strstr(packet, "\r\n\r\n");
    if (header_end == NULL) {
        return -1; // Вернуть -1, если конец заголовка не найден
    }

    // Указатель на тело запроса
    char* body = header_end + 4;

    // Подсчёт длины тела
    int body_length = packet_size - (body - (char*)packet);

    return (body_length - 1);
}
*/


void WriteToFlash(uint32_t startAddress, uint8_t* data, uint32_t length)
{
   
    // Разблокируем область флеш для записи
    HAL_FLASH_Unlock();
    FLASH_Erase_Sector(FLASH_SECTOR_12, VOLTAGE_RANGE_3);
    
    // Процесс записи по адресу
    for (uint32_t i = 0; i < length; i++) {
        // Записываем данные
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, startAddress + i, data[i]) != HAL_OK) {
            // Ошибка записи
            HAL_FLASH_Lock();
            return;
        }
    }

    // Блокируем флеш после записи
    HAL_FLASH_Lock();
}


// Основная функция для работы с сектором
void WriteFlash(FlashDataType type, uint8_t* data)
{
    size_t minFreeHeapSize = xPortGetMinimumEverFreeHeapSize();
    taskENTER_CRITICAL();
    uint8_t sectorData[0x1000]; // Буфер для данных сектора (4 KB)
    uint32_t *address = 0;
    uint32_t dataSize = 0;
  
    // Чтение всего 3-го сектора во флеш
    for (uint32_t i = 0; i < 0x1000; i++) {
        sectorData[i] = *(volatile uint8_t*)(0x0800C000 + i);
    }
    
    // Изменение значений в буфере
    sectorData[BOOT_OS_OK_ADDRESS - 0x0800C000] = boot_os_ok;
    //sectorData[BOOT_FLAG_CRC_ADDRESS - 0x0800C000] = boot_flag_crc;
    sectorData[BOOT_FLAG_NEW_ADDRESS - 0x0800C000] = boot_flag_new;
    sectorData[SECTOR_ENABLED_ADDRESS - 0x0800C000] = sector_enabled;
    sectorData[BOOT_CRC_ADDRESS - 0x0800C000] = crc_os;
    
    
    
    
      switch (type) 
      {
      case MAC:
        address = (uint32_t *)FLASH_ADDRESS_MAC;
        dataSize = 6;
        break;
      case IP:
        address = (uint32_t *)FLASH_ADDRESS_IP;
        dataSize = 4;
        break;
      case NETMASK:
        address = (uint32_t *)FLASH_ADDRESS_NETMASK;
        dataSize = 4;
        break;
      case GATEWAY:
        address = (uint32_t *)FLASH_ADDRESS_GATEWAY;
        dataSize = 4;
        break;
      case SERIAL:
        address = (uint32_t *)FLASH_ADDRESS_SERIAL;
        dataSize = 6;
        break;
      case RS485SPEED:
        address = (uint32_t *)FLASH_ADDRESS_RS485_SPEED;
        dataSize = 3;
        break;
      case RS485PARITIY:
        address = (uint32_t *)FLASH_ADDRESS_RS485_PARITIY;
        dataSize = 10;
        break;
      case RS485STOPBIT:
        address = (uint32_t *)FLASH_ADDRESS_RS485_STOPBIT;
        dataSize = 1;
        break;
      default:
        return; 
      }
    
    if(data)
    {
    memcpy(address, data, dataSize);
    }
    
    
    
    // Запись обратно в 3-й сектор
    WriteToFlash(0x0800C000, sectorData, 0x1000);
    taskEXIT_CRITICAL();
}


/*
const char * UPLOAD_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
  
        setrelay(0);
      
      if (iNumParams > 0 && strcmp(pcParam[0], "file") == 0) {
        uint32_t packetIndex = atoi(pcParam[1]);  // Номер текущего пакета
        uint32_t totalPackets = atoi(pcParam[2]);  // Общее количество пакетов
        char *packetData = pcValue[0];  // Данные пакета
        
        static uint32_t received_size;
        static uint32_t received_size_now;
      
        received_size += strlen(packetData);
        
        uint16_t packet_size = strlen(packetData);
        packet_size = get_body_length(packetData, packet_size);
        packetData = extract_body(packetData);
        
        
        Flash_WritePacket(packetData, packet_size);
        
        // Если мы получили все пакеты
        if (packetIndex == (totalPackets-1)) {
            return "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nFile uploaded successfully!";
            received_size = 0;          
            boot_flag_new  = 1;  
        }

        // Ответ на успешный прием пакета
        return "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nPacket received";
    
    }
}
*/



/*
err_t httpd_post_receive_data(void *connection, struct pbuf *p) 
{


    struct file_upload_context *context = (struct file_upload_context *)connection;

    // Извлечение данных из pbuf
    uint16_t packet_size = p->tot_len; // Общий размер данных пакета
    char *packetData = (char *)p->payload; // Указатель на полезные данные
    
    if (Flash_WritePacket(packetData, packet_size) != HAL_OK)
    {
      return ERR_BUF;
    }



    return ERR_OK; // Информируем об успешной обработке пакета
}

err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd)
{
    // Начало обработки POST-запроса
    return ERR_OK;
}



void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len) 
{
    // Завершение обработки POST-запроса

}




//---------------------------------------------------------------------------------ANOTHER-CODE-END---
*/
//возвращает указатель на элемент массива[указанный тег + 0x0d 0x0a 0x0d 0x0a]
char *parser(uint8_t *buf, uint16_t len, const char *str) {
    uint8_t flags[] = {0x0D, 0x0A, 0x0D, 0x0A};
    size_t str_len = strlen(str);
    size_t buf_len = len; 

    for (size_t i = 0; i < buf_len; i++) {
        if (strncmp((char *)&buf[i], str, str_len) == 0) {
            // Проверяем флаги после строки
            if (memcmp(&buf[i + str_len], flags, sizeof(flags)) == 0) {
                // Возвращаем указатель на элемент после флагов
                return &buf[i + str_len + sizeof(flags)];
            }
        }
    }
    error_flash = 1;
    return NULL;
}

//вытаскивает числовые значения, переданные в multipart/form-data пакете по тегу
uint16_t parser_num(uint8_t *buf, uint16_t len, const char *str) {
    uint8_t flags[] = {0x22, 0x0D, 0x0A, 0x0D, 0x0A};
    size_t str_len = strlen(str);
    size_t buf_len = len;

    for (size_t i = 0; i < buf_len; i++) {
        if (strncmp((char *)&buf[i], str, str_len) == 0) {
            // Проверяем флаги после строки
            if (i + str_len + sizeof(flags) >= buf_len) {
                // Данные выходят за пределы буфера
                return 0;
                error_flash = 1;
            }

            if (memcmp(&buf[i + str_len], flags, sizeof(flags)) == 0) {
                // Позиция начала числа
                size_t num_start = i + str_len + sizeof(flags);

                // Ищем конец числа (0x0D 0x0A)
                size_t num_end = num_start;
                while (num_end + 1 < buf_len && !(buf[num_end] == 0x0D && buf[num_end + 1] == 0x0A)) {
                    num_end++;
                }

                if (num_end + 1 >= buf_len) {
                    // Конец строки не найден
                    return 0;
                    error_flash = 1;
                }

                // Извлекаем строку числа
                size_t num_len = num_end - num_start;
                if (num_len == 0 || num_len > 5) { // Число не может быть больше 65535
                    return 0;
                    error_flash = 1;
                }

                char num_str[6] = {0}; // Максимум 5 символов + \0
                memcpy(num_str, &buf[num_start], num_len);
                num_str[num_len] = '\0';

                // Преобразуем строку в число
                uint16_t result = (uint16_t)atoi(num_str);
                return result;
            }
        }
    }
    error_flash = 1;
    return 0; // Если строка не найдена
}



err_t httpd_post_receive_data(void *connection, struct pbuf *p) 
{

    // Извлечение данных из pbuf
    uint16_t packet_size = p->tot_len; // Общий размер данных пакета
    char *packetData = (char *)p->payload; // Указатель на полезные данные
    
    uint16_t packetNow = parser_num(packetData, packet_size, "chunkIndex");
    uint16_t packetTotal = parser_num(packetData, packet_size, "totalChunks");
    uint16_t BinDataLength = parser_num(packetData, packet_size, "rawDataLength");
    
    
    packetData = parser(packetData, packet_size, "stream");
    
    if(packetNow == (packetTotal-1))
    {
      crc_os = 
      ((uint32_t)packetData[BinDataLength-4] << 24) |  // Старший байт
      ((uint32_t)packetData[BinDataLength-3] << 16) |  
      ((uint32_t)packetData[BinDataLength-2] << 8)  |  
      ((uint32_t)packetData[BinDataLength-1]); 
      packetData[BinDataLength-4] = 0xFF;
      packetData[BinDataLength-1] = 0xFF;
      packetData[BinDataLength-2] = 0xFF;
      packetData[BinDataLength-3] = 0xFF;
      boot_flag_new = 1;
      WriteFlash(0, 0);
    }
    
    // Обработка данных (например, запись во флеш)
    if (Flash_WritePacket(packetData, BinDataLength) != HAL_OK)
    {
        pbuf_free(p);
        return ERR_BUF;
        error_flash = 1;
        
    }


    httpd_post_data_recved(connection, packet_size);
    pbuf_free(p);
    return ERR_OK;
}

err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd)
{
    //xSemaphoreTake(xPacketSaved, portMAX_DELAY);
    *post_auto_wnd = 0;
    return ERR_OK;
}

void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len) 
{

    const char *success_message = "/updateprocess.shtml"; // Путь к странице успешного ответа
    size_t message_len = strlen(success_message);
    
    strncpy(response_uri, success_message, response_uri_len - 1);
    response_uri[response_uri_len - 1] = '\0';
    /*
      if (xSemaphoreTake(xPacketSaved, portMAX_DELAY) == pdTRUE)
  {
  }
    */
}

void CRC_Config(void) 
{
    // Указываем базовый адрес 
    hcrc.Instance = CRC;

    // Вызываем инициализацию HAL
    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        // Если что-то пошло не так, можно добавить обработку ошибки
        while (1);
    }
}

uint32_t calculate_flash_crc(uint32_t start_address, uint32_t end_address) 
{
    // Проверяем, что адреса выровнены по 4 байта
    if (start_address % 4 != 0) {
        return 0xFFFFFFFF; // Ошибка
    }

    // Вычисляем длину данных в 32-битных словах
    uint32_t length = (end_address - start_address) / 4;

    // Преобразуем адреса в указатели на uint32_t
    uint32_t* data = (uint32_t*)start_address;
    
    uint32_t answer = HAL_CRC_Calculate(&hcrc, data, length);
    // Вычисляем 
    return answer;
}

//---------------------------------------------------------------------------------
static uint8_t reverse (uint8_t val8)
{
   uint8_t result = 0;
   uint8_t maskSRC = 0x01;
   uint8_t maskDST = 0x80;
 
   for (int i=0; i < 8; i++)
   {
      if (val8 & maskSRC)
         result |= maskDST;
      maskSRC <<= 1;
      maskDST >>= 1;
   }
   
   return result;
}
 
static uint32_t reflect32 (uint32_t val32)
{
   uint32_t result = 0;
   uint32_t maskSRC = 0x00000001;
   uint32_t maskDST = 0x80000000;
 
   for (int i=0; i < 32; i++)
   {
      if (val32 & maskSRC)
         result |= maskDST;
      maskSRC <<= 1;
      maskDST >>= 1;
   }
   
   return result;
}
 
uint32_t crc32_formula_normal( size_t len,
                               const void *data )
{
#define POLY 0x04C11DB7
   const unsigned char *buffer = (const unsigned char*) data;
   uint32_t crc = -1;
 
   while( len-- )
   {
      crc = crc ^ ((uint32_t)reverse(*buffer++) << 24);
      for( int bit = 0; bit < 8; bit++ )
      {
         if( crc & (1L << 31)) crc = (crc << 1) ^ POLY;
         else                  crc = (crc << 1);
      }
   }
   return reflect32( ~crc );
}



uint32_t crc32_hw_equivalent(size_t len, const void *data) {
    #define POLY 0x04C11DB7
    const unsigned char *buffer = (const unsigned char *)data;
    uint32_t crc = 0xFFFFFFFF; // Инициализация, как в аппаратном модуле STM32
    //uint32_t crc = 0x0; 
    
    while (len--) {
        crc ^= ((uint32_t)(*buffer++) << 24); // XOR с входным байтом, смещённым в старший байт
        for (int bit = 0; bit < 8; bit++) {
            if (crc & (1U << 31)) {
                crc = (crc << 1) ^ POLY; // Применение полинома при переполнении
            } else {
                crc = (crc << 1); // Простое сдвигание влево
            }
        }
    }

    return crc; // Без финального отражения и инверсии
}

static uint8_t *reorder4 (uint8_t *src, uint32_t len)
{
   static uint8_t dst[4];
   uint8_t appendlen, idx;
 
   len = (len % 4)+4;
   appendlen = (len % 4) ? 4-(len % 4) : 0;
   idx = 0;
   while(appendlen--)
   {
      dst[idx] = 0xFF;
      idx++;
   }
   while(len--)
   {
      dst[idx] = src[3-idx];
      idx++;
   }
   return dst;
}
 
uint32_t crc32_formula_normal_STM32( size_t len,
                                     void *data )
{
#define POLY 0x04C11DB7
   uint8_t *buffer = (uint8_t*)data;
   uint32_t crc = -1;
   uint32_t portion;
   uint8_t *reordered;
 
   while( len )
   {
      portion = len < 4 ? len : 4;
      reordered = reorder4(buffer, portion);
      for (uint8_t i=0; i < 4; i++)
      {
         crc = crc ^ ((uint32_t)reordered[i] << 24);
         for( int bit = 0; bit < 8; bit++ )
         {
            if( crc & (1L << 31)) crc = (crc << 1) ^ POLY;
            else                  crc = (crc << 1);
         }
      }
      buffer += portion;
      len -= portion;
   }
   return crc;
} 



uint32_t test()
{
  uint32_t val_1 = 0x010A03AA;
  uint32_t val_2 = 0x13B000E1;
  uint32_t val_3 = 0x001BD050;
  uint32_t val_FF = 0xFFFFFFFF;
  
  //->DR = val_FF;
  
  CRC->DR = val_1;
  CRC->DR = val_2;
  CRC->DR = val_3;
    
  return CRC->DR;
}
/* USER CODE END Application */

