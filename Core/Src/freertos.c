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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rtc.h"
#include "stm32f4xx.h"
#include <stdbool.h>


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
SemaphoreHandle_t xHighPrioritySemaphore;




extern struct netif gnetif;
extern UART_HandleTypeDef huart3;
extern  uint8_t rxBuffer[50];
extern uint8_t IP_ADDRESS[4];
extern uint8_t NETMASK_ADDRESS[4];
extern uint8_t GATEWAY_ADDRESS[4];
extern volatile uint16_t REGISTERS[9];
extern uint32_t usart_speed;
uint8_t SERIAL_ADDRESS[6] = {0};


extern void MX_USART3_UART_Init(void);
//----------------------------ADC---LOGIC---------------------------------------

volatile uint8_t last_position = 5;
volatile uint8_t temp_count = 0;
volatile uint8_t mode = 0;
volatile uint8_t log_ready = 0;
uint32_t g_tick;   //-------//------//-----
uint32_t diff;     //-------//------//-----


//volatile uint8_t manual_mode = 0;

uint8_t reley_auto_protection = 1;

float C_phase_A = 0;
float R_leak_A = 0;
float C_phase_B = 0;
float R_leak_B = 0;
float C_phase_C = 0;
float R_leak_C = 0;
uint8_t TARGET_VALUE = 0;
uint8_t TARGET_VALUE_DEF = 25; //25
uint8_t WARNING_VALUE = 0;
uint8_t WARNING_VALUE_DEF = 20; //20

//uint8_t hw_protection = 0;
#define FLASH_ADDRESS_C_PHASE_A 0x0800C0A0
#define FLASH_ADDRESS_R_LEAK_A  0x0800C0B0
#define FLASH_ADDRESS_C_PHASE_B 0x0800C0C0
#define FLASH_ADDRESS_R_LEAK_B  0x0800C0D0
#define FLASH_ADDRESS_C_PHASE_C 0x0800C0E0
#define FLASH_ADDRESS_R_LEAK_C  0x0800C0F0
#define FLASH_ADDRESS_TARGET_VALUE 0x0800C100
#define FLASH_ADDRESS_HW_PROTECTION 0x0800C110
#define FLASH_ADDRESS_WARNING_VALUE 0x0800C120



#define SQ3         1.732050807f    // sqrt(3)
#define COS30       0.866025403f    // cos(30°)
#define OMEGA       314.1592653f    // 2 * M_PI * 50 ≈ 6.283185307 * 50
#define MULT_UP     43824.0f        // 14608 * 3
#define MULT_DOWN   2.057065f       // 11.365 * 0.181



volatile uint8_t button_ivent = 0;

//------------------------------------------------------------------------------

uint8_t USART_3_SPEED[10];
uint8_t uartStopBits[10];
char uartPARITY[10];  
uint8_t output[200] = {0};


uint16_t ADC_BUFFER_SIZE = 457;  //900
extern uint16_t adcBuffer[457];  //900
uint8_t adc_ready = 0;



//-------------------------------------------------------------------
uint8_t SOFTWARE_VERSION[3] = {0x01, 0x01, 0x02};
uint16_t soft_ver_modbus = 112;

extern struct httpd_state *hs;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct netconn * nc;
struct netconn * in_nc;
struct netbuf * nb;
struct netconn *newconn = NULL; 

const char *ssi_tags[] = {"MAC", "IP", "MASK", "GETAWEY", "AMP", "SEC", "MIN",
"HOUR", "DAY", "PIN", "RELAY", "SERIAL", "SOFT", "RS485", "SPEED", "PARITY",
"STOPB", "CPHASEA", "RLEAKA", "CPHASEB", "RLEAKB", "CPHASEC", "RLEAKC", "TVALUE", "MODE" , "CRCACC", "JSON", "WVALUE", "ALERT", "LOG"};


typedef enum 
{
  MAC,
  IP,
  NETMASK,
  GATEWAY,
  SERIAL,
  RS485SPEED,
  RS485PARITIY,
  RS485STOPBIT,
  C_PHASE_A,
  R_LEAK_A,
  C_PHASE_B, 
  R_LEAK_B, 
  C_PHASE_C,
  R_LEAK_C,
  TaRGET_VALUE,
  HW_PROTECTION,
  Warning_VALUE
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
extern volatile uint8_t theme;





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
#define M_PI 3.14159265


#define BOOT_OS_OK_ADDRESS 0x0800C080
#define BOOT_CRC_ADDRESS 0x0800C200
#define BOOT_FLAG_NEW_ADDRESS 0x0800C084 
#define SECTOR_1_ADDRESS 0x08020000 //end in 0x08080000 (sector 5/6/7) 384kb in total
#define SECTOR_2_ADDRESS 0x08080000 // end in 0x080E0000 (sector 8/9/10) 384kb in total
#define BOOTLOADER_ADDRESS 0x08000000 //end in 0x0800C080  48kb in total
#define SECTOR_ENABLED_ADDRESS 0x0800C088 //1 - нет новой ОС, 2 - есть новоя ОС
#define LOG_SECTOR_ACTIVE 0x0800C204 //в каком секторе мы пишем лог сейчас



#define MINUTE_10_SIZE 10
#define HOUR_1_SIZE 6
#define DAY_2_SIZE 48

volatile uint16_t avg1h = 0;
volatile uint16_t avg2d = 0;

//-------------------------TIME-&-LOG--SECTION---------------------------------------
UBaseType_t uxHighWaterMark1;
UBaseType_t uxHighWaterMark2;
UBaseType_t uxHighWaterMark3;
UBaseType_t uxHighWaterMark4;
UBaseType_t uxHighWaterMark5;
UBaseType_t uxHighWaterMark6;
UBaseType_t uxHighWaterMark;


typedef struct {
    uint8_t seconds;   // 0-59
    uint8_t minutes;   // 0-59
    uint8_t hours;     // 0-23 (24-часовой формат)
    uint8_t day;       // 1-31 (день месяца)
    uint8_t month;     // 1-12
    uint16_t year;     // Пример: 2023 (полный год)   //not used
} DateTime;


 typedef struct {
    uint8_t timestamp[6];  // ss:mm:hh:dd:mm:yy (временная метка)
    uint8_t event_code;    // код события
    uint8_t data[5];       //поле данных
  } LogEntry; 
                           //12 байт


volatile uint8_t time_acepted = 0;  //not used

volatile uint32_t log_ptr = 0;
RTC_HandleTypeDef hrtc;

volatile uint32_t LOG_START_ADDR = 0;
volatile uint32_t LOG_END_ADDR = 0;

volatile uint8_t log_sector_active = 0;


#define LOG_START_ADDR_BLOCK 0x08120000  //17-23 sectors (128х7)
#define LOG_END_ADDR_BLOCK 0x081FFFF8 
#define LOG_ENTRY_SIZE  (12U)
#define LOG_BUFF_SIZE 1026

volatile uint8_t log_for_wed[LOG_BUFF_SIZE] = {0}; //1kb


static int first_call = 1;

//--------------------------(critical flags)------------------------------------
//после изменения этих флагов нужно вызвать WriteFlash(0, 0); 
//для их автоматической запиши во флеш
volatile uint8_t boot_os_ok = 0;
volatile uint8_t boot_flag_new = 0;
volatile uint8_t sector_enabled = 0;
volatile uint32_t crc_os = 0;
volatile uint8_t crc_accepted = 5;
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
volatile uint8_t RS485 = 0;
uint8_t RX_Flag = 0;
uint8_t i9 = 0;
uint8_t fff = 1;
uint8_t ch = 0;
volatile uint32_t er = 0;
uint8_t response_data[50] = {0};

//-------------------------------------------------------------------------------------------------------------------
typedef struct {
    uint16_t arr[MINUTE_10_SIZE];
    uint8_t index;
    uint32_t sum; 
} CircularBuffer10Min;

typedef struct {
    uint16_t arr[HOUR_1_SIZE];
    uint8_t index;
    uint32_t sum;
} CircularBuffer1Hour;

typedef struct {
    uint16_t arr[DAY_2_SIZE];
    uint8_t index;
    uint32_t sum;
} CircularBuffer2Day;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

osThreadId_t HighPriorityTaskHandle;
const osThreadAttr_t HighPriorityTask_attributes = {
   .name = "HighPriorityTask",
   .stack_size = 1024 * 10, // Размер стека (измените при необходимости)
   .priority = (osPriority_t) osPriorityHigh, // Приоритет выше остальных      osPriorityHigh
};

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
  .stack_size = 1128 * 6,
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
  .stack_size = 528 * 4,
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
void startMyTimer_UPDATE(uint32_t timeout_ms);
void vMyTimer2Callback(TimerHandle_t xTimer);
void startMyTimer_RESET(uint32_t timeout_ms);
void vMyTimerCallback(TimerHandle_t xTimer);
void convert_str_to_uint8_array_serial(const char* input, uint8_t* output);
void EXTI12_Init(void);
void EXTI15_10_IRQHandler(void);


void log_for_web_init();


void Swipe_Log_Sector();
void parse_http_time_request(uint8_t *http_request, uint8_t temp_arr[6]);

void WriteToFlash(uint32_t startAddress, uint8_t* data, uint32_t length);
void UpdateSector3();
void load_flags_from_flash(void);
HAL_StatusTypeDef Flash_WritePacket(uint8_t *packet, uint16_t packet_size);
uint16_t get_body_length(uint8_t *packet, uint16_t packet_size);
char* extract_body(uint8_t *packet);
const char *handle_file_upload(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
void WriteToFlash(uint32_t startAddress, uint8_t* data, uint32_t length);
//=======
//void EXTI6_Init(void);
void EXTI9_5_IRQHandler(void);
void RTC_Init(void);

uint32_t find_next_free_log_address(void);

void convert_str_to_float_bytes(const char* input, uint8_t* output);
void load_values_from_flash(void);

void erise_update_sector(void);


void send_ethernet(uint8_t *data, uint16_t len, struct netconn *newconn);
uint16_t adc_get_rms(uint16_t *arr, uint16_t length);
void CleanupResources(struct netconn *nc, struct netconn *newconn, struct netbuf *buf);
void write_to_log(uint8_t code, uint8_t log_data[], uint16_t copy_len);
void save_time_to_rtc(uint8_t* arr);
void swichSector();
uint32_t calculate_flash_crc(uint32_t start_address, uint32_t end_address);
void CRC_Config(void);
uint16_t parser_num(uint8_t *buf, uint16_t len, const char *str);
char *parser(uint8_t *buf, uint16_t len, const char *str);


void get_current_timestamp(uint8_t *timestamp);
void initBuffer10Min(CircularBuffer10Min *buffer);
void initBuffer1Hour(CircularBuffer1Hour *buffer);
void initBuffer2Day(CircularBuffer2Day *buffer);
void init_circular_buffers(CircularBuffer10Min *buffer, CircularBuffer1Hour *buffer2, CircularBuffer2Day *buffer3);
void addValue10Min(CircularBuffer10Min *buffer, uint16_t value);
void addValue1Hour(CircularBuffer1Hour *buffer, uint16_t value);
void addValue2Day(CircularBuffer2Day *buffer, uint16_t value);
float getAverage10Min(CircularBuffer10Min *buffer);
float getAverage1Hour(CircularBuffer1Hour *buffer);
float getAverage2Day(CircularBuffer2Day *buffer);


err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd);
err_t httpd_post_receive_data(void *connection, struct pbuf *p);
void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len);

static void vRelayReleaseCallback(TimerHandle_t xTimer);

float calculate_rms_B_macros(uint16_t rms);
float calculate_rms_C_macros(uint16_t rms);
float calculate_rms_A_macros(uint16_t rms);
extern void OLED_1in5_rgb_run();
void EXTI6_DeInit(void);

const char * SAVE_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char * JSON_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char * LOG_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);

const tCGI SAVE_CGI = {"/save", SAVE_CGI_Handler};
const tCGI JSON_CGI = {"/json", JSON_CGI_Handler};
const tCGI LOG_CGI = {"/logdata", LOG_CGI_Handler};

tCGI CGI_TAB[3];
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void HighPriorityTask(void *argument);

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

  xHighPrioritySemaphore = xSemaphoreCreateBinary();
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
  Relay_taskHandle = osThreadNew(StartTask02, NULL, &Relay_task_attributes); //не помогает
  
  /* creation of mobdus */
  mobdusHandle = osThreadNew(StartTask03, NULL, &mobdus_attributes);  //помогает
  
  /* creation of Relay_control */
  Relay_controlHandle = osThreadNew(StartTask04, NULL, &Relay_control_attributes); //не помогает
  
  /* creation of LWGL_control */
  LWGL_controlHandle = osThreadNew(StartTask05, NULL, &LWGL_control_attributes); //не помогает
  
  /* creation of WDI */
  WDIHandle = osThreadNew(StartTask06, NULL, &WDI_attributes); //помогает
  
  /* USER CODE BEGIN RTOS_THREADS */
  HighPriorityTaskHandle = osThreadNew(HighPriorityTask, NULL, &HighPriorityTask_attributes); //не помогает
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
  
  CGI_TAB[0] = SAVE_CGI;
  CGI_TAB[1] = JSON_CGI;
  CGI_TAB[2] = LOG_CGI;
  http_set_cgi_handlers(CGI_TAB, 3);
  xPacketSemaphore = xSemaphoreCreateBinary();
  xPacketSaved = xSemaphoreCreateBinary();
  
  load_values_from_flash();
  
  /*
  if(hw_protection)
  {
    EXTI6_Init();
  }
  */
  
  REGISTERS[4] = (REGISTERS[4] |= 0x04);
  REGISTERS[0] = soft_ver_modbus;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
  load_flags_from_flash();
  osDelay(100);
  RTC_Init();
  xSemaphoreGive(xPacketSaved);

  /* Infinite loop */

  
  
  for(;;)     //------------------------------------------------------modbus_RTU------------------------------
  {
    uxHighWaterMark1 = uxTaskGetStackHighWaterMark(NULL);
    
    
    static uint8_t first = 1;
    if(first)
    {
      
      uint8_t *flash_ptr = (uint8_t *)0x08080000;
      uint8_t all_ff = 1;
      for (int i = 0; i < 20; i++) {
          if (flash_ptr[i] != 0xFF) {
              all_ff = 0;
              break;
          }
      }

      if(!all_ff)
      {
          erise_update_sector();
          first = 0;
      }
    }
    
    if (REGISTERS[4] & (1 << 4)) 
    {
    // 4-й бит установлен (1)
    mode = 1;
    } 
    else 
    {
    // 4-й бит сброшен (0)
    mode = 0;
    }
    
    
     if (xSemaphoreTake(xPacketSemaphore, pdMS_TO_TICKS(1500)) == pdTRUE)
    {
      
      packetReceived = 0;
      uint8_t data[20] = {0};
      uint16_t len_ext = 0;
      
      uint16_t len = (sizeof(rxBuffer) - __HAL_DMA_GET_COUNTER(huart3.hdmarx));
      
      memcpy(data, rxBuffer, len);
      
      modbus(data, len, response_data, &len_ext, RTU); 
      
      send_uart(response_data, len_ext);   
      
    }

    
    if (
        __HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE)  ||  // Frame Error
        __HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE)  ||  // Noise Error
        __HAL_UART_GET_FLAG(&huart3, UART_FLAG_PE)  ||  // Parity Error
        __HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE)     // Overrun Error
        ) 
    {
      

    volatile uint32_t dummy = huart3.Instance->DR;
    (void)dummy;  

    memset(rxBuffer, 0, sizeof(rxBuffer));
    
    HAL_UART_DMAStop(&huart3);
    
    
    
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        
    HAL_UART_DeInit(&huart3);
        
    
    MX_USART3_UART_Init();
    
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
    
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
  
  
  /*
  uint16_t mask = (1 << 2);  
  REGISTERS[4] = (REGISTERS[4] | mask); 
  */
  
  HAL_GPIO_WritePin(UART1_RE_DE_GPIO_Port, UART1_RE_DE_Pin, GPIO_PIN_RESET);
  ReadFlash(SERIAL, SERIAL_ADDRESS);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);

  
  CircularBuffer10Min buffer10;
  CircularBuffer1Hour buffer1Hour;
  CircularBuffer2Day buffer2Day;
  
  
  initBuffer10Min(&buffer10);
  initBuffer1Hour(&buffer1Hour);
  initBuffer2Day(&buffer2Day);
  
  for(;;)
  {
    
    //---------------------------------ADC-and-Realay-logic-------------------------
    uxHighWaterMark2 = uxTaskGetStackHighWaterMark(NULL);

    if(adc_ready == 1)
    {
      

        /*
        uint16_t rms = adc_get_rms(adcBuffer, ADC_BUFFER_SIZE);
        REGISTERS[1] = (uint16_t)(rms * 0.019922);  // REGISTERS[1] = ((((rms / 4096) * 3.3) * 3) / (121.1775) * 1000);
        */
        /*
        uint16_t rms = adc_get_rms(adcBuffer, ADC_BUFFER_SIZE);
        //rms = 300; //for test
        //osDelay(50);
        float leak_phase_A_macros = calculate_rms_A_macros(rms);
        float leak_phase_B_macros = calculate_rms_B_macros(rms);
        float leak_phase_C_macros = calculate_rms_C_macros(rms);
        
        uint16_t AA = (uint16_t)leak_phase_A_macros;
        uint16_t BB = (uint16_t)leak_phase_B_macros;
        uint16_t CC = (uint16_t)leak_phase_C_macros;
        
        uint16_t max_val = (uint16_t) fmax(fmax(leak_phase_A_macros, leak_phase_B_macros), leak_phase_C_macros);
        REGISTERS[1] = max_val;
        */
      
        
//----------------------------------------------------------------------------------WARNING-LOGIC-------------
        static uint8_t lasttime = 0;
        static uint8_t lasttime_hour = 0;
       
        static uint8_t count = 0;
        
        if(lasttime != time.minutes)
        {
          static uint8_t count = 0;
          lasttime = time.minutes;
          count++;
          addValue10Min(&buffer10, REGISTERS[1]);
          
          if((time.minutes % 10) == 0)
          {
            uint16_t Average10min = (uint16_t)getAverage10Min(&buffer10);
            count = 0;
            addValue1Hour(&buffer1Hour, Average10min);
            if(lasttime_hour != time.hours)
            {
              uint16_t Average1h = (uint16_t) getAverage1Hour(&buffer1Hour);
              avg1h = Average1h;
              lasttime_hour = time.hours;
              addValue2Day(&buffer2Day, Average1h);
              avg2d = getAverage2Day(&buffer2Day);
            }
          }
        }
//----------------------------------------------------------------------------------WARNING-LOGIC-END----------
    }
    
    
    /*
    if(REGISTERS[1] >= TARGET_VALUE)
    {
      setrelay(0);
      theme = 2;
      taskENTER_CRITICAL();
      uint8_t temp_value = (uint8_t)REGISTERS[1];
      write_to_log(0x33, &temp_value, 1);
      taskEXIT_CRITICAL();
    }
    else if(REGISTERS[1] < TARGET_VALUE)
    {
      theme = 1;
    }
    */
    
    
if(!start)
{
    static uint8_t value_was_changed = 1;
    //warning 2
    if(REGISTERS[1] >= WARNING_VALUE) 
    {
      if(value_was_changed == 1)
      {
      REGISTERS[4] = (REGISTERS[4] |= 0x02);
      uint8_t data = REGISTERS[1];
      taskENTER_CRITICAL();
      write_to_log(0x32, &data, 1);
      taskEXIT_CRITICAL();
      value_was_changed = 0;
      }
    }
    else if(REGISTERS[1] < WARNING_VALUE)
    {
      value_was_changed = 1;
    }
    
    
    static uint8_t flag_1 = 0; // или bool flag_1 = false;

    if(avg1h)
    {
      // Если мгновенный ток на 10% выше часового среднего и флаг не выставлен – записываем лог
      if((REGISTERS[1] > (avg1h * 1.1)) && (flag_1 == 0))
      {
        uint8_t data = REGISTERS[1];
        taskENTER_CRITICAL();
        write_to_log(0x30, &data, 1);
        flag_1 = 1;
        taskEXIT_CRITICAL();
      }
      // Сброс флага, когда ток снижается ниже среднего
      else if ((REGISTERS[1] <= (avg1h)) && (flag_1 == 1))
      {
        flag_1 = 0;
      }
    }
 }
    
     
      
    /*  
    if(hw_protection)
    {
      if (HAL_GPIO_ReadPin(Fixing_the_leak_GPIO_Port, Fixing_the_leak_Pin) == GPIO_PIN_SET)
      {
        setrelay(0);
      } 
    }
    */
    
    
    
    if(restart == 1)
    {
      startMyTimer_RESET(25000);
      restart = 0;
    } 

//--------------------------------------------------------------OS-UPDATE--------
    
    if((boot_flag_new == 1) && (next_free_addr != 0))
    { 
          taskENTER_CRITICAL();
          address = SECTOR_2_ADDRESS;
          uint32_t crc_stm = calculate_flash_crc(address, (next_free_addr-4));  
          if(crc_os == crc_stm)
          {
            uint8_t data = 0x00;
            write_to_log(0x07, &data, 1);
            crc_accepted = 1;
            sector_enabled = 2;
            WriteFlash(0, 0);
            startMyTimer_RESET(7000);
            next_free_addr = 0;
          }
          else if(crc_os != crc_stm)
          {
            uint8_t data = 0xFF;
            write_to_log(0x07, &data, 1);
            crc_accepted = 0;
            sector_enabled = 2;
            next_free_addr = 0;
            WriteFlash(0, 0);
            startMyTimer_RESET(7000);
          }

          taskEXIT_CRITICAL();
        
    }
    
//----------------------------------------------------------------OS-UPDATE-END--
     osDelay(25);
     //taskYIELD();
    
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
      uxHighWaterMark3 = uxTaskGetStackHighWaterMark(NULL);

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
    
    uxHighWaterMark4 = uxTaskGetStackHighWaterMark(NULL);
    
    
    if(mode)
    {
      
      
      switch(REGISTERS[2])
      {
      case 0:
        if(last_position != 0)
        {
           HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_RESET);
           uint8_t temp[1] = {0x00};
           write_to_log(0x05, &temp[0], 1);
           last_position = 0;
        }
        break;
        
      case 1:
        if(last_position != 1)
        {
           HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_SET);
           last_position = 1;
           uint8_t temp[1] = {0x01};
           write_to_log(0x05, &temp[0], 1);
        }
        break;  
      }
    }
    
    
    if(fff)
    {
      reley_auto_protection = 0;
      setrelay(0);
      osDelay(5000);
      reley_auto_protection = 1;
      
      start = 0;
      fff = 0;
    }

    if(button_ivent)
    {
      HAL_GPIO_WritePin(Checking_for_leaks_GPIO_Port, Checking_for_leaks_Pin, GPIO_PIN_SET);
      //HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_RESET);
      osDelay(3000);
      //HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(Checking_for_leaks_GPIO_Port, Checking_for_leaks_Pin, GPIO_PIN_RESET);
      osDelay(500);  
      button_ivent = 0;
      taskENTER_CRITICAL();
      write_to_log(0x31, 0x00, 1);
      taskEXIT_CRITICAL();
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
     uxHighWaterMark5 = uxTaskGetStackHighWaterMark(NULL);

    if(start == 0)
    {
      uxHighWaterMark5 = uxTaskGetStackHighWaterMark(NULL);

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
    
    

    uxHighWaterMark6 = uxTaskGetStackHighWaterMark(NULL);


    /*
    size_t minFreeHeapSize = xPortGetMinimumEverFreeHeapSize();
    if(HAL_GPIO_ReadPin(RS485_1_ON_GPIO_Port, RS485_1_ON_Pin) == GPIO_PIN_SET)
    {
      RS485 = 0;
    }
    else
    {
      RS485 = 1;
    }
    */
    
    
    //warning 1
    if ((!(REGISTERS[4] & 0x01)) && (time.days >= 2))
    {     
      static uint8_t period = 0;
      if(avg1h > avg2d)
      {   
        if(period != time.hours)
        {
        period = time.hours;
        uint16_t delta = (avg2d - avg1h);
        if(delta >= 5)
        {
          REGISTERS[4] = (REGISTERS[4] |= 0x01);
          uint8_t data = 0x02;
          taskENTER_CRITICAL();
          write_to_log(0x32, &data, 1);
          taskEXIT_CRITICAL();
        }
        }
      }
    }
   
    
          if(ch == 1)
          {
            HAL_GPIO_WritePin(Checking_for_leaks_GPIO_Port, Checking_for_leaks_Pin, GPIO_PIN_SET);
            reley_auto_protection = 0;
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
            reley_auto_protection = 1;
            ch = 0;
          }
    
  }
  /* USER CODE END StartTask06 */
  
}



void HighPriorityTask(void *argument) 
{
  

   uint32_t timetag = HAL_GetTick(); //-------//------//-----
   //static uint8_t last_state = 5;
    for(;;) 
    {   
      
        if (xSemaphoreTake(xHighPrioritySemaphore, portMAX_DELAY) == pdTRUE) 
        {
          
          uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
          
          uint16_t rms = adc_get_rms(adcBuffer, ADC_BUFFER_SIZE);
          //rms = 300; //for test
          //osDelay(50);
          float leak_phase_A_macros = calculate_rms_A_macros(rms);
          float leak_phase_B_macros = calculate_rms_B_macros(rms);
          float leak_phase_C_macros = calculate_rms_C_macros(rms);
          
          uint16_t AA = (uint16_t)leak_phase_A_macros;
          uint16_t BB = (uint16_t)leak_phase_B_macros;
          uint16_t CC = (uint16_t)leak_phase_C_macros;
          
          uint16_t max_val = (uint16_t) fmax(fmax(leak_phase_A_macros, leak_phase_B_macros), leak_phase_C_macros);
          REGISTERS[1] = max_val;
          //REGISTERS[1] = 19;
          
          
          
          
          if(!mode)
          {
              if((REGISTERS[1] >= TARGET_VALUE) && reley_auto_protection)
              {
                osMutexWait(RelayMutexHandle, osWaitForever);
                REGISTERS[2] = 0;
                osMutexRelease(RelayMutexHandle);
              }
              else if ((REGISTERS[1] <= TARGET_VALUE) && reley_auto_protection)
              {
                osMutexWait(RelayMutexHandle, osWaitForever);
                REGISTERS[2] = 1;
                osMutexRelease(RelayMutexHandle);
              }
              

                  

             
              if((REGISTERS[2] == 0) && (last_position != REGISTERS[2]))
              {
                HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_RESET);
                theme = 2;
                if(!start)
                {
                   taskENTER_CRITICAL();
                   uint8_t temp_value = (uint8_t)REGISTERS[1];
                   write_to_log(0x33, &temp_value, 1);
                   uint8_t temp[1] = {0x00};
                   write_to_log(0x05, &temp[0], 1);
                   taskEXIT_CRITICAL();
                }
                last_position = REGISTERS[2];
              }
              else if((REGISTERS[2] == 1) && (last_position != REGISTERS[2]))
              {
                 HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_SET);
                 last_position = REGISTERS[2];
                 uint8_t temp[1] = {0x01};
                 write_to_log(0x05, &temp[0], 1);
                 theme = 1;
              }
              
       
              
          }
          
          

          HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);


          
          
          osDelay(1); 
          g_tick = HAL_GetTick();   //-------//------//-----
          diff = g_tick - timetag;  //-------//------//-----
          
          timetag = g_tick;         //-------//------//-----
          //osDelay(100);
          
        }
    
}
}








/* Private application code */
/* USER CODE BEGIN Application */

void setrelay(uint16_t i)
{
  static uint8_t last_i = 0;
  
  if(last_i != i)
  {
     last_i = i;
     uint8_t data = (uint8_t) i;
     taskENTER_CRITICAL();
     write_to_log(0x05, &data, 1);
     osMutexWait(RelayMutexHandle, osWaitForever);
     REGISTERS[2] = i;
     osMutexRelease(RelayMutexHandle);
     taskEXIT_CRITICAL();
  }
}



//---------------------------------------------------------------------------------HTTPD-SERVER-LOGICS-START---

uint16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen) 
{
  uint8_t buffer[50] = {0};
  uint8_t bufferSize = 50;
  
  
  
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
    if(!first_call)
    {
    first_call = 1;
    }
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
    snprintf((char*)buffer, bufferSize, "%.9f", C_phase_A);
  }
  else if(iIndex == 18)
  {
    snprintf((char*)buffer, bufferSize, "%.9f", R_leak_A);
  }
  else if(iIndex == 19)
  {
    snprintf((char*)buffer, bufferSize, "%.9f", C_phase_B);
  }
    else if(iIndex == 20)
  {
    snprintf((char*)buffer, bufferSize, "%.9f", R_leak_B);
  }
    else if(iIndex == 21)
  {
    snprintf((char*)buffer, bufferSize, "%.9f", C_phase_C);
  }
    else if(iIndex == 22)
  {
    snprintf((char*)buffer, bufferSize, "%.9f", R_leak_C);
  }
    else if(iIndex == 23)
  {
    snprintf((char*)buffer, bufferSize, "%d", TARGET_VALUE);
  }
    else if(iIndex == 24)
  {
    /*
    if(hw_protection)
    {
      snprintf((char*)buffer, bufferSize, "checked");
    }
    */
    snprintf((char*)buffer, bufferSize, "%d", mode);
  }
  else if(iIndex == 25)
  {
    snprintf((char*)buffer, bufferSize, "%d", crc_accepted);
  }
  else if(iIndex == 26)
  {

    snprintf((char*)buffer, bufferSize, "{\"current\":%u}", REGISTERS[1]);   //like a JSON
  }
  else if(iIndex == 27)
  {
    snprintf((char*)buffer, bufferSize, "%d", WARNING_VALUE);
  }
  else if(iIndex == 28)
  {
    snprintf((char*)buffer, bufferSize, "%d", REGISTERS[4]);
  }
  else if(iIndex == 29)
  {
     log_for_web_init();
     memcpy(pcInsert, (void const *)log_for_wed, 1025);
     return 1025;
  }

  
  snprintf(pcInsert, iInsertLen, "%s", buffer);
  return strlen(pcInsert);
}


void httpd_ssi_init(void) 
{
  http_set_ssi_handler(ssi_handler, ssi_tags, 30);
}

void log_for_web_init()
{
      // Порог пустых записей (0xFF)
    uint8_t FF_THRESHOLD = 5;
    
    // Статические переменные для отслеживания состояния между вызовами
    // current_addr – текущая позиция чтения (идём в обратном направлении)
    // initial_addr – адрес, с которого начался текущий цикл передачи (для определения полного круга)
    // first_call – флаг первого вызова в рамках одной последовательной передачи логов
    static uint32_t current_addr = 0;
    static uint32_t initial_addr = 0;

    memset((void *)log_for_wed, 0, LOG_BUFF_SIZE);
    
    int buf_index = 0;
    int max_buf_size = sizeof(log_for_wed);  // Размер выходного буфера (например, 1 КБ)
    int max_entries = max_buf_size / LOG_ENTRY_SIZE; // Количество записей, умещающихся в буфере
    int consecutive_ff = 0; // Счётчик подряд идущих пустых (0xFF) записей

    // При первом вызове начинаем с самой новой записи
    if (first_call)
    {
        if (log_ptr == LOG_START_ADDR_BLOCK)
            // Если указатель равен началу, начинаем с последней валидной записи
            current_addr = LOG_END_ADDR_BLOCK - LOG_ENTRY_SIZE;
        else
            current_addr = log_ptr - LOG_ENTRY_SIZE;
        // Запоминаем стартовую позицию для определения полного круга
        initial_addr = current_addr;
        first_call = 0;
    }

    // Чтение логов в обратном порядке (от самой новой записи)
    for (int i = 0; i < max_entries; i++)
    {
        // Если уже прошли полный круг, прекращаем чтение
        if (i > 0 && current_addr == initial_addr)
            break;

        // Проверка корректности указателя; если он вне диапазона,
        // устанавливаем его на последнюю валидную запись
        if (current_addr < LOG_START_ADDR_BLOCK || current_addr > (LOG_END_ADDR_BLOCK - LOG_ENTRY_SIZE)) {
            current_addr = LOG_END_ADDR_BLOCK - LOG_ENTRY_SIZE;
        }
        
        uint8_t entry[LOG_ENTRY_SIZE];
        // Считываем одну запись по текущему адресу
        
        
        
        //Логика для пропуска лишних 8-ми байт в конце каждого сектора (иначе сбивается кратность 12-ти)
        if ((current_addr & 0xDFFFF) == 0xDFFF4) 
        {
          current_addr -= 8;
        }
        else if ((current_addr & 0xBFFFF) == 0xBFFF4)
        {
          current_addr -= 8;
        }
        else if ((current_addr & 0x9FFFF) == 0x9FFF4)
        {
          current_addr -= 8;
        }
        else if ((current_addr & 0x7FFFF) == 0x7FFF4)
        {
          current_addr -= 8;
        }
        else if ((current_addr & 0x5FFFF) == 0x5FFF4)
        {
          current_addr -= 8;
        }
        else if ((current_addr & 0x3FFFF) == 0x3FFF4)
        {
          current_addr -= 8;
        }
          
          
          
          
        memcpy(entry, (const void *)current_addr, LOG_ENTRY_SIZE);

        // Проверяем, является ли запись "пустой" (все байты равны 0xFF)
        int is_ff = 1;
        for (int j = 0; j < LOG_ENTRY_SIZE; j++)
        {
            if (entry[j] != 0xFF)
            {
                is_ff = 0;
                break;
            }
        }

        // Подсчёт подряд идущих пустых записей
        if (is_ff)
            consecutive_ff++;
        else
            consecutive_ff = 0;

        // Если встречено достаточное число пустых записей, считаем, что валидных логов больше нет
        if (consecutive_ff >= FF_THRESHOLD)
            break;

        // Копируем запись в выходной буфер
        memcpy((void *)&log_for_wed[buf_index], entry, LOG_ENTRY_SIZE);
        buf_index += LOG_ENTRY_SIZE;


        // Обновляем указатель. Если вычитание размера записи привело бы к значению ниже начала сектора,
        // переносим указатель на последнюю валидную запись
        if (current_addr - LOG_ENTRY_SIZE < LOG_START_ADDR_BLOCK)
            current_addr = LOG_END_ADDR_BLOCK - LOG_ENTRY_SIZE;
        else
            current_addr -= LOG_ENTRY_SIZE;
    }

    // Если достигнут конец логов (либо по порогу пустых записей, либо полный круг пройден),
    // добавляем сигнальную последовательность в конец выходного буфера
    if (consecutive_ff >= FF_THRESHOLD || ((!first_call) && current_addr == initial_addr) || current_addr == (initial_addr - 1) ||current_addr == (initial_addr - 2) ||current_addr == (initial_addr + 2) ||current_addr == (initial_addr + 1) ||current_addr == (initial_addr + 3) ||current_addr == (initial_addr -3)) 
    {
        const char marker[] = "\r\n-\r\n";
        int marker_len = sizeof(marker) - 1; // Без завершающего '\0'
        if (buf_index + marker_len < max_buf_size)
        {
            memcpy((void *)&log_for_wed[buf_index], marker, marker_len);
            buf_index += marker_len;
        }
        // Сброс состояния для следующей передачи – в следующий раз начинаем с самой свежей записи
        first_call = 1;
    }
    else if(first_call)
    {
      first_call = 0;
    }
    // Функция используется для SSI (<!--LOG-->),
    // поэтому возвращаем 0 

}


//Логика обработки /Save запросов от клиента в веб интерфейсе
const char * SAVE_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
  
  first_call = 1;
  
  
  char ip[16] = {0};
  char mask[16] = {0};
  char gateway[16] = {0};
  char mac[18] = {0};
  char serial[18] = {0};
  char rs485speed[18] = {0};
  char rs485paritiy[18] = {0};
  char rs485stopbit[18] = {0};
  
  
  
  char c_phase_a_str[18]  = {0};
  char r_leak_a_str[18]   = {0};
  char c_phase_b_str[18]  = {0};
  char r_leak_b_str[18]   = {0};
  char c_phase_c_str[18]  = {0};
  char r_leak_c_str[18]   = {0};
  char target_value_str[18] = {0};
  char hw_protection_arr[18] = {0};
  char warning_value_str[18] = {0};
  char date_arr[50] = {0};

  uint8_t c_phase_a_flag     = 0;
  uint8_t r_leak_a_flag      = 0;
  uint8_t c_phase_b_flag     = 0;
  uint8_t r_leak_b_flag      = 0;
  uint8_t c_phase_c_flag     = 0;
  uint8_t r_leak_c_flag      = 0;
  uint8_t target_value_flag  = 0;
  uint8_t hw_protection_flag = 0;
  uint8_t warning_value_flag = 0;
  uint8_t date_flag          = 0;
  
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
     else if (strcmp(pcParam[i], "c_phase_a") == 0)
    {
      strncpy(c_phase_a_str, pcValue[i], sizeof(c_phase_a_str) - 1);
      c_phase_a_flag = 1;
    }
    else if (strcmp(pcParam[i], "r_leak_a") == 0)
    {
      strncpy(r_leak_a_str, pcValue[i], sizeof(r_leak_a_str) - 1);
      r_leak_a_flag = 1;
    }
    else if (strcmp(pcParam[i], "c_phase_b") == 0)
    {
      strncpy(c_phase_b_str, pcValue[i], sizeof(c_phase_b_str) - 1);
      c_phase_b_flag = 1;
    }
    else if (strcmp(pcParam[i], "r_leak_b") == 0)
    {
      strncpy(r_leak_b_str, pcValue[i], sizeof(r_leak_b_str) - 1);
      r_leak_b_flag = 1;
    }
    else if (strcmp(pcParam[i], "c_phase_c") == 0)
    {
      strncpy(c_phase_c_str, pcValue[i], sizeof(c_phase_c_str) - 1);
      c_phase_c_flag = 1;
    }
    else if (strcmp(pcParam[i], "r_leak_c") == 0)
    {
      strncpy(r_leak_c_str, pcValue[i], sizeof(r_leak_c_str) - 1);
      r_leak_c_flag = 1;
    }
    else if (strcmp(pcParam[i], "target_value") == 0)
    {
      strncpy(target_value_str, pcValue[i], sizeof(target_value_str) - 1);
      target_value_flag = 1;
    }
    else if (strcmp(pcParam[i], "warning_value") == 0)
    {
      strncpy(warning_value_str, pcValue[i], sizeof(warning_value_str) - 1);
      warning_value_flag = 1;
    }
    else if (strcmp(pcParam[i], "checked") == 0) //   WH_PROTECTION
    {
      strncpy(hw_protection_arr, pcValue[i], sizeof(target_value_str) - 1);
      hw_protection_flag = 1;
    }
    else if(strcmp(pcParam[i], "test") == 0)
    {
      button_ivent = 1;
    }
    else if(strcmp(pcParam[i], "date") == 0)
    {
      strncpy(date_arr, pcValue[i], sizeof(date_arr) - 1);
      date_flag = 1;
    }
    else if (strcmp(pcParam[i], "givetime") == 0) 
    {
       uint16_t mask = (1 << 2);  
       REGISTERS[4] = (REGISTERS[4] | mask); 
    }
    else if (strcmp(pcParam[i], "swichmode") == 0) 
    {
       REGISTERS[4] ^= (1 << 4);
       
       if(mode)
       {
         mode = 0;
       }
       else
       {
         mode = 1;
       }
    }
    

  
  }
  
  if(date_flag != 0)
  {
    taskENTER_CRITICAL();
    save_time_to_rtc((uint8_t*) date_arr);
    date_flag = 0;
    taskEXIT_CRITICAL();
  }
  
  if(mac_flag != 0)
  {
    convert_str_to_uint8_array(mac, output, 1);
    taskENTER_CRITICAL();
    WriteFlash(MAC, output);
    memset(output, 0, sizeof(output));
    mac_flag = 0;
    ReadFlash(MAC, gnetif.hwaddr);
    restart = 1;
    pinaccept = 1;
    taskEXIT_CRITICAL();
  }
  if(serial_flag != 0)
  {
    convert_str_to_uint8_array_serial(serial, output);
    taskENTER_CRITICAL();
    WriteFlash(SERIAL, output);
    memset(output, 0, sizeof(output));
    mask_flag = 0;
    ReadFlash(SERIAL, SERIAL_ADDRESS);
    restart = 1;
    pinaccept = 1;
    taskEXIT_CRITICAL();
  }

  
  
    if (c_phase_a_flag != 0)
    {
      convert_str_to_float_bytes(c_phase_a_str, output); 
      taskENTER_CRITICAL();
      WriteFlash(C_PHASE_A, output);
      memset(output, 0, sizeof(output));
      c_phase_a_flag = 0;
      C_phase_A = *((float *)FLASH_ADDRESS_C_PHASE_A);
      write_to_log(0x12, (uint8_t *)&C_phase_A, sizeof(C_phase_A));
      taskEXIT_CRITICAL();
    }

    if (r_leak_a_flag != 0)
    {
      convert_str_to_float_bytes(r_leak_a_str, output);
      taskENTER_CRITICAL();
      WriteFlash(R_LEAK_A, output);
      memset(output, 0, sizeof(output));
      r_leak_a_flag = 0;
      R_leak_A  = *((float *)FLASH_ADDRESS_R_LEAK_A);
      
      write_to_log(0x15, (uint8_t *)&R_leak_A, sizeof(R_leak_A));
      taskEXIT_CRITICAL();
    }

    if (c_phase_b_flag != 0)
    {
      taskENTER_CRITICAL();
      convert_str_to_float_bytes(c_phase_b_str, output); 
      WriteFlash(C_PHASE_B, output);
      memset(output, 0, sizeof(output));
      c_phase_b_flag = 0;
      C_phase_B = *((float *)FLASH_ADDRESS_C_PHASE_B);
      
      write_to_log(0x13, (uint8_t *)&C_phase_B, sizeof(C_phase_B));
      taskEXIT_CRITICAL();
    }

    if (r_leak_b_flag != 0)
    {
      taskENTER_CRITICAL();
      convert_str_to_float_bytes(r_leak_b_str, output);
      WriteFlash(R_LEAK_B, output);
      memset(output, 0, sizeof(output));
      r_leak_b_flag = 0;
      R_leak_B  = *((float *)FLASH_ADDRESS_R_LEAK_B);
      
      write_to_log(0x16, (uint8_t *)&R_leak_B, sizeof(R_leak_B));
      taskEXIT_CRITICAL();
    }

    if (c_phase_c_flag != 0)
    {
      taskENTER_CRITICAL();
      convert_str_to_float_bytes(c_phase_c_str, output);
      WriteFlash(C_PHASE_C, output);
      memset(output, 0, sizeof(output));
      c_phase_c_flag = 0;
      C_phase_C = *((float *)FLASH_ADDRESS_C_PHASE_C);
      
      write_to_log(0x14, (uint8_t *)&C_phase_C, sizeof(C_phase_C));
      taskEXIT_CRITICAL();
    }

    if (r_leak_c_flag != 0)
    {
      taskENTER_CRITICAL();
     convert_str_to_float_bytes(r_leak_c_str, output);
      WriteFlash(R_LEAK_C, output);
      memset(output, 0, sizeof(output));
      r_leak_c_flag = 0;
      R_leak_C  = *((float *)FLASH_ADDRESS_R_LEAK_C);
      
      write_to_log(0x17, (uint8_t *)&R_leak_C, sizeof(R_leak_C));
      taskEXIT_CRITICAL();
    }

    if (target_value_flag != 0)
    {
      taskENTER_CRITICAL();
      output[0] = (uint8_t)atoi(target_value_str);
      WriteFlash(TaRGET_VALUE, output);
      memset(output, 0, sizeof(output));
      target_value_flag = 0;
      TARGET_VALUE = *((uint8_t *)FLASH_ADDRESS_TARGET_VALUE);
      
      write_to_log(0x18, &TARGET_VALUE, 1);
      taskEXIT_CRITICAL();
    }
  
      if (warning_value_flag != 0)
    {
      taskENTER_CRITICAL();
      output[0] = (uint8_t)atoi(warning_value_str);
      WriteFlash(Warning_VALUE, output);
      memset(output, 0, sizeof(output));
      warning_value_flag = 0;
      WARNING_VALUE = *((uint8_t *)FLASH_ADDRESS_WARNING_VALUE);
      
      write_to_log(0x19, &WARNING_VALUE, 1);
      taskEXIT_CRITICAL();
    }
  
    /*
    if (hw_protection_flag != 0)
    {
      output[0] = (uint8_t)atoi(hw_protection_arr);
      WriteFlash(HW_PROTECTION, output);
      memset(output, 0, sizeof(output));
      hw_protection_flag = 0;
      hw_protection = *((uint8_t *)FLASH_ADDRESS_HW_PROTECTION);
      
      write_to_log(0x20, &hw_protection, 1);
    }
    */
    
    




   
    

    //отключить аппаратное срабатывание защиты, т.к. настройки фаз изменились
    /*
    if(hw_protection == 0)
    {
      EXTI6_DeInit();  
    }
    else if(hw_protection == 1)
    {
      EXTI6_Init();
    }
    */
 
  
  
  if(pinaccept)
  {
    taskENTER_CRITICAL();
    if(ip_flag != 0)
    {
      convert_str_to_uint8_array(ip, output, 0);
      WriteFlash(IP, output);
      memset(output, 0, sizeof(output));
      ip_flag = 0; 
      ReadFlash(IP, IP_ADDRESS);
      
      write_to_log(0x02, IP_ADDRESS, 4);
    }  
    if(mask_flag != 0)
    {
      convert_str_to_uint8_array(mask, output, 0);
      WriteFlash(NETMASK, output);
      memset(output, 0, sizeof(output));
      mask_flag = 0;
      ReadFlash(NETMASK, NETMASK_ADDRESS);
      
      write_to_log(0x03, NETMASK_ADDRESS, 4);
    } 
    if(gateway_flag != 0)
    {
      convert_str_to_uint8_array(gateway, output, 0);
      WriteFlash(GATEWAY, output);
      memset(output, 0, sizeof(output));
      gateway_flag = 0;   
      ReadFlash(GATEWAY, GATEWAY_ADDRESS);
      
      write_to_log(0x04, GATEWAY_ADDRESS, 4);
    }      
    if(speed_flag != 0)
    {
      memcpy(output, rs485speed, sizeof(rs485speed)); 
      WriteFlash(RS485SPEED, output);
      memset(output, 0, sizeof(output));
      speed_flag = 0;   
      ReadFlash(RS485SPEED, USART_3_SPEED);
      
      write_to_log(0x09, USART_3_SPEED, 4);
    }
    if(paritiy_flag != 0)
    {
      //convert_str_to_uint8_array(rs485paritiy, output, 2);
      WriteFlash(RS485PARITIY, (uint8_t*)rs485paritiy);
      memset(output, 0, sizeof(output));
      paritiy_flag = 0;   
      ReadFlash(RS485PARITIY, (uint8_t*)uartPARITY);
      
      write_to_log(0x10, (uint8_t*)uartPARITY, 4);
    }
    if(stopbit_flag != 0)
    {
      convert_str_to_uint8_array(rs485stopbit, output, 2);
      WriteFlash(RS485STOPBIT, output);
      memset(output, 0, sizeof(output));
      stopbit_flag = 0;   
      ReadFlash(RS485STOPBIT, uartStopBits);
      
      write_to_log(0x11, uartStopBits, 1);
    }
    
    taskEXIT_CRITICAL();
    restart = 1;

    return 0;
  }

  
  
  
  
  return 0;
}



const char * JSON_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
  first_call = 1;
  
  return 0; //там просто подставить SSI тег текущего тока
}

uint8_t loghhh = 0;


const char * LOG_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{       
  
  
  /*
    // Порог пустых записей (0xFF)
    uint8_t FF_THRESHOLD = 5;
    
    // Статические переменные для отслеживания состояния между вызовами
    // current_addr – текущая позиция чтения (идём в обратном направлении)
    // initial_addr – адрес, с которого начался текущий цикл передачи (для определения полного круга)
    // first_call – флаг первого вызова в рамках одной последовательной передачи логов
    static uint32_t current_addr = 0;
    static uint32_t initial_addr = 0;

    memset((void *)log_for_wed, 0, LOG_BUFF_SIZE);
    
    int buf_index = 0;
    int max_buf_size = sizeof(log_for_wed);  // Размер выходного буфера (например, 1 КБ)
    int max_entries = max_buf_size / LOG_ENTRY_SIZE; // Количество записей, умещающихся в буфере
    int consecutive_ff = 0; // Счётчик подряд идущих пустых (0xFF) записей

    // При первом вызове начинаем с самой новой записи
    if (first_call)
    {
        if (log_ptr == LOG_START_ADDR_BLOCK)
            // Если указатель равен началу, начинаем с последней валидной записи
            current_addr = LOG_END_ADDR_BLOCK - LOG_ENTRY_SIZE;
        else
            current_addr = log_ptr - LOG_ENTRY_SIZE;
        // Запоминаем стартовую позицию для определения полного круга
        initial_addr = current_addr;
        first_call = 0;
    }

    // Чтение логов в обратном порядке (от самой новой записи)
    for (int i = 0; i < max_entries; i++)
    {
        // Если уже прошли полный круг, прекращаем чтение
        if (i > 0 && current_addr == initial_addr)
            break;

        // Проверка корректности указателя; если он вне диапазона,
        // устанавливаем его на последнюю валидную запись
        if (current_addr < LOG_START_ADDR_BLOCK || current_addr > (LOG_END_ADDR_BLOCK - LOG_ENTRY_SIZE)) {
            current_addr = LOG_END_ADDR_BLOCK - LOG_ENTRY_SIZE;
        }
        
        uint8_t entry[LOG_ENTRY_SIZE];
        // Считываем одну запись по текущему адресу
        
        
        
        //Логика для пропуска лишних 8-ми байт в конце каждого сектора (иначе сбивается кратность 12-ти)
        if ((current_addr & 0xDFFFF) == 0xDFFF4) 
        {
          current_addr -= 8;
        }
        else if ((current_addr & 0xBFFFF) == 0xBFFF4)
        {
          current_addr -= 8;
        }
        else if ((current_addr & 0x9FFFF) == 0x9FFF4)
        {
          current_addr -= 8;
        }
        else if ((current_addr & 0x7FFFF) == 0x7FFF4)
        {
          current_addr -= 8;
        }
        else if ((current_addr & 0x5FFFF) == 0x5FFF4)
        {
          current_addr -= 8;
        }
        else if ((current_addr & 0x3FFFF) == 0x3FFF4)
        {
          current_addr -= 8;
        }
          
          
          
          
        memcpy(entry, (const void *)current_addr, LOG_ENTRY_SIZE);

        // Проверяем, является ли запись "пустой" (все байты равны 0xFF)
        int is_ff = 1;
        for (int j = 0; j < LOG_ENTRY_SIZE; j++)
        {
            if (entry[j] != 0xFF)
            {
                is_ff = 0;
                break;
            }
        }

        // Подсчёт подряд идущих пустых записей
        if (is_ff)
            consecutive_ff++;
        else
            consecutive_ff = 0;

        // Если встречено достаточное число пустых записей, считаем, что валидных логов больше нет
        if (consecutive_ff >= FF_THRESHOLD)
            break;

        // Копируем запись в выходной буфер
        memcpy((void *)&log_for_wed[buf_index], entry, LOG_ENTRY_SIZE);
        buf_index += LOG_ENTRY_SIZE;


        // Обновляем указатель. Если вычитание размера записи привело бы к значению ниже начала сектора,
        // переносим указатель на последнюю валидную запись
        if (current_addr - LOG_ENTRY_SIZE < LOG_START_ADDR_BLOCK)
            current_addr = LOG_END_ADDR_BLOCK - LOG_ENTRY_SIZE;
        else
            current_addr -= LOG_ENTRY_SIZE;
    }

    // Если достигнут конец логов (либо по порогу пустых записей, либо полный круг пройден),
    // добавляем сигнальную последовательность в конец выходного буфера
    if (consecutive_ff >= FF_THRESHOLD || ((!first_call) && current_addr == initial_addr) || current_addr == (initial_addr - 1) ||current_addr == (initial_addr - 2) ||current_addr == (initial_addr + 2) ||current_addr == (initial_addr + 1) ||current_addr == (initial_addr + 3) ||current_addr == (initial_addr -3)) 
    {
        const char marker[] = "\r\n-\r\n";
        int marker_len = sizeof(marker) - 1; // Без завершающего '\0'
        if (buf_index + marker_len < max_buf_size)
        {
            memcpy((void *)&log_for_wed[buf_index], marker, marker_len);
            buf_index += marker_len;
        }
        // Сброс состояния для следующей передачи – в следующий раз начинаем с самой свежей записи
        first_call = 1;
    }
    else if(first_call)
    {
      first_call = 0;
    }
    // Функция используется для SSI (<!--LOG-->),
    // поэтому возвращаем 0 
*/
    return 0;
}
//---------------------------------------------------------------------------------HTTPD-SERVER-LOGICS-END---




//---------------------------------------------------------------------------------FLASH-LOGICS-START--------

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

void convert_str_to_float_bytes(const char* input, uint8_t* output)
{
    // Преобразуем строку в float
    float value = atof(input);

    // Копируем байты float в массив output
    memcpy(output, &value, sizeof(float));
}

void convert_str_to_uint8_array_serial(const char* input, uint8_t* output)
{
  for (int i = 0; i < 6; i++) 
  {
    output[i] = input[i] - '0';
  }
}

void WriteToFlash(uint32_t startAddress, uint8_t* data, uint32_t length)
{

    HAL_FLASH_Unlock();
    
    if((startAddress >= 0x0800C000) && (startAddress <= 0x0800FFFF))
    {
      FLASH_Erase_Sector(FLASH_SECTOR_3, VOLTAGE_RANGE_3);
    }
    else if((startAddress >= 0x08100000) && (startAddress <= 0x08103FFF))
    {
      FLASH_Erase_Sector(FLASH_SECTOR_12, VOLTAGE_RANGE_3);
    }
    
    for (uint32_t i = 0; i < length; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, startAddress + i, data[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return;
        }
    }

    HAL_FLASH_Lock();
}


// Функция для работы с переменными в памяти
void WriteFlash(FlashDataType type, uint8_t* data)
{
    size_t minFreeHeapSize = xPortGetMinimumEverFreeHeapSize();
    taskENTER_CRITICAL();
    uint8_t sectorData[0x1000]; // Формат сектора памяти (4 KB)
    uint32_t *address = 0;
    uint32_t dataSize = 0;
  
    // Считывание полного сектора
    for (uint32_t i = 0; i < 0x1000; i++) {
        sectorData[i] = *(volatile uint8_t*)(0x0800C000 + i);
    }
    
    
    // Изменение значений в буфере 
    sectorData[BOOT_OS_OK_ADDRESS - 0x0800C000] = boot_os_ok;
    //sectorData[BOOT_FLAG_CRC_ADDRESS - 0x0800C000] = boot_flag_crc;
    sectorData[BOOT_FLAG_NEW_ADDRESS - 0x0800C000] = boot_flag_new;
    sectorData[SECTOR_ENABLED_ADDRESS - 0x0800C000] = sector_enabled;
    sectorData[LOG_SECTOR_ACTIVE - 0x0800C000] = log_sector_active;
    

    uint32_t index = BOOT_CRC_ADDRESS - 0x0800C000;
    
    sectorData[index + 0] = (uint8_t)(crc_os & 0xFF);        // младший байт
    sectorData[index + 1] = (uint8_t)((crc_os >> 8) & 0xFF);  // второй байт
    sectorData[index + 2] = (uint8_t)((crc_os >> 16) & 0xFF); // третий байт
    sectorData[index + 3] = (uint8_t)((crc_os >> 24) & 0xFF); // старший байт
    
    
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
        dataSize = 10;
        break;
      case RS485PARITIY:
        address = (uint32_t *)FLASH_ADDRESS_RS485_PARITIY;
        dataSize = 10;
        break;
      case RS485STOPBIT:
        address = (uint32_t *)FLASH_ADDRESS_RS485_STOPBIT;
        dataSize = 1;
        break;
      case C_PHASE_A:
        address = (uint32_t *)FLASH_ADDRESS_C_PHASE_A;
        dataSize = 4;
        break;
      case R_LEAK_A:
        address = (uint32_t *)FLASH_ADDRESS_R_LEAK_A;
        dataSize = 4;
        break;
      case C_PHASE_B:
        address = (uint32_t *)FLASH_ADDRESS_C_PHASE_B;
        dataSize = 4;
        break;
      case R_LEAK_B:
        address = (uint32_t *)FLASH_ADDRESS_R_LEAK_B;
        dataSize = 4;
        break;
      case C_PHASE_C:
        address = (uint32_t *)FLASH_ADDRESS_C_PHASE_C;
        dataSize = 4;
        break;
      case R_LEAK_C:
        address = (uint32_t *)FLASH_ADDRESS_R_LEAK_C;
        dataSize = 4;
        break;
      case TaRGET_VALUE:
        address = (uint32_t *)FLASH_ADDRESS_TARGET_VALUE;
        dataSize = 1;
        break;
      case HW_PROTECTION:
        address = (uint32_t *)FLASH_ADDRESS_HW_PROTECTION;
        dataSize = 1;
        break;
      case Warning_VALUE:
        address = (uint32_t *)FLASH_ADDRESS_WARNING_VALUE;
        dataSize = 1;
        break;
      default:
        return; 
      }
    
    if (data) 
    {
    memcpy(&sectorData[(uint32_t)address - 0x0800C000], data, dataSize);
    }
    
    
    osDelay(150);
    // Запись полного сектора в память
    WriteToFlash(0x0800C000, sectorData, 0x1000);
    taskEXIT_CRITICAL();
}

void load_values_from_flash(void)
{
    uint8_t i = 0;
    uint32_t temp; 
    
    // Чтение и проверка C_phase_A
    temp = *(volatile uint32_t *)FLASH_ADDRESS_C_PHASE_A;
    if (temp != 0xFFFFFFFF) {
       memcpy(&C_phase_A, &temp, sizeof(float));
    } else {
        //C_phase_A = 0.3f;
      C_phase_A = 0.5f;
    }

    // Чтение и проверка R_leak_A
    temp = *(volatile uint32_t *)FLASH_ADDRESS_R_LEAK_A;
    if (temp != 0xFFFFFFFF) {
        memcpy(&R_leak_A, &temp, sizeof(float));
    } else {
        R_leak_A = 10.0f;
    }

    // Чтение и проверка C_phase_B
    temp = *(volatile uint32_t *)FLASH_ADDRESS_C_PHASE_B;
    if (temp != 0xFFFFFFFF) {
        memcpy(&C_phase_B, &temp, sizeof(float));
    } else {
        //C_phase_B = 0.3f;
      C_phase_B = 0.5f;
   }

    // Чтение и проверка R_leak_B
    temp = *(volatile uint32_t *)FLASH_ADDRESS_R_LEAK_B;
    if (temp != 0xFFFFFFFF) {
        memcpy(&R_leak_B, &temp, sizeof(float));
    } else {
        R_leak_B = 10.0f;
    }

    // Чтение и проверка C_phase_C
    temp = *(volatile uint32_t *)FLASH_ADDRESS_C_PHASE_C;
    if (temp != 0xFFFFFFFF) {
        memcpy(&C_phase_C, &temp, sizeof(float));
    } else {
        //C_phase_C = 0.3f;
        C_phase_C = 0.5f;
    }

    // Чтение и проверка R_leak_C
    temp = *(volatile uint32_t *)FLASH_ADDRESS_R_LEAK_C;
    if (temp != 0xFFFFFFFF) {
        memcpy(&R_leak_C, &temp, sizeof(float));
    } else {
        R_leak_C = 10.0f;
    }

    // Чтение и проверка TARGET_VALUE
    TARGET_VALUE = *(volatile uint8_t *)FLASH_ADDRESS_TARGET_VALUE;
    if (TARGET_VALUE == 0xFF) {
        TARGET_VALUE = TARGET_VALUE_DEF;
    }
    
    /*
    hw_protection = *(volatile uint8_t *)FLASH_ADDRESS_HW_PROTECTION;
    if (hw_protection == 0xFF) {
        hw_protection = 1;
    }
    */
    
    WARNING_VALUE = *(volatile uint8_t *)FLASH_ADDRESS_WARNING_VALUE;
    if(WARNING_VALUE == 0xFF) {
       WARNING_VALUE = WARNING_VALUE_DEF;
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





#define ADC_BUFFER_SIZE 457  // Пример размера буфера; 
#define MA_WINDOW_SIZE    5   // Размер окна для скользящего среднего



uint16_t adc_get_rms(uint16_t *arr, uint16_t length)
{

    uint16_t rms = 0;
    float sum_sq = 0.0f;
    uint32_t cnt = (length < ADC_BUFFER_SIZE) ? length : ADC_BUFFER_SIZE;
  
    
  
  
    
    for (uint32_t k = 0; k < cnt; k++)
    {
        sum_sq += (float)arr[k] * (float)arr[k];
    }
    float rms_f = sqrtf(sum_sq / (float)cnt);
    rms = (uint16_t)rms_f;
      

    return rms;
}






//показывает что данные с ADC готовы к обработке
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) 
  {
    adc_ready = 1;
    HAL_ADC_Stop_DMA(&hadc1);
    //----------------------------
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xHighPrioritySemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
  __disable_irq();
  SCB->VTOR = FLASH_BASE;  // обычно 0x08000000
  __DSB();

  __ISB();
  HAL_NVIC_SystemReset();
}



void startMyTimer_UPDATE(uint32_t timeout_ms) 
{
  TimerHandle_t myTimer2 = xTimerCreate
    (
     "OneShotTimer",                       
     pdMS_TO_TICKS(timeout_ms),            
     pdFALSE,                              
     (void *) 0,                           
     vMyTimer2Callback                      
       );
  
  if (myTimer2 != NULL) 
  {
    HAL_NVIC_DisableIRQ(ADC_IRQn);
    osThreadSuspend(HighPriorityTaskHandle);
    osThreadSuspend(defaultTaskHandle);
    osThreadSuspend(Relay_taskHandle);
    osThreadSuspend(mobdusHandle);
    osThreadSuspend(Relay_controlHandle);
    osThreadSuspend(LWGL_controlHandle);
    xTimerStart(myTimer2, 0);  
  } 
  else 
  {
    while(1);
  }
}
//---↑
//---↑
void vMyTimer2Callback(TimerHandle_t xTimer) 
{
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  osThreadResume(HighPriorityTaskHandle);
  osThreadResume(defaultTaskHandle);
  osThreadResume(Relay_taskHandle);
  osThreadResume(mobdusHandle);
  osThreadResume(Relay_controlHandle);
  osThreadResume(LWGL_controlHandle);
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
    

    button_ivent = 1;
    
    for (int i = 0; i < 100; i++) 
    {
    }
    
  }
}
//HAL_GPIO_WritePin(Checking_for_leaks_GPIO_Port, Checking_for_leaks_Pin, GPIO_PIN_SET);



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

void EXTI6_DeInit(void)
{
    // Отключить маску прерывания для EXTI6
    EXTI->IMR &= ~EXTI_IMR_MR6;
    
    // Отключить триггер на восходящий фронт
    EXTI->RTSR &= ~EXTI_RTSR_TR6;
    
    // Отключить триггер на нисходящий фронт (если он был настроен)
    EXTI->FTSR &= ~EXTI_FTSR_TR6;
    
    // Отключить прерывание EXTI9_5 в NVIC
    NVIC_DisableIRQ(EXTI9_5_IRQn);
    
    // Опционально: сбросить конфигурацию SYSCFG для EXTI6
    SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI6);
}
//---------------------------------------------------------------------------------ANOTHER-CODE-END---

void load_flags_from_flash(void)
{
    boot_os_ok = *(volatile uint8_t *)BOOT_OS_OK_ADDRESS;
    if (boot_os_ok == 0xFF)
    {
      boot_os_ok = 0;
    }
    

    
    sector_enabled = *(volatile uint8_t *)SECTOR_ENABLED_ADDRESS;
    if((sector_enabled != 1) && (sector_enabled != 2))
    {
      sector_enabled = 1;
    }
    
    log_sector_active = *(volatile uint8_t *)LOG_SECTOR_ACTIVE;
    if(log_sector_active == 0xFF)
    {
      log_sector_active = 1;
    }
    
    switch (log_sector_active){
      case 1:
        LOG_START_ADDR = 0x08120000; 
        LOG_END_ADDR = 0x0813FFFF;
        break;
      case 2: 
        LOG_START_ADDR = 0x08140000; 
        LOG_END_ADDR = 0x0815FFFF;
        break;
      case 3:
        LOG_START_ADDR = 0x08160000; 
        LOG_END_ADDR = 0x0817FFFF;
        break;
      case 4:
        LOG_START_ADDR = 0x08180000; 
        LOG_END_ADDR = 0x0819FFFF;
        break;
      case 5:
        LOG_START_ADDR = 0x081A0000; 
        LOG_END_ADDR = 0x081BFFFF;
        break;
      case 6:
        LOG_START_ADDR = 0x081C0000; 
        LOG_END_ADDR = 0x081DFFFF;
        break;
      case 7:
        LOG_START_ADDR = 0x081E0000; 
        LOG_END_ADDR = 0x081FFFFF;
        break;      
      
      
      
    }
    
    log_ptr = find_next_free_log_address();
    if (log_ptr == 0xFF)
    {
       Swipe_Log_Sector();
    }
    
}



        
HAL_StatusTypeDef Flash_WritePacket(uint8_t *packet, uint16_t packet_size)
{
  
  taskENTER_CRITICAL();
  HAL_FLASH_Unlock();
  HAL_StatusTypeDef status = HAL_OK;
  
    static uint8_t first = 1;
    if(first)
    {
      address = SECTOR_2_ADDRESS;
      len = 393216;
      next_free_addr = SECTOR_2_ADDRESS;
      
      first = 0;
    }
  
  
    // Проверка на переполнение области
    if ((next_free_addr + packet_size) >= (address + len + 1))
    {
        HAL_FLASH_Lock();
        taskEXIT_CRITICAL();
        return HAL_ERROR; // Флеш память переполнена
    }
    


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
            taskEXIT_CRITICAL();
            return status; // Ошибка при записи
        }
    }

    // Сохраняем указатель на свободную ячейку для следующего пакета
    next_free_addr += packet_size;  // ---invalid
    // Блокировка флеш памяти после записи
    iteration++;
    HAL_FLASH_Lock();
   
    taskEXIT_CRITICAL();
    //xSemaphoreGive(xPacketSaved);
    return status;
}

void erise_update_sector(void)
{
  
    taskENTER_CRITICAL();
    // Разблокировка флеш памяти для записи
    HAL_FLASH_Unlock();
    __disable_irq();
    
    HAL_StatusTypeDef status = HAL_OK;
    

    
    
    
    
    if(next_free_addr == 0)
    {
            address = SECTOR_2_ADDRESS;
            len = 393216;
            next_free_addr = SECTOR_2_ADDRESS;
            FLASH_Erase_Sector(FLASH_SECTOR_8, VOLTAGE_RANGE_3);
            FLASH_Erase_Sector(FLASH_SECTOR_9, VOLTAGE_RANGE_3);
            FLASH_Erase_Sector(FLASH_SECTOR_10, VOLTAGE_RANGE_3);
    }
    
    
    __enable_irq();
    HAL_FLASH_Lock();
    taskEXIT_CRITICAL();
}

//---------------------------------------------------------------------------------OS-UDATE-FUNCTIONS---

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
    }
    

    if (Flash_WritePacket(packetData, BinDataLength) != HAL_OK)
    {
        pbuf_free(p);
        error_flash = 1;
        boot_flag_new = 1;
        WriteFlash(0, 0);
        return ERR_BUF; 
    }


    httpd_post_data_recved(connection, packet_size);
    pbuf_free(p);
    
    if(packetNow == (packetTotal-1))
    {
      boot_flag_new = 1;
      WriteFlash(0, 0);
    }
    

       
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

    const char *success_message = "/update.shtml"; // Путь к странице успешного ответа
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
    __HAL_CRC_DR_RESET(&hcrc);
    
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
    uint32_t reversed_answer = 0;
    reversed_answer |= (answer & 0x000000FF) << 24;  // младший байт в старший
    reversed_answer |= (answer & 0x0000FF00) << 8;   // второй байт на второй
    reversed_answer |= (answer & 0x00FF0000) >> 8;   // третий байт на третий
    reversed_answer |= (answer & 0xFF000000) >> 24;  // старший байт на младший

    return reversed_answer;
}

void swichSector()
{
  switch (sector_enabled){
    
  case 1: 
    sector_enabled = 2;
    break;
  case 2: 
    sector_enabled = 1;
    break;

  }    
}

//---------------------------------------------------------------------------------

//--------------------------RMS--BY--MACROS----------------------------------------
float calculate_rms_A_macros(uint16_t rms)
{
    float I_s = rms * 0.0000466f;

    float XC_phase_A = 1.0f / (OMEGA * (C_phase_A * 1e-6f));


    float R_eq_phase_A = ((R_leak_A * 1e6f) * XC_phase_A) /
                         ((R_leak_A * 1e6f) + XC_phase_A);

    float up_formula = 2 * I_s * COS30 * MULT_UP;

    float down_formula = MULT_DOWN * R_eq_phase_A * SQ3;

    float result = (up_formula / down_formula) * 1000.0f;
    return result;
}

float calculate_rms_B_macros(uint16_t rms)
{
    float I_s = rms * 0.0000466f;

    float XC_phase_B = 1.0f / (OMEGA * (C_phase_B * 1e-6f));


    float R_eq_phase_B = ((R_leak_B * 1e6f) * XC_phase_B) /
                         ((R_leak_B * 1e6f) + XC_phase_B);

    float up_formula = 2 * I_s * COS30 * MULT_UP;

    float down_formula = MULT_DOWN * R_eq_phase_B * SQ3;

    float result = (up_formula / down_formula) * 1000.0f;
    return result;
}


float calculate_rms_C_macros(uint16_t rms)
{
    float I_s = rms * 0.0000466f;

    float XC_phase_C = 1.0f / (OMEGA * (C_phase_C * 1e-6f));


    float R_eq_phase_C = ((R_leak_C * 1e6f) * XC_phase_C) /
                         ((R_leak_C * 1e6f) + XC_phase_C);

    float up_formula = 2 * I_s * COS30 * MULT_UP;

    float down_formula = MULT_DOWN * R_eq_phase_C * SQ3;

    float result = (up_formula / down_formula) * 1000.0f;
    return result;
}

//--------------------------------------------------------------------------------------Positive Leakage Trend Analysis

//------------------------------------
void initBuffer10Min(CircularBuffer10Min *buffer) {
    for(int i = 0; i < MINUTE_10_SIZE; i++) buffer->arr[i] = 0;
    buffer->index = 0;
    buffer->sum = 0;
}
void initBuffer1Hour(CircularBuffer1Hour *buffer) {
    for(int i = 0; i < HOUR_1_SIZE; i++) buffer->arr[i] = 0;
    buffer->index = 0;
    buffer->sum = 0;
}
void initBuffer2Day(CircularBuffer2Day *buffer) {
    for(int i = 0; i < DAY_2_SIZE; i++) buffer->arr[i] = 0;
    buffer->index = 0;
    buffer->sum = 0;
}
//-----------------------------------------
void init_circular_buffers(CircularBuffer10Min *buffer, CircularBuffer1Hour *buffer2, CircularBuffer2Day *buffer3)
{
  initBuffer10Min(buffer);
  initBuffer1Hour(buffer2);
  initBuffer2Day(buffer3);
}
//---------------------------------------------

void addValue10Min(CircularBuffer10Min *buffer, uint16_t value) {
    // Вычитаем старое значение из суммы
    buffer->sum -= buffer->arr[buffer->index];
    // Добавляем новое значение
    buffer->arr[buffer->index] = value;
    buffer->sum += value;
    // Сдвигаем индекс
    buffer->index = (buffer->index + 1) % MINUTE_10_SIZE;
}

void addValue1Hour(CircularBuffer1Hour *buffer, uint16_t value) {
    buffer->sum -= buffer->arr[buffer->index];
    buffer->arr[buffer->index] = value;
    buffer->sum += value;
    buffer->index = (buffer->index + 1) % HOUR_1_SIZE;
}

void addValue2Day(CircularBuffer2Day *buffer, uint16_t value) {
    buffer->sum -= buffer->arr[buffer->index];
    buffer->arr[buffer->index] = value;
    buffer->sum += value;
    buffer->index = (buffer->index + 1) % DAY_2_SIZE;
}

void RTC_Init(void) 
{
 
  
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();


    __HAL_RCC_LSI_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET) {}

    __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSI);
    
    __HAL_RCC_RTC_ENABLE();

    
    HAL_Delay(100);
  
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    
    /** Инициализация RTC с использованием LSI**/
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    
    HAL_StatusTypeDef rtc_status;
    rtc_status = HAL_RTC_Init(&hrtc);
    
    if (rtc_status != HAL_OK) {
        // Обработка ошибки инициализации
        Error_Handler();
    }

    /** Установка времени: 00:00:00 **/
    sTime.Hours = 0;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    sTime.TimeFormat = RTC_HOURFORMAT12_AM;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }

    /** Установка даты: 01.01.2025 **/
    sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 1;
    sDate.Year = 25; // 2025 год (HAL использует 2 последних цифры)
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }
}




float getAverage10Min(CircularBuffer10Min *buffer) {
    return (float)buffer->sum / MINUTE_10_SIZE;
}

float getAverage1Hour(CircularBuffer1Hour *buffer) {
    return (float)buffer->sum / HOUR_1_SIZE;
}

float getAverage2Day(CircularBuffer2Day *buffer) {
    return (float)buffer->sum / DAY_2_SIZE;
}
//----------------------------------------------RTC-SECTION--------------------
void save_time_to_rtc(uint8_t *arr)
{

    uint16_t mask = (1 << 2);  
    REGISTERS[4] &= ~mask;
    
      uint8_t day   = (arr[0] - '0') * 10 + (arr[1] - '0');
      uint8_t month = (arr[3] - '0') * 10 + (arr[4] - '0');
      uint16_t year = (arr[6] - '0') * 1000 + (arr[7] - '0') * 100 +
                      (arr[8] - '0') * 10   + (arr[9] - '0');
      uint8_t hour  = (arr[11] - '0') * 10 + (arr[12] - '0');
      uint8_t minute = (arr[14] - '0') * 10 + (arr[15] - '0');
      uint8_t second = (arr[17] - '0') * 10 + (arr[18] - '0');

      // Настройка времени RTC
      RTC_TimeTypeDef sTime = {0};
      sTime.Hours = hour;
      sTime.Minutes = minute;
      sTime.Seconds = second;
      sTime.TimeFormat = RTC_HOURFORMAT12_AM;
      sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
      sTime.StoreOperation = RTC_STOREOPERATION_RESET;

      if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
      {
          Error_Handler();
      }

      // Настройка даты RTC
      RTC_DateTypeDef sDate = {0};
      sDate.Date = day;
      sDate.Month = month; 
      sDate.Year = year % 100;  // HAL использует две последние цифры года
      sDate.WeekDay = RTC_WEEKDAY_SUNDAY;

      if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
      {
          Error_Handler();
      }
    
    

}


void get_current_timestamp(uint8_t *timestamp)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    timestamp[0] = sTime.Seconds;
    timestamp[1] = sTime.Minutes;
    timestamp[2] = sTime.Hours;
    timestamp[3] = sDate.Date;
    timestamp[4] = sDate.Month;
    timestamp[5] = sDate.Year;
}


void parse_http_time_request(uint8_t *http_request, uint8_t temp_arr[6]) {
    int index = 0;
    char *token = strtok(http_request, "-");
    while (token != NULL && index < 6) {
        int val = atoi(token);

        // Если значение меньше 2000, сохраняем как есть
        if(val < 2000) {
            temp_arr[index++] = (uint8_t)val;
        }
        // Иначе сохраняем разницу (год относительно 2000)
        else {
            uint8_t year = (uint8_t)(val - 2000);
            temp_arr[index++] = year;
        }

        token = strtok(NULL, "-");
    }
}

//------------------------------------------------LOG-SECTION-------------------
void write_to_log(uint8_t code, uint8_t log_data[], uint16_t copy_len)
{

  /* вынесена в шапку
  
  typedef struct {
    uint8_t timestamp[6];  // ss:mm:hh:dd:mm:yy (временная метка)
    uint8_t event_code;    // код события
    uint8_t data[5];       //поле данных
  } LogEntry; 
                           //12 байт
  */
  
  //код события (основные)
  //0x30 - изменение тока утечки на 10%
  //0x31 - ТЕСТ
  //0x32 - Предупреждение
  //0x33 - сработала защита

  
  
  //код события (настройки)
  //0х02 - IP адрес 
  //0х03 - Маска подсети 
  //0х04 - Шлюз 
  //0х05 - REGISTERS[2] (состоянее реле изменилось (кнопка в вебе, moodbus, аппаратная защита и т.д.) (0х00/0х01)
  //0x07 - Приняли новое П.О.
  //0x09 - USART скорость (4 байта, которые надо собрать в 4 первые цифры)
  //0x10 - USART четность (char)(прям словом ~odd)
  //0x11 - USART стоп бит
  
  //0х12 - C_phase_A (все значения тут - float, без скоращения)
  //0x13 - C_phase_B
  //0x14 - C_phase_C
  //0x15 - R_leak_A
  //0x16 - R_leak_B
  //0x17 - R_leak_C
  
  //0x18 - TARGET_VALUE (uint8_t пороговое значение тока)
  //0x19 - WARNING_VALUE (uint8_t пороговое значение тока)
  //0x20 - Аппаратная защита (0/1)
  
  if(log_ready)
  {

  uint32_t startAddress = log_ptr;
  uint8_t final_data[12] = {0};
    
    
  LogEntry frame;
  
    
    while ((log_ptr + sizeof(LogEntry)) >= LOG_END_ADDR)
    {
       Swipe_Log_Sector();
       startAddress = log_ptr;
    }

  
    get_current_timestamp(frame.timestamp); 
    
    frame.event_code = code;
    
    
    memset(frame.data, 0, sizeof(frame.data)); // Заполняем массив данных нулями
    memcpy(frame.data, log_data, copy_len);
    
    
    memcpy(final_data, &frame, sizeof(frame));
    HAL_FLASH_Unlock();

      
      for (uint32_t i = 0; i < sizeof(LogEntry); i++) {
          if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, startAddress + i, final_data[i]) != HAL_OK) {
              HAL_FLASH_Lock();
              return;
          }
      }

     HAL_FLASH_Lock();
     
     log_ptr += sizeof(LogEntry);
  }
}

static uint8_t flash_read_byte(uint32_t address)
{
    return *((volatile uint8_t*)address);
}


static void flash_write_byte(uint32_t address, uint8_t value)
{
    taskENTER_CRITICAL();
    HAL_FLASH_Unlock();

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, value) != HAL_OK)
    {
        HAL_FLASH_Lock();
        taskEXIT_CRITICAL();
        return;
    }

    HAL_FLASH_Lock();
    taskEXIT_CRITICAL();
}



uint32_t find_next_free_log_address(void)
{

    uint32_t addr = LOG_END_ADDR - 1;

    // 1. Ищем «последний занятый байт», т.е. первый не 0xFF с конца.
    //    Если таких нет, значит память пуста.
    while (1)
    {
        // Если ушли за начало лога, значит вся область пуста.
        if (addr < LOG_START_ADDR)
        {
            // Нет записанных данных
            return LOG_START_ADDR; 
        }

        if (flash_read_byte(addr) != 0xFF)
        {
            temp_count = 9;
            // Нашли байт, который НЕ равен 0xFF – это и будет последний занятый байт.
            break;
        }

        addr--;
    }

    // 2. Подсчитываем, сколько байт всего занято
    //    (LOG_START_ADDR ... addr включительно).
    //    Например, если addr = LOG_START_ADDR, то размер = 1.
    uint32_t used_size = (addr - LOG_START_ADDR) + 1; 

    log_ready = 1;
    
    // 3. Проверяем кратность размеру записи (12 байт).
    uint32_t remainder = used_size % LOG_ENTRY_SIZE;
    if (remainder == 0)
    {
        // 3.1. Если всё чётко по границам 12 байт, 
        //      то свободный адрес – сразу за последним занятым байтом.
      if((LOG_START_ADDR + used_size) < LOG_END_ADDR)
      {
        return LOG_START_ADDR + used_size; 
      }
      else
      {
        return 0xFF; //заполнено все
      }
    }
    else
    {
        // 3.2. Иначе — нужно «дозаписать» остаток до ближайших 12 байт символами 0xF0,
        //      чтобы при следующем чтении было понятно, что эта запись недействительна.

        // Сколько байт не хватает до кратного размера
        uint32_t need_fill = LOG_ENTRY_SIZE - remainder;

        // Адрес начала «обрывка». 
        // Текущее «начало» последней записи по границе 12 байт:
        // Например, если used_size=25, remainder=1, тогда последняя корректная граница – 24.
        uint32_t partial_start = (LOG_START_ADDR + used_size) - remainder; 
        // Верхняя граница, которую мы зальём 0xF0, чтобы закрыть 12-байтную запись:
        uint32_t partial_end   = partial_start + LOG_ENTRY_SIZE - 1; 
        if (partial_end >= LOG_END_ADDR) 
        {
            // На всякий случай проверяем, чтобы не выйти за пределы.
            return 0xFF;
        }

        // Заполняем нужный диапазон 0xF0, превращая остаток в "невалидную запись".
        for (uint32_t fill_addr = partial_start; fill_addr <= partial_end; fill_addr++)
        {
            flash_write_byte(fill_addr, 0xF0);
        }

        // Теперь свободный адрес — сразу за этой 12-байтной «фейковой» записью.
        return (partial_end + 1);
    }
}
   




void Swipe_Log_Sector()
{
  
  log_sector_active = (log_sector_active % 7) + 1;
  
 
  
  taskENTER_CRITICAL();
  HAL_FLASH_Unlock();
  switch (log_sector_active){
        
      case 1:
        LOG_START_ADDR = 0x08120000; 
        log_ptr = 0x08120000;
        LOG_END_ADDR = 0x0813FFFF;
        FLASH_Erase_Sector(FLASH_SECTOR_17, VOLTAGE_RANGE_3);
        break;
      case 2: 
        LOG_START_ADDR = 0x08140000; 
        log_ptr = LOG_START_ADDR;
        LOG_END_ADDR = 0x0815FFFF;
        FLASH_Erase_Sector(FLASH_SECTOR_18, VOLTAGE_RANGE_3);
        break;
      case 3:
        LOG_START_ADDR = 0x08160000; 
        log_ptr = LOG_START_ADDR;
        LOG_END_ADDR = 0x0817FFFF;
        FLASH_Erase_Sector(FLASH_SECTOR_19, VOLTAGE_RANGE_3);
        break;
      case 4:
        LOG_START_ADDR = 0x08180000; 
        log_ptr = LOG_START_ADDR;
        LOG_END_ADDR = 0x0819FFFF;
        FLASH_Erase_Sector(FLASH_SECTOR_20, VOLTAGE_RANGE_3);
        break;
      case 5:
        LOG_START_ADDR = 0x081A0000;
        log_ptr = LOG_START_ADDR;
        LOG_END_ADDR = 0x081BFFFF;
        FLASH_Erase_Sector(FLASH_SECTOR_21, VOLTAGE_RANGE_3);
        break;
      case 6:
        LOG_START_ADDR = 0x081C0000; 
        log_ptr = LOG_START_ADDR;
        LOG_END_ADDR = 0x081DFFFF;
        FLASH_Erase_Sector(FLASH_SECTOR_22, VOLTAGE_RANGE_3);
        break;
      case 7:
        LOG_START_ADDR = 0x081E0000; 
        log_ptr = LOG_START_ADDR;
        LOG_END_ADDR = 0x081FFFF8;
        FLASH_Erase_Sector(FLASH_SECTOR_23, VOLTAGE_RANGE_3);
        break;      
    }
  HAL_FLASH_Lock();
  taskEXIT_CRITICAL();
  WriteFlash(0,0);
}


/*
void save_time_unix()
{
  
    uint16_t mask = (1 << 2);  
    REGISTERS[4] &= ~mask;
  
  
  
  
    // Считываем 64-битное Unix время из массива регистров.
    // Предполагается, что каждый регистр (REGISTERS[5]...REGISTERS[8]) содержит по 16 бит.
    uint64_t timestamp = ((uint64_t)REGISTERS[5] << 48) |
                         ((uint64_t)REGISTERS[6] << 32) |
                         ((uint64_t)REGISTERS[7] << 16) |
                         ((uint64_t)REGISTERS[8]);

    // Извлекаем компоненты времени (секунды, минуты, часы)
    uint32_t sec    = timestamp % 60;
    uint32_t minute = (timestamp / 60) % 60;
    uint32_t hour   = (timestamp / 3600) % 24;
    // Количество дней, прошедших с 1 января 1970 (эпоха Unix)
    uint32_t days = timestamp / 86400;

    // Вычисляем год, месяц и день
    int year = 1970;
    while (1) {
        // Определяем количество дней в текущем году с учётом високосного года
        int daysInYear = ((year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 366 : 365);
        if (days >= daysInYear) {
            days -= daysInYear;
            year++;
        } else {
            break;
        }
    }
    int month = 1;
    int daysInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    // Если год високосный, февраль имеет 29 дней
    if (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) {
        daysInMonth[1] = 29;
    }
    while (days >= daysInMonth[month - 1]) {
        days -= daysInMonth[month - 1];
        month++;
    }
    int day = days + 1;

    // Вычисляем день недели.
    // 1 января 1970 был четвергом. При условии, что в RTC дни недели нумеруются от 1 (понедельник) до 7 (воскресенье),
    // можно использовать формулу: weekday = ((daysSinceEpoch + 3) % 7) + 1.
    //uint32_t weekday = ((timestamp / 86400 + 3) % 7) + 1;

    // Настройка времени RTC
    RTC_TimeTypeDef sTime = {0};
    sTime.Hours         = hour;
    sTime.Minutes       = minute;
    sTime.Seconds       = sec;
    sTime.TimeFormat    = RTC_HOURFORMAT12_AM;  // Если нужен 24-часовой формат, настройте соответствующим образом
    sTime.DayLightSaving= RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation= RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        Error_Handler();
    }

    // Настройка даты RTC
    RTC_DateTypeDef sDate = {0};
    sDate.Date    = day;
    sDate.Month   = month;
    sDate.Year    = year % 100;  // HAL принимает две последние цифры года
    //sDate.WeekDay = weekday;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        Error_Handler();
    }
}

*/


/* USER CODE END Application */

