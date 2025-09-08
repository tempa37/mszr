#include "measurement.h"
#include "main.h"
#include "math.h"
#include "cmsis_os.h"

extern void write_to_log(uint8_t code, uint8_t log_data[], uint16_t copy_len);

extern osMutexId_t RelayMutexHandle;
extern osThreadId_t HighPriorityTaskHandle;

extern volatile uint16_t relay_timeout;
extern volatile uint8_t last_position;
extern volatile uint8_t protection_pause;
extern volatile uint16_t REGISTERS[];
extern uint8_t reley_auto_protection;
extern volatile uint8_t theme;
extern uint16_t start;
extern volatile uint8_t test_leak;
extern volatile uint8_t mode; 

extern float C_phase_A;
extern float R_leak_A;
extern float C_phase_B;
extern float R_leak_B;
extern float C_phase_C;
extern float R_leak_C;

extern uint8_t TARGET_VALUE;
extern uint8_t TARGET_VALUE_DEF;

extern uint8_t WARNING_VALUE;
extern uint8_t WARNING_VALUE_DEF;

static uint16_t adc_get_rms(uint16_t *arr, uint16_t length);
static float calculate_rms(uint16_t rms, float C_phase, float R_leak);
static void vRelayReleaseCallback(void *argument);

static osTimerId_t xRelayReleaseTimer = NULL;
volatile uint8_t adc_full_buf = 0;
volatile uint8_t adc_half_buf = 0;

#if TEST_TIME == 1
int32_t code_time[6] = {0};
#endif

uint16_t adcBuffer[ADC_BUF] = {0};
uint8_t adc_ready = 0;     //Говорит о том, что данные с ADC готовы

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    adc_half_buf = 1;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(HighPriorityTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    adc_full_buf = 1;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(HighPriorityTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }  
}

static uint16_t adc_get_rms(uint16_t *arr, uint16_t length) {
  float sum_sq = 0.0f;

  uint16_t cnt = (length < ADC_BUF) ? length : ADC_BUF;

  for (uint16_t i = 0; i < cnt; i++) {
    sum_sq += (float)arr[i] * (float)arr[i];
  }

  return (uint16_t)sqrtf(sum_sq / (float)cnt);
}

static float calculate_rms(uint16_t rms, float C_phase, float R_leak) {
  float I_s = rms * 0.0000466f;
  
  float XC_phase_A = 1.0f / (OMEGA * (C_phase * 1e-6f));
  
  float tmp = R_leak * 1e6f;
  
  float R_eq_phase = (tmp * XC_phase_A) / (tmp + XC_phase_A);
  
  float up_formula = 2* I_s * COS30 * MULT_UP;
  
  float down_formula = MULT_DOWN * R_eq_phase * SQ3;
  
  return (up_formula / down_formula) * 1000.0f;
}

static void vRelayReleaseCallback(void *argument) {
  // 1) синхронизируем программное состояние
  last_position = 0;
  
  osMutexWait(RelayMutexHandle, osWaitForever);
  REGISTERS[2] = 1;
  osMutexRelease(RelayMutexHandle);

  // 3) поднимаем реле
#if OUT == 0
  HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_SET);
#endif
  
  osDelay(50);
  // 2) снимаем паузу – следующие решения можно принимать
  protection_pause = 0;
}

void HighPriorityTask(void *argument) {
  xRelayReleaseTimer = osTimerNew(vRelayReleaseCallback, osTimerOnce, (void *)0, NULL);
 
  uint16_t rms = 0;

#if ALGORITHM_COS == 1 
  float leak_phase_A_macros = 0;
  float leak_phase_B_macros = 0;
  float leak_phase_C_macros = 0;
  uint16_t max_leak_val = 0;
#endif  
  
  uint32_t notification = 0;
  
  uint8_t log_value = 0;
  
  while (1) {   //--------------------------------------------------------------------------------------------
    
    notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    if (notification > 0) {
      adc_ready = 1;

#if DEBUG == 1
      WM_ADC = uxTaskGetStackHighWaterMark(NULL);
#endif
      
      if (adc_half_buf) {
#if TEST_TIME == 1
        code_time[0] = HAL_GetTick();
#endif
        adc_half_buf = 0;
        adc_full_buf = 0;
        rms = adc_get_rms(adcBuffer, ADC_HALF_BUF);
        
      } else if (adc_full_buf) {
#if TEST_TIME == 1
        code_time[1] = HAL_GetTick();
#endif
        adc_half_buf = 0;
        adc_full_buf = 0;
        rms = adc_get_rms((uint16_t *)(adcBuffer + ADC_HALF_BUF), ADC_HALF_BUF);
      }
#if TEST_TIME == 1
      code_time[2] = code_time[1] - code_time[0];
#endif

#if TEST_TIME == 1
      code_time[3] = HAL_GetTick();
#endif
      
#if ALGORITHM_COS == 1
      leak_phase_A_macros = calculate_rms(rms, C_phase_A, R_leak_A);
      leak_phase_B_macros = calculate_rms(rms, C_phase_B, R_leak_B);
      leak_phase_C_macros = calculate_rms(rms, C_phase_C, R_leak_C);
      
      max_leak_val = (uint16_t) fmaxf(fmaxf(leak_phase_A_macros, leak_phase_B_macros), leak_phase_C_macros);
      
      REGISTERS[1] = max_leak_val;
#else
      
#define A3_Q20   ( 10)         //  0.00001 * 2^20
#define A2_Q20   (-2307)       // -0.00220 * 2^20
#define A1_Q20   (488209)      //  0.46580 * 2^20
#define A0_Q20   (4687953)     //  4.47160 * 2^20
#define Q        20            // ?????????? ??????? ???
      
      int32_t x = (int32_t)rms - 199;
      x = (x < 0) ? 0 : x;    
      
      int64_t acc = A3_Q20;             
      acc = acc * x;                     
      acc += A2_Q20;                    
      acc = acc * x;                  
      acc += A1_Q20;                    
      acc = acc * x;                     
      acc += A0_Q20;                     
      
      uint32_t y = (uint32_t)(acc >> Q);   
      osMutexWait(RelayMutexHandle, osWaitForever);
      REGISTERS[1] = (y > 65535U) ? 65535U : (uint16_t)y;
      osMutexRelease(RelayMutexHandle);
#endif

      if ((!mode) && (protection_pause == 0)) {
        if ((REGISTERS[1] >= TARGET_VALUE) && reley_auto_protection) {
          osMutexWait(RelayMutexHandle, osWaitForever);
          REGISTERS[2] = 0;
          last_position = 1;
          osMutexRelease(RelayMutexHandle);

          if (relay_timeout != 0) {
            protection_pause = 1;
            osTimerStart(xRelayReleaseTimer, relay_timeout);
          }
        } else if ((REGISTERS[1] < TARGET_VALUE) && reley_auto_protection) {
          osMutexWait(RelayMutexHandle, osWaitForever);
          REGISTERS[2] = 1;
          osMutexRelease(RelayMutexHandle);
        }
        
        if ((REGISTERS[2] == 0) && (last_position != REGISTERS[2])) {
          
#if OUT == 0
          HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_SET);
#else
          HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_RESET);
#endif
          theme = 2;
          
          if (test_leak == 1) {
            test_leak = 0;
            HAL_GPIO_WritePin(Checking_for_leaks_GPIO_Port, Checking_for_leaks_Pin, GPIO_PIN_RESET);
          }
          
          if (!start) {
            taskENTER_CRITICAL();
            log_value = (uint8_t)REGISTERS[1];
            write_to_log(0x33, &log_value, 1);
            log_value = 0x00;
            write_to_log(0x05, &log_value, 1);
            taskEXIT_CRITICAL();
          }
          last_position = REGISTERS[2];
        } else if ((REGISTERS[2] == 1) && (last_position != REGISTERS[2])) {
          
#if OUT == 0
          HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_RESET);
#else
          HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_SET);
#endif

          last_position = REGISTERS[2];
          log_value = 0x01;
          write_to_log(0x05, &log_value, 1);
          theme = 1;
        }    
      }
      
#if TEST_TIME == 1
      code_time[4] = HAL_GetTick();
      code_time[5] = code_time[4] - code_time[3];
#endif
    }
  }
}