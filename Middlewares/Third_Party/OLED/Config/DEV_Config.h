/******************************************************************************
**************************Hardware interface layer*****************************
* | file      		:	DEV_Config.c
* |	version			:	V1.0
* | date			:	2020-06-17
* | function		:	Provide the hardware underlying interface	
******************************************************************************/
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "main.h"
#include <stdint.h>
#include <stdlib.h>

/**
 * data
**/
#define UBYTE uint8_t
#define UWORD uint16_t
#define UDOUBLE uint32_t

#define USE_SPI_4W 1
#define USE_IIC 0
#define USE_IIC_SOFT 0

#define IIC_CMD 0X00
#define IIC_RAM 0X40

//OLED GPIO
#define OLED_CS_0 HAL_GPIO_WritePin(GPIOE, SPI_NCC_Pin, GPIO_PIN_RESET)
#define OLED_CS_1 HAL_GPIO_WritePin(GPIOE, SPI_NCC_Pin, GPIO_PIN_SET)

#define OLED_DC_0 HAL_GPIO_WritePin(GPIOE, LED_DISPLAY_Pin, GPIO_PIN_RESET)
#define OLED_DC_1 HAL_GPIO_WritePin(GPIOE, LED_DISPLAY_Pin, GPIO_PIN_SET)

#define OLED_RST_0 HAL_GPIO_WritePin(GPIOE, OLED_RST_Pin, GPIO_PIN_RESET)
#define OLED_RST_1 HAL_GPIO_WritePin(GPIOE, OLED_RST_Pin, GPIO_PIN_SET)

/*------------------------------------------------------------------------------------------------------*/

UBYTE System_Init(void);
void System_Exit(void);

UBYTE SPI4W_Write_Byte(UBYTE value);
void I2C_Write_Byte(UBYTE value, UBYTE Cmd);

void write_data(uint8_t *data, uint16_t length);

void Driver_Delay_ms(uint32_t xms);
void Driver_Delay_us(uint32_t xus);

#endif
