#ifndef MODBUS_H

#define MODBUS_H

#include <stdint.h>

	typedef enum {
              TCP,
              RTU
	} Protocol;
	
        extern  uint16_t REGISTERS[4];
	
	#define SELF_ADDR 1

	//---------------------------------0x03--REGISTERS-----------------------------

	#define READ_REGISTERS 					REGISTERS   //массив из регистров для чтения
	#define LEN_0X03_REGISTERS      		        4			//количество доступных для чтения регистров

	//---------------------------------0x06--REGISTERS-----------------------------

	#define WRITE_REGISTERS 				REGISTERS  //массив регистров для записи
	#define LEN_0X06_REGISTERS				1		   //количество регистров для записи
	#define WRITE_REGISTERS_ADDRESS			        2		   //адрес регистра 1

	//---------------------------------END--REGISTERS------------------------------


	void modbus(uint8_t *data, uint16_t len, uint8_t *data_out, uint16_t *len_out, Protocol protocol);
	void readREG (Protocol protocol, uint8_t *data, uint16_t len, uint8_t *data_out, uint16_t *len_out);
	void writeREG(Protocol protocol, uint8_t *data, uint16_t len, uint8_t *data_out, uint16_t *len_out);
	void modbus_response_err(uint8_t protocol, uint8_t *data, uint8_t *data_out, uint16_t *len_out, uint8_t err_code);
	uint16_t mbcrc(unsigned char* data, int32_t len);

#endif