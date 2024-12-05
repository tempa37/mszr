#include "modbus.h"
#include "stm32f4xx_hal.h"
#include "main.h"


void modbus(uint8_t *data, uint16_t len, uint8_t *data_out, uint16_t *len_out, Protocol protocol)
{
	uint16_t crc = 0;
	uint16_t rx_crc = 0;
	uint8_t fcode = 0;
        
    
          if(((data[0] == SELF_ADDR) && (protocol == RTU))|| ((protocol == TCP) && data[6] == SELF_ADDR))
            {     

              switch (protocol)
                {
                case TCP:
                  fcode = data[7];
                  break;
                case RTU:
                  crc = mbcrc(data, len-1);
                  rx_crc = (data[len-1] << 8) | data[len];
                  if ((crc == rx_crc) || (protocol == TCP))
                    {
                      fcode = data[1];
                    }
                  break;
                }

              switch(fcode)
                {
                  case 03:							
                    readREG(protocol, data, len, data_out, len_out);
                    break;
                  
                  case 06:
                    writeREG(protocol, data, len, data_out, len_out);
                    break;
                    
                  case 0:  //crc error

                    break;
                    
                  default:
                    modbus_response_err(protocol, data, data_out, len_out, 0x01);  //Illegal Function
                    break;
                }

            }
	}






void readREG (Protocol protocol, uint8_t *data, uint16_t len, uint8_t *data_out, uint16_t *len_out)
{
    uint16_t start_address; 
    uint16_t quantity;       
    uint16_t crc_calculated; 
    uint16_t i;
    uint8_t unit_id;

	if(protocol == RTU)
	{
	
		start_address = (data[2] << 8) | data[3];
		quantity = (data[4] << 8) | data[5];
		
		if (quantity > LEN_0X03_REGISTERS || start_address + quantity > LEN_0X03_REGISTERS) 
		{
			modbus_response_err(protocol,data, data_out, len_out, 0x02);  //Illegal Data Address
			return;
		}
		
		data_out[0] = data[0];  // ID запроса
		data_out[1] = 0x03;     // код функции
		data_out[2] = quantity * 2;  // количество байт с данными
		
		for (i = 0; i < quantity; i++) 
		{
                  if((start_address + i) <= (LEN_0X03_REGISTERS - 1))
                  {
			data_out[3 + i * 2] = (READ_REGISTERS[start_address + i] >> 8) & 0xFF;  //Data
			data_out[4 + i * 2] = READ_REGISTERS[start_address + i] & 0xFF;       
                  }
		}
		
		crc_calculated = mbcrc(data_out, 3 + quantity * 2);
		data_out[3 + quantity * 2] = (crc_calculated >> 8) & 0xFF; 
		data_out[4 + quantity * 2] = crc_calculated & 0xFF;
		*len_out = 5 + quantity * 2; 
	 
	}
	else if(protocol == TCP)
	{
          
		unit_id = data[6]; 

		// Начало PDU (Protocol Data Unit)
		start_address = (data[8] << 8) | data[9];  
		quantity = (data[10] << 8) | data[11];     

		if (quantity > LEN_0X03_REGISTERS || start_address + quantity > LEN_0X03_REGISTERS) 
		{
			modbus_response_err(protocol, data, data_out, len_out, 0x02); //Illegal Data Address
			return;
		}
		
		data_out[0] = data[0];  // Transaction ID 
		data_out[1] = data[1];  // Transaction ID 
		data_out[2] = 0x00;     // Protocol ID 
		data_out[3] = 0x00;     // Protocol ID 
		data_out[4] = 0x00;     // Длина 
		data_out[5] = quantity * 2 + 3;  // Длина 
		data_out[6] = unit_id;  // Unit Identifier
		data_out[7] = 0x03;  
		data_out[8] = quantity * 2;  
		for (i = 0; i < quantity; i++) 
		{
                  if((start_address + i) <= (LEN_0X03_REGISTERS - 1))
                  {
			data_out[9 + i * 2] = (READ_REGISTERS[start_address + i] >> 8) & 0xFF;  //Data
			data_out[10 + i * 2] = READ_REGISTERS[start_address + i] & 0xFF;      
                  }
		}
		*len_out = 9 + quantity * 2;		
	}
}



void writeREG(Protocol protocol, uint8_t *data, uint16_t len, uint8_t *data_out, uint16_t *len_out)
{
	if(protocol == RTU)
	{
		uint16_t address = (data[2] << 8) | data[3];
		
		if (address < WRITE_REGISTERS_ADDRESS || 
			address >= WRITE_REGISTERS_ADDRESS + LEN_0X06_REGISTERS)
		{
			modbus_response_err(protocol, data, data_out, len_out, 0x02); //Illegal Data Address
			return;
		}
		
		uint16_t value = (data[4] << 8) | data[5];
		
		WRITE_REGISTERS[address] = value;
		
		for (int i = 0; i < len; i++) 
		{
			data_out[i] = data[i];
		}
		
		*len_out = len + 2;
	}
	else if(protocol == TCP)
	{
		uint16_t transaction_id = (data[0] << 8) | data[1];
		uint16_t protocol_id = (data[2] << 8) | data[3];
		uint16_t message_len = (data[4] << 8) | data[5];
                
		if (protocol_id != 0x0000 || message_len < 6)
		{
			modbus_response_err(protocol, data, data_out, len_out, 0x02); //Illegal Data Address
			return;
		}

		uint8_t unit_id = data[6]; // Unit ID
		uint16_t register_address = (data[8] << 8) | data[9]; 
		uint16_t register_value = (data[10] << 8) | data[11]; 
		if (register_address < WRITE_REGISTERS_ADDRESS || 
			register_address >= WRITE_REGISTERS_ADDRESS + LEN_0X06_REGISTERS)
		{
			modbus_response_err(protocol, data, data_out, len_out, 0x02); //Illegal Data Address
			return;
		}

		REGISTERS[register_address] = register_value;

		data_out[0] = (transaction_id >> 8) & 0xFF;
		data_out[1] = transaction_id & 0xFF;
		data_out[2] = (protocol_id >> 8) & 0xFF;
		data_out[3] = protocol_id & 0xFF;
		data_out[4] = 0x00;
		data_out[5] = 0x06; 
		data_out[6] = unit_id; 
		data_out[7] = 0x06; 
		data_out[8] = (register_address >> 8) & 0xFF;
		data_out[9] = register_address & 0xFF;
		data_out[10] = (register_value >> 8) & 0xFF;
		data_out[11] = register_value & 0xFF;
                *len_out = 12;
            }
}





void modbus_response_err(uint8_t protocol, uint8_t *data, uint8_t *data_out, uint16_t *len_out, uint8_t err_code) 
{
	if(protocol == RTU)
	{
            uint8_t function_code = err_code;      

            data_out[0] = SELF_ADDR;             
            data_out[1] = function_code | 0x80;  
            data_out[2] = err_code;             

            // Вычисляем CRC для первых трех байтов (адрес, функция, код ошибки)
            uint16_t crc = mbcrc(data_out, 3);
            data_out[3] = (crc >> 8) & 0xFF;    
            data_out[4] = crc & 0xFF;    
            *len_out = 5;
	}
	else if(protocol == TCP)
	{
            uint16_t transaction_id = (data[0] << 8) | data[1];  
            uint16_t protocol_id = (data[2] << 8) | data[3]; 
            //uint16_t length = (data[4] << 8) | data[5];          
            uint8_t unit_id = data[6];                         
            uint8_t function_code = data[7];                   


            data_out[0] = (transaction_id >> 8) & 0xFF;  
            data_out[1] = transaction_id & 0xFF;        
            data_out[2] = (protocol_id >> 8) & 0xFF;    
            data_out[3] = protocol_id & 0xFF;           
            data_out[4] = 0x00;                         
            data_out[5] = 0x03;                        
            data_out[6] = unit_id;                       
            data_out[7] = function_code | 0x80;          
            data_out[8] = err_code;               
            *len_out = 9;
    }
}



static const uint16_t crc16table[] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 
};

uint16_t mbcrc(uint8_t * data, int32_t len)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < len; i++)
    crc = (crc >> 8) ^ crc16table[(crc & 0xFF) ^ data[i]];
    
    unsigned char temp = crc >> 8;
    crc = (crc << 8) | temp; 
    return crc;
}