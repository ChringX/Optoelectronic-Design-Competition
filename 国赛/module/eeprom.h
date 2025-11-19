#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "main.h"

#define Address_Write 0xA0
#define Address_Read  0xA1

uint8_t Send_Byte(uint16_t address,uint8_t pBuffer);
uint8_t Read_Byte(uint16_t address,uint8_t *data);
uint8_t EEPROM_check_device(void);
uint8_t EEPROM_read_bytes(uint16_t address,uint8_t *data,uint16_t size);
uint8_t EEPROM_write_bytes(uint16_t address,uint8_t *data,uint16_t size);
uint8_t EEPROM_wait(void);
#endif
