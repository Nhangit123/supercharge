#ifndef AT24C08C_H_
#define AT24C08C_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

// at24c08c.h
// Địa chỉ 7-bit gốc của 24Cxx là 0x50 (0b1010 000)
#define AT24C08_BASE_ADDR_7B  0x50
#define AT24C08_A2            0   // =1 nếu chân A2 nối VCC, =0 nếu A2 nối GND

#define PAGE_NUM  64
#define PAGE_SIZE 16

void EEPROM_WriteByte(uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
uint8_t EEPROM_ReadByte(uint16_t page, uint16_t offset);


#endif
