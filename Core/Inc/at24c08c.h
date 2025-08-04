#ifndef AT24C08C_H_
#define AT24C08C_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

//8kbit 1024 word => 10 bits for word addr
//      8 bit each word



// Device address depends on A2 pin:
// - A2 connected to GND: 0xA0 (0b10100000)
// - A2 connected to VCC: 0xA2 (0b10100010)
// A1 A0 for page addr
// R/W for read or write

//[P1][P0] = 00: Addresses 0–255.
//[P1][P0] = 01: Addresses 256–511.
//[P1][P0] = 10: Addresses 512–767.
//[P1][P0] = 11: Addresses 768–1023.

//page 0 to 15: [P1][P0] = 00.
//page 16 to 31: [P1][P0] = 01.
//page 32 to 47: [P1][P0] = 10.
//page 48 to 63: [P1][P0] = 11.


#define DEVICE_ADDR 0b10100000
#define PAGE_NUM 64 //num of page
#define PAGE_SIZE 16 //each page have 16 bytes



/* write byte
page is which page wanna write (from 0 to PAGE_NUM-1)
offset is which byte in page (from 0 to PAGE_SIZE-1)
data is data wanna write
size is size of data
*/
void EEPROM_WriteByte(uint16_t page,uint16_t offset, uint8_t *data,uint16_t size);




uint8_t EEPROM_ReadByte(uint16_t addr,uint16_t offset);

#endif
