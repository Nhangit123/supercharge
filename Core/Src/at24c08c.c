#include "at24c08c.h"
extern I2C_HandleTypeDef hi2c1;

// Gom thành hàm nhỏ: tính addr 7-bit sau khi nhét A2, P1, P0
static inline uint8_t eeprom_devaddr7(uint16_t mem_addr_10bit)
{
    uint8_t block = (mem_addr_10bit >> 8) & 0x03;  // A9..A8 → P1,P0
    return (AT24C08_BASE_ADDR_7B | (AT24C08_A2 << 2) | block); // 7-bit
}

void EEPROM_WriteByte(uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
    if (page >= PAGE_NUM || offset >= PAGE_SIZE || size == 0) return;
    if (offset + size > PAGE_SIZE) size = PAGE_SIZE - offset;

    uint16_t mem_addr = (page * PAGE_SIZE) + offset;     // 0..1023 (10-bit)
    uint8_t  dev7     = eeprom_devaddr7(mem_addr);       // 7-bit device address
    uint8_t  mem8     = (uint8_t)(mem_addr & 0xFF);      // địa chỉ ô nhớ thấp 8 bit

    // HAL mong DevAddress là địa chỉ 7-bit << 1
    HAL_I2C_Mem_Write_DMA(&hi2c1, (uint16_t)(dev7 << 1), mem8, I2C_MEMADD_SIZE_8BIT, data, size);

}

uint8_t EEPROM_ReadByte(uint16_t page, uint16_t offset)
{
    if (page >= PAGE_NUM || offset >= PAGE_SIZE) return 0;

    uint16_t mem_addr = (page * PAGE_SIZE) + offset;     // 0..1023
    uint8_t  dev7     = eeprom_devaddr7(mem_addr);
    uint8_t  mem8     = (uint8_t)(mem_addr & 0xFF);
    uint8_t  data     = 0;

    HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(dev7 << 1), mem8, I2C_MEMADD_SIZE_8BIT, &data, 1,100);
    return data;
}
