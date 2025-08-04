#include "at24c08c.h"

extern I2C_HandleTypeDef hi2c1; // Ensure I2C handle is declared

// Write bytes to EEPROM
void EEPROM_WriteByte(uint16_t page, uint16_t offset, uint8_t *data, uint16_t size) {
    // Validate inputs
    if (page >= PAGE_NUM || offset >= PAGE_SIZE || size == 0) {
        return; // Invalid page, offset, or size
    }

    // Calculate total bytes to write and check if it exceeds page boundary
    if (offset + size > PAGE_SIZE) {
        size = PAGE_SIZE - offset; // Limit to page boundary
    }

    // Calculate the 10-bit memory address: page * PAGE_SIZE + offset
    uint16_t mem_addr = (page * PAGE_SIZE) + offset;

    // Device address: 0b1010[A2][P1][P0][R/W]
    // A2 is 0 (in DEVICE_ADDR), P1/P0 are the upper 2 bits of the 10-bit address
    uint8_t dev_addr = (DEVICE_ADDR & 0xF8) | ((mem_addr >> 8) & 0x06); // Set P1, P0 bits

    // Lower 8 bits of the 10-bit address for the memory address byte
    uint8_t mem_addr_byte = (uint8_t)(mem_addr & 0xFF);

    // Perform the write operation
    HAL_I2C_Mem_Write(
        &hi2c1,           // I2C handle
        dev_addr,         // Device address (7-bit, excluding R/W bit)
        mem_addr_byte,    // Memory address (lower 8 bits)
        1,                // Address size (1 byte for AT24C08C)
        data,             // Data to write
        size,             // Number of bytes to write
        100               // Timeout (ms)
    );

    // Wait for the write cycle to complete (t_WR = 5ms max as per datasheet)
    HAL_Delay(5);
}


// Read a single byte from EEPROM
uint8_t EEPROM_ReadByte(uint16_t page, uint16_t offset) {
    // Validate inputs
    if (page >= PAGE_NUM || offset >= PAGE_SIZE) {
        return 0; // Invalid page or offset, return 0 as an error value
    }

    // Calculate the 10-bit memory address: page * PAGE_SIZE + offset
    uint16_t mem_addr = (page * PAGE_SIZE) + offset;

    // Device address: 0b1010[A2][P1][P0][R/W]
    // A2 is set in DEVICE_ADDR (0 in this case), P1/P0 are the upper 2 bits of the page number
    uint8_t dev_addr = (DEVICE_ADDR & 0xF8) | ((page >> 4) & 0x06); // Update P1, P0 bits

    // Split 10-bit memory address into 2 bytes for I2C (only lower 8 bits sent as address byte)
    uint8_t mem_addr_byte = (uint8_t)(mem_addr & 0xFF); // Lower 8 bits

    // Buffer to store the read data
    uint8_t data = 0;

    // Perform the read operation
    HAL_I2C_Mem_Read(
        &hi2c1,           // I2C handle
        dev_addr,         // Device address (7-bit, shifted left with R/W bit)
        mem_addr_byte,    // Memory address (lower 8 bits)
        1,                // Address size (1 byte for AT24C08C, as upper bits are in dev_addr)
        &data,            // Buffer to store read data
        1,                // Data size (1 byte)
        100               // Timeout (ms)
    );

    return data; // Return the read byte
}
