/*
 * modbus.h
 *
 *  Created on: Mar 28, 2025
 *      Author: Nhan
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

#define  HOLDING_REGISTERS_READ_FUCTION_CODE 3           // Read up to 125 contiguous memory words
#define  MODBUS_ID	1
#define  MODBUS_GPIO_WRITE_PORT GPIOD
#define  MODBUS_WRITE_PORT GPIO_PIN_4
#include "stm32h7xx_hal.h"
#include "string.h"

void modbus_init();
void modbus_send_data(uint8_t *data,uint8_t num);
void modbus_request_data(uint8_t slave_id,uint8_t addr,uint8_t number_of_registers);

#endif /* INC_MODBUS_H_ */
