/*
 * modbus.c
 *
 *  Created on: Mar 28, 2025
 *      Author: Nhan
 */
#include "modbus.h"
#include "modbus_crc.h"
extern UART_HandleTypeDef huart2;
uint8_t modbus_send[20];
uint8_t modbus_receive[20];
uint16_t final_data[10];
uint8_t is_requesting = 0;
uint8_t modbus_database[50];
static void resetArray(uint8_t *arr, size_t size)
{
  memset(arr, 0, size);
}
void modbus_send_data(uint8_t *data,uint8_t num)
{
	HAL_GPIO_WritePin(MODBUS_GPIO_WRITE_PORT,MODBUS_WRITE_PORT,GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2,data,num,100);
	HAL_GPIO_WritePin(MODBUS_GPIO_WRITE_PORT,MODBUS_WRITE_PORT,GPIO_PIN_RESET);
}
void modbus_respone_data()
{
	uint8_t slave_id_ = 1;
	if(slave_id_ == modbus_receive[0])
	{
		uint16_t start_addr = (modbus_receive[2] << 8) | modbus_receive[3]; // start address
    uint16_t num_registers = (modbus_receive[4] << 8) | modbus_receive[5]; // number of registers
		modbus_send[0] = MODBUS_ID;  // slave ID
	  modbus_send[1] = HOLDING_REGISTERS_READ_FUCTION_CODE;  // function code
	  modbus_send[2] = num_registers*2;  // Byte count
		int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

		for (int i=0; i<num_registers; i++)   // Load the actual data into TxData buffer
		{
			modbus_send[indx++] = (modbus_database[start_addr]>>8)&0xFF;  // extract the higher byte
			modbus_send[indx++] = (modbus_database[start_addr])&0xFF;   // extract the lower byte
			start_addr++;  // increment the register address
		}
	uint16_t crc = crc16(modbus_send, num_registers);
	modbus_send[num_registers] = crc&0xFF;   // CRC LOW
	modbus_send[num_registers+1] = (crc>>8)&0xFF;  // CRC HIGH
	num_registers = num_registers+2;
	modbus_send_data(modbus_send,num_registers);  // send data... CRC will be calculated in the function itself
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
		if (huart->Instance == USART1)
    {
			  if (is_requesting)
        {
           uint8_t numRegisters = modbus_receive[2] / 2;
           for (uint8_t i = 0; i < numRegisters; i++)
           {
               final_data[i] = (modbus_receive[3 + (i * 2)] << 8) | modbus_receive[4 + (i * 2)];
           }
           is_requesting = 0;
        }
        else
        {
           modbus_respone_data();
        }
    }
}

void modbus_request_data(uint8_t slave_id,uint8_t addr,uint8_t number_of_registers)
{
	//uint8_t offset,
resetArray(modbus_receive,20);
resetArray(modbus_send,10);
modbus_send[0]	= slave_id;
modbus_send[1] = 3;
modbus_send[2] = 0;
modbus_send[3] = addr; //+ offset; offset 0x01   actual = fake + offset
modbus_send[4] = 0;
modbus_send[5] = number_of_registers;
uint16_t crc = crc16(modbus_send,6);
modbus_send[6] = crc&0xFF;   // CRC LOW
modbus_send[7] = (crc>>8)&0xFF;  // CRC HIGH
is_requesting = 1;
modbus_send_data(modbus_send,8);
}

void modbus_init()
{
	HAL_UARTEx_ReceiveToIdle_IT(&huart2,modbus_receive,20);
}



