/*
 * winform.c
 *
 *  Created on: Jun 21, 2025
 *      Author: Nhan
 */
#include"winform.h"
#include"adc.h"
extern UART_HandleTypeDef huart7;
void send_data_to_winform(double isense, double voutsense, double vinsense, double vcbsense)
{
    char msg[50];
    snprintf(msg, sizeof(msg), "%.2f\r\n%.2f\r\n%.2f\r\n%.2f\r\n", isense, voutsense, vinsense, vcbsense);
    HAL_UART_Transmit(&huart7, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}


