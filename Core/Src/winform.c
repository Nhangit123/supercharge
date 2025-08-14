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
    char msg[128]; // rộng hơn 50 để đủ chỗ cho nhãn + đơn vị + separator
    int len = snprintf(msg, sizeof(msg),
                       "I=%.2f A\r\n"
                       "Vout=%.2f V\r\n"
                       "Vin=%.2f V\r\n"
                       "Vcb=%.2f V\r\n"
                       "----------------------\r\n",
                       isense, voutsense, vinsense, vcbsense);

    if (len < 0) return;                    // lỗi format
    if (len > (int)sizeof(msg)) len = sizeof(msg); // phòng tràn

    HAL_UART_Transmit(&huart7, (uint8_t*)msg, (uint16_t)len, HAL_MAX_DELAY);
}



