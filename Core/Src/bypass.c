/*
 * bypass.c
 *
 *  Created on: Aug 3, 2025
 *      Author: Nhan
 */
#include "bypass.h"
void enable_bypass()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
}
void disable_bypass()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
}
