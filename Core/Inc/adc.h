/*
 * adc.h
 *
 *  Created on: May 18, 2025
 *      Author: Nhan
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include"stm32h7xx_hal.h"
#include"stdint.h"
#include"stm32h7xx_hal_adc.h"
#include "winform.h"
#define VREF 3.3f
#define ADC_RES 65536.0f
void adc_get_value();

#endif /* INC_ADC_H_ */
