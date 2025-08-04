/*
 * adc.c
 *
 *  Created on: May 18, 2025
 *      Author: Nhan
 */
#include "adc.h"
extern ADC_HandleTypeDef hadc1;
extern uint32_t adc_value[4];
extern double isense,voutsense,vinsense,vcbsense;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//	uint16_t isense = adc_value[0];
//	uint16_t voutsense = adc_value[1];
//	uint16_t vinsense = adc_value[2];
	double isense = (((double)adc_value[0] / 65536.0f) * 3.3f - 2.5f) * 10.0f;
	double voutsense = ((double)adc_value[0] / 65536.0f) * 3.3f  * 10.0f;
	double vinsense = ((double)adc_value[0] / 65536.0f) * 3.3f  * 10.0f;

	send_data_to_winform(isense, voutsense, vinsense, adc_value[4]);
}

void adc_get_value()
{
	HAL_ADC_Start_DMA(&hadc1, adc_value, 4);
}




//  uint16_t adc_get_i()
//  {
//	  HAL_ADC_Start(&hadc1);
//	  HAL_ADC_PollForConversion(&hadc1, 10);
//	  int value = HAL_ADC_GetValue(&hadc1);
//	  return value;
//  }
//
//  uint16_t adc_get_vout()
//  {
//	  HAL_ADC_Start(&hadc1);
//	  HAL_ADC_PollForConversion(&hadc1, 10);
//	  int value = HAL_ADC_GetValue(&hadc1);
//	  return value;
//  }
//
//
//  uint16_t adc_get_vin()
//  {
//	  HAL_ADC_Start(&hadc1);
//	  HAL_ADC_PollForConversion(&hadc1, 10);
//	  int value = HAL_ADC_GetValue(&hadc1);
//	  return value;
//  }



