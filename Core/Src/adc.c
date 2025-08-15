/*
 * adc.c
 *
 *  Created on: May 18, 2025
 *      Author: Nhan
 */
#include "adc.h"
extern ADC_HandleTypeDef hadc1;
extern uint16_t adc_value[4];
extern double isense,voutsense,vinsense,vcbsense;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    double v_adc0 = ((double)adc_value[0] / ADC_RES) * VREF;
    double v_adc1 = ((double)adc_value[1] / ADC_RES) * VREF;
    double v_adc2 = ((double)adc_value[2] / ADC_RES) * VREF;
    double v_adc3 = ((double)adc_value[3] / ADC_RES) * VREF;

    isense = (((1.38-v_adc0)/0.7+1.38)/3.3*4.3-2.5)*10 ;
    voutsense = v_adc1*10 ;
    vinsense  = v_adc2*10 ;
    vcbsense  = v_adc3*10 ;

//    send_data_to_winform(isense, voutsense, vinsense, vcbsense);
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



