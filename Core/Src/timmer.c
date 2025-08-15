#include "timmer.h"
#include "at24c08c.h"
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern uint8_t page_of_eeprom_to_write_isense;
extern double isense;
extern uint8_t page_write_test_eeprom;
extern uint8_t data_write_test_eeprom;
// Use volatile for variables shared between ISR and main
volatile int check = 0;
volatile int a = 0;
static uint8_t eep_buf[2];
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        if(check < a)
        {
//
        	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
        	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
        	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
        }
        else if (check < 20)  // Changed condition
        {

        	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
        	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
        	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
        }
        else
        {
            check = 0;
        }

        check++;
    }
    else if(htim->Instance == TIM2)
    {
        double is = isense;
        int16_t q = (int16_t)(is * 100.0);      // 12.34 A -> 1234

        // Pack little-endian
        eep_buf[0] = (uint8_t)(q & 0xFF);
        eep_buf[1] = (uint8_t)((q >> 8) & 0xFF);
    	EEPROM_WriteByte(page_of_eeprom_to_write_isense, 0, eep_buf, 2);
    	if(page_of_eeprom_to_write_isense == 64)
    	{
    		page_of_eeprom_to_write_isense = 0;
    	}
    	page_of_eeprom_to_write_isense++;
//    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
//    	EEPROM_WriteByte(page_write_test_eeprom, 0, &data_write_test_eeprom, 1);
//    	page_write_test_eeprom++;
//    	data_write_test_eeprom++;
    }
}

void mosfet(float duty)
{
    a = (int)(duty * 20);
    HAL_TIM_Base_Start_IT(&htim4);
}

void mosfet_stop()
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_TIM_Base_Stop_IT(&htim4);
    check = 0;
}
// Hàm delay 100ns sử dụng TIM1
// Hàm delay 100ns sử dụng TIM1
void delay_ns(uint32_t ns) {
    uint32_t ticks = (uint32_t)(ns / 27.77f);
    HAL_TIM_Base_Start(&htim1);
    uint32_t start = TIM1->CNT;
    while ((uint32_t)(TIM1->CNT - start) < ticks);
}
