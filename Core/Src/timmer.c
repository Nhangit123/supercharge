#include "timmer.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;
// Use volatile for variables shared between ISR and main
volatile int check = 0;
volatile int a = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        if(check <= a)
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
        }
        else if (check <= 1000)  // Changed condition
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
        }
        else
        {
            check = 0;
        }

        check++;
    }
}

void mosfet(float duty)
{
    a = (int)(duty * 1000);
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
