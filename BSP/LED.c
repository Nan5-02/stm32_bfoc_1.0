#include "LED.h"


void LED_Task_Init(void)
{
    LED_Init();
}

void LED_Task_Run(void)
{
    while (1)
    {
        // LED_control(1, 1);
        // osDelay(500);
        // LED_control(1, 0);
        // osDelay(500);
    }
}



void LED_Init(void)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
}

void LED_control(uint8_t led, uint8_t state)
{
    if (led == 1)
    {
        if(state == 1)
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        else if(state == 0)
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    }
    else if (led == 2)
    {
        if(state == 1)
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        else if(state == 0)
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    }
}
