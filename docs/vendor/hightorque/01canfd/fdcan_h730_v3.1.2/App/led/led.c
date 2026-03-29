#include "led.h"



void led_toggle()
{
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}


void led_toggle_err()
{
    while (1)
    {
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        HAL_Delay(500);
    }
}

