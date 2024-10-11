//
// Created by gl on 24.05.24.
//

#include "gpio.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

void gpio_setup()
{
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_set_mode(LED1_GPIO_Port,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,LED1_Pin);
    gpio_set_mode(CAN_Drv_Port,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_OPENDRAIN,CAN_Drv_Pin);
}

void CAN_Drv_EN()
{
    gpio_clear(CAN_Drv_Port,CAN_Drv_Pin);
}

void CAN_Drv_Slp()
{
    gpio_set(CAN_Drv_Port,CAN_Drv_Pin);
}

void LED1_ON()
{
    gpio_set(LED1_GPIO_Port,LED1_Pin);
}

void LED1_OFF()
{
    gpio_clear(LED1_GPIO_Port,LED1_Pin);
}

void LED1_TOGGLE()
{
    gpio_toggle(LED1_GPIO_Port,LED1_Pin);
}
