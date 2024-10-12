//
// Created by gl on 24.05.24.
//

#ifndef GSMCAN_GPIO_H
#define GSMCAN_GPIO_H

#include <stdbool.h>

#define CAN_Drv_Pin (GPIO12 | GPIO13)
#define CAN_Drv_Port GPIOB

#define LED1_Pin GPIO12
#define LED1_GPIO_Port GPIOC

void gpio_setup(void);

void LED1_ON();
void LED1_OFF();
void LED1_TOGGLE();

void CAN_Drv_EN();
void CAN_Drv_Slp();

#endif //GSMCAN_GPIO_H
