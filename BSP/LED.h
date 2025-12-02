#ifndef LED_H
#define LED_H

#include "main.h"

void LED_Task_Init(void);
void LED_Task_Run(void);

void LED_Init(void);
void LED_control(uint8_t led, uint8_t state);

#endif // LED_H
