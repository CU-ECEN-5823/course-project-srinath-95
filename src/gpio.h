/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>
#include "log.h"

void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();

bool INTERRUPT_BUTTON0;
bool INTERRUPT_BUTTON1;

//#define	LED0_port gpioPortF
//#define LED0_pin	4
//#define LED1_port gpioPortF
//#define LED1_pin 5
//#define PB0_port gpioPortF
//#define PB0_pin 6
//#define PB1_port gpioPortF
//#define PB1_pin 7

#endif /* SRC_GPIO_H_ */
