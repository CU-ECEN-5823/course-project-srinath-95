/*
 * timer.h
 *
 *  Created on: Apr 3, 2019
 *      Author: srina
 */

#ifndef SRC_TIMER_H_
#define SRC_TIMER_H_

/*******************************
 * Include Files
 *******************************/
#include "em_letimer.h"
//#include "sleep_timer.h"
#include "cmu.h"
#include <math.h>
#include "em_core.h"
#include "gpio.h"
//#include "scheduler.h"
#include "display.h"

/*******************************
 * MACROS
 *******************************/
#define CLOCK_PERIOD (1)	// Setting the Clock Period
#define LED_ON (0.175)	// Setting the Led on time


/*************************************
 * GLOBAL Variables
 *************************************/
uint16_t rollover_count;
uint16_t div_factor;
int OSC_FREQ;
int reqfreq;
bool INTERRUPT_COMP0;
extern const int lowest_sleep_mode;

#endif /* SRC_TIMER_H_ */
