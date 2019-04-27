/*
 * adc.h
 *
 *  Created on: Apr 18, 2019
 *      Author: srina
 */

#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#include "em_gpio.h"
#include "log.h"

#include "em_device.h"
#include "em_adc.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "native_gecko.h"
#include "gpio.h"


volatile uint32_t Signal;

void setupSensor(void);
void adc_reading();

bool TX_done_flag;

#endif /* SRC_ADC_H_ */
