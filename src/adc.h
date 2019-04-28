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

/* function name: setupSensor
 * description: This function is used to setup the AD8232 Sensor
 * arguments: void
 * return type: void
 */
void setupSensor(void);

/* function name: adc_reading
 * description: To read the ADC value from the sensor AD8232
 * arguments: void
 * return type: void
 */
void adc_reading();

/* Flag to check if Transmission is complete*/
bool TX_done_flag;

extern uint32_t adc_data;

#endif /* SRC_ADC_H_ */
