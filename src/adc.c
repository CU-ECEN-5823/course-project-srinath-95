/*
 * adc.c
 *
 *  Created on: Apr 18, 2019
 *      Author: srina
 */
#include "adc.h"

void setupSensor(void)
{
	CMU_ClockEnable(cmuClock_ADC0, true);
  /* Base the ADC configuration on the default setup. */
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;

  init.prescale = ADC_PrescaleCalc(16000000, 0); // Init to max ADC clock for Series 1

  sInit.diff       = false;        // single ended
  sInit.reference  = adcRef2V5;    // internal 2.5V reference
  sInit.resolution = adcRes12Bit;  // 12-bit resolution
  sInit.acqTime    = adcAcqTime4;  // set acquisition time to meet minimum requirement

  sInit.posSel = adcPosSelAPORT2XCH9;

   ADC_Init(ADC0, &init);
   ADC_InitSingle(ADC0, &sInit);
}

void adc_reading()
{
	ADC_Start(ADC0, adcStartSingle);
	LOG_INFO("\n Entered Adc_reading");
	while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK))
	{

	}

	LOG_INFO("\n check");
	uint32_t ADC_data = ADC_DataSingleGet(ADC0);
	uint32_t millivolts = (ADC_data * 2500) / 4096;
	LOG_INFO("\n The signal value is: %d",millivolts);

}
