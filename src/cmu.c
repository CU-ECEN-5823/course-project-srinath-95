/*
 * cmu.c
 *
 *  Created on: Jan 29, 2019
 *      Author: srinath
 */

#include "cmu.h"

extern const int lowest_sleep_mode;

void cmu_init(void)
{
	// Initialize the CMU block for Low-Frequency EM1 and EM2 mode
	if(lowest_sleep_mode <= 2)
	{
		CMU_OscillatorEnable(cmuOsc_LFXO,true,false);
		CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
		CMU_ClockEnable(cmuClock_LFA,true);
		CMU_ClockEnable(cmuClock_LETIMER0,true);
	}
	// Initialize the CMU block for ultra-low frequency EM3 mode
	else if(lowest_sleep_mode > 2) //
	{
		CMU_OscillatorEnable(cmuOsc_ULFRCO,true,false);
		CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);
		CMU_ClockEnable(cmuClock_LFA,true);
		CMU_ClockEnable(cmuClock_LETIMER0,true);
	}

}
