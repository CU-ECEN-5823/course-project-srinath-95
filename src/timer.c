/*
 * timer.c
 *
 *  Created on: Apr 3, 2019
 *      Author: srina
 */

#include "timer.h"



// Initializing the LETIMER block
void timers_init(void)
{
	// Setting up of the desired frequency based on sleep mode

	if (lowest_sleep_mode <= 2)
		{
			OSC_FREQ = 32768;
		}

		else if(lowest_sleep_mode > 2)
		{
			OSC_FREQ = 1000;
		}
	const LETIMER_Init_TypeDef letimerInit =
	{
		  .enable         = false,                   /* Start counting when init completed. */
		  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
		  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
		  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
		  .out0Pol        = 0,                      /* Idle value for output 0. */
		  .out1Pol        = 0,                      /* Idle value for output 1. */
		  .ufoa0          = letimerUFOANone,         /* PWM output on output 0 */
		  .ufoa1          = letimerUFOANone,       	/* PWM output on output 1*/
		  .repMode        = letimerRepeatFree       /* Count until stopped */
	};

	// INitialize LETIMER0
	LETIMER_Init(LETIMER0, &letimerInit);

	while (LETIMER0->SYNCBUSY != 0);

	// Enabling the LETIMER0 interupt anf COMP0
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0);

	// Enabling NVIC interupt
	NVIC_EnableIRQ(LETIMER0_IRQn);

	// calling the prescalar function
	prescalar();

	// Clearing the interupt
	LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);

	// Enabling LETIMER0
	LETIMER_Enable(LETIMER0,true);

}


void LETIMER0_IRQHandler()
{
	uint32_t count_flag;

	CORE_ATOMIC_IRQ_DISABLE();
	count_flag = LETIMER_IntGet(LETIMER0);

	if(count_flag & LETIMER_IF_COMP0)
	{
		INTERRUPT_COMP0 = 1;
		rollover_count+= 1;
		LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0);
	}
	CORE_ATOMIC_IRQ_ENABLE();
	gecko_external_signal(INTERRUPT_COMP0);
}

void prescalar(void)
{
	div_factor = 1;
	if (CLOCK_PERIOD>1.99)

	{
		// calculating the prescalar value with respect to the desired frequency
		int maxfreq = CLOCK_PERIOD*OSC_FREQ;
		int raise = maxfreq/65535;
		div_factor = pow(2,raise);
		CMU_ClockDivSet(cmuClock_LETIMER0,div_factor);
		int newfreq= OSC_FREQ/div_factor;
		reqfreq = CLOCK_PERIOD * newfreq;

		// Setting LETIMER0 comp0 register
		LETIMER_CompareSet(LETIMER0,0,reqfreq);
		while (LETIMER0->SYNCBUSY!=0);

	}
	else
	{
		// calculating the prescalar value with respect to the desired frequency

		reqfreq = CLOCK_PERIOD * OSC_FREQ;

		// Setting LETIMER0 comp0 register
		LETIMER_CompareSet(LETIMER0,0,reqfreq);
		while (LETIMER0->SYNCBUSY!=0);
	}

}

void timerWait(uint32_t ms_wait)
{
	CORE_ATOMIC_IRQ_DISABLE();
	uint32_t begin;
	uint32_t value;
	begin = LETIMER_CounterGet(LETIMER0);

	uint32_t ticks_to_wait = (((reqfreq/CLOCK_PERIOD) * ms_wait) / 1000 );		// calculating ms ticks

	if(begin >= ticks_to_wait)
		value = begin - ticks_to_wait;
	else
		value = reqfreq - (ticks_to_wait - begin);

	LETIMER_CompareSet(LETIMER0,1,value);
	LETIMER_IntClear(LETIMER0,LETIMER_IF_COMP1);
	LETIMER_IntEnable(LETIMER0,(LETIMER_IEN_COMP1));
	CORE_ATOMIC_IRQ_ENABLE();

}
