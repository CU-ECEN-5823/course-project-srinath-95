/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>


#define	LED0_port gpioPortF
#define LED0_pin	4
#define LED1_port gpioPortF
#define LED1_pin 5
#define PB0_port gpioPortF
#define PB0_pin 6

int pending_irq;	// gets the interrupt flag register to know the status pending interrupts
int PB0_pressed;	//To get the status of PB0 button


void gpioInit()
{
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

	GPIO_PinModeSet(PB0_port,PB0_pin, gpioModeInput, false);
		NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);

		NVIC_EnableIRQ(GPIO_EVEN_IRQn);
		GPIO_IntConfig(PB0_port,PB0_pin,true,true,true);
}

void gpioSetDisplayExtcomin(bool high)
{
	if(high)
	{
		GPIO_PinOutSet(gpioPortD,13);
	}
	else
	{
		GPIO_PinOutClear(gpioPortD,13);
	}
}

void GPIO_EVEN_IRQHandler()
{
	CORE_AtomicDisableIrq();
	printf("\n Entered the GPIO interrupt handler");
	//CORE_ATOMIC_IRQ_DISABLE();
	pending_irq = GPIO_IntGet();	// gets the interrupt flag register to know the status pending interrupts
	PB0_pressed = GPIO_PinInGet(PB0_port,PB0_pin);	// Getting the status of PB0 button
	printf("\n button state in interrupt; %d", PB0_pressed);
	INTERRUPT_BUTTON = true;
	 gecko_external_signal(INTERRUPT_BUTTON);	// Call the external signal(State Machine) to notify the button press event
	 //LOG_INFO("The pressed value is: %d",PB0_pressed );

	 GPIO_IntClear(pending_irq);	// Clearing the GPIO interrupts
	 //CORE_ATOMIC_IRQ_ENABLE();
	 CORE_AtomicEnableIrq();

}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}
