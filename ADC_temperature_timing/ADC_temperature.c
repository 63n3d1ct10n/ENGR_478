#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"

#include "inc/hw_gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
//*****************************************************************************
//
//!
//! In this project we use ADC0, SS1 to measure the data from the on-chip 
//! temperature sensor. The ADC sampling is triggered by software whenever 
//! four samples have been collected. Both the Celsius and the Fahreheit 
//! temperatures are calcuated.
//
//*****************************************************************************

uint32_t ui32ADC0Value; 					// data array to store samples from ADC0 SS1
//volatile uint32_t ui32TempAvg;			// averaged measurement from temp sensor
volatile uint32_t ui32TempValueC;		// Celsius temperature
volatile uint32_t ui32TempValueF;		// Fahrenheit temperature


//*****************************************************************************
//
// GPIO port initialization for logic analyzer monitoring
//
//*****************************************************************************
void
PortFunctionInit(void)
{
    //
    // Enable Peripheral Clocks 
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable pin PA2 for GPIOOutput
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);
		
		//my modification 
		//MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);
    //
    // Enable pin PA4 for GPIOOutput
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);

    //
    // Enable pin PA3 for GPIOOutput
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
}

//ADC0 initializaiton
void ADC0_Init(void)
{
	
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // configure the system clock to be 40MHz
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	//activate the clock of ADC0
		SysCtlDelay(2);	//insert a few cycles after enabling the peripheral to allow the clock to be fully activated.

		ADCSequenceDisable(ADC0_BASE, 3); //disable ADC0 before the configuration is complete
		ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); // will use ADC0, SS1, processor-trigger, priority 0
		//ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_TS); //ADC0 SS1 Step 0, sample from internal temperature sensor
		//ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_TS); //ADC0 SS1 Step 1, sample from internal temperature sensor
	//	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_TS); //ADC0 SS1 Step 2, sample from internal temperature sensor
		//ADC0 SS1 Step 0, sample from internal temperature sensor, completion of this step will set RIS, last sample of the sequence
		ADCSequenceStepConfigure(ADC0_BASE,3,3,ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END); 
	
		IntPrioritySet(INT_ADC0SS3, 0x00);  	 // configure ADC0 SS1 interrupt priority as 0
		IntEnable(INT_ADC0SS3);    				// enable interrupt 31 in NVIC (ADC0 SS1)
		ADCIntEnableEx(ADC0_BASE, ADC_INT_SS3);      // arm interrupt of ADC0 SS1
	
		ADCSequenceEnable(ADC0_BASE, 3); //enable ADC0
}
		
//interrupt handler
void ADC0_Handler(void)
{
	
		//GPIO_PORTA_DATA_R ^= 0x04; //PA2
		//GPIO_PORTA_DATA_R ^= 0x10; //PA4
		ADCIntClear(ADC0_BASE, 3); //Clear interrupt flag of ADC0 SS1
		ADCProcessorTrigger(ADC0_BASE, 3); //Software trigger the next ADC sampling 
		//GPIO_PORTA_DATA_R ^= 0x04; //PA2

	
	
		ADCSequenceDataGet(ADC0_BASE, 3, &ui32ADC0Value); //Load the captured data from FIFO; The FIFO depth is 4 for SS1 

		//ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4; //Average four samples from SS1
		ui32TempValueC = (1475 - ((2475 * ui32ADC0Value)) / 4096)/10; //Calculate the Celsius temperature
		ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5; //Calculate the Fahrenheit temperature
		SysCtlDelay(SysCtlClockGet()/100000);

	
		//GPIO_PORTA_DATA_R ^= 0x04; //PA2
	
}

int main(void)
{
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // configure the system clock to be 40MHz

		PortFunctionInit();	
		GPIO_PORTA_DATA_R &= ~0x04; //PA2
		GPIO_PORTA_DATA_R &= ~0x10; //PA4	
	
		ADC0_Init(); 
		IntMasterEnable();       		// globally enable interrupt
		ADCProcessorTrigger(ADC0_BASE, 3); //Software trigger ADC sampling 
	
		while(1)
		{
			
		}
}
