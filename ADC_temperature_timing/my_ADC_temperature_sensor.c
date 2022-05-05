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
#include "driverlib/timer.h"
#include "driverlib/uart.h"
//#include "tm4c123gh6pm.h"
//*****************************************************************************
//
//!
//! In this project we use ADC0, SS3 to measure the data from the on-chip 
//! temperature sensor. The ADC sampling is triggered by software whenever 
//! four samples have been collected. Both the Celsius and the Fahreheit 
//! temperatures are calcuated.
//
//*****************************************************************************
uint32_t ui32ADC0Value[1]; 					// data array to store samples from ADC0 SS3
//volatile uint32_t ui32TempAvg;			// averaged measurement from temp sensor
volatile uint32_t ui32TempValueC;		// Celsius temperature
volatile uint32_t ui32TempValueF;		// Fahrenheit temperature


//*****************************************************************************
//
// GPIO port initialization for logic analyzer monitoring
//
//*****************************************************************************
void PortFunctionInit(void)
{
    //
    // Enable Peripheral Clocks 
    //
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Enable clocks on GPIO PORT E 
		GPIO_PORTE_DIR_R &= ~GPIO_PIN_3;			//set direction of PortE pin 3 as input
		GPIO_PORTE_DATA_R &= ~GPIO_PIN_3;
		GPIO_PORTE_DEN_R  &= ~GPIO_PIN_3;							//disable port PE3 for digital fucntion
		GPIO_PORTE_AFSEL_R |= GPIO_PIN_3;							//Use PE4 for alternate function
		GPIO_PORTE_AMSEL_R |= GPIO_PIN_3;    				  //turn on analog function
		
		//GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_4);			//enable interrupt on pin 4

    //
    // Enable pin PA2 for GPIOOutput
    //
		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);
		
		//my modification 
		//enable pin 3 on PORTE for ADC input
    //MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_3);
	
    //
    // Enable pin PA4 for GPIOOutput
    //
  //MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);

    //
    // Enable pin PA3 for GPIOOutput
    //
	//MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
}
void UART_Init(void){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);			//enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);			//enable GPIOA port pins

    GPIOPinConfigure(GPIO_PA0_U0RX);					//configure pin PA0 for recieving
    //GPIOPinConfigure(GPIO_PA1_U0TX);					//configure pin PA1 for transmitting
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);		

    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
    //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); //enable pin for LED PF2

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));				//allow 8 bits/sec bandwidth, 
		
	  IntPrioritySet(UART0_BASE, 1); //set the priority of the ADC0 to 1
    IntMasterEnable(); //enable processor interrupts
    IntEnable(INT_UART0); //enable the UART interrupt
		UARTIntEnable(UART0_BASE, UART_INT_RX);  //only enable RX interrupts
    //UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
}

void Timer0A_Init(void){
	
		//Enable Peripheral Clocks
	
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0) ;
		TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);				//Configure for 32-bit timer
		TimerLoadSet(TIMER0_BASE, TIMER_A, 16000-1);					  //reload value
		IntPrioritySet(INT_TIMER0A, 0x00);											//configure Timer0A interrupt as priority 00
		IntEnable(INT_TIMER0A);																	//enable interrupt 19 in NVIC (TImer0A)
		TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);				//arm timeout interrupt
		TimerEnable(TIMER0_BASE, TIMER_A); 											//enable timer0A 
}
//ADC0 initializaiton
void ADC0_Init(void)
{		
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // configure the system clock to be 40MHz
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	//configure ADC0 module
		//SysCtlDelay(2);
		ADCSequenceDisable(ADC0_BASE, 3);	//Disable Sample Sequencer by clearing bit3
		//SYSCTL->RCGCADC = (1UL<<0);		//enable clock on ADC0
		//SYSCTL_RCGC0_R |= SYSCTL_RCGC0_ADC0;  //enable clock on ADC0
	
		//SYSCTL->RCGCGPIO |= (1UL<<5);   //enable clock on port E
		//SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;    //enable clock GPIOA port E
		
		//ADC0_ACTSS_R &= ~(ADC_ACTSS_ASEN3);		//Disable Sample Sequencer by clearing bit3
	
		//configure ADC to be triggered by processor
		ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);  //configuring sample sequencer priorities
	
		//select event triggering mode
		//ADC0_EMUX_R |=(ADC_EMUX_EM3_PROCESSOR);		//processor trigger
		
		//ADC0_EMUX_R &= ~0xF000;			//select a software event as a start conversion trigger
		//ADC0_SSMUX3_R = 0;					//select AN0 analog channel for sample sequencer 3 or SS3, get input from channel 0
		//ADC0_PC_R = 0x3;		//sets ADC sampling to 250ksps
	
		//ADC0_PSSI_R |= (1UL<<3);		//set the SS3 bit of ADCPSSI register to start ADC conversion for sample sequencer 3
		//ADC0_SSCTL3_R |= (1<<1)|(1<<2);  //take one sample at a time, set flag at first sample
		//ADC0_SSCTL3_R |= (ADC_CTL_CH0)|)(1<<1)|(1<<2);
		//ADCSequenceStepConfigure(ADC0_BASE,3,3,ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END); 
		ADCSequenceStepConfigure(ADC0_BASE,3,3,ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END); 
		
		//SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // configure the system clock to be 40MHz
		//SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	//activate the clock of ADC0
		//SysCtlDelay(2);	//insert a few cycles after enabling the peripheral to allow the clock to be fully activated.

		//ADCSequenceDisable(ADC0_BASE, 3); //disable ADC0 before the configuration is complete
	//	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); // will use ADC0, SS3, processor-trigger, priority 0
	//	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END); //ADC0 SS3 Step 0, sample from internal temperature sensor
	 
		//ADCSequenceStepConfigure(ADC0_BASE, 3, 1, ADC_CTL_TS); //ADC0 SS1 Step 1, sample from internal temperature sensor
		//ADCSequenceStepConfigure(ADC0_BASE, 3, 2, ADC_CTL_TS); //ADC0 SS1 Step 2, sample from internal temperature sensor
		//ADC0 SS1 Step 0, sample from internal temperature sensor, completion of this step will set RIS, last sample of the sequence
		//ADCSequenceStepConfigure(ADC0_BASE,3,1,ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END); 
	
		IntPrioritySet(INT_ADC0SS3, 0x00);  	 // configure ADC0 SS3 interrupt priority as 0
		IntEnable(INT_ADC0SS3);    				// enable interrupt 31 in NVIC (ADC0 SS3)
		ADCIntEnableEx(ADC0_BASE, ADC_INT_SS3);      // arm interrupt of ADC0 SS3
	
		ADCSequenceEnable(ADC0_BASE, 3); //enable ADC0 after configuration
		//ADC0_ACTSS_R |= (1UL<<3);    //enable ADC0 SS3
		//ADC0_ACTSS_R |= (ADC_ACTSS_ASEN3);    //enable ADC0 SS3
}
//void GPIOPORTE_Int(){
		// IntPrioritySet(GPIO_PORTE_BASE, 0x01); 
		// IntEnable(GPIO_PORTE_BASE);
		
		 	
//}
//void GPIOPORTE_Handler(){
		 //IntEnable(GPIO_PORTE_BASE);
		 //GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);
		 //ADCSequenceDataGet(ADC0_BASE, 3, ui32ADC0Value);	//Load the captured data from FIFO; The FIFO depth is 1 for SS3 

//}
//interrupt handler
void ADC0_Handler(void)
{
	  //IntEnable(GPIO_PORTE_BASE);
		// GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);
	
		//GPIO_PORTA_DATA_R ^= 0x04; //PA2
		//GPIO_PORTA_DATA_R ^= 0x10; //PA4
		ADCIntClear(ADC0_BASE, 3); //Clear interrupt flag of ADC0 SS3
		//ADCProcessorTrigger(ADC0_BASE, 3); //Software trigger the next ADC sampling 
		//GPIO_PORTA_DATA_R ^= 0x04; //PA2
		//ADC0_PSSI_R |= (1<<3);	//Enable SS3 to start sampling from AN0
		//
		//ADC0_SSFIFO3_R
		//ui32ADC0Value = ADC0_SSFIFO3_R;		//read ADC coversion result from SS3 FIFO
	
		ADCSequenceDataGet(ADC0_BASE, 3, ui32ADC0Value); //Load the captured data from FIFO; The FIFO depth is 1 for SS3 

		//ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4; //Average four samples from SS1
		ui32TempValueC = (1475 - ((2475 * ui32ADC0Value[0])) / 4096)/10; //Calculate the Celsius temperature
		ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5; //Calculate the Fahrenheit temperature
		//SysCtlDelay(SysCtlClockGet()/100000);
		
		//TimerLoadSet(TIMER0_BASE, TIMER_A, 80000);		//load delay value in timer
		//TimerEnable(TIMER0_BASE, TIMER_A); 						//enable timer
	
		//GPIO_PORTA_DATA_R ^= 0x04; //PA2
	
}
void Timer0A_Handler(void){
		  //clear timer interrupt sources
			TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);		
			
	    //sample every milli second	at 1000Hz						
			ADCProcessorTrigger(ADC0_BASE, 3);	

}
void UART0IntHandler(){
		uint32_t ui32Status;
		//get interrupt status
    ui32Status = UARTIntStatus(UART0_BASE, true); 
		//clear the asserted interrupts
    UARTIntClear(UART0_BASE, ui32Status); 
		
		//send Celcius temperature value to terminal
		UARTCharPutNonBlocking(UART0_BASE, ui32TempValueC);
		UARTCharPutNonBlocking(UART0_BASE, '\r');
		UARTCharPutNonBlocking(UART0_BASE, '\n');
	  //send Fahrenheit temperature value to terminal
		UARTCharPutNonBlocking(UART0_BASE, ui32TempValueF);	
	 
}		

int main(void)
{		// configure the system clock to be 40MHz
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); 
    //inialize Ports
		PortFunctionInit();
    //inialize UART 	
		UART_Init();		
		
		Timer0A_Init();		//inialize timers
		ADC0_Init();			//inialize ADC0 
		IntMasterEnable();  // globally enable interrupt
		//ADCProcessorTrigger(ADC0_BASE, 3); //Software trigger ADC sampling 
	
		while(1)
		{
			
		}
}
