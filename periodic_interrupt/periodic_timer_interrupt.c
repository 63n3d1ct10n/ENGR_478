#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"


#define RED_MASK GPIO_PIN_1 
#define BLUE_MASK GPIO_PIN_2
#define GREEN_MASK GPIO_PIN_3

volatile unsigned long count = 0;  //initalize count as a global variable

//timer initialization to handle the debounce
void Timer1A_Init(void){
	
		//Enable Peripheral Clocks
	
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1) ;
		TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);				//Configure a oneshot timer
		//TimerLoadSet(TIMER1_BASE, TIMER_A, period2-1);					  //reload value
		IntPrioritySet(INT_TIMER1A, 0x01);											//configure Timer1A interrupt as priority 1
		IntEnable(INT_TIMER1A);																	//enable interrupt 19 in NVIC (TImer1A)
		TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);				//arm timeout interrupt
		//TimerEnable(TIMER1_BASE, TIMER_A); 											//enable timer1A 

}

//timer initialization to handle the 1 second count ups
void Timer0A_Init(unsigned long period){
	
		//Enable Peripheral Clocks
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0) ;
		TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);				//Configure for 32-bit timer
		TimerLoadSet(TIMER0_BASE, TIMER_A, period-1);					  //reload value
		IntPrioritySet(INT_TIMER0A, 0x00);											//configure Timer0A interrupt as priority 00
		IntEnable(INT_TIMER0A);																	//enable interrupt 19 in NVIC (TImer0A)
		TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);				//arm timeout interrupt
		TimerEnable(TIMER0_BASE, TIMER_A); 											//enable timer0A 

}

//port initalization function
void PortFunctionInit(){
	
		volatile uint32_t ui32Loop;          //initialize clock counter variable
	
		SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF; //activate clock on Port F
		
		ui32Loop = SYSCTL_RCGC2_R;						//allow time for clock to start

			//unlock port F
			GPIO_PORTF_LOCK_R = 0x4C4F434B;
			GPIO_PORTF_CR_R |= 0x01;
		
			//set direction for PF1, PF2, and PF3 as output
			GPIO_PORTF_DIR_R |= 0x0E;
			//GPIO_PORTF_DIR_R |= 0x04;
			
			//set the direction of PF0 and PF4 as input by clearing the bits
			GPIO_PORTF_DIR_R &= ~0x11;
			//GPIO_PORTF_DIR_R &= ~0x01;
		
			//enable PF0, PF1, PF2, PF3, and PF4 for digital function
			GPIO_PORTF_DEN_R |= 0x1F;
		
			//enable pull_up on PF0 and PF4
			GPIO_PORTF_PUR_R |= 0x11;
		
}

//interrupt initialization for GPIOF
void Interrupt_Init(){
	NVIC_EN0_R |= 0x40000000;			//Enable interrupt 30 in NVIC (GPIOF)
	//NVIC_PRI7_R &= ~0x00E00000;		//configure GPIOF as priority 0
	IntPrioritySet(INT_GPIOF, 0x02);	//configure GPIOF as priority 2
	GPIO_PORTF_IM_R |= 0x11;		//arm interrupt of PF0 and PF4
	GPIO_PORTF_IS_R &= ~0x11;		//enable edge triggering on PF0 and PF4 by setting IS to 0
	GPIO_PORTF_IBE_R &= ~0x11;	//PF0 andPF4 not both-edge triggered
	GPIO_PORTF_IEV_R &= ~0x11;		//PF0 and PF4 rising edge event triggering
	IntMasterEnable();
}

//function to swtich through LEDs as count changes
void switchLeds(){
	//	if(count>7){							//check for count greater than 7
			count = count%8;				//set bound for count; 0 <= count <=7
//		}
	//	if(count<0){							//check for count less than 0
	//		count = 7;		 					//then reset count back to seven; make a loop
	//	}	
		
		switch(count){												//switch for count between 0 and 7
			case 0:															//when count is 0, all LEDs are turned off
				GPIO_PORTF_DATA_R &= ~(RED_MASK|BLUE_MASK|GREEN_MASK);		
				GPIO_PORTF_DATA_R &= ~GREEN_MASK;
				GPIO_PORTF_DATA_R &= ~BLUE_MASK;
				break;
			
			case 1:															//when count is 1, red LED is on, green and blue off
				GPIO_PORTF_DATA_R |=  RED_MASK;
				GPIO_PORTF_DATA_R &= ~(GREEN_MASK|BLUE_MASK);
				GPIO_PORTF_DATA_R &= ~BLUE_MASK;
				break;
			
			case 2:															//when count is 2, blue LED is on, red and green are off
				GPIO_PORTF_DATA_R |= BLUE_MASK;
				GPIO_PORTF_DATA_R &= ~(RED_MASK|GREEN_MASK);
				GPIO_PORTF_DATA_R &= ~GREEN_MASK;
				break;
			
			case 3:															//when count is 3, blue LED is on, red is on, and green is off
				GPIO_PORTF_DATA_R |= (BLUE_MASK|RED_MASK);
				GPIO_PORTF_DATA_R &= ~GREEN_MASK;
				break;
			
			case 4:															//when count is 4, green LED is on, red and green are off
				GPIO_PORTF_DATA_R |= GREEN_MASK;
				GPIO_PORTF_DATA_R &= ~(BLUE_MASK|RED_MASK);
				break;
			
			case 5:															//when count is 5, green LED is on, red is on, and blue is off
				GPIO_PORTF_DATA_R |= (GREEN_MASK|RED_MASK);
				GPIO_PORTF_DATA_R &= ~BLUE_MASK;

				break;
			
			case 6:															//when count is 6, blue LED is on, green is on, and red is off
				GPIO_PORTF_DATA_R |= (GREEN_MASK|BLUE_MASK);
				GPIO_PORTF_DATA_R &= ~RED_MASK;
				break;
			
			case 7:															//when count is 7, all LEDs are on 
				GPIO_PORTF_DATA_R |= (GREEN_MASK|BLUE_MASK|RED_MASK);
				break;
		}

}

//interrupt handler
void GPIOPortF_Handler(){
		//switch debounce
		
	/*
		NVIC_EN0_R &= ~0x40000000;		//disable interrupt
		SysCtlDelay(53333);						//delay for a few milliseconds
		NVIC_EN0_R |= 0x40000000;			//enable interrupt again
	*/
		//GPIO_PORTF_ICR_R |= 0x11;				//acknowledge/clear interrupt flag
		
		NVIC_EN0_R &= ~0x40000000;		//disable interrupts on GPIOF
	
		if(GPIO_PORTF_RIS_R&0x10){								//action of SW1(PF4)
			GPIO_PORTF_ICR_R |= 0x10; 							//acknowledge/clear interrupt flags
					if((GPIO_PORTF_DATA_R&0x10)==0x00){			//SW1 is pressed
						 count++;													//increment count
					}
		}
			
		if(GPIO_PORTF_RIS_R&0x01){								//action on SW2 (PF0)
				GPIO_PORTF_ICR_R |= 0x01;							//acknowledge/clear flag
				if((GPIO_PORTF_DATA_R&0x01)==0x00){		//SW2 is pressed
					count--;														//decrement count
			}
		}		
		switchLeds();													//call function to operate the switches
		TimerLoadSet(TIMER1_BASE, TIMER_A, 80000);		//load delay value in timer
		TimerEnable(TIMER1_BASE, TIMER_A); 						//enable timer
		//TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);		//enable timer
}

//timer 1 handler for debouncing
void Timer1A_Handler(){
	
		TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);		//clear timer interrupts
		//GPIO_PORTF_ICR_R |= 0x01;							//acknowledge/clear flag
		//GPIO_PORTF_ICR_R |= 0x10;							//acknowledge/clear flag
	
		NVIC_EN0_R |= 0x40000000;							//Enable GPIOF interrupts	
}

//timer0 interrupt handler
	void Timer0A_Handler(void){
		
			TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);					//clear timer interrupt sources
			count++;										//increment count every second
		
			switchLeds();							//call function to operate the switches

	}
int main(){
		unsigned long period = 16000000; 	//reload value to Timer0A
		//unsigned long period2 = 80000;  	//reload value to Timer1A
		//initialize Ports
		PortFunctionInit();
	
		//configure interrupts
		Interrupt_Init();
	
		Timer0A_Init(period);				//configure timer0A interrupts
		Timer1A_Init();			//configure timer1A interrupts
	
		//keep system running  
		while(true){
			 
		}

}


