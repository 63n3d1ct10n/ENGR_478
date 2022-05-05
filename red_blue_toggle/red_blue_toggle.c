#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"

#define RED_MASK GPIO_PIN_1
#define BLUE_MASK GPIO_PIN_2

void PortFunctionInit(){
	
		volatile uint32_t ui32Loop;
	
		SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;
		
		ui32Loop = SYSCTL_RCGC2_R;

		//unlock port F
		GPIO_PORTF_LOCK_R = 0x4C4F434B;
		GPIO_PORTF_CR_R |= 0x01;
	
		//set direction for PF1, PF2 as output
		GPIO_PORTF_DIR_R |= 0x06;
		
		//set the direction of PF0 and PF4 as input by clearing thr bits
		GPIO_PORTF_DIR_R &= ~0x11;
		//GPIO_PORTF_DIR_R &= ~0x10;
	
		//enable PF0, PF1, PF2, and PF4 for digital function
		GPIO_PORTF_DEN_R |= 0x17;
	
		//enable pull_up on PF0 and PF4
		GPIO_PORTF_PUR_R |= 0x11;
		
}

int main(void){

		//define clock frequency variable
		unsigned long clk_freq;
	
		//initalize the GPIO ports
		PortFunctionInit();
		
		//set the required frequency of the system
		SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //400 MHz/2/10 = 20MHz
		
		//initialize clock set frequency
		clk_freq = SysCtlClockGet();

		uint32_t BLUE_TOGGLE;		//define Blue toggle variable as a 32-bit integer
		
		uint32_t RED_TOGGLE;		//define Red toggle variable as a 32-bit integer
		
		uint32_t LEDs_OFF;			//define variable to store off LEDs as a 32-bit integer
		
		while(1){
			//loop 1000 times
			for(uint32_t index = 0; index < 1000; index++){
				
				if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)== 0x00){   //switch 1 is pressed
							
								//turn off blue LED toggle
								BLUE_TOGGLE &= ~GPIO_PIN_2;
								GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_TOGGLE);
							
								//delay for half a second
								SysCtlDelay(clk_freq/6);
						
								//toggle PF1 RED LED
								RED_TOGGLE ^= GPIO_PIN_1;
								GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_TOGGLE);
						
			}else if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)!= 0x00){	//switch 1 is not pressed
					
								//turn off RED LED
								RED_TOGGLE &= ~GPIO_PIN_1;
								GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_TOGGLE);
					
								//delay for half a second
								SysCtlDelay(clk_freq/6);
					
								
								//toggle PF2 BLUE LED
								BLUE_TOGGLE ^= GPIO_PIN_2;
								GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_TOGGLE);
		
				}
				if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)==0x00){			//switch 2 is pressed
								
								//turn off RED and BLUE LEDs
								LEDs_OFF &= ~(GPIO_PIN_1|GPIO_PIN_2);
								GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2, LEDs_OFF);
				
				}
				//increment index
				index++;
				
				//delay for a bit
				SysCtlDelay(5333);
			}
		}
	}
