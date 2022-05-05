#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#define RED_MASK GPIO_PIN_1 
#define GREEN_MASK GPIO_PIN_3

volatile signed long count = 0;  //initalize count as a global variable

//port initalization function
void PortFunctionInit(){
	
		volatile uint32_t ui32Loop;          //initialize clock counter variable
	
		SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF; //activate clock on Port F
		
		ui32Loop = SYSCTL_RCGC2_R;						//allow time for clock to start

			//unlock port F
			GPIO_PORTF_LOCK_R = 0x4C4F434B;
			GPIO_PORTF_CR_R |= 0x01;
		
			//set direction for PF1, PF3 as output
			GPIO_PORTF_DIR_R |= 0x0A;
			//GPIO_PORTF_DIR_R |= 0x04;
			
			//set the direction of PF0 and PF4 as input by clearing the bits
			GPIO_PORTF_DIR_R &= ~0x11;
			//GPIO_PORTF_DIR_R &= ~0x01;
		
			//enable PF0, PF1, PF3, and PF4 for digital function
			GPIO_PORTF_DEN_R |= 0x1B;
		
			//enable pull_up on PF0 and PF4
			GPIO_PORTF_PUR_R |= 0x11;
		
}

void Interrupt_Init(){
	NVIC_EN0_R |= 0x40000000;			//Enable interrupt 30 in NVIC (GPIOF)
	NVIC_PRI7_R &= ~0x00E00000;		//configure GPIOF as priority 0
	GPIO_PORTF_IM_R |= 0x11;		//arm interrupt of PF0 and PF4
	GPIO_PORTF_IS_R &= ~0x11;		//enable edge triggering on PF0 and PF4 by setting IS to 0
	GPIO_PORTF_IBE_R &= ~0x11;
	GPIO_PORTF_IEV_R &= ~0x11;		//PF0 and PF4 rising edge event triggering
	IntMasterEnable();
}


//interrupt handler
void GPIOPortF_Handler(){
	//switch debounce
	NVIC_EN0_R &= ~0x40000000;		//disable interrupt
	SysCtlDelay(53333);						//delay for a few milliseconds
	NVIC_EN0_R |= 0x40000000;			//enable interrupt again
	
	
		//action on SW1
		if(GPIO_PORTF_RIS_R&0x10){												//poll SW1
			
			GPIO_PORTF_ICR_R |= 0x10; 							 //clear flag on PF4 (SW1)
			if((GPIO_PORTF_DATA_R&0x10)==0x00){							//check if SW1 is pressed
					count++; 																		//increment count when SW1 is pressed
					if(count == 0x01){
						GPIO_PORTF_DATA_R |= GREEN_MASK;					//turn the green LED on if count is equal to 1
					}
					if(count == 0x02){
						GPIO_PORTF_DATA_R &= ~GREEN_MASK;					//turn the green LED off if count is equal to 2
						GPIO_PORTF_DATA_R |= RED_MASK;						//turn the red LED on if count is equal to 2
					}
					if(count == 0x03){
						GPIO_PORTF_DATA_R |= GREEN_MASK|RED_MASK; //turn green and red LED is count is equal to 3
						
					}
					if(count>0x03){
							GPIO_PORTF_DATA_R &= ~GREEN_MASK;			//turn off green LED if counter is greter than 3
							GPIO_PORTF_DATA_R &= ~RED_MASK;				//turn off red LED if counter is greter than 3
						
					}
					if(((GPIO_PORTF_DATA_R&0x01)==0x00)&&count == 0x00){
							
							GPIO_PORTF_DATA_R |= GREEN_MASK;									//turn on green LED
							GPIO_PORTF_DATA_R |= RED_MASK;										//turn on red LED
					}	
					
			}
		}	
		
			//action on SW2
			if(GPIO_PORTF_RIS_R&0x01){							//poll SW2
				
					GPIO_PORTF_ICR_R |= 0x01; 					 //clear flag on PF0 (SW2)
					if((GPIO_PORTF_DATA_R&0x01)==0x00){												//check if SW2 is pressed
							count--;															//decrement count
						
							if(count<0x00){
								count = 0x03;				//reset count
								
							}
							if( count == 0x03){											//count is equal to 3
																												
								GPIO_PORTF_DATA_R |= GREEN_MASK;						//turn on green LED when count is 3
								GPIO_PORTF_DATA_R |= RED_MASK;							//turn on red LED when count is 3
								
							}
							if(count == 0x02){				             //count is equal to 2
								
								GPIO_PORTF_DATA_R &= ~GREEN_MASK;									//turn off green LED
								GPIO_PORTF_DATA_R |= RED_MASK;								    //turn on red LED
								
							}
							if(count == 0x01){										//count is equal to 1
								
								GPIO_PORTF_DATA_R |= GREEN_MASK;		 // decrement count and turn off green LED;
								GPIO_PORTF_DATA_R &= ~RED_MASK;			 //decreement count and turn on red LED
						
							}
							if(count == 0x00){											//count is equal to 0
								GPIO_PORTF_DATA_R &= ~GREEN_MASK; 		// turn off green LED;
								GPIO_PORTF_DATA_R &= ~RED_MASK;				// turn off red LED;
							}
							
					}
			}
			count = count%0x04;			
	}

int main(){
		//initialize Ports
		PortFunctionInit();
	
		//configure interrupts
		Interrupt_Init();
	
	//loop forever
	while(true){
      //delay for a while		
			SysCtlDelay(6000000);
	}

}
