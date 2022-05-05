#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"
#define PWM_FREQUENCY 55
/*
//#define uint_32 GPIO_PIN
#define two_sec 1000000  //two second delay

uint32_t count = 0;   //for debugging

void PortInitialization(){
		 SYSCTL_RCGCGPIO_R = SYSCTL_RCGC2_GPIOA;		//enable clock on PORTA
	
		 SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF; //activate clock on Port F
			
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
	
		 //little finger
		 GPIO_PORTA_DIR_R |= GPIO_PIN_6;  //set direction of pin as output
		 GPIO_PORTA_DEN_R |= GPIO_PIN_6;  //enable pin 2 for digital function
			
			//ring finger
		 GPIO_PORTA_DIR_R |= GPIO_PIN_5;  //set direction of pin as output
		 GPIO_PORTA_DEN_R |= GPIO_PIN_5;  //enable pin 3 for digital function
	
		 //middle finger
		 GPIO_PORTA_DIR_R |= GPIO_PIN_4;  //set direction of pin as output
		 GPIO_PORTA_DEN_R |= GPIO_PIN_4;  //enable pin 4 for digital function
	
		 //index
		 GPIO_PORTA_DIR_R |= GPIO_PIN_3;  //set direction of pin as output
		 GPIO_PORTA_DEN_R |= GPIO_PIN_3;  //enable pin 5 for digital function
	
		 //thumb
		 GPIO_PORTA_DIR_R |= GPIO_PIN_2;  //set direction of pin as output
		 GPIO_PORTA_DEN_R |= GPIO_PIN_2;  //enable pin 6 for digital function
		 
}
		 

//create a microsecond delay using TIMER1A
void DelaySomeSeconds(int time){
		 SYSCTL_RCGCTIMER_R |= 2;  //enable clock on TIMER1A
		 //SYSCTL->RCGCTIMER |= 2;			//enable clock on TIMER1A
		 TIMER1_CTL_R = 0;						//disable timer before initialization
		 TIMER1_CFG_R |= 0x04;					//enable timer for 16bit option
		 TIMER1_TAMR_R |= 0x02; 			  //enabe timer for periodic function
		 TIMER1_TAILR_R = 16-1;				//TimerA interval load value
		 TIMER1_ICR_R = 0x01;					//clear timer flag
		 TIMER1_CTL_R |= 0x01;						//enable timer after initialization
		 
		 for(int index = 0; index<time; index++){
				 while((TIMER1_RIS_R&0x01)==0){
							 TIMER1_ICR_R = 0x01; //clear TIMER1 interrupt flag
				 }
		 }
}

//function will generate 3% duty cycle from 20ms PWM signal
void servoZeroRange(int GPIO_PIN){
		 for(int index = 0; index<1000; index++){
				 GPIO_PORTA_DATA_R |= GPIO_PIN;  //enable Pin 4 on portA fro ditital function
				 //DelaySomeSeconds(600);  //generate 0.6ms delays
				 GPIO_PORTA_DATA_R &= ~GPIO_PIN;  //Disable Pin on port A
				 //DelaySomeSeconds(19400); 		//generate 1.94ms delays
		 }

}

//generate a 7% duty cycle from 20ms PWM signal
void servoHalfRange(int GPIO_PIN){
		 for(int index = 0; index<1000; index++){
				 GPIO_PORTA_DATA_R |= GPIO_PIN;  //enable Pin 4 on portA fro ditital function
				 //DelaySomeSeconds(1400);  //generate 1.4ms delays
				 GPIO_PORTA_DATA_R &= ~GPIO_PIN;  //Disable Pin on port A
				 //DelaySomeSeconds(18600); 		//generate 1.86ms delays
		 }
	
}	

void servoFullRange(int GPIO_PIN){
			for(int index = 0; index<1000; index++){
				 GPIO_PORTA_DATA_R |= GPIO_PIN;  //enable Pin 4 on portA fro ditital function
				 //DelaySomeSeconds(2400);  //generate 0.6ms delays
				 GPIO_PORTA_DATA_R &= ~GPIO_PIN;  //Disable Pin on port A
				 //DelaySomeSeconds(17600); 		//generate 1.76ms delays
		 }

}

//start moving fingers when called
void fingerMovement(int GPIO_PIN){
			while(true){
					//servoZeroRange(GPIO_PIN);					//servo at zero range
					//DelaySomeSeconds(two_sec);				//delay two seconds
					//servoHalfRange(GPIO_PIN);					//servo at half range
					DelaySomeSeconds(two_sec);				//delay a two seconds
					servoFullRange(GPIO_PIN);					//servo at full range
					//DelaySomeSeconds(two_sec);				//delay two seconds
		  }
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

void GPIOPortF_Handler(){
			//switch debounce
			NVIC_EN0_R &= ~0x40000000;		//disable interrupt
			SysCtlDelay(53333);						//delay for a few milliseconds
			NVIC_EN0_R |= 0x40000000;			//enable interrupt again
			
			//action on SW1
			if(GPIO_PORTF_RIS_R&0x10){												//poll SW1
			
					GPIO_PORTF_ICR_R |= 0x10; 							 //clear flag on PF4 (SW1)
					if((GPIO_PORTF_DATA_R&0x10)==0x00){							//check if SW1 is pressed
							fingerMovement(GPIO_PIN_4);							//call to begin moving servos
							//count ++;
					}
			}	
}	
int main(){
		//initialize GPIO pins
		PortInitialization();
		
		//initalize interrupts
		Interrupt_Init();
	
		//loop while running called servo servo
		while(true){
		//	if((GPIO_PORTF_DATA_R&GPIO_PIN_0)==0x01){							//check if SW1 is pressed
							//fingerMovement(GPIO_PIN_1);							//call to begin moving servos
			//}	
			//if((GPIO_PORTF_DATA_R&GPIO_PIN_4)==0x01){							//check if SW1 is pressed
						//	fingerMovement(GPIO_PIN_4);							//call to begin moving servos
			}
					fingerMovement(GPIO_PIN_2);							//call to begin moving servos
				  //fingerMovement(GPIO_PIN_3);							//call to begin moving servos
					//fingerMovement(GPIO_PIN_4);							//call to begin moving servos
					//fingerMovement(GPIO_PIN_5);							//call to begin moving servos
					//fingerMovement(GPIO_PIN_6);							//call to begin moving servos
}



#include "tm4c123gh6pm.h"


#define PWM_FREQUENCY 55
void delay(int time){
		 for(int index = 0; index< time; index++){
				 for(int indexj=0; indexj<3180; indexj++){
				 }
		 }
	 }
int main(void){
	
		//delay for a few milliseconds
		//void delay(int n);
		
		//int duty_cycle = 4999;
		SysCtlDelay(3180);
	
		//SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);		//enable clock on PWM1
		SYSCTL_RCGCPWM_R |= 2;			    //enable clock on PWM1
		SYSCTL_RCGCGPIO_R |= 0x20;			//enable clock on GPIO
		SYSCTL_RCC_R |= (1<<20);				//enable system clock divisor function
		SYSCTL_RCC_R &= ~0x00100000; 	  //Directly feed clock to PWM1 module without pre-divider
		
		//setting PF2 pin for M1PWM6 channel output pin
		GPIO_PORTF_AFSEL_R |= (1<<2);			 //set PF2 for alternate function
		GPIO_PORTF_PCTL_R &= ~0x00000F00;  //set PF2 as output pin
		GPIO_PORTF_PCTL_R |= 0x00000500;		//make PF2 PWM output pin
		GPIO_PORTF_DEN_R |= (1<<2);					//set PF2 as a digital pin
		
		//PWM Channel 6 setting
		PWM1_3_CTL_R &= ~(1<<0);  		//Disable Generator 3 counter
		PWM1_3_CTL_R &= ~(1<<1);			//select down count mode of count 3
		PWM1_3_GENA_R = 0x0000008C;		//set PWM output when counter reloaded and clear when matches PWMCMPA
		PWM1_3_LOAD_R = 16000;				//set load value for 50kHz (16MHz/16000) 16MHz/65 = 250kHz and (250kHz/5000)
		PWM1_3_CMPA_R = 7999-1;				//set duty cycle to minimum value
		
		PWM1_3_CTL_R = 1;							//Enable Generator 3 counter
		PWM1_ENABLE_R	= 0x40;					//Enable PWM1 channel 6 output
		//GPIO_PORTF_DATA_R |= GPIO_PIN_2;
		//loop forever
		while(true){
					
					duty_cycle = duty_cycle - 40;			//the change in the voltage output will be 
																						//determined by the value subtracted from the duty cycle
		  		if(duty_cycle<=0){
						 duty_cycle = 5000;							//Reset duty cycle back to maximum value
					}
					
					PWM1->_3_CMPA = duty_cycle;
					delay(100);
					
		}
	}
	
	


	volatile uint32_t ui32Load;
	volatile uint32_t ui32PWMClock;
	volatile uint8_t ui8Adjust;
	ui8Adjust = 83;

	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);

	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);

	while(1)
	{

		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
		{
			ui8Adjust--;
			if (ui8Adjust < 56)
			{
				ui8Adjust = 56;
			}
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
		}

		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
		{
			ui8Adjust++;
			if (ui8Adjust > 111)
			{
				ui8Adjust = 111;
			}
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
		}

		SysCtlDelay(100000);
	}


//need to know clocknspeed
*/
/* Generates 50HZ and variable duty cycle on PF2 pin of TM4C123 Tiva C Launchpad */
/* PWM1 module and PWM generator 3 of PWM1 module is used. Hence PWM channel*/
//#include "TM4C123GH6PM.h"

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint8_t ui8Adjust;

void PortInitialization(){
	
		 //SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);			//set system clock
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);		//set clock on Port f
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);		//set clock on Port E
	
		 //unlock GPIO port F
		 HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
		 HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x01;
		 
	   //enable pull up on PF0 AND PF4 for the switches
		 GPIO_PORTF_PUR_R |= 0x11;
	
		 //configure port F, E pins for output
		 GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
		 GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5);
	
		 //SET DIRECTION FOR THE PINS	
		// GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT);
		// GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT);
	
		 GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT);
		 GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT);
		 GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_DIR_MODE_OUT);
	
		 //gSpecify the amp/strength from each pin
		 GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
		 GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
		 
		 //enable pin PE4 for M1PWM2
		 GPIOPinConfigure(GPIO_PE4_M1PWM2);
		 GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
		 
		 //enable pin PE5 for M1PWM3
		 GPIOPinConfigure(GPIO_PE5_M1PWM3);
		 GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
	
		 //enable pin PF0 for M1PWM4
		 GPIOPinConfigure(GPIO_PF0_M1PWM4);
		 GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
	
		 //enable pin PF1 for M1PWM5
		 GPIOPinConfigure(GPIO_PF1_M1PWM5);
		 GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
		 
		 //enable pin PF2 for M1PWM6
		 GPIOPinConfigure(GPIO_PF2_M1PWM6);
		 GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
		 
		 ui32PWMClock = SysCtlClockGet() / 64;
		 ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
		 //PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
		 //PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

		 //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Load / 1000);
		 
		 //PWM Setup 
		 //Configure the PWM generator for count up and down with immediate updates to parameters
		 PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC); //PE4, PE5
		 PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 16000);  //frequency 1/512 of the system and range for widthset
		 
		 PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC); //PF0, PF1
		 PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 16000);  //frequency 1/512 of the system and range for widthset
		 
		 PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC); //PF2
		 PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 16000);  //frequency 1/512 of the system and range for widthset
		 
		 //PWM1_1_LOAD_R = 5000;
		 //PWM1_2_LOAD_R = 5000;
		 //PWM1_3_LOAD_R = 5000;
		 
		 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 1);  //SET THE PULSE WIDTH
		 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 1);  //SET THE PULSE WIDTH
		 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, 1);  //SET THE PULSE WIDTH
		 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 1);  //SET THE PULSE WIDTH
		 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 1);  //SET THE PULSE WIDTH
		 
		 //Once setup, enable the generators
		 //Start th timers on all the generators
		 PWMGenEnable(PWM1_BASE, PWM_GEN_1);
		 PWMGenEnable(PWM1_BASE, PWM_GEN_2);
		 PWMGenEnable(PWM1_BASE, PWM_GEN_3);
		 PWMOutputState(PWM1_BASE, (PWM_OUT_2_BIT|PWM_OUT_3_BIT|PWM_OUT_4_BIT|PWM_OUT_5_BIT|PWM_OUT_6_BIT), true); //enable all the pins
		 //PWMOutputState(PWM1_BASE, (PWM_OUT_6_BIT), true); //enable all the pins
}

int main(void){
		//inialize ports
		PortInitialization();
	
		while(true){
				if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)==0x00){
						//PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
				}
			//PWMOutputState(PWM1_BASE, (PWM_OUT_2_BIT|PWM_OUT_3_BIT|PWM_OUT_4_BIT|PWM_OUT_5_BIT|PWM_OUT_6_BIT), true); //enable all the pins
		}


}

