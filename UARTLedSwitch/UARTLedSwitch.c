#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"

#define RED_MASK GPIO_PIN_1 
#define BLUE_MASK GPIO_PIN_2
#define GREEN_MASK GPIO_PIN_3

//error message array
char error[] = "INVALID INPUT";

//start up message
char initMess[] = "Enter G,g,R,r,B,b to turn LEDs ON and OFF: ";

void UART_Init(void){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);			//enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);			//enable GPIOA port pins

    GPIOPinConfigure(GPIO_PA0_U0RX);					//configure pin PA0 for recieving
    GPIOPinConfigure(GPIO_PA1_U0TX);					//configure pin PA1 for transmitting
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);		

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); //enable pin for LED PF2

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));				//allow 8 bits/sec bandwidth, 
																																								//allow one stop bit and no parity bit

    IntMasterEnable(); //enable processor interrupts
    IntEnable(INT_UART0); //enable the UART interrupt
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
}

//terminal input function
void terminalInput(char *message){
			
		//loop trhough message while printing on terminal
		for(int index = 0;index < message[index]!= '\0'; index++){
					UARTCharPut(UART0_BASE, message[index]);
		}
}
void UART0IntHandler(){
		uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status

    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts
		
		while(UARTCharsAvail(UART0_BASE)){
			
					switch(UARTCharGet(UART0_BASE)){			//switch through characters on terminal
						case 'R':	
							if((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)&GPIO_PIN_1) == 0){			//check if red LED is off
									GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_2, GPIO_PIN_3&GPIO_PIN_2);  //turn all other LEDs off
									GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);			//turn red LED on
									UARTCharPut(UART0_BASE, 'R');									//show character on terminal
							}
						break;
						
						case 'r':
								if((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)&GPIO_PIN_1) == GPIO_PIN_1){		//check if red LED is on
										GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);			//turn off red LED
										UARTCharPut(UART0_BASE, 'r');							//show character on terminal
								}
						break;
						
						case 'B':
							if((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2)&GPIO_PIN_2) == 0){			//check if blue LED is off
									GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_1&GPIO_PIN_3);	//turn all other LEDs off
									GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);			//turn blue LED on
									UARTCharPut(UART0_BASE, 'B');								//show character on terminal
							}
						break;
						
						case 'b':
							if((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2)&GPIO_PIN_2) == GPIO_PIN_2){		//check if blue LED is on
										GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);			//turn off blue LED
										UARTCharPut(UART0_BASE, 'b');								//show character on terminal
							}
						break;
						
						case 'G':
							if((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3)&GPIO_PIN_3) == 0){			//check if green LED is off
									GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_1&GPIO_PIN_2);	//turn all other LEDs off
									GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);			//turn green LED on
									UARTCharPut(UART0_BASE, 'G');								//show character on terminal
							}
						break;
						
						case 'g':
								if((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3)&GPIO_PIN_3) == GPIO_PIN_3){		//check if green LED is on
										GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);			//turn off green LED
										UARTCharPut(UART0_BASE, 'g');									//show character on terminal
								}
						break;
						default:
							terminalInput("INVALID INPUT");       //display error message if not G,B,R,g,b,r
							}
						terminalInput("\r\n");   
		}
		
}
//message initialization and display function 

int main(){
		
		//initialize UART ports
		UART_Init();
	
		//send prompt to terminal
		terminalInput(initMess);

		//continue while true
		while(true){}
			
}


/*
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_gpio.h"
#include "driverlib/timer.h"


int main(void) {
	
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	GPIOPinConfigure(GPIO_PC4_U1RX);
	GPIOPinConfigure(GPIO_PC5_U1TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);	

	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 38400, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 38400, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	while (1)
	{
		if (UARTCharsAvail(UART1_BASE)) // if char received on UART 1, transmit that char on UART 0
			UARTCharPut(UART0_BASE, UARTCharGet(UART1_BASE));
		if (UARTCharsAvail(UART0_BASE)){ // if char received on UART 0, transmit that char on UART 1
			UARTCharPut(UART1_BASE, UARTCharGet(UART0_BASE));
		}
	}

}
*/

