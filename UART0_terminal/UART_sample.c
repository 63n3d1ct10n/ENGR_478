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

#define RED_MASK GPIO_PIN_1 
#define BLUE_MASK GPIO_PIN_2
#define GREEN_MASK GPIO_PIN_3
/*
void UART_Init(void){
	
		//SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);			//set system clock

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);			//enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);			//enable GPIOA port pins

    GPIOPinConfigure(GPIO_PA0_U0RX);					//configure pin PA0 for recieving
    GPIOPinConfigure(GPIO_PA1_U0TX);					//configure pin PA1 for transmitting
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);		

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); //enable pin for LED PF2

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));				//allow 8 bits/sec bandwidth, 
																																								//allow one stop bit and no parity bit

    IntMasterEnable(); //enable processor interrupts
    IntEnable(INT_UART0); //enable the UART interrupt
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
}
*/
void UARTIntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status

    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts

    while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
    {
        UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE)); //echo character
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //blink LED
        SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay ~1 msec
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //turn off LED
    }
/*	
		char readMsg[3];
	
		//acknowledge flag for Timer0A timeout
		//TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		
		int index = 0;
	
		while(readMsg[index] != '\0'){
				if(UARTCharGet(UART0_BASE)== 'R'){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
				
				}
				if(UARTCharGet(UART0_BASE)== 'r'){
					 GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK&BLUE_MASK);
				}
				if(UARTCharGet(UART0_BASE)== 'B'){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
				
				}
				if(UARTCharGet(UART0_BASE)== 'b'){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, RED_MASK&BLUE_MASK);
				}
				if(UARTCharGet(UART0_BASE)== 'G'){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GREEN_MASK);
				
				}
				if(UARTCharGet(UART0_BASE)== 'g'){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, RED_MASK&BLUE_MASK);
				}
				
		}
		*/
}

int main(void) {
	
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); //enable pin for LED PF2

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    IntMasterEnable(); //enable processor interrupts
    IntEnable(INT_UART0); //enable the UART interrupt
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
/*
    UARTCharPut(UART0_BASE, 'E');
    UARTCharPut(UART0_BASE, 'n');
    UARTCharPut(UART0_BASE, 't');
    UARTCharPut(UART0_BASE, 'e');
    UARTCharPut(UART0_BASE, 'r');
    UARTCharPut(UART0_BASE, ' ');
    UARTCharPut(UART0_BASE, 'T');
    UARTCharPut(UART0_BASE, 'e');
    UARTCharPut(UART0_BASE, 'x');
    UARTCharPut(UART0_BASE, 't');
    UARTCharPut(UART0_BASE, ':');
    UARTCharPut(UART0_BASE, ' ');
		
		UART_Init();
		*/
		//char readMsg[3];
		
		char initMess[] = {'E','n','t','e','r',
											' ','a',' ','l','e',
											't','t','e','r',' ','t',
											'o',' ','t','u','r',
											'n',' ','o','n',' ','L','E',
											'D','s',':'};
		//int index = 0;
		for(int index = 0; index < 31 ;index++){
					UARTCharPut(UART0_BASE, initMess[index]);
		}
	 
		
	//	while(readMsg[inde != '\0'){
				if(UARTCharGet(UART0_BASE)== 'R'){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
				
				}
				if(UARTCharGet(UART0_BASE)== 'r'){
					 GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK&BLUE_MASK);
				}
				if(UARTCharGet(UART0_BASE)== 'B'){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
				
				}
				if(UARTCharGet(UART0_BASE)== 'b'){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, RED_MASK&BLUE_MASK);
				}
				if(UARTCharGet(UART0_BASE)== 'G'){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GREEN_MASK);
				
				}
				if(UARTCharGet(UART0_BASE)== 'g'){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, RED_MASK&BLUE_MASK);
				}
				
	//	}
		
    while (1) //let interrupt handler do the UART echo function
    {
//    	if (UARTCharsAvail(UART0_BASE)) UARTCharPut(UART0_BASE, UARTCharGet(UART0_BASE));
    }

	}

