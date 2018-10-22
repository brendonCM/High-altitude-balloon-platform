/*
******************************************************************************
File:     main.c

******************************************************************************
*/

/*==========================================================================*
 * 																			*
 * 					EEE4022s: Final Year Project					        *
 * 							CubeSat interface							    *
 * 																			*
 *==========================================================================*
 *
 *	Brendon Crispen Maongera
 *
 *
 *
 *
 ****************************************************************************
 */

//==========================================================================*
//	INCLUDES
//==========================================================================*

	/*Include your header files here*/

#include "stm32f0xx.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

//==========================================================================*
//	MACROS
//==========================================================================*

	/*Define your macros here*/



//==========================================================================*
//	GLOBAL CONSTANTS
//==========================================================================*

	/*Define your global constants here*/
#define GPIO_AFRL_PIN6_AF_0  0b00000001000000000000000000000000
#define GPIO_AFRL_PIN6_AF_1  0b00000010000000000000000000000000
#define GPIO_AFRL_PIN6_AF_2  0b00000100000000000000000000000000
#define GPIO_AFRL_PIN6_AF_3  0b00001000000000000000000000000000

#define GPIO_AFRL_PIN7_AF_0  0b00010000000000000000000000000000
#define GPIO_AFRL_PIN7_AF_1  0b00100000000000000000000000000000
#define GPIO_AFRL_PIN7_AF_2  0b01000000000000000000000000000000
#define GPIO_AFRL_PIN7_AF_3  0b10000000000000000000000000000000

#define GPIO_AFRL_PIN2_AF_0  0b000100000000
#define GPIO_AFRL_PIN2_AF_1  0b001000000000
#define GPIO_AFRL_PIN2_AF_2  0b010000000000
#define GPIO_AFRL_PIN2_AF_3  0b100000000000

#define GPIO_AFRL_PIN3_AF_0  0b0001000000000000
#define GPIO_AFRL_PIN3_AF_1  0b0010000000000000
#define GPIO_AFRL_PIN3_AF_2  0b0100000000000000
#define GPIO_AFRL_PIN3_AF_3  0b1000000000000000




//==========================================================================*
//	GLOBAL VARIABLES
//==========================================================================*

	/*Define your global variables here*/
unsigned int packet_counter;
unsigned int TM_packet_type, TC_packet_type;
int Origin_ID[2] = {1,2}; // For TM, TC
int Destination_ID[2] = {2,1}; // For TM, TC
int time, sub_seconds, date;
char packet[150];     // need an array to store multiple packets due to delays
char data[50]; // from UART
char sending_to_cube = 'Y';
int cube_detect = 0;
int nom =0;


//==========================================================================*
//	FUNCTION PROTOTYPES
//==========================================================================*

	/*Declare your functions here*/
void init();
void UART_communication_init();
void Timer_read();
void UART1_send_data();
void UART2_send_data();
void UART1_receive_data();
void UART2_receive_data();
void Cube_detection();
void RTC_init();
void Create_packet();
void getTime();



//==========================================================================*
//	FUNCTIONS
//==========================================================================*

	/*Write your function descriptions here*/
void init()
{
	// Enable A and B GPIO
	RCC ->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC ->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB ->MODER |= GPIO_MODER_MODER0_0;

	// for cube detection
	GPIOA ->MODER &= ~GPIO_MODER_MODER0;
	GPIOA ->PUPDR |= GPIO_PUPDR_PUPDR0_0;
}

void getTime()
{
	// Reads the time and date for the timestamp
	while ((RTC->ISR & RTC_ISR_RSF)==0);
	sub_seconds = RTC->SSR;
	time = RTC ->TR;
	date = RTC->DR;
	RTC ->ISR &= ~RTC_ISR_RSF;
}

void RTC_init()
{
	// initializes the time stamp for each packet sent/received
	RCC ->BDCR |= RCC_BDCR_BDRST; //Resets the RTC domain
	RCC ->BDCR &= ~RCC_BDCR_BDRST; // Undo the reset
	RCC ->APB1ENR |= RCC_APB1ENR_PWREN; // enable clock for power control
	PWR ->CR |= PWR_CR_DBP;  	// to enable RTC registers write access

	RCC ->CSR |= RCC_CSR_LSION;    // switch on LSI oscillator
	while ((RCC ->CSR & RCC_CSR_LSIRDY) == 0);  // Wait till the LSI is ready
	RCC ->BDCR |= RCC_BDCR_RTCSEL_LSI; //setting the LSI as the RTC clock

	RCC ->BDCR |= RCC_BDCR_RTCEN; // enable RTC clock

	RTC ->WPR = 0xCA;	// Unlock write protection on all RTC registers
	RTC ->WPR = 0x53;	// Unlock write protection on all RTC registers

	// Calendar initialization and configuration
	RTC ->ISR |= RTC_ISR_INIT;
	RTC ->ISR |= RTC_ISR_INITF;

	// To generate a 1Hz clock
	RTC ->PRER = 0b011111110000000000000000; // Set 127 for the PREDIV_A
	RTC ->PRER |= 0b100111000;	// Set 312 for the PREDIV_S

	// Set the seconds (30 SECONDS)
	RTC ->TR &= ~RTC_TR_SU_0;
	RTC ->TR &= ~RTC_TR_SU_1;
	RTC ->TR &= ~RTC_TR_SU_2;
	RTC ->TR &= ~RTC_TR_SU_3;
	RTC ->TR |= RTC_TR_ST_1|RTC_TR_ST_0;
	RTC ->TR &= ~RTC_TR_ST_2;

	// Set the minutes (30 MINUTES)
	RTC ->TR &= ~RTC_TR_MNU;
	RTC ->TR |= RTC_TR_MNT_1|RTC_TR_MNT_0;
	RTC ->TR &= ~RTC_TR_MNT_2;

	// Set the hours (14 HOURS)
	RTC ->TR |= RTC_TR_HU_2;
	RTC ->TR |= RTC_TR_HT_0;
	RTC ->TR &= ~RTC_TR_HT_1;


	// for the day (12)
	RTC ->DR |= RTC_DR_DU_1;
	RTC ->DR |= RTC_DR_DT_0;
	RTC ->DR &= ~RTC_DR_DT_1;

	// for the month (09)
	RTC ->DR |= RTC_DR_MU_0;
	RTC ->DR |= RTC_DR_MU_3;

	// for the year (18)
	RTC ->DR |= RTC_DR_YU_3;
	RTC ->DR |= RTC_DR_YT_0;

	RTC ->CR &= ~RTC_CR_FMT;	// Set as 24 hour format
	RTC ->ISR &= ~RTC_ISR_INIT; // To initialize the date and time
}

void UART_communication_init()
{
	RCC ->APB2ENR |= RCC_APB2ENR_USART1EN; // enable clock for USART1
	RCC ->APB1ENR |= RCC_APB1ENR_USART2EN; // enable clock for USART2

	// For USART1
	GPIOB -> MODER |= GPIO_MODER_MODER6_1; // set PB6 to AF mode
	GPIOB -> MODER |= GPIO_MODER_MODER7_1; // set PB7 to AF mode

	GPIOB ->AFR[0] &= ~GPIO_AFRL_PIN6_AF_0; // clear bit 0 of AFR6[3:0]
	GPIOB ->AFR[0] &= ~GPIO_AFRL_PIN6_AF_1; // clear bit 1 of AFR6[3:0]
	GPIOB ->AFR[0] &= ~GPIO_AFRL_PIN6_AF_2; // clear bit 2 of AFR6[3:0]
	GPIOB ->AFR[0] &= ~GPIO_AFRL_PIN6_AF_3; // clear bit 3 of AFR6[3:0]

	GPIOB -> AFR[0] &= ~GPIO_AFRL_PIN7_AF_0; // clear bit 0 of AFR7[3:0]
	GPIOB -> AFR[0] &= ~GPIO_AFRL_PIN7_AF_1; // clear bit 1 of AFR7[3:0]
	GPIOB -> AFR[0] &= ~GPIO_AFRL_PIN7_AF_2; // clear bit 2 of AFR7[3:0]
	GPIOB -> AFR[0] &= ~GPIO_AFRL_PIN7_AF_3; // clear bit 3 of AFR7[3:0]

	USART1 -> CR1 &= ~USART_CR1_M; // clear M0 (bit 12) of USARTx_CR1
	USART1 -> BRR = 48000000/921600; // setting a baud rate of 921.6kbps
	USART1 -> CR1 &= ~USART_CR1_PCE; // no parity
	USART1 -> CR2 &= ~USART_CR2_STOP; // 1 stop bit

	// Interrupt code below
	USART1 ->CR1 |= USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART1_IRQn);

	USART1 -> CR1 |= USART_CR1_UE;   // enable USART1
	USART1 -> CR1 |= USART_CR1_TE;  // enable USART1_TX
	USART1 -> CR1 |= USART_CR1_RE;  // enable USART1_RX

	// For USART2
	GPIOA -> MODER |= GPIO_MODER_MODER2_1; // set PA2 to AF mode
	GPIOA -> MODER |= GPIO_MODER_MODER3_1; // set PA3 to AF mode

	GPIOA ->AFR[0] |=  GPIO_AFRL_PIN2_AF_0; // set bit 0 of AFR2[3:0]
	GPIOA ->AFR[0] &= ~GPIO_AFRL_PIN2_AF_1; // clear bit 1 of AFR2[3:0]
	GPIOA ->AFR[0] &= ~GPIO_AFRL_PIN2_AF_2; // clear bit 2 of AFR2[3:0]
	GPIOA ->AFR[0] &= ~GPIO_AFRL_PIN2_AF_3; // clear bit 3 of AFR2[3:0]

	GPIOA -> AFR[0] |=  GPIO_AFRL_PIN3_AF_0; // set bit 0 of AFR3[3:0]
	GPIOA -> AFR[0] &= ~GPIO_AFRL_PIN3_AF_1; // clear bit 1 of AFR3[3:0]
	GPIOA -> AFR[0] &= ~GPIO_AFRL_PIN3_AF_2; // clear bit 2 of AFR3[3:0]
	GPIOA -> AFR[0] &= ~GPIO_AFRL_PIN3_AF_3; // clear bit 3 of AFR3[3:0]

	USART2 -> CR1 &= ~USART_CR1_M; // clear M0 (bit 12) of USARTx_CR1
	USART2 -> BRR = 48000000/9600; // setting a baud rate of 9.6kbps
	USART2 -> CR1 &= ~USART_CR1_PCE; // no parity
	USART2 -> CR2 &= ~USART_CR2_STOP; // 1 stop bit

	USART2 ->CR1 |= USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART2_IRQn);

	USART2 -> CR1 |= USART_CR1_UE;   // enable USART2
	USART2 -> CR1 |= USART_CR1_TE;  // enable USART2_TX
	USART2 -> CR1 |= USART_CR1_RE;  // enable USART2_RX
}

void USART2_IRQHandler(void)
{
	// receives data sent from the pocketbeagle and creates a packet to send to the cube
	unsigned char DataRx2; // define a variable to store data received
	while((USART2 -> ISR & USART_ISR_RXNE) == 0); // exits loop when data is received
	DataRx2= USART2 -> RDR; // store received data into ‘DataRx’ variable
	nom++;
	data[nom] = DataRx2;
	sending_to_cube = 'Y';
	Create_packet();
}

void USART1_IRQHandler(void)
{
	// receives data sent from the cube and creates a packet to store in the pocketbeagle
	unsigned char DataRx2; // define a variable to store data received
	while((USART1 -> ISR & USART_ISR_RXNE) == 0); // exits loop when data is received
	DataRx2= USART1 -> RDR; // store received data into ‘DataRx’ variable
	nom++;
	data[nom] = DataRx2;
	sending_to_cube = 'N';
	Create_packet();
}

void UART1_send_data()
{
	// sends each byte to the cube
	int datalength = sizeof(packet)/sizeof(packet[0]); // get length of packet
	int count = 0;
	while (count < datalength)
	{
		USART1 -> TDR = packet[count]; 		// write character ‘c’ to the USART1_TDR
		while((USART1 -> ISR & USART_ISR_TC) == 0); // exits loop when TC is high
		count++;
	}

}

void UART2_send_data()
{
	// send each byte to the pocket
	unsigned char DataToTx2 = packet;	// define a variable to store data to transmit
	USART2 -> TDR = DataToTx2; 		// write character ‘c’ to the USART1_TDR
	while((USART2 -> ISR & USART_ISR_TC) == 0); // exits loop when TC is high
}

void Cube_Detection()
{
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // enable clock for the sysconfcontroller
	SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // map PA0 to EXTI0
	EXTI-> IMR |= EXTI_IMR_MR0; // unmask external interrupt 0
	EXTI-> FTSR |= EXTI_FTSR_TR0; // trigger on falling edge
	NVIC_EnableIRQ(EXTI0_1_IRQn); // enable EXTI0_1interrupt in the NVIC
}

void EXTI0_1_IRQHandler(void)
{
	// initializes all the variables need for the TM/TC protocol
	EXTI-> PR |= EXTI_PR_PR0; // clear the interrupt pending bit
	GPIOB ->BSRR |= GPIO_BSRR_BS_0; //setting the led to indicate the cube is detected
	for (int x=0;x<3000000;x++)
	{}
	GPIOB ->BSRR |= GPIO_BSRR_BR_0;
	UART_communication_init();
	RTC_init();
}

void Create_packet()
{
	// create TM/TC protocol packet to send to the cube o the pocketbeagle
	getTime();
	if (sending_to_cube == 'Y')
	{
		TC_packet_type++;

		snprintf (packet ,sizeof(packet), "%06dT%06d.%03d,%07d,%01c%01c,%01d,%01d,%07d,%050c" , date, time , sub_seconds, packet_counter,'T','C', Origin_ID[1] , Destination_ID[1], TC_packet_type , data);
		UART1_send_data();
	}
	else if (sending_to_cube == 'N')
	{
		TM_packet_type++;
		snprintf (packet ,sizeof(packet), "%06dT%06d.%03d,%07d,%01c%01c,%01d,%01d,%07d,%050d" , date, time , sub_seconds, packet_counter, 'T','M' , Origin_ID[0] , Destination_ID[0], TM_packet_type , data);
		UART2_send_data();
	}
	packet_counter++;
}


//==========================================================================*

/****************************************************************************
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	init();
	Cube_Detection(); // initialize for cube detection
	/* Infinite loop */
	while (1)
	{
		// Wait for cube to be detected
		nom = 0;
	}
}
