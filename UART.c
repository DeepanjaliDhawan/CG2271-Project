#include "MKL25Z4.h"        // Device header
#include "system_MKL25Z4.h" 
#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"

#define BAUD_RATE 9600 
#define UART_TX_PORTE22 22 // UART2 (162)
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128


/* UART code @ 48MHz core clk freq and 24MHz Bus clk freq */
/*
void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	//enable clock to UART2 and PORTE
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	//connect UART to pins for PTE22, PTE23
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4); // ALT 4 (162)
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	//Ensure Tx and Rx are disabled before configuration
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK)); // Off TE and RE (754)
	
	// Set Baud Rate to desired value
	bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2; //UART module runs at half the clk speed: system design
	divisor = bus_clock / (baud_rate * 16); // UART oversampling *16
	UART2->BDH = UART_BDH_SBR(divisor >> 8); // higher & lower baud rate registers (751)
	UART2->BDL = UART_BDL_SBR(divisor);
	
	// No parity, 8-bits settings
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	// Enable interrupts
	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	// Enable Tx and Rx
	//UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	// Only need to enable Rx since not transmitting
	UART2->C2 |= UART_C2_RE_MASK;
}
*/
