#include "MKL25Z4.h"        // Device header
#include "system_MKL25Z4.h" 
#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"
//#include "UART.c"

#define FUNCTIONBITSMASK(x)	(0xF0 & x)	// Obtains only the function bits, which are the 4 MSB, leaving 4 LSB as 0

#define BAUD_RATE 9600 
#define UART_TX_PORTE22 22 // UART2 (162)
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128


osSemaphoreId_t brainSem;
osSemaphoreId_t motorSem;
osSemaphoreId_t ledSem;
osSemaphoreId_t bgMusicSem;  		// Semaphore for the continuous music
osSemaphoreId_t finishMusicSem;	// Semaphore for the music played when finish the run

volatile uint8_t rx_data;


// To set threads to above normal priority
const osThreadAttr_t priorityAboveNormal = {
	.priority = osPriorityAboveNormal
};

// To set threads to high priority
const osThreadAttr_t priorityHigh = {
	.priority = osPriorityHigh
};


/* UART code @ 48MHz core clk freq and 24MHz Bus clk freq */
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


/* INT code for UART2 */
void UART2_IRQHandler()
{
	// Clear Pending IRQ
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	// Receive data from ESP32
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		rx_data = UART2->D;
		osSemaphoreRelease(brainSem);	// Release brainSem to start brain_thread
	}
	
	//Clear INT Flag
	PORTD->ISFR |= 0xffffffff;
}


/* Brain Thread */
void brain_thread (void *argument) {
	for (;;) {
		osSemaphoreAcquire(brainSem, osWaitForever);
		
		if (FUNCTIONBITSMASK(rx_data) == 0x00) {
			osSemaphoreRelease(motorSem);
		} 
		else if (FUNCTIONBITSMASK(rx_data) == 0x10) {
			osSemaphoreRelease(finishMusicSem);
		}
		// add led command
	}
}


/* Motor Thread */
void motor_thread (void *argument) {
	for (;;) {
		osSemaphoreAcquire(motorSem, osWaitForever);
		
		// include motor move code/function
	}
}


/* LED Thread */
void led_thread (void *argument) {
	for (;;) {
		osSemaphoreAcquire(ledSem, osWaitForever);
	}
}


/* Background Continuous Music Thread */
void bg_music_thread (void *argument) {
	for (;;) {
		osSemaphoreAcquire(bgMusicSem, osWaitForever);
		
		// code to play continuous music
	}
}


/* Finish Music Thread */
void finish_music_thread (void *argument) {
	for (;;) {
		osSemaphoreRelease(bgMusicSem);	// stop playing background music
		osSemaphoreAcquire(finishMusicSem, osWaitForever);
		
		// code to play finish music
	}
}


/* MAIN function */
int main(void)
{
	SystemCoreClockUpdate();
	//InitPWM();
	//InitSwitch();
	initUART2(BAUD_RATE);
	
	osKernelInitialize();	// Initialize CMSIS-RTOS
	
	brainSem = osSemaphoreNew(1, 0, NULL);
	motorSem = osSemaphoreNew(1, 0, NULL);
	ledSem = osSemaphoreNew(1, 0, NULL);     // CHECK if led sem needs to be blocked initially*****
	bgMusicSem = osSemaphoreNew(1, 1, NULL); 			// Continuous background music plays immediately
	finishMusicSem = osSemaphoreNew(1, 0, NULL);	// Music to be played at the end only upon receiving the command, initially blocked

	// DUTY CYCLE: change VOLUME + can generate different PWM simulataneously (557)
	// completely HARDWARE driven
	// TPM1_C0V = 0xEA6; // 0xEA6 = 3750 (half of 7500) -> 50% duty cycle 
	TPM1_C1V = 0x753; // half of 3750
	
	osThreadNew(brain_thread, NULL, &priorityHigh);	// brain thread should have the highest priority to control the others
	osThreadNew(motor_thread, NULL, NULL);
	osThreadNew(led_thread, NULL, NULL);
	osThreadNew(bg_music_thread, NULL, NULL);
	osThreadNew(finish_music_thread, NULL, &priorityAboveNormal);
	
	osKernelStart();
	
	while(1)
	{
		// TODO setup PWM for motors
		// TODO MOD, CnV for motors
		
		/**
		// AUDIO
		uint16_t MOD_VALUE[] = {note_C, note_D, note_E, note_F, note_G, note_A, note_B};
		for (int i = 0; i < 7; i++){
			// change FREQUENCY by changing MOD
			TPM1->MOD = MOD_VALUE[i];
			delay(0xFF000);
		}
		**/
	}
}
