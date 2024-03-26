#include "MKL25Z4.h"        // Device header
#include "system_MKL25Z4.h" 
#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"
//#include "UART.c"

#define FUNCTIONBITSMASK(x)	(0xF0 & x)	// Obtains only the function bits, which are the 4 MSB, leaving 4 LSB as 0

#define RED_LED     18      // PortB Pin 18
#define GREEN_LED   19      // PortB Pin 19
#define BLUE_LED    1       // PortD Pin 1
#define MASK(x)     (1 << (x))

#define BAUD_RATE 9600 
#define UART_TX_PORTE22 22 // UART2 (162)
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128


osSemaphoreId_t brainSem;
osSemaphoreId_t motorSem;
osSemaphoreId_t ledSem;
osSemaphoreId_t bgMusicSem;  		// Semaphore for the continuous music
osSemaphoreId_t finishMusicSem;	// Semaphore for the music played when finish the run

volatile uint8_t rx_data = 0x00;


typedef enum
{
	red_led,
	green_led,
	blue_led,
} led_colors_t;

typedef enum
{
	led_on,
	led_off,
} led_toggle_t;


volatile led_colors_t global_led = blue_led;

// To set threads to above normal priority
const osThreadAttr_t priorityAboveNormal = {
	.priority = osPriorityAboveNormal
};

// To set threads to high priority
const osThreadAttr_t priorityHigh = {
	.priority = osPriorityHigh
};


/* Delay Function */
static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}


/* input/output for LED @ PORTB, PORTD */
void initGPIO(void)
{
    // Enable Clock to PORTB and PORTD
    SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));

    // Configure MUX settings to make all 3 pins GPIO
    PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK; // clear
    PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1); // set
    PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
    PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);

    // Set Data Direction Registers for PortB and PortD
    PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
    PTD->PDDR |= MASK(BLUE_LED);
}

/* OFF all LED */
void offRGB()
{
    // Set bit to 1 to turn off LED since active low
    PTB->PSOR = MASK(RED_LED);
    PTB->PSOR = MASK(GREEN_LED);
    PTD->PSOR = MASK(BLUE_LED);
}


void ledControl(led_colors_t led_color, led_toggle_t led_toggle)
{
	offRGB();
  switch(led_color)
  {
		// Set bit to 0 to turn on LED since active low
		case red_led:  // Red LED
			if (led_toggle == led_on){
				PTB->PCOR = MASK(RED_LED);
			} else {
				PTB->PSOR = MASK(RED_LED);
			}				
			break;
		case green_led:  // Green LED
			if (led_toggle == led_on) {
				PTB->PCOR = MASK(GREEN_LED);
			} else {
				PTB->PSOR = MASK(GREEN_LED);
			}
			break;
		case blue_led:  // Blue LED
			if (led_toggle == led_on) {
				PTD->PCOR = MASK(BLUE_LED);
			} else {
				PTD->PSOR = MASK(BLUE_LED);
			}
			break;
		default:
			offRGB();
			break;
	}
}


/* UART code @ 48MHz core clk freq and 24MHz Bus clk freq */
void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	//enable clock to UART2 and PORTE
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	//Ensure Tx and Rx are disabled before configuration
	UART2->C2 &= ~(UART_C2_RE_MASK); // Off TE and RE (754)
	
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
	UART2->C2 |= UART_C2_RIE_MASK;
	
	UART2->C2 |= UART_C2_RE_MASK;
}


/* INT code for UART2 */
void UART2_IRQHandler()
{
	// Clear Pending IRQ
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	offRGB();
	
	// Receive data from ESP32
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		rx_data = UART2->D;
		osSemaphoreRelease(brainSem);	// Release brainSem to start brain_thread
	} 
	
	//Clear INT Flag
	//PORTD->ISFR |= 0xffffffff;
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
		
		//Testing
		//ledControl(red_led, led_on);
		//delay(0x80000);
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
	initGPIO();
	initUART2(BAUD_RATE);
	
	
	osKernelInitialize();	// Initialize CMSIS-RTOS
	
	brainSem = osSemaphoreNew(1, 0, NULL);
	motorSem = osSemaphoreNew(1, 0, NULL);
	ledSem = osSemaphoreNew(1, 0, NULL);     // CHECK if led sem needs to be blocked initially*****
	bgMusicSem = osSemaphoreNew(1, 1, NULL); 			// Continuous background music plays immediately
	finishMusicSem = osSemaphoreNew(1, 0, NULL);	// Music to be played at the end only upon receiving the command, initially blocked

	// DUTY CYCLE: change VOLUME + can generate different PWM simulataneously (557)
	// completely HARDWARE driven
	
	osThreadNew(brain_thread, NULL, &priorityHigh);	// brain thread should have the highest priority to control the others
	osThreadNew(motor_thread, NULL, NULL);
	osThreadNew(led_thread, NULL, NULL);
	osThreadNew(bg_music_thread, NULL, NULL);
	osThreadNew(finish_music_thread, NULL, &priorityAboveNormal);
	
	//ledControl(global_led, led_on);
	osKernelStart();

	while(1)
	{
		// TODO setup PWM for motors
		// TODO MOD, CnV for motors
		
		//ledControl(global_led, led_on);
		//delay(0x80000);
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
