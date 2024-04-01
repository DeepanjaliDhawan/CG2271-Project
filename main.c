#include "MKL25Z4.h"        // Device header
#include "system_MKL25Z4.h" 
#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"
#include <stdbool.h>
//#include "UART.c"
//#include "motor.c"

#define FUNCTIONBITSMASK(x)	(0xF0 & x)	// Obtains only the function bits, which are the 4 MSB, leaving 4 LSB as 0

#define RED_LED     18      // PortB Pin 18
#define GREEN_LED   19      // PortB Pin 19
#define BLUE_LED    1       // PortD Pin 1
#define MASK(x)     (1 << (x))

#define BAUD_RATE 9600 
#define UART_TX_PORTE22 22 // UART2 (162)
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128
// motors
#define MOTOR_BACK_LEFT		0	// PTB0 TPM1_CH0 RIGHT
#define MOTOR_BACK_RIGHT 	1	// PTB1 TPM1_CH1 RIGHT
#define MOTOR_FRONT_LEFT	2	// PTB2 TPM2_CH0 LEFT
#define MOTOR_FRONT_RIGHT 	3	// PTB3 TPM2_CH1 LEFT

#define DIRECTIONS 		6
#define MOD_VAL 		7500
#define FULL_MOD 		0x1D4C			// 7500	// for actual run
#define HALF_MOD 		0xEA6			// 3750	// for adjustment
#define QUARTER_MOD 	1875		// 1875 // for test runs
#define TEST_MOD (MOD_VAL / 8)			// 937	// for test runs// temp

#define SW_POS		6		// PORTD Pin 6: for temporary push btn Interrupt


// Define LED pins
//green -> front
#define GREEN_PTC4 4 // Port C Pin 4
#define GREEN_PTC5 5 // Port C Pin 5
#define GREEN_PTC6 6 // Port C Pin 6
#define GREEN_PTC10 10 // Port C Pin 10
#define GREEN_PTC11 11 // Port C Pin 11
#define GREEN_PTC12 12 // Port C Pin 12
#define GREEN_PTC13 13 // Port C Pin 13
#define GREEN_PTC16 16 // Port C Pin 16

// red -> back 
#define RED_PTC7 7 // Port C Pin 7


// audio 
#define PTD0 0 // for PWM (TPM0_CH0)

#define CLOCK (48000000 / 128) // 375000
#define note_C 		1047
#define note_D 		1175
#define note_E 		1319
#define note_F 		1397
#define note_G 		1568
#define note_A 		1760
#define note_B 		1865
#define note_C1 	2093
#define note_D1 	2349
#define note_E1 	2637
#define note_F1 	2794
#define note_G1 	3136
#define note_A1 	3520
#define note_B1 	3729

// ESP commands
#define STOP 			0x00
#define FORWARD 		0x01
#define BACKWARD 		0x02
#define LEFT	 		0x03
#define RIGHT	 		0x04
#define FRONT_LEFT 		0x05
#define FRONT_RIGHT 	0x06
#define REVERSE_LEFT	0x07
#define REVERSE_RIGHT	0x08
#define SONG 			0x10

osSemaphoreId_t brainSem;
osSemaphoreId_t motorSem;
//osSemaphoreId_t ledSem;
osSemaphoreId_t bgMusicSem;  		// Semaphore for the continuous music
osSemaphoreId_t finishMusicSem;	// Semaphore for the music played when finish the run

volatile uint8_t rx_data = 0x00;

volatile bool is_moving = false;

volatile uint32_t test_var = 0x00000000;


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
/*
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
*/

void InitGPIO(void) {
	// Enable Clock to PORTC
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	// Configure MUX Setting for all pins to be GPIO
	PORTC->PCR[GREEN_PTC4] &= ~PORT_PCR_MUX_MASK; // Port C Pin 4
	PORTC->PCR[GREEN_PTC4] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_PTC5] &= ~PORT_PCR_MUX_MASK; // Port C Pin 5
	PORTC->PCR[GREEN_PTC5] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_PTC6] &= ~PORT_PCR_MUX_MASK; // Port C Pin 6
	PORTC->PCR[GREEN_PTC6] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_PTC10] &= ~PORT_PCR_MUX_MASK; // Port C Pin 10
	PORTC->PCR[GREEN_PTC10] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_PTC11] &= ~PORT_PCR_MUX_MASK; // Port C Pin 11
	PORTC->PCR[GREEN_PTC11] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_PTC12] &= ~PORT_PCR_MUX_MASK; // Port C Pin 12
	PORTC->PCR[GREEN_PTC12] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_PTC13] &= ~PORT_PCR_MUX_MASK; // Port C Pin 13
	PORTC->PCR[GREEN_PTC13] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_PTC16] &= ~PORT_PCR_MUX_MASK; // Port C Pin 16
	PORTC->PCR[GREEN_PTC16] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[RED_PTC7] &= ~PORT_PCR_MUX_MASK; // Port C Pin 7
	PORTC->PCR[RED_PTC7] |= PORT_PCR_MUX(1);
	
	// Set Data Direction Registers for Port C
	PTC->PDDR |= (MASK(GREEN_PTC4) | MASK(GREEN_PTC5) | MASK(GREEN_PTC6) | MASK(GREEN_PTC10) | MASK(GREEN_PTC11) | 
								MASK(GREEN_PTC12) | MASK(GREEN_PTC13) | MASK(GREEN_PTC16) | MASK(RED_PTC7));
}


void greenLED_off(void) {
	PTC->PCOR = (MASK(GREEN_PTC4) | MASK(GREEN_PTC5) | MASK(GREEN_PTC6) | MASK(GREEN_PTC10) | MASK(GREEN_PTC11) | 
							 MASK(GREEN_PTC12) | MASK(GREEN_PTC13) | MASK(GREEN_PTC16));
}

void redLED_off(void) {
	PTC->PCOR = MASK(RED_PTC7);
}

// Bot moving -> green LEDs must be running 1 LED at a time
void greenLED_moving(void) {
	const uint32_t GREEN_LEDs_MASK[] = {
		MASK(GREEN_PTC4), MASK(GREEN_PTC5), MASK(GREEN_PTC6), MASK(GREEN_PTC10), MASK(GREEN_PTC11), MASK(GREEN_PTC12),
		MASK(GREEN_PTC13), MASK(GREEN_PTC16)
	};
	
	const int NUM_LEDs = sizeof(GREEN_LEDs_MASK) / sizeof(GREEN_LEDs_MASK[0]);
	
	for (int i = 0; i < NUM_LEDs; i++) {
		greenLED_off(); // Turn off all LEDs before lighting the next
		PTC->PSOR = GREEN_LEDs_MASK[i]; // turn on one LED at a time
		osDelay(100); // wait for a while
		PTC->PCOR = GREEN_LEDs_MASK[i]; // turn off the LED
	}
}

// Bot stationary -> all Green LEDs should light up together
void greenLED_stationary(void) {
	greenLED_off(); // Ensure all LEDs are off first
	PTC->PSOR = (MASK(GREEN_PTC4) | MASK(GREEN_PTC5) | MASK(GREEN_PTC6) | MASK(GREEN_PTC10) | MASK(GREEN_PTC11) | 
				 MASK(GREEN_PTC12) | MASK(GREEN_PTC13) | MASK(GREEN_PTC16)); // Turn on all LEDs
}

// Bot moving -> all Red LEDs should be flashing continuously at rate of 500ms
void redLED_moving(void) {
	PTC->PSOR = MASK(RED_PTC7); // Turn on the red LED
	osDelay(500); // LED on for 500ms
	PTC->PCOR = MASK(RED_PTC7); // Turn off the red LED
	osDelay(500); // LED off for 500ms
}

// Bot stationary -> all Red LEDs should be flashing at rate of 250ms
void redLED_stationary(void) {
	PTC->PSOR = MASK(RED_PTC7); // Turn on the red LED
	osDelay(250); // LED on for 250ms
	PTC->PCOR = MASK(RED_PTC7); // Turn off the red LED
	osDelay(250); // LED off for 250ms
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

/* Init 50 Hz rising edge PWM for PORTB TPM0,1 CH0,1 */
void InitPWM(void){
	// Enable Clock Gating for PORTB
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// Configure MODE 3 for PWM TPM (163)
	PORTB->PCR[MOTOR_BACK_LEFT] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[MOTOR_BACK_LEFT] |= PORT_PCR_MUX(3); 
	
	PORTB->PCR[MOTOR_BACK_RIGHT] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[MOTOR_BACK_RIGHT] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[MOTOR_FRONT_LEFT] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[MOTOR_FRONT_LEFT] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[MOTOR_FRONT_RIGHT] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[MOTOR_FRONT_RIGHT] |= PORT_PCR_MUX(3);
	
	// Enable Clock gating for TPM1,2 (207)
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
	// Select Clock for TPM module
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // internal clk
	
	// init PWM for TIMER 1
	// up-counting, prescaler = 128 (553)
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= TPM_SC_CMOD(1) | TPM_SC_PS(7); 
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// TPM1_CH0 ie MOTOR_BACK_LEFT
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) |(TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Edge align PWM, high-true pulses: BABA = 1010 (555)
	// TPM1_CH1 ie MOTOR_BACK_RIGHT
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) |(TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));	
	
	// init PWM for TIMER 2
	// up-counting, prescaler = 128 (553)
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= TPM_SC_CMOD(1) | TPM_SC_PS(7); 
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// TPM2_CH0 ie MOTOR_FRONT_LEFT
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) |(TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	// TPM2_CH1 ie MOTOR_FRONT_RIGHT
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) |(TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Set Modulo Value 48_000_000(48MHz) / 128 = 375_000 / 7500 = 50 Hz (Clk / MOD = freq) (554)
	TPM1->MOD = MOD_VAL;
	TPM2->MOD = MOD_VAL;
}

void motor_left_forward() {
	TPM2_C0V = 0x0;
	TPM2_C1V = FULL_MOD;
}
void motor_left_stop() {
	TPM2_C0V = 0x0;
	TPM2_C1V = 0x0;
}
void motor_right_forward() {
	TPM1_C0V = 0x0;
	TPM1_C1V = FULL_MOD;
}
void motor_right_stop() {
	TPM1_C0V = 0x0;
	TPM1_C1V = 0x0;
}

void run_motor() {
	// For reference:
	// Left wheels, AIN1: PTB3, TPM2_CH1
	// Left wheels, AIN2: PTB2, TPM2_CH0
	// Right wheels, AIN1: PTB1, TPM1_CH1
	// Right wheels, AIN2: PTB0, TPM1_CH0
	
	switch(rx_data){
	case STOP: // Stationary
		TPM2_C1V = TPM2_C0V = TPM1_C1V = TPM1_C0V = 0x0;
		is_moving = false;
		break;
	case FORWARD: // Move forward in straight line
		motor_left_forward();
		motor_right_forward();	
	
		is_moving = true;
		break;

	case FRONT_LEFT: // Turn left	// NOT WORKING
		// Configure left wheels
		TPM2_C0V = 0x0;
		TPM2_C1V = 0x0;

		// Configure right wheels
		TPM1_C0V = 0x0;
		TPM1_C1V = FULL_MOD;
		test_var = TPM1_C1V;
	
		is_moving = true;
		break;
	
	case FRONT_RIGHT: // Turn right // NOT WORKING
		// Configure left wheels
		TPM2_C0V = 0x0;
		TPM2_C1V = FULL_MOD;

		// Configure right wheels
		TPM1_C0V = 0x0;
		TPM1_C1V = 0x0;	
	
		is_moving = true;
		break;

		
	case BACKWARD: // Reverse in straight line
		// Configure left wheels
		TPM2_C1V = 0x0;
		TPM2_C0V = HALF_MOD;
		// Configure right wheels
		TPM1_C1V = 0x0;
		TPM1_C0V = HALF_MOD;
	
		is_moving = true;
		break; 
	
	case LEFT: // pivot L
		// left wheels reverse
		TPM2_C1V = 0x0;
		TPM2_C0V = HALF_MOD;
		// right wheels forward
		TPM1_C0V = 0x0;
		TPM1_C1V = HALF_MOD;
	
		is_moving = true;
		break;
	
	case RIGHT: // pivot R
		// left wheels forward
		TPM2_C0V = 0x0;
		TPM2_C1V = HALF_MOD;
		// right wheels reverse
		TPM1_C1V = 0x0;
		TPM1_C0V = HALF_MOD;
	
		is_moving = true;
		break;
	
	case REVERSE_LEFT: // Reverse in straight line
	// Configure left wheels
	TPM2_C1V = 0x0;
	TPM2_C0V = HALF_MOD;
	// Configure right wheels
	TPM1_C1V = 0x0;
	TPM1_C0V = FULL_MOD;

	is_moving = true;
	break; 
	
	case REVERSE_RIGHT: // Reverse in straight line
	// Configure left wheels
	TPM2_C1V = 0x0;
	TPM2_C0V = FULL_MOD;
	// Configure right wheels
	TPM1_C1V = 0x0;
	TPM1_C0V = HALF_MOD;

	is_moving = true;
	break; 
		
	default:
		break;
	}
}

void InitAudio(void){
	// Enable Clock Gating for PORTD (on power)
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	// Configure MODE 3 for PWM TPM (Chpt 10 163) // choose PWM TPM module
	PORTD->PCR[PTD0] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0] |= PORT_PCR_MUX(4); // ALT4
	
	// Enable Clock gating for TPM1 (Search SCGC6 207)
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; //0x2
	
	// Select Clock for TPM module
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // internal clk
		
	/* Edge-aligned PWM */
	// Update SnC register : CMOD = 01(up counting), PS = 111 (Prescalar 128) Chapter 31 Timer (553)
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= TPM_SC_CMOD(1) | TPM_SC_PS(7);
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK); // disable center aligned PWM
	
	// Enable PWM on TPM0 Channel 0 -> PTB0 Edge Align PWM 1010 (555)
	TPM0_C0SC &= ~ ((TPM_CnSC_ELSB_MASK) |(TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // BABA = 1010
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

// change FREQUENCY by changing MOD
// change VOLUME by changing CnV (DUTY CYCLE)
void play_continuous_song() {
    uint16_t freq[] = { 
			note_C1, note_B, note_A, note_F,
			note_G, note_G, note_C1, 
			note_B,	note_A, 
			note_A, note_A, note_C1,
			note_B, note_A, note_G, 
			note_B1, note_A1, note_B1, note_A1, 
			note_B1, note_G, note_G, 
			note_B1, note_A1, note_B1, note_A1, note_B1};
	uint16_t length[] = { 1, 1, 1, 1,
						2, 1, 1,
						2, 2,
						1, 1, 2,
						1, 1, 2,
						1, 1, 1, 1,
						1, 2, 1,
						1, 1, 1, 1, 1};
    int continuousSongLength = sizeof(freq) / sizeof(uint16_t);
	
	for (int i = 0; i < continuousSongLength; i++) {
		TPM0->MOD = CLOCK / freq[i];
		TPM0_C0V = (CLOCK / freq[i]) / 8; // Set duty cycle to 12.5%
		//osDelay(0x3F000 * length[i]);
		osDelay(100 * length[i]);
		TPM0_C0V = 0x0;
		//osDelay(0x3F000);
		osDelay(100);
    }	
}

void play_ending_song() {
    uint16_t freq[] = { 
			note_C, note_D, note_E, note_F,
			note_G, note_A, note_B, note_C1,
			note_B, note_A, note_G, note_F,
			note_E, note_D, note_C
	};		
    int continuousSongLength = sizeof(freq) / sizeof(uint16_t);

	for (int i = 0; i < continuousSongLength; i++) {
		
		TPM0->MOD = CLOCK / freq[i];
		TPM0_C0V = (CLOCK / freq[i]) / 8; // Set duty cycle to 12.5%
		delay(0x1F000);
		//osDelay(100);
		TPM0_C0V = 0x0;
		delay(0x1F000);
		//osDelay(100);
	}


	while(1) {
	}
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
		ledControl(red_led, led_on);
		
		if (FUNCTIONBITSMASK(rx_data) == 0x00) {
			osSemaphoreRelease(motorSem);
		} 
		else if (FUNCTIONBITSMASK(rx_data) == 0x10) {
			osSemaphoreRelease(finishMusicSem);
		}
		offRGB();
	}
}


/* Motor Thread */
void motor_thread (void *argument) {
	for (;;) {
		osSemaphoreAcquire(motorSem, osWaitForever);
		ledControl(blue_led, led_on);

		// TODO: remove push btn interrupt
		
		// include motor move code/function
		run_motor();
		
		offRGB();
	}
}


/* LED Green Thread */
void led_green_thread (void *argument) {
	for (;;) {
		//osSemaphoreAcquire(ledSem, osWaitForever);
		
		if (is_moving) {
			greenLED_moving();
		}
		else {
			greenLED_stationary();
		}
		//Testing
		//ledControl(red_led, led_on);
		//delay(0x80000);
	}
}


/* LED Red Thread */
void led_red_thread (void *argument) {
	for (;;) {
		//osSemaphoreAcquire(ledSem, osWaitForever);
		
		if (is_moving) {
			redLED_moving();
		}
		else {
			redLED_stationary();
		}
	}
}


/* Background Continuous Music Thread */
void bg_music_thread (void *argument) {
	for (;;) {
		osSemaphoreAcquire(bgMusicSem, osWaitForever);
		
		// code to play continuous music
		play_continuous_song();
		osSemaphoreRelease(bgMusicSem);	// stop playing background music
	}
}

/* Finish Music Thread */
void finish_music_thread (void *argument) {
	
	osSemaphoreAcquire(finishMusicSem, osWaitForever);
	//osSemaphoreAcquire(bgMusicSem, osWaitForever);
	
	// code to play finish music
	play_ending_song();
	osSemaphoreAcquire(bgMusicSem, osWaitForever);
	osSemaphoreRelease(finishMusicSem);	// stop playing background music
}


/* MAIN function */
int main(void){
	SystemCoreClockUpdate();
	//InitPWM();
	//InitSwitch();
	InitGPIO();
	initUART2(BAUD_RATE);
	InitPWM();
	InitAudio();
	
	osKernelInitialize();	// Initialize CMSIS-RTOS
	
	brainSem = osSemaphoreNew(1, 0, NULL);
	motorSem = osSemaphoreNew(1, 0, NULL);
	//ledSem = osSemaphoreNew(1, 1, NULL);     // CHECK if led sem needs to be blocked initially*****
	bgMusicSem = osSemaphoreNew(1, 1, NULL); 			// Continuous background music plays immediately
	finishMusicSem = osSemaphoreNew(1, 0, NULL);	// Music to be played at the end only upon receiving the command, initially blocked

	// DUTY CYCLE: change VOLUME + can generate different PWM simulataneously (557)
	// completely HARDWARE driven
	
	osThreadNew(brain_thread, NULL, &priorityHigh);	// brain thread should have the highest priority to control the others
	osThreadNew(motor_thread, NULL, NULL);
	osThreadNew(led_green_thread, NULL, NULL);
	osThreadNew(led_red_thread, NULL, NULL);
	osThreadNew(bg_music_thread, NULL, NULL);
	osThreadNew(finish_music_thread, NULL, &priorityAboveNormal);
	
	//ledControl(global_led, led_on);
	osKernelStart();
}
