#include "RTE_Components.h"
#include  CMSIS_device_header
#include "MKL25Z4.h"                    // Device header


/*
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

#define MASK(x)      (1 << (x))

//volatile int bot_move = 1;

// Setup GPIO Pins for the following
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
*/
