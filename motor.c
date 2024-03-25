#include "MKL25Z4.h"        // Device header
#include "system_MKL25Z4.h" 
#define MASK(x)     (1 << (x))

#define SW_POS		6		// PORTD Pin 6 (INT)

#define PTB0_Pin 0 // for PWM TPM1 CH0 (PWM)
#define PTB1_Pin 1

#define CLOCK (48000000 / 128) // 375000 (AUDIO PWM)
#define note_C (uint16_t)(CLOCK / 262)
#define note_D (uint16_t)(CLOCK / 294)
#define note_E (uint16_t)(CLOCK / 330)
#define note_F (uint16_t)(CLOCK / 349)
#define note_G (uint16_t)(CLOCK / 392)
#define note_A (uint16_t)(CLOCK / 440)
#define note_B (uint16_t)(CLOCK / 494)

#define BAUD_RATE 9600 
#define UART_TX_PORTE22 22 // UART2 (162)
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

/* Delay Function */
static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}

/* Init 50 Hz rising edge PWM for PORTB TPM1 CH0,1 */
void InitPWM(void){
	// Enable Clock Gating for PORTB
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// Configure MODE 3 for PWM TPM (163)
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3); 
	
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	// Enable Clock gating for TPM1 (207)
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	// Select Clock for TPM module
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // internal clk
	
	// Set Modulo Value 48_000_000(48MHz) / 128 = 375_000 / 7500 = 50 Hz (Clk / MOD = freq) (554)
	TPM1->MOD = 7500;
	//TPM1->MOD = 5000;
	
	/* Edge-aligned PWM */
	// Update SnC register : CMOD = 01(up counting), PS = 111 (Prescalar 128) Chapter 31 Timer (553)
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= TPM_SC_CMOD(1) | TPM_SC_PS(7);
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Enable PWM on TPM1 Channel 0 -> PTB0 Edge Align PWM 1010 (555)
	TPM1_C0SC &= ~ ((TPM_CnSC_ELSB_MASK) |(TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on TPM1 Channel 1-> PTB0
	TPM1_C1SC &= ~ ((TPM_CnSC_ELSB_MASK) |(TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));		
}

// to select LED for INT code
volatile unsigned int counter = 0;

/* INT code for PORTD */
void PORTD_IRQHandler()
{
	// Clear Pending IRQ
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	
	// Updating some variable / flag
	counter++;
	if(counter >= 3) {
			counter = 0;
	}		
	delay(0x80000); // debouncing
	
	//Clear INT Flag: set 1 to clear
	PORTD->ISFR |= MASK(SW_POS);
}

/* Init input, enable interrupt for switch at PORTD */
void InitSwitch(void)
{
	// enable clock for PortD
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	/* Select GPIO and enable pull-up resistors and interrupts on 
	falling edges of pin connected to switch*/
	PORTD->PCR[SW_POS] |= (PORT_PCR_MUX(1)	| //set GPIO
						PORT_PCR_PS_MASK	| 
						PORT_PCR_PE_MASK	|
						PORT_PCR_IRQC(0x0a)); //INT configuration
	
	// Set PORT D Switch bit to INPUT
	PTD->PDDR &= ~MASK(SW_POS);
	
	//Enable Interrupts
	NVIC_SetPriority(PORTD_IRQn, 2); // highest priority lvl = 0
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	NVIC_EnableIRQ(PORTD_IRQn);
}

/* MAIN function*/
int main(void)
{
	SystemCoreClockUpdate();
	InitPWM();
	InitSwitch();

	// DUTY CYCLE: change VOLUME + can generate different PWM simulataneously (557)
	// completely HARDWARE driven
	// TPM1_C0V = 0xEA6; // 0xEA6 = 3750 (half of 7500) -> 50% duty cycle 
	TPM1_C1V = 0x753; // half of 3750
	
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


