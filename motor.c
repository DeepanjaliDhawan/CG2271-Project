/**
#include "MKL25Z4.h"        // Device header
#include "system_MKL25Z4.h" 
#define MASK(x)     (1 << (x)) 

#define SW_POS		6		// PORTD Pin 6: for temporary push btn Interrupt

#define MOTOR_BACK_LEFT		0	// PTB0 TPM1_CH0
#define MOTOR_BACK_RIGHT 	1	// PTB1 TPM1_CH1
#define MOTOR_FRONT_LEFT	2	// PTB2 TPM2_CH0
#define MOTOR_FRONT_RIGHT 	3	// PTB3 TPM2_CH1

#define DIRECTIONS 6
#define MOD_VAL 7500
#define FULL_MOD (MOD_VAL)			// 7500	// for actual run
#define QUARTER_MOD (MOD_VAL / 4)		// 1875 // for test runs
#define TEST_MOD (MOD_VAL / 8)			// 937	// for test runs

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

/**
// Delay Function
static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}

// Init 50 Hz rising edge PWM for PORTB TPM0,1 CH0,1
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


// to select LED for INT code
volatile unsigned int counter = 0;

// INT code for PORTD
void PORTD_IRQHandler()
{
	// Clear Pending IRQ
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	
	// Updating some variable / flag
	counter++;
	if(counter > DIRECTIONS) {
			counter = 0;
	}		
	delay(0x80000); // debouncing
	
	//Clear INT Flag: set 1 to clear
	PORTD->ISFR |= MASK(SW_POS);
}

// Temp: Init input, enable interrupt for switch at PORTD

void InitSwitch(void)
{
	// enable clock for PortD
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	// Select GPIO and enable pull-up resistors and interrupts on 
	falling edges of pin connected to switch
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

void run_motor() {
	// For reference:
	// Left wheels, AIN1: PTB3, TPM2_CH1
	// Left wheels, AIN2: PTB2, TPM2_CH0
	// Right wheels, AIN1: PTB1, TPM1_CH1
	// Right wheels, AIN2: PTB, TPM1_CH0

	switch(counter){
	case 0: // Stationary
		TPM2_C1V = TPM2_C0V = TPM1_C1V = TPM1_C0V = 0x0;
		break;
	case 1: // Move forward in straight line
		// Configure left wheels
		TPM2_C0V = 0;
		TPM2_C1V = TEST_MOD;

		// Configure right wheels
		TPM1_C0V = 0;
		TPM1_C1V = TEST_MOD;	
		break;

	case 2: // Turn left
		// Configure left wheels
		TPM2_C0V = 0;
		TPM2_C1V = TEST_MOD;

		// Configure right wheels
		TPM1_C0V = 0;
		TPM1_C1V = QUARTER_MOD;
		break;
	
	case 3: // Turn right
		// Configure left wheels
		TPM2_C0V = 0;
		TPM2_C1V = QUARTER_MOD;
		
		// Configure right wheels
		TPM1_C0V = 0;
		TPM1_C1V = TEST_MOD;
		break;
	
	case 4: // Reverse in straight line
		// Configure left wheels
		TPM2_C1V = 0;
		TPM2_C0V = TEST_MOD;
		// Configure right wheels
		TPM1_C1V = 0;
		TPM1_C0V = TEST_MOD;
		break; // use H bridge or what ?
	case 5: // pivot L
		// left wheels reverse
		TPM2_C1V = 0;
		TPM2_C0V = TEST_MOD;
		// right wheels forward
		TPM1_C0V = 0;
		TPM1_C1V = TEST_MOD;
	

		break;
	case 6: // pivot R
		// left wheels forward
		TPM2_C0V = 0;
		TPM2_C1V = TEST_MOD;
		// right wheels reverse
		TPM1_C1V = 0;
		TPM1_C0V = TEST_MOD;
	default:
		break;
	}
}

// MAIN function

int main(void)
{
	SystemCoreClockUpdate();
	InitPWM();
	InitSwitch();
	
	while(1)
	{
		
		// Run the motors
		run_motor();
		
		// AUDIO
		uint16_t MOD_VALUE[] = {note_C, note_D, note_E, note_F, note_G, note_A, note_B};
		for (int i = 0; i < 7; i++){
			// change FREQUENCY by changing MOD
			TPM1->MOD = MOD_VALUE[i];
			delay(0xFF000);
		}
		
	}
}
**/

