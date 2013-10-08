#include "msp430fr5739.h"
#include <stdio.h>

#define LINE_SENSOR_MAX 300
#define FORCE_SENSOR_MAX 900

#define STATE_DIR P3DIR
#define STATE_OUT P3OUT
#define STATE_LED0 BIT4
#define STATE_LED1 BIT5
#define STATE_LED2 BIT6
#define STATE_LED3 BIT7

#define GEARED_DIR P2DIR
#define GEARED_OUT P2OUT
#define GEARED_PIN BIT0

#define FINISHED_OPERATION 0
#define CONTINUE_OPERATION 1

typedef enum {IDLE, FERRIS, PRE_DUCKS, DUCKS, STRENGTH, FINISH} STATE;

void state_machine(STATE *, unsigned char);
void state_idle(unsigned char *);
void state_ferris(unsigned char *);
void state_pre_ducks(unsigned char *);
void state_ducks(unsigned char *);
void state_strength(unsigned char *);

volatile unsigned int ADCResult;

void delayMillis(unsigned int delay)
{
	while (delay--)
	{
		__delay_cycles(1000);	// CPU clock speed divided by 1000
	}
}

void updateFPGASignal(unsigned int in_bit)
{
	switch (in_bit)
	{
		case 0:		// 000
			STATE_OUT &= ~(STATE_LED0+STATE_LED1+STATE_LED2);
			break;
		case 1:		// 001
			STATE_OUT |= (STATE_LED0);
			STATE_OUT &= ~(STATE_LED1+STATE_LED2);
			break;
		case 2:		// 010
			STATE_OUT |= (STATE_LED1);
			STATE_OUT &= ~(STATE_LED0+STATE_LED2);
			break;
		case 3:		// 011
			STATE_OUT |= (STATE_LED0+STATE_LED1);
			STATE_OUT &= ~(STATE_LED2);
			break;
		case 4:		// 100
			STATE_OUT |= (STATE_LED2);
			STATE_OUT &= ~(STATE_LED0+STATE_LED1);
			break;
		case 5:		// 101
			STATE_OUT |= (STATE_LED0+STATE_LED2);
			STATE_OUT &= ~(STATE_LED1);
			break;
		case 6:		// 110
			STATE_OUT |= (STATE_LED1+STATE_LED2);
			STATE_OUT &= ~(STATE_LED0);
			break;
		case 7:		// 111
			STATE_OUT |= (STATE_LED0+STATE_LED1+STATE_LED2);
			break;
		default: break;
	}
}

void SetupADC(unsigned int in_bit)
{
	// Configure ADC
	P1SEL1 |= in_bit;  
	P1SEL0 |= in_bit; 

	// Allow for settling delay 
	delayMillis(50);

	// Configure ADC
	ADC10CTL0 &= ~ADC10ENC; 
	ADC10CTL0 = ADC10SHT_7 + ADC10ON;		// ADC10ON, S&H=192 ADC clks
	ADC10CTL1 = ADC10SHS_0 + ADC10SHP + ADC10SSEL_0; 
	ADC10CTL2 = ADC10RES;					// 10-bit conversion results
	
	switch (in_bit)
	{
		case BIT0:
			ADC10MCTL0 = ADC10INCH_0;
			break;
		case BIT1:
			ADC10MCTL0 = ADC10INCH_1;
			break;
		case BIT2:
			ADC10MCTL0 = ADC10INCH_2;
			break;
		case BIT3:
			ADC10MCTL0 = ADC10INCH_3;
			break;
		case BIT4:
			ADC10MCTL0 = ADC10INCH_4;
			break;
	}	
	
	ADC10IE = ADC10IE0;						// Enable ADC conv complete interrupt
}

void TakeADCMeas(void)
{  
	while (ADC10CTL1 & BUSY); 
	ADC10CTL0 |= ADC10ENC | ADC10SC ;		// Start conversion 
	__bis_SR_register(CPUOFF + GIE);		// LPM0, ADC10_ISR will force exit
	__no_operation();						// For debug only
}

int main(void)
{
	WDTCTL = WDTPW + WDTHOLD;
	// Init SMCLK = MCLk = ACLK = 1MHz
	CSCTL0_H = 0xA5;						// Password
	CSCTL1 |= DCOFSEL0 + DCOFSEL1;			// Set max. DCO setting = 8MHz
	CSCTL2 = SELA_3 + SELS_3 + SELM_3;		// set ACLK = MCLK = DCO/8
	CSCTL3 = DIVA_3 + DIVS_3 + DIVM_3;		// set all dividers to 1MHz
	
	// P1.4 is used as input from NTC voltage divider
	// Set it to output low
	P1OUT &= ~BIT4;      
	P1DIR |= BIT4; 
	
	// Setup FPGA state signal
	STATE_DIR |= (BIT4+BIT5+BIT6+BIT7);
	STATE_OUT &= ~(BIT4+BIT5+BIT6+BIT7);
	
	// Setup geared motor
	GEARED_DIR |= GEARED_PIN;
	GEARED_OUT &= ~GEARED_PIN;
	
	// Initial state
	STATE state = IDLE;
	updateFPGASignal(0);
	SetupADC(BIT0);
	
	unsigned char operation;
	while(1)
	{
		switch(state)
		{
			case IDLE:
				delayMillis(50);
				state_idle(&operation);
				break;
			case FERRIS:
				delayMillis(50);
				state_ferris(&operation);
				break;
			case PRE_DUCKS:
				delayMillis(50);
				state_pre_ducks(&operation);
				break;
			case DUCKS:
				delayMillis(50);
				state_ducks(&operation);
				break;
			case STRENGTH:
				delayMillis(50);
				state_strength(&operation);
				break;
		}
		delayMillis(50);
		state_machine(&state, operation);
	}
}

void state_machine(STATE *state, unsigned char operation)
{
	switch(*state)
	{
		case IDLE:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = FERRIS;
					updateFPGASignal(1);
					SetupADC(BIT1);
					P2OUT |= BIT0;		// Turn on geared motor
					break;
				default: break;
			}
			break;
		case FERRIS:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = PRE_DUCKS;
					updateFPGASignal(2);
					SetupADC(BIT2);
					break;
				default: break;
			}
			break;
		case PRE_DUCKS:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = DUCKS;
					updateFPGASignal(3);
					SetupADC(BIT3);
					break;
				default: break;
			}
			break;
		case DUCKS:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = STRENGTH;
					updateFPGASignal(4);
					SetupADC(BIT4);
					break;
				default: break;
			}
			break;
		case STRENGTH:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = IDLE;
					updateFPGASignal(0);
					SetupADC(BIT0);
					P2OUT &= ~BIT0;		// Turn off geared motor
					break;
				default: break;
			}
			break;
	}
}

void state_idle(unsigned char *operation)
{
	
	delayMillis(50);
	TakeADCMeas();
	if(ADCResult < FORCE_SENSOR_MAX)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_ferris(unsigned char *operation)
{
	delayMillis(50);
	SetupADC(BIT1);
	TakeADCMeas();
	if(ADCResult < FORCE_SENSOR_MAX)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_pre_ducks(unsigned char *operation)
{
	delayMillis(50);
	SetupADC(BIT2);
	TakeADCMeas();
	if(ADCResult < FORCE_SENSOR_MAX)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_ducks(unsigned char *operation)
{
	delayMillis(50);
	SetupADC(BIT3);
	TakeADCMeas();
	if(ADCResult < FORCE_SENSOR_MAX)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_strength(unsigned char *operation)
{
	delayMillis(50);
	SetupADC(BIT4);
	TakeADCMeas();
	if(ADCResult < FORCE_SENSOR_MAX)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
	switch(__even_in_range(ADC10IV,ADC10IV_ADC10IFG))
	{
		case ADC10IV_NONE: break;				// No interrupt
		case ADC10IV_ADC10OVIFG: break;			// conversion result overflow
		case ADC10IV_ADC10TOVIFG: break;		// conversion time overflow
		case ADC10IV_ADC10HIIFG: break;			// ADC10HI
		case ADC10IV_ADC10LOIFG: break;			// ADC10LO
		case ADC10IV_ADC10INIFG: break;			// ADC10IN
		case ADC10IV_ADC10IFG: 
			ADCResult = ADC10MEM0;
			__bic_SR_register_on_exit(CPUOFF);                                              
			break;								// Clear CPUOFF bit from 0(SR)                         
		default: break;
	}
}
