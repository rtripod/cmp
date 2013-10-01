#include "msp430fr5739.h"
#include <stdio.h>

#define LINE_SENSOR_MAX 300
#define FORCE_SENSOR_MAX 900

#define FINISHED_OPERATION 1
#define CONTINUE_OPERATION 0

typedef enum {STATE_START, STATE_POST_FERRIS, STATE_DUCKS, STATE_STRENGTH, STATE_FINISH} STATE;

void state_machine(STATE *, int);
void state_start(int *);
void state_post_ferris(int *);
void state_ducks(int *);
void state_strength(int *);
void state_finish(int *);

volatile unsigned int ADCResult;

void SetupADC(unsigned int in_bit)
{
	// Configure ADC
	P1SEL1 |= in_bit;  
	P1SEL0 |= in_bit; 

	// Allow for settling delay 
	__delay_cycles(50000);

	// Configure ADC
	ADC10CTL0 &= ~ADC10ENC; 
	ADC10CTL0 = ADC10SHT_7 + ADC10ON;        // ADC10ON, S&H=192 ADC clks
	// ADCCLK = MODOSC = 5MHz
	ADC10CTL1 = ADC10SHS_0 + ADC10SHP + ADC10SSEL_0; 
	ADC10CTL2 = ADC10RES;                    // 10-bit conversion results
	
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
	
	ADC10IE = ADC10IE0;                      // Enable ADC conv complete interrupt

	// Setup Thresholds for relative difference in Thermistor measurements
}

unsigned int CalibrateADC(void)
{
	unsigned char CalibCounter =0;
	unsigned int Value = 0;

	// Disable interrupts & user input during calibration

	while(CalibCounter <50)
	{
		P3OUT ^= BIT4;
		CalibCounter++;
		while (ADC10CTL1 & BUSY); 
		ADC10CTL0 |= ADC10ENC | ADC10SC ;       // Start conversion 
		__bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
		__no_operation(); 
		Value += ADCResult;
	}
	Value = Value/50;
	// Reenable switches after calibration
	return Value;
}

void TakeADCMeas(void)
{  
	while (ADC10CTL1 & BUSY); 
	ADC10CTL0 |= ADC10ENC | ADC10SC ;       // Start conversion 
	__bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
	__no_operation();                       // For debug only
}

int main(void)
{
	WDTCTL = WDTPW + WDTHOLD;
	// Init SMCLK = MCLk = ACLK = 1MHz
	CSCTL0_H = 0xA5;
	CSCTL1 |= DCOFSEL0 + DCOFSEL1;          // Set max. DCO setting = 8MHz
	CSCTL2 = SELA_3 + SELS_3 + SELM_3;      // set ACLK = MCLK = DCO
	CSCTL3 = DIVA_3 + DIVS_3 + DIVM_3;      // set all dividers to 1MHz
	
	// P1.4 is used as input from NTC voltage divider
	// Set it to output low
	P1OUT &= ~BIT4;      
	P1DIR |= BIT4; 
	
	PJDIR |= (BIT0+BIT1+BIT2+BIT3);
	P3DIR |= (BIT4+BIT5+BIT6+BIT7);
	
	PJOUT &= ~(BIT0+BIT1+BIT2+BIT3);
	P3OUT &= ~(BIT4+BIT5+BIT6+BIT7);
	
	STATE state = STATE_START;
	int operation;
	while(1)
	{
		PJOUT &= ~(BIT0+BIT1+BIT2+BIT3);
		P3OUT &= ~(BIT4+BIT5+BIT6+BIT7);
		switch(state)
		{
			case STATE_START:
				PJOUT |= BIT0;
				__delay_cycles(50000);
				state_start(&operation);
				break;
			case STATE_POST_FERRIS:
				PJOUT |= BIT1;
				__delay_cycles(50000);
				state_post_ferris(&operation);
				break;
			case STATE_DUCKS:
				PJOUT |= BIT2;
				__delay_cycles(50000);
				state_ducks(&operation);
				break;
			case STATE_STRENGTH:
				PJOUT |= BIT3;
				__delay_cycles(50000);
				state_strength(&operation);
				break;
			case STATE_FINISH:
				P3OUT |= BIT4;
				__delay_cycles(50000);
				state_finish(&operation);
				break;
		}
		__delay_cycles(50000);
		state_machine(&state, operation);
	}
}

void state_machine(STATE *state, int operation)
{
	switch(*state)
	{
		case STATE_START:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = STATE_POST_FERRIS;
					break;
			}
			break;
		case STATE_POST_FERRIS:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = STATE_DUCKS;
					break;
			}
			break;
		case STATE_DUCKS:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = STATE_STRENGTH;
					break;
			}
			break;
		case STATE_STRENGTH:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = STATE_FINISH;
					break;
			}
			break;
		case STATE_FINISH:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = STATE_START;
					break;
			}
			break;
	}
}

void state_start(int *operation)
{
	__delay_cycles(50000);
	SetupADC(BIT0);
	TakeADCMeas();
	if(ADCResult < FORCE_SENSOR_MAX)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_post_ferris(int *operation)
{
	__delay_cycles(50000);
	SetupADC(BIT1);
	TakeADCMeas();
	if(ADCResult < FORCE_SENSOR_MAX)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_ducks(int *operation)
{
	__delay_cycles(50000);
	SetupADC(BIT2);
	TakeADCMeas();
	if(ADCResult < LINE_SENSOR_MAX)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_strength(int *operation)
{
	__delay_cycles(50000);
	SetupADC(BIT3);
	TakeADCMeas();
	if(ADCResult < LINE_SENSOR_MAX)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_finish(int *operation)
{
	__delay_cycles(50000);
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
		case ADC10IV_NONE: break;               // No interrupt
		case ADC10IV_ADC10OVIFG: break;         // conversion result overflow
		case ADC10IV_ADC10TOVIFG: break;        // conversion time overflow
		case ADC10IV_ADC10HIIFG: break;         // ADC10HI
		case ADC10IV_ADC10LOIFG: break;         // ADC10LO
		case ADC10IV_ADC10INIFG: break;         // ADC10IN
		case ADC10IV_ADC10IFG: 
			ADCResult = ADC10MEM0;
			__bic_SR_register_on_exit(CPUOFF);                                              
			break;                          // Clear CPUOFF bit from 0(SR)                         
		default: break;
	}
}
