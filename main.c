#include "msp430fr5739.h"
#include <stdio.h>

#define LINE_SENSOR_MAX 300
#define FORCE_SENSOR_MAX 900

volatile unsigned int ADCResult;

void SetupADC(void)
{
	// Configure ADC
	P1SEL1 |= BIT4;  
	P1SEL0 |= BIT4; 

	// Allow for settling delay 
	__delay_cycles(50000);

	// Configure ADC
	ADC10CTL0 &= ~ADC10ENC; 
	ADC10CTL0 = ADC10SHT_7 + ADC10ON;        // ADC10ON, S&H=192 ADC clks
	// ADCCLK = MODOSC = 5MHz
	ADC10CTL1 = ADC10SHS_0 + ADC10SHP + ADC10SSEL_0; 
	ADC10CTL2 = ADC10RES;                    // 10-bit conversion results
	ADC10MCTL0 = ADC10INCH_4;                // A4 ADC input select; Vref=AVCC
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

/**********************************************************************//**
* @brief  Take ADC Measurement
* 
* @param  none 
*  
* @return none
*************************************************************************/
void TakeADCMeas(void)
{  
	while (ADC10CTL1 & BUSY); 
	ADC10CTL0 |= ADC10ENC | ADC10SC ;       // Start conversion 
	__bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
	__no_operation();                       // For debug only
}

/**********************************************************************//**
* @brief  Initializes Accelerometer
* 
* @param  none 
*  
* @return none
*************************************************************************/
void main(void)
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
	PJDIR |= (BIT0+BIT1);

	SetupADC();
	//unsigned int compare = CalibrateADC();
	while(1)
	{      
	TakeADCMeas();
     
     //printf("%d \n", ADCResult);

/*	if( ADCResult < LINE_SENSOR_MAX )
		PJOUT &= ~BIT0;
	else
		PJOUT |= BIT0;*/
     
	if( ADCResult < FORCE_SENSOR_MAX )
		PJOUT &= ~BIT1;
	else
		PJOUT |= BIT1;  
   }
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
