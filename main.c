#include "main.h"

void delayMillis(unsigned int delay)
{
	while (delay--)
	{
		__delay_cycles(1000);	// CPU clock speed divided by 1000
	}
}

// duty_cycle range: 700 - 2300
void pwmControl(unsigned char in_component, int duty_cycle)
{
	switch (in_component)
	{
		case DUCK1_SERVO:
			TA0CCR0 = SERVO_PERIOD;						// PWM Period
			TA0CCTL1 = OUTMOD_7;						// CCR1 reset/set
			TA0CCR1 = duty_cycle;						// CCR1 PWM duty cycle
			TA0CTL = TASSEL_2 + MC_1 + TACLR;			// SMCLK, up mode, clear TAR
			break;
		case DUCK2_SERVO:
			TA1CCR0 = SERVO_PERIOD;						// PWM Period
			TA1CCTL1 = OUTMOD_7;						// CCR1 reset/set
			TA1CCR1 = duty_cycle;						// CCR1 PWM duty cycle
			TA1CTL = TASSEL_2 + MC_1 + TACLR;			// SMCLK, up mode, clear TAR
			break;
		case DUCK3_SERVO:
			TB0CCR0 = SERVO_PERIOD;						// PWM Period
			TB0CCTL1 = OUTMOD_7;						// CCR1 reset/set
			TB0CCR1 = duty_cycle;						// CCR1 PWM duty cycle
			TB0CTL = TBSSEL_2 + MC_1 + TBCLR;			// SMCLK, up mode, clear TBR
			break;
		case STRENGTH_SERVO:
			TB1CCR0 = SERVO_PERIOD;						// PWM Period
			TB1CCTL1 = OUTMOD_7;						// CCR1 reset/set
			TB1CCR1 = duty_cycle;						// CCR1 PWM duty cycle
			TB1CTL = TBSSEL_2 + MC_1 + TBCLR;			// SMCLK, up mode, clear TBR
			break;
		case HBRIDGE_EN:
			TB2CCR0 = HBRIDGE_PERIOD;					// PWM Period
			TB2CCTL1 = OUTMOD_7;						// CCR1 reset/set
			TB2CCR1 = duty_cycle;						// CCR1 PWM duty cycle
			TB2CTL = TBSSEL_2 + MC_1 + TBCLR;			// SMCLK, up mode, clear TBR
			break;
		default: break;
	}
}

void shiftOut(unsigned char in_data)
{
	SHIFTER3_OUT &= ~SHIFT3_RCK;
	unsigned char ii;
	for (ii = 0; ii < 8; ++ii)
	{
		SHIFTER3_OUT &= ~SHIFT3_SRCK;
		if (in_data & BIT7)
			SHIFTER2_OUT |= SHIFT2_SERIN;
		else
			SHIFTER2_OUT &= ~SHIFT2_SERIN;
		in_data = in_data << 1;
		SHIFTER3_OUT |= SHIFT3_SRCK;
	}
	SHIFTER3_OUT |= SHIFT3_RCK;
}

void SetupADC(unsigned char in_port, unsigned char in_bit)
{
	// Configure ADC
	switch (in_port)
	{
		case SENSOR_PORT1:
			P1SEL0 |= in_bit;
			P1SEL1 |= in_bit;
			break;
		case SENSOR_PORT3:
			P3SEL0 |= in_bit;
			P3SEL1 |= in_bit;
			break;
		default: break;
	}
	
	// Allow for settling delay
	delayMillis(DEFAULT_DELAY);
	
	// Configure ADC
	ADC10CTL0 &= ~ADC10ENC;
	ADC10CTL0 = ADC10SHT_7 + ADC10ON;		// ADC10ON, S&H=192 ADC clks
	ADC10CTL1 = ADC10SHS_0 + ADC10SHP + ADC10SSEL_0;
	ADC10CTL2 = ADC10RES;					// 10-bit conversion results
	
	switch (in_port)
	{
		case SENSOR_PORT1:
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
				case BIT5:
					ADC10MCTL0 = ADC10INCH_5;
					break;
				default: break;
			}
			break;
		case SENSOR_PORT3:
			switch (in_bit)
			{
				case BIT0:
					ADC10MCTL0 = ADC10INCH_12;
					break;
				case BIT1:
					ADC10MCTL0 = ADC10INCH_13;
					break;
				case BIT2:
					ADC10MCTL0 = ADC10INCH_14;
					break;
				case BIT3:
					ADC10MCTL0 = ADC10INCH_15;
					break;
				default: break;
			}
			break;
		default: break;
	}
	
	ADC10IE = ADC10IE0;						// Enable ADC conv complete interrupt
}

void TakeADCMeas(void)
{
	ADCResult = 1000;						// TEST
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
	STATE_DIR |= (STATE_0+STATE_1+STATE_2+STATE_3);
	STATE_OUT &= ~(STATE_0+STATE_1+STATE_2+STATE_3);
	
	// Setup geared motor
	GEARED_DIR |= GEARED_MOTOR;
	GEARED_OUT &= ~GEARED_MOTOR;
	
	// Setup servos
	SERVO_DIR |= (DUCK1_SERVO+DUCK2_SERVO+DUCK3_SERVO+STRENGTH_SERVO);
	SERVO_SEL0 |= (DUCK1_SERVO+DUCK2_SERVO+DUCK3_SERVO+STRENGTH_SERVO);
		
	// Setup lift motor
	HBRIDGE_DIR |= (LIFT_DIR1+HBRIDGE_EN+LIFT_DIR2+WAVE_DIR);
	HBRIDGE_SEL0 |= HBRIDGE_EN;
	HBRIDGE_OUT &= ~(LIFT_DIR1+LIFT_DIR2+WAVE_DIR);
	
	// Setup shifter
	SHIFTER3_DIR |= (SHIFT3_RCK+SHIFT3_SRCK);
	SHIFTER3_OUT &= ~(SHIFT3_RCK+SHIFT3_SRCK);
		
	SHIFTER2_DIR |= SHIFT2_SERIN;
	SHIFTER2_OUT &= ~SHIFT2_SERIN;
		
	// Initial state
	STATE state = IDLE;
	STATE_OUT = IDLE;
	SetupADC(SENSOR_PORT1, ENTRY_FSR);
	
	pwmControl(DUCK1_SERVO, DUCK1_UP);
	pwmControl(DUCK2_SERVO, DUCK2_UP);
	pwmControl(DUCK3_SERVO, DUCK3_UP);
	
	pwmControl(STRENGTH_SERVO, MALLET_DOWN);
	shiftOut(0);
	shiftOut(0);								// TEST
	
	unsigned char operation;
	while(1)
	{
		switch(state)
		{
			case IDLE:
				state_idle(&operation);
				break;
			case FERRIS:
				state_ferris(&operation);
				break;
			case PRE_DUCKS:
				state_pre_ducks(&operation);
				break;
			case DUCK1:
				state_duck1(&operation);
				break;
			case DUCK2:
				state_duck2(&operation);
				break;
			case DUCK3:
				state_duck3(&operation);
				break;
			case PRE_STRENGTH:
				state_pre_strength(&operation);
				break;
			case STRENGTH1:
				state_strength1(&operation);
				break;
			case STRENGTH2:
				state_strength2(&operation);
				break;
			case STRENGTH3:
				state_strength3(&operation);
				break;
			case POST_STRENGTH:
				state_post_strength(&operation);
				break;
			default: break;
		}
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
					STATE_OUT = *state;					// FPGA: Turn on music and LEDs
					SetupADC(SENSOR_PORT1, PRE_DUCKS_FSR);
					GEARED_OUT |= GEARED_MOTOR;			// Turn on geared motor
					break;
				default: break;
			}
			break;
		case FERRIS:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = PRE_DUCKS;
					STATE_OUT = *state;					// FPGA: Dim Ferris LEDs, brighten duck LEDs
					pwmControl(HBRIDGE_EN, LIFT_DOWN);
					HBRIDGE_OUT |= WAVE_DIR;			// DIR = HIGH
					SetupADC(SENSOR_PORT3, DUCK_IR);
					break;
				default: break;
			}
			break;
		case PRE_DUCKS:
			switch(operation)
			{
				case FINISHED_OPERATION:
					delayMillis(DUCK_DELAY);
					*state = DUCK1;
					STATE_OUT = *state;					// FPGA: Play duck SFX
					break;
				default: break;
			}
			break;
		case DUCK1:
			switch(operation)
			{
				case FINISHED_OPERATION:
					delayMillis(DUCK_DELAY);
					*state = DUCK2;
					STATE_OUT = *state;					// FPGA: Play duck SFX
					break;
				default: break;
			}
			break;
		case DUCK2:
			switch(operation)
			{
				case FINISHED_OPERATION:
					delayMillis(DUCK_DELAY);
					*state = DUCK3;
					STATE_OUT = *state;					// FPGA: Play duck SFX
					break;
				default: break;
			}
			break;
		case DUCK3:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = PRE_STRENGTH;
					STATE_OUT = *state;					// FPGA: Dim duck LEDs
					HBRIDGE_OUT &= ~WAVE_DIR;			// DIR = LOW
					SetupADC(SENSOR_PORT3, STRENGTH_IR);
					break;
				default: break;
			}
			break;
		case PRE_STRENGTH:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = STRENGTH1;
					STATE_OUT = *state;
					break;
				default: break;
			}
			break;
		case STRENGTH1:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = STRENGTH2;
					STATE_OUT = *state;
					break;
				default: break;
			}
			break;
		case STRENGTH2:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = STRENGTH3;
					STATE_OUT = *state;
					break;
				default: break;
			}
			break;
		case STRENGTH3:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = POST_STRENGTH;
					STATE_OUT = *state;					// FPGA: Play bell SFX
					SetupADC(SENSOR_PORT1, EXIT_FSR);
					break;
				default: break;
			}
			break;
		case POST_STRENGTH:
			switch(operation)
			{
				case FINISHED_OPERATION:
					*state = IDLE;
					STATE_OUT = *state;					// FPGA: Turn off music and LEDs
					unsigned char ii;
					for (ii = 0; ii < 3; ++ii)
					{
						shiftOut(0x00);
						delayMillis(LED_RISE_DELAY);
						shiftOut(0xFF);
						delayMillis(LED_RISE_DELAY);
					}
					shiftOut(0x00);
					pwmControl(DUCK1_SERVO, DUCK1_UP);
					pwmControl(DUCK2_SERVO, DUCK2_UP);
					pwmControl(DUCK3_SERVO, DUCK3_UP);
					GEARED_OUT &= ~GEARED_MOTOR;		// Turn off geared motor
					HBRIDGE_OUT |= LIFT_DIR2;			// DIR1 = LOW, DIR2 = HIGH
					SetupADC(SENSOR_PORT3, STRENGTH_IR);
					TakeADCMeas();
					while (ADCResult >= IR_LIFT_TRIGGER)
					{
						TakeADCMeas();
					}
					delayMillis(LIFT_DELAY);
					HBRIDGE_OUT &= ~LIFT_DIR2;			// DIR1 = LOW, DIR2 = LOW
					SetupADC(SENSOR_PORT1, ENTRY_FSR);
					break;
				default: break;
			}
			break;
		default: break;
	}
}

void state_idle(unsigned char *operation)
{
	TakeADCMeas();
	if(ADCResult < FSR_TRIGGER)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_ferris(unsigned char *operation)
{
	TakeADCMeas();
	if(ADCResult < FSR_TRIGGER)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_pre_ducks(unsigned char *operation)
{
	TakeADCMeas();
	if(ADCResult < IR_DUCK_TRIGGER)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_duck1(unsigned char *operation)
{
	pwmControl(DUCK1_SERVO, DUCK1_DOWN);
	*operation = FINISHED_OPERATION;
}

void state_duck2(unsigned char *operation)
{
	pwmControl(DUCK2_SERVO, DUCK2_DOWN);
	*operation = FINISHED_OPERATION;
}

void state_duck3(unsigned char *operation)
{
	pwmControl(DUCK3_SERVO, DUCK3_DOWN);
	*operation = FINISHED_OPERATION;
}

void state_pre_strength(unsigned char *operation)
{
	pwmControl(STRENGTH_SERVO, MALLET1_UP);
	TakeADCMeas();
	if(ADCResult < IR_LIFT_TRIGGER)
		*operation = FINISHED_OPERATION;
	else
		*operation = CONTINUE_OPERATION;
}

void state_strength1(unsigned char *operation)
{
	pwmControl(STRENGTH_SERVO, MALLET_DOWN);
	unsigned char ii = 0x01;
	pwmControl(HBRIDGE_EN, LIFT_UP);
	HBRIDGE_OUT |= LIFT_DIR1;					// DIR1 = HIGH, DIR2 = LOW
	shiftOut(ii);
	delayMillis(LED_RISE_DELAY);
	while (ii < LED_ATTEMPT1)
	{
		ii = (ii << 1) + 0x01;
		shiftOut(ii);
		delayMillis(LED_RISE_DELAY);
	}
	HBRIDGE_OUT &= ~LIFT_DIR1;					// DIR1 = LOW, DIR2 = LOW
	pwmControl(HBRIDGE_EN, LIFT_DOWN);
	delayMillis(DEFAULT_DELAY);
	HBRIDGE_OUT |= LIFT_DIR2;					// DIR1 = LOW, DIR2 = HIGH
	while (ii > 0x00)
	{
		ii = (ii >> 1);
		shiftOut(ii);
		delayMillis(LED_FALL_DELAY);
	}
	pwmControl(STRENGTH_SERVO, MALLET2_UP);
	TakeADCMeas();
	while (ADCResult >= IR_LIFT_TRIGGER)
	{
		TakeADCMeas();
	}
	delayMillis(LIFT_DELAY);
	HBRIDGE_OUT &= ~LIFT_DIR2;					// DIR1 = LOW, DIR2 = LOW
	*operation = FINISHED_OPERATION;
}

void state_strength2(unsigned char *operation)
{
	pwmControl(STRENGTH_SERVO, MALLET_DOWN);
	unsigned char ii = 0x01;
	pwmControl(HBRIDGE_EN, LIFT_UP);
	HBRIDGE_OUT |= LIFT_DIR1;					// DIR1 = HIGH, DIR2 = LOW
	shiftOut(ii);
	delayMillis(LED_RISE_DELAY);
	while (ii < LED_ATTEMPT2)
	{
		ii = (ii << 1) + 0x01;
		shiftOut(ii);
		delayMillis(LED_RISE_DELAY);
	}
	HBRIDGE_OUT &= ~LIFT_DIR1;					// DIR1 = LOW, DIR2 = LOW
	pwmControl(HBRIDGE_EN, LIFT_DOWN);
	delayMillis(DEFAULT_DELAY);
	HBRIDGE_OUT |= LIFT_DIR2;					// DIR1 = LOW, DIR2 = HIGH
	while (ii > 0x00)
	{
		ii = (ii >> 1);
		shiftOut(ii);
		delayMillis(LED_FALL_DELAY);
	}
	pwmControl(STRENGTH_SERVO, MALLET3_UP);
	TakeADCMeas();
	while (ADCResult >= IR_LIFT_TRIGGER)
	{
		TakeADCMeas();
	}
	delayMillis(LIFT_DELAY);
	HBRIDGE_OUT &= ~LIFT_DIR2;					// DIR1 = LOW, DIR2 = LOW
	*operation = FINISHED_OPERATION;
}

void state_strength3(unsigned char *operation)
{
	pwmControl(STRENGTH_SERVO, MALLET_DOWN);
	unsigned char ii = 0x01;
	pwmControl(HBRIDGE_EN, LIFT_UP);
	HBRIDGE_OUT |= LIFT_DIR1;					// DIR1 = HIGH, DIR2 = LOW
	shiftOut(ii);
	delayMillis(LED_RISE_DELAY);
	while (ii < LED_ATTEMPT3)
	{
		ii = (ii << 1) + 0x01;
		shiftOut(ii);
		delayMillis(LED_RISE_DELAY);
	}
	*operation = FINISHED_OPERATION;
}

void state_strength(unsigned char max_led, unsigned char *operation)
{
	pwmControl(STRENGTH_SERVO, MALLET_DOWN);
	unsigned char ii = 0x01;
	pwmControl(HBRIDGE_EN, LIFT_UP);
	HBRIDGE_OUT |= LIFT_DIR1;					// DIR1 = HIGH, DIR2 = LOW
	shiftOut(ii);
	delayMillis(LED_RISE_DELAY);
	while (ii < max_led)
	{
		ii = (ii << 1) + 0x01;
		shiftOut(ii);
		delayMillis(LED_RISE_DELAY);
	}
	if (max_led == LED_ATTEMPT3)
	{
		*operation = FINISHED_OPERATION;
	}
	else
	{
		HBRIDGE_OUT &= ~LIFT_DIR1;				// DIR1 = LOW, DIR2 = LOW
		pwmControl(HBRIDGE_EN, LIFT_DOWN);
		delayMillis(DEFAULT_DELAY);
		HBRIDGE_OUT |= LIFT_DIR2;				// DIR1 = LOW, DIR2 = HIGH
		while (ii > 0x00)
		{
			ii = (ii >> 1);
			shiftOut(ii);
			delayMillis(LED_FALL_DELAY);
		}
		if (max_led == LED_ATTEMPT1)
		{
			pwmControl(STRENGTH_SERVO, MALLET2_UP);
		}
		else
		{
			pwmControl(STRENGTH_SERVO, MALLET3_UP);
		}		
		TakeADCMeas();
		while (ADCResult >= IR_LIFT_TRIGGER)
		{
			TakeADCMeas();
		}
		delayMillis(LIFT_DELAY);
		HBRIDGE_OUT &= ~LIFT_DIR2;				// DIR1 = LOW, DIR2 = LOW
		*operation = FINISHED_OPERATION;
	}
}

void state_post_strength(unsigned char *operation)
{
	HBRIDGE_OUT &= ~LIFT_DIR1;					// DIR1 = LOW, DIR2 = LOW
	pwmControl(HBRIDGE_EN, LIFT_DOWN);
	TakeADCMeas();
	if(ADCResult < FSR_TRIGGER)
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