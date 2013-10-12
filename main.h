#include "msp430fr5739.h"
#include <stdio.h>

#define LINE_SENSOR_MAX 300
#define FORCE_SENSOR_MAX 900

// States P3.4 to P3.7
#define STATE_DIR P3DIR
#define STATE_OUT P3OUT

// Geared motor P4.0
#define GEARED_DIR P4DIR
#define GEARED_OUT P4OUT
#define GEARED_MOTOR BIT0

// Servos P1.0, P1.2, P1.4, P1.6
#define SERVO_DIR P1DIR
#define SERVO_SEL0 P1SEL0

#define DUCK1_SERVO BIT0
#define DUCK2_SERVO BIT2
#define DUCK3_SERVO BIT4
#define STRENGTH_SERVO BIT6

// Lift motor P2.1, P2.2
#define LIFT_DIR P2DIR
#define LIFT_OUT P2OUT
#define LIFT_DIN1 BIT1
#define LIFT_DIN2 BIT2

// Wave motor P1.7
#define WAVE_DIR P1DIR
#define WAVE_OUT P1OUT
#define WAVE_DIN BIT7

// Force sensitive resistor P1.1, P1.3, P1.5
#define FSR_PORT 1
#define FSR_SEL0 P1SEL0
#define FSR_SEL1 P1SEL1

#define ENTRY_FSR BIT1
#define PRE_DUCKS_FSR BIT3
#define EXIT_FSR BIT5

// Line sensor P3.3, P3.4
#define IR_PORT 3
#define IR_SEL0 P3SEL0
#define IR_SEL1 P3SEL1

#define DUCK_IR BIT0
#define STRENGTH_IR BIT1

// Strength LEDs P2.0, P2.2, P2.5 
#define SHIFTER_DIR P2DIR
#define SHIFTER_OUT P2OUT

#define SHIFT_SERIN BIT0
#define SHIFT_SRCK BIT2
#define SHIFT_RCK BIT5

// State operation
#define FINISHED_OPERATION 0
#define CONTINUE_OPERATION 1

// Delays
#define DEFAULT_DELAY 50
#define DUCK_DELAY 100
#define LED_RISE_DELAY 60
#define LED_FALL_DELAY 180
#define MOTOR_DELAY 3000

typedef enum
{
	IDLE = 0x00, FERRIS = 0x10,
	PRE_DUCKS = 0x20, DUCK1 = 0x30, DUCK2 = 0x40, DUCK3 = 0x50,
	PRE_STRENGTH = 0x60, STRENGTH1 = 0x70, STRENGTH2 = 0x80, STRENGTH3 = 0x90, POST_STRENGTH = 0xA0
} STATE;

void state_machine(STATE *, unsigned char);
void state_idle(unsigned char *);
void state_ferris(unsigned char *);
void state_pre_ducks(unsigned char *);
void state_duck1(unsigned char *);
void state_duck2(unsigned char *);
void state_duck3(unsigned char *);
void state_pre_strength(unsigned char *);
void state_strength(unsigned char, unsigned char *);
void state_post_strength(unsigned char *);

volatile unsigned int ADCResult;
