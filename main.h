#include "msp430fr5739.h"
#include <stdio.h>

/***********************************
 * Delays (milliseconds)
 ***********************************/
#define DEFAULT_DELAY 50
#define DUCK_DELAY 100
#define LED_RISE_DELAY 200
#define LED_FALL_DELAY (int)((float)LED_RISE_DELAY*1.2 + 0.5)
#define LIFT_DELAY 2

/***********************************
 *	Geared motor: P4.0
 ***********************************/
#define GEARED_DIR P4DIR
#define GEARED_OUT P4OUT
#define GEARED_MOTOR BIT0

/***********************************
 *	Servos: P1.0, P1.2, P1.4, P1.6
 ***********************************/
#define SERVO_DIR P1DIR
#define SERVO_SEL0 P1SEL0

#define DUCK1_SERVO BIT0
#define DUCK2_SERVO BIT2
#define DUCK3_SERVO BIT4
#define STRENGTH_SERVO BIT6

// PWM values
#define SERVO_PERIOD 19999	// 20000 - 1
#define DUCK_UP 1900
#define DUCK_DOWN 1200
#define MALLET1_UP 1400
#define MALLET2_UP 1800
#define MALLET3_UP 2200
#define MALLET_DOWN 800

/***********************************
 *	H-Bridge Motors: P2.1
 ***********************************/
#define HBRIDGE_DIR P2DIR
#define HBRIDGE_SEL0 P2SEL0
#define HBRIDGE_OUT P2OUT
#define HBRIDGE_EN BIT1

// Lift motor: P2.0, P2.2
#define LIFT_DIR1 BIT0
#define LIFT_DIR2 BIT2

// Wave motor: P2.5
#define WAVE_DIR BIT5

// PWM values
#define HBRIDGE_PERIOD 499		// 500 - 1
#define LIFT_UP 499				// Full speed
#define LIFT_DOWN (int)((float)LIFT_UP*0.3*(float)LED_RISE_DELAY/(float)LED_FALL_DELAY + 0.5) // Descend same speed fraction as LEDs

/***********************************
 *	Sensors
 ***********************************/
#define SENSOR_PORT1 1
#define SENSOR_PORT3 3

// Force sensitive resistor: P3.0, P3.1, P3.2
#define ENTRY_FSR BIT0
#define PRE_DUCKS_FSR BIT1
#define EXIT_FSR BIT2

// Line sensor: P1.1, P1.3
#define DUCK_IR BIT1
#define STRENGTH_IR BIT3

#define IR_TRIGGER 300		// Port1 = 300, Port3 = 150
#define FSR_TRIGGER 450		// Port1 = 900, Port3 = 450

/***********************************
 *	8bit Shifter: P1.5, P1.7, P2.6
 ***********************************/
#define SHIFTER1_DIR P1DIR
#define SHIFTER1_OUT P1OUT
#define SHIFTER2_DIR P2DIR
#define SHIFTER2_OUT P2OUT

#define SHIFT1_RCK BIT5
#define SHIFT1_SRCK BIT7
#define SHIFT2_SERIN BIT6

// LEDs to light up
#define LED_ATTEMPT1 0x07
#define LED_ATTEMPT2 0x1F
#define LED_ATTEMPT3 0xFF

/***********************************
 *	States: P3.4 to P3.7
 ***********************************/
#define STATE_DIR P3DIR
#define STATE_OUT P3OUT

#define STATE_0 BIT4
#define STATE_1 BIT5
#define STATE_2 BIT6
#define STATE_3 BIT7

// State operation
#define FINISHED_OPERATION 0
#define CONTINUE_OPERATION 1

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
void state_strength1(unsigned char *);
void state_strength2(unsigned char *);
void state_strength3(unsigned char *);
void state_strength(unsigned char, unsigned char *);
void state_post_strength(unsigned char *);

volatile unsigned int ADCResult;