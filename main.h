#include "msp430fr5739.h"
#include <stdio.h>

typedef enum { FALSE, TRUE } boolean;

/***********************************
 * Delays (milliseconds)
 ***********************************/
#define DEFAULT_DELAY 50

#define DUCK1_DELAY 300*0.5
#define DUCK2_DELAY 390*0.5
#define DUCK3_DELAY 390*0.5

#define STRENGTH_DELAY 1500
#define LED_RISE_DELAY 200
#define LED_FALL_DELAY (int)((float)LED_RISE_DELAY*1.2 + 0.5)
#define LIFT_DELAY 300

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

#define DUCK1_UP 700
#define DUCK2_UP 700
#define DUCK3_UP 600

#define DUCK1_DOWN 1300
#define DUCK2_DOWN 1400
#define DUCK3_DOWN 1300

#define MALLET1_UP 1000
#define MALLET2_UP 800
#define MALLET3_UP 600
#define MALLET_DOWN 2200

/***********************************
 *	H-Bridge
 ***********************************/
#define HBRIDGE_DIR P2DIR
#define HBRIDGE_SEL0 P2SEL0
#define HBRIDGE_OUT P2OUT

// Lift motor: P2.0, P2.1, P2.5
#define LIFT_DIR1 BIT0
#define LIFT_EN BIT1
#define LIFT_DIR2 BIT5

// PWM values
#define HBRIDGE_PERIOD 499		// 500 - 1
#define LIFT_UP 250				// Full speed
#define LIFT_DOWN (int)((float)LIFT_UP*0.5*(float)LED_RISE_DELAY/(float)LED_FALL_DELAY + 0.5) // Descend same speed fraction as LEDs

/***********************************
 *	Sensors
 ***********************************/
#define SENSOR_PORT1 1
#define SENSOR_PORT3 3

// Force sensitive resistor: P1.1, P1.3, P1.5
#define ENTRY_FSR BIT1
#define PRE_DUCKS_FSR BIT3
#define EXIT_FSR BIT5

// Line sensor: P3.0, P3.1
#define DUCK_IR BIT1
#define STRENGTH_IR BIT0

#define MAX_READINGS 100
#define IR_DUCK_TRIGGER 415
#define IR_LIFT_TRIGGER 435
#define FSR_ENTRY_TRIGGER 850	// Port1 = 900, Port3 = 450
#define FSR_MIDDLE_TRIGGER 900
#define FSR_EXIT_TRIGGER 980

/***********************************
 *	8bit Shifter: P2.6, P3.2, P3.3, 
 ***********************************/
#define SHIFTER2_DIR P2DIR
#define SHIFTER2_OUT P2OUT
#define SHIFTER3_DIR P3DIR
#define SHIFTER3_OUT P3OUT

#define SHIFT2_SERIN BIT6
#define SHIFT3_SRCK BIT2
#define SHIFT3_RCK BIT3

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