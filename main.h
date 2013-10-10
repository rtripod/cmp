#include "msp430fr5739.h"
#include <stdio.h>

#define LINE_SENSOR_MAX 300
#define FORCE_SENSOR_MAX 900

#define STATE_DIR P3DIR
#define STATE_OUT P3OUT

#define GEARED_DIR P2DIR
#define GEARED_OUT P2OUT
#define GEARED_PIN BIT0

#define DUCK1_SERVO BIT0
#define DUCK2_SERVO BIT2
#define DUCK3_SERVO BIT4
#define STRENGTH_SERVO BIT6
#define LIFT_MOTOR BIT1

#define ENTRY_FSR BIT1
#define PRE_DUCKS_FSR BIT3
#define EXIT_FSR BIT5

#define DUCK_IR BIT3
#define STRENGTH_IR BIT4

#define FSR_PORT 1
#define IR_PORT 3

#define SERVO_DIR P1DIR
#define SERVO_SEL0 P1SEL0

#define LIFT_DIR P2DIR
#define LIFT_SEL0 P2SEL0

#define FSR_SEL0 P1SEL0
#define FSR_SEL1 P1SEL1

#define IR_SEL0 P3SEL0
#define IR_SEL1 P3SEL1

#define FINISHED_OPERATION 0
#define CONTINUE_OPERATION 1

#define DUCK_DELAY 100

typedef enum {IDLE = 0x00, FERRIS = 0x10, PRE_DUCKS = 0x20, DUCK1 = 0x30, DUCK2 = 0x40, DUCK3 = 0x50, STRENGTH = 0x60} STATE;

void state_machine(STATE *, unsigned char);
void state_idle(unsigned char *);
void state_ferris(unsigned char *);
void state_pre_ducks(unsigned char *);
void state_duck1(unsigned char *);
void state_duck2(unsigned char *);
void state_duck3(unsigned char *);
void state_strength(unsigned char *);

volatile unsigned int ADCResult;
