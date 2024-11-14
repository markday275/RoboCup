/************************************
 *            motors.h              *
 * 
 * Motor outputs functions header
 * 
 * Arthors 
 *        Mark   Day
 *        Jack   Wilson
 *        Kieran James
 *************************************/

#ifndef MOTORS_H_
#define MOTORS_H_

#include <stdint.h>

/**********************************************************************************
* Init All Motors
*
* Needs to be done before trying to write to motors  
**********************************************************************************/
void motor_init(void);


/**********************************************************************************
* Sets speed limits for motors 
*
* Check defines for limits 
**********************************************************************************/
uint16_t check_speed_limits(uint16_t speed);


/**********************************************************************************
* Set Motor
*
* Sets the Motor speed based on globals
* Is called by the schedular
**********************************************************************************/
void set_motor(void);


/**********************************************************************************
* Move Forwards
*
* Moves to robot continuously forwards
* 0 -> 10 
**********************************************************************************/
void MoveForward(uint8_t speed);


/**********************************************************************************
* Move Backwards
*
* Moves to robot continuously backwards
* 0 -> 10 
**********************************************************************************/
void MoveBackward(uint8_t speed);


/**********************************************************************************
* Turn Left
*
* Turns robot continuously left
* 0 -> 10 
**********************************************************************************/
void TurnLeft(uint8_t speed);


/**********************************************************************************
* Turn right
*
* Turns robot continuously right
* 0 -> 10 
**********************************************************************************/
void TurnRight(uint8_t speed);


/**********************************************************************************
* Turn Left
*
* Turns robot left for a given time
* 0 -> 10 
* Duration in ms
**********************************************************************************/
void TurnLeftForTime(uint8_t speed, unsigned long duration);


/**********************************************************************************
* Turn right
*
* Turns robot right for a given time
* 0 -> 10 
* Duration in ms
**********************************************************************************/
void TurnRightForTime(uint8_t speed, unsigned long duration);


/**********************************************************************************
* ENCODER SETUP
*
* Do before state machine 
**********************************************************************************/
void encoderSetUp(void);


/**********************************************************************************
* Prints raw Encoder values
*
* "Index: {ENCODER1} : {ENCODER2}"
**********************************************************************************/
void printEncoderSerial(void);


void doEncoder1A(void);


void doEncoder2A(void);


/**********************************************************************************
* Updates Odometry 
*
* Saves value to global in mm 
**********************************************************************************/
void OdomReading(void);


/**********************************************************************************
* Returns current Odometry in mm
**********************************************************************************/
float GetOdom(void);


int getFrontGatePOS(void);


void frontGateCentre(void);


void frontGateOpen(void);


void frontGateClose(void);


void frontGateHalfOpen(void);


void frontGateHalfClose(void);


int getBackGatePOS(void);


void BackGateOpen(void);


void BackGateClose(void);


#endif /* MOTORS_H_ */
