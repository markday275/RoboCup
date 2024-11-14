/************************************
 *            motors.cpp            *
 * 
 * Motor outputs functions
 * 
 * Arthors 
 *        Mark   Day
 *        Jack   Wilson
 *        Kieran James
 *************************************/


#include <Servo.h>
#include <stdint.h>
#include "Arduino.h"

#include "motors.h"
#include "sensors.h"


//**********************************************************************************
//motor speed globals
//**********************************************************************************
#define LEFT_MOTOR_ADDRESS              0      
#define RIGHT_MOTOR_ADDRESS             1     
#define MIN_SPEED_CAP                   1050           
#define MAX_SPEED_CAP                   1950
#define MOTOR_STOP_SPEED                1500
#define MOTOR_SPEED_MULITPLIER          450
#define MOTOR_DRIFT                     30
static uint16_t g_rightMotorSpeed =     MOTOR_STOP_SPEED; //above 1500 forwards
static uint16_t g_leftMotorSpeed =      MOTOR_STOP_SPEED; //below 1500 forwards


//**********************************************************************************
//globals for dc motor encoder
//**********************************************************************************
volatile int encoderPos1 =  0;
int lastReportedPos1 =      1;
volatile int encoderPos2 =  0;
int lastReportedPos2 =      1;
boolean A_set1 =            false;
boolean B_set1 =            false;
boolean A_set2 =            false;
boolean B_set2 =            false;

//globals for odometry in mm
#define ENCODER_TO_1000MM 85
static float g_odom = 0;
static float prevEncoder1;
static float prevEncoder2;
enum PinAssignments {
  encoder1PinA = 2,
  encoder1PinB = 3,
  encoder2PinA = 4,
  encoder2PinB = 5,
};


//**********************************************************************************
// Servo gate assignments
//**********************************************************************************
#define BACKGATE_PIN      7
#define FRONTGATE_PIN     8

#define FRONT_CENTRE      1700
#define FRONT_CLOSE       800
#define FRONT_OPEN        2500
#define FRONT_HALF_OPEN   2100
#define FRONT_HALF_CLOSE  1350

#define BACKOPEN          2250
#define BACKCLOSED        1750

typedef enum {
  CENTRE,
  LEFT,
  RIGHT,
  OPEN,
  CLOSED,
} GatePos;
GatePos g_FrontGatePOS = CENTRE;
GatePos g_BackGatePOS = CLOSED;

Servo leftMotor, rightMotor, frontGate, backGate;


//**********************************************************************************
// Motor Init
//**********************************************************************************
void motor_init(void) {
  leftMotor.attach(LEFT_MOTOR_ADDRESS);
  rightMotor.attach(RIGHT_MOTOR_ADDRESS);
  frontGate.attach(FRONTGATE_PIN);
  backGate.attach(BACKGATE_PIN);

  frontGateCentre();
  BackGateClose();
}

//**********************************************************************************
// Check Speed Limits
//**********************************************************************************
uint16_t check_speed_limits(uint16_t speed) {
  if (speed > MAX_SPEED_CAP ){
    speed = MAX_SPEED_CAP;
  } else if (speed < MIN_SPEED_CAP) {
    speed = MIN_SPEED_CAP;
  }
  return speed;
}


//**********************************************************************************
// Set Motor
//**********************************************************************************
void set_motor(void) {
  g_leftMotorSpeed = check_speed_limits(g_leftMotorSpeed);
  g_rightMotorSpeed = check_speed_limits(g_rightMotorSpeed);
  leftMotor.writeMicroseconds(g_leftMotorSpeed);
  rightMotor.writeMicroseconds(g_rightMotorSpeed);
}


//**********************************************************************************
// Basic Move Functions 
//**********************************************************************************
void MoveForward(uint8_t speed) 
{
  g_rightMotorSpeed = MOTOR_STOP_SPEED - ((MOTOR_SPEED_MULITPLIER * speed)/ 10);
  g_leftMotorSpeed = MOTOR_STOP_SPEED + ((MOTOR_SPEED_MULITPLIER * speed)/ 10) - MOTOR_DRIFT; 
}


void MoveBackward(uint8_t speed)
{
  g_rightMotorSpeed = MOTOR_STOP_SPEED + (MOTOR_SPEED_MULITPLIER * speed)/ 10;
  g_leftMotorSpeed = MOTOR_STOP_SPEED - (MOTOR_SPEED_MULITPLIER * speed)/ 10;
}


void TurnLeft(uint8_t speed)
{
  g_rightMotorSpeed = MOTOR_STOP_SPEED - (MOTOR_SPEED_MULITPLIER * speed)/ 10;
  g_leftMotorSpeed = MOTOR_STOP_SPEED - (MOTOR_SPEED_MULITPLIER * speed)/ 10;
}


void TurnRight(uint8_t speed)
{
  g_rightMotorSpeed = MOTOR_STOP_SPEED + (MOTOR_SPEED_MULITPLIER * speed)/ 10;
  g_leftMotorSpeed = MOTOR_STOP_SPEED + (MOTOR_SPEED_MULITPLIER * speed)/ 10;
}

//**********************************************************************************
// Move for time
//**********************************************************************************
void TurnLeftForTime(uint8_t speed, unsigned long duration)
{
  TurnLeft(speed);
  set_motor();
  delay(duration);
  g_rightMotorSpeed = MOTOR_STOP_SPEED;  
  g_leftMotorSpeed = MOTOR_STOP_SPEED;  
}


void TurnRightForTime(uint8_t speed, unsigned long duration)
{
  TurnRight(speed);
  set_motor();
  delay(duration);
  g_rightMotorSpeed = MOTOR_STOP_SPEED;  
  g_leftMotorSpeed = MOTOR_STOP_SPEED;  
}


//**********************************************************************************
// Encoder
//**********************************************************************************
void encoderSetUp(void)
{
  pinMode(encoder1PinA, INPUT);       
  pinMode(encoder1PinB, INPUT); 
  pinMode(encoder2PinA, INPUT); 
  pinMode(encoder2PinB, INPUT); 
  attachInterrupt(digitalPinToInterrupt(2), doEncoder1A, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(4), doEncoder2A, CHANGE);
}


void printEncoderSerial(void)
{
  if ((lastReportedPos1 != encoderPos1)||(lastReportedPos2 != encoderPos2)) 
  {
    Serial.print("Index:");
    Serial.print(encoderPos1, DEC);
    Serial.print(":");
    Serial.print(encoderPos2, DEC);
    Serial.println();
    lastReportedPos1 = encoderPos1;
    lastReportedPos2 = encoderPos2;
  }
}


void doEncoder1A(void)
{
  A_set1 = digitalRead(encoder1PinA) == HIGH;
  encoderPos1 += (A_set1 != B_set1) ? -1 : +1;
  B_set1 = digitalRead(encoder1PinB) == HIGH;
  encoderPos1 += (A_set1 == B_set1) ? -1 : +1;
}


void doEncoder2A(void)
{
  A_set2 = digitalRead(encoder2PinA) == HIGH;
  encoderPos2 += (A_set2 != B_set2) ? +1 : -1;
  B_set2 = digitalRead(encoder2PinB) == HIGH;
  encoderPos2 += (A_set2 == B_set2) ? +1 : -1;
}


//**********************************************************************************
// Odometry
//**********************************************************************************
void OdomReading(void)
{
  float localencoder1 = encoderPos1;
  float localencoder2 = encoderPos2;
  float deltaEncoder = ((localencoder1 - prevEncoder1) + (localencoder2 - prevEncoder2)) / 2;
  g_odom += ENCODER_TO_1000MM * deltaEncoder / 1000; 
  prevEncoder1 = localencoder1;
  prevEncoder2 = localencoder2;
}


float GetOdom(void)
{
  return g_odom;
}


//**********************************************************************************
// Front Servo
//**********************************************************************************
int getFrontGatePOS(void)
{
  return g_FrontGatePOS;
}


void frontGateCentre(void)
{
  frontGate.writeMicroseconds(FRONT_CENTRE);
  g_FrontGatePOS = CENTRE;
}


void frontGateClose(void)
{
  frontGate.writeMicroseconds(FRONT_CLOSE);
  g_FrontGatePOS = LEFT;
}


void frontGateOpen(void)
{
  frontGate.writeMicroseconds(FRONT_OPEN);
  g_FrontGatePOS = RIGHT;
}


void frontGateHalfOpen(void)
{
  frontGate.writeMicroseconds(FRONT_HALF_OPEN);
  g_FrontGatePOS = RIGHT;
}


void frontGateHalfClose(void)
{
  frontGate.writeMicroseconds(FRONT_HALF_CLOSE);
  g_FrontGatePOS = RIGHT;
}


//**********************************************************************************
// Back Servo
//**********************************************************************************
int getBackGatePOS(void)
{
  return g_BackGatePOS;
}


void BackGateOpen(void)
{
  backGate.writeMicroseconds(BACKOPEN);
  g_BackGatePOS = OPEN;
}


void BackGateClose(void)
{
  backGate.writeMicroseconds(BACKCLOSED);
  g_BackGatePOS = CLOSED;
}





