/************************************
 *            sensors.h             *
 * 
 * Header file for all inputs into the robot.
 * 
 * Authors:
 *        Mark   Day
 *        Jack   Wilson
 *        Kieran James
 *************************************/

#ifndef SENSORS_H_
#define SENSORS_H_

#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


/**********************************************************************************
* INITIALIZE INFRARED SENSOR
*
* Sets the digital pin for the IR sensor as input.
**********************************************************************************/
void init_infrared(void);


/**********************************************************************************
* READ INFRARED SENSOR
*
* Reads and stores the state of the IR sensor in `infrared_value`.
* Should be called periodically by the scheduler.
**********************************************************************************/
void read_infrared(void);


/**********************************************************************************
* GET INFRARED SENSOR VALUE
*
* Returns the current value of `infrared_value`.
**********************************************************************************/
bool get_infrared(void);


/**********************************************************************************
* INITIALIZE INDUCTIVE SENSOR
*
* Sets the digital pin mode for the inductive sensor as input.
**********************************************************************************/
void init_inductive(void);


/**********************************************************************************
* READ INDUCTIVE SENSOR
*
* Reads and stores the state of the inductive sensor in `inductive_value`.
* Should be called periodically by the scheduler.
**********************************************************************************/
void read_inductive(void);


/**********************************************************************************
* GET INDUCTIVE SENSOR VALUE
*
* Returns the current value of `inductive_value`.
**********************************************************************************/
bool get_inductive(void);


/**********************************************************************************
* INITIALIZE TOF SENSORS
*
* Sets up and initializes all Time-of-Flight (ToF) sensors.
**********************************************************************************/
void TofSensorInit(void);


/**********************************************************************************
* PRINT TOF SENSOR VALUES TO SERIAL
*
* Outputs the current ToF sensor readings to the serial monitor.
**********************************************************************************/
void PrintTofSerial(void);


/**********************************************************************************
* UPDATE TOF SENSOR VALUES
*
* Reads and updates the current distance values for each ToF sensor.
**********************************************************************************/
void updateTof(void);


/**********************************************************************************
* GET TOF CENTER VALUE
*
* Returns the minimum distance value from the two center ToF sensors.
**********************************************************************************/
int TofcentreValue(void);


/**********************************************************************************
* GET TOF LEFT-CENTER VALUE
*
* Returns the distance value from the left-center ToF sensor.
**********************************************************************************/
int TofLeftCentreValue(void);


/**********************************************************************************
* GET TOF RIGHT-CENTER VALUE
*
* Returns the distance value from the right-center ToF sensor.
**********************************************************************************/
int TofRightCentreValue(void);


/**********************************************************************************
* GET TOF LEFT VALUE
*
* Returns the distance value from the left ToF sensor.
**********************************************************************************/
int TofleftValue(void);


/**********************************************************************************
* GET TOF RIGHT VALUE
*
* Returns the distance value from the right ToF sensor.
**********************************************************************************/
int TofrightValue(void);


/**********************************************************************************
* GET TOF BOTTOM LEFT VALUE
*
* Returns the distance value from the bottom-left ToF sensor.
**********************************************************************************/
int TofbottomLeftValue(void);


/**********************************************************************************
* GET TOF BOTTOM RIGHT VALUE
*
* Returns the distance value from the bottom-right ToF sensor.
**********************************************************************************/
int TofbottomRightValue(void);


/**********************************************************************************
* INITIALIZE IMU SENSOR
*
* Sets up the IMU sensor.
**********************************************************************************/
void IMUsetup(void);


/**********************************************************************************
* UPDATE IMU SENSOR DATA
*
* Reads and updates the IMU sensor data, including heading and angle of attack.
**********************************************************************************/
void IMUloop(void);


/**********************************************************************************
* PRINT IMU EVENT DATA
*
* Prints IMU event data to the serial monitor.
* Parameter: sensors_event_t* event - The event data to print.
**********************************************************************************/
void IMUprintEvent(sensors_event_t* event);


/**********************************************************************************
* GET HEADING IN DEGREES
*
* Returns the current heading from the IMU in degrees.
**********************************************************************************/
float GetHeadingDeg(void);


/**********************************************************************************
* GET HEADING IN RADIANS
*
* Returns the current heading from the IMU in radians.
**********************************************************************************/
float GetHeadingRad(void);


/**********************************************************************************
* GET ANGLE OF ATTACK
*
* Returns the current angle of attack from the IMU.
**********************************************************************************/
float GetAngleofAttack(void);


/**********************************************************************************
* INITIALIZE COLOR SENSOR
*
* Sets up the color sensor.
**********************************************************************************/
void colourSetup(void);


/**********************************************************************************
* READ COLOR SENSOR VALUES
*
* Reads the raw RGB values from the color sensor and updates `g_currentcolour`.
**********************************************************************************/
void read_colour(void);


/**********************************************************************************
* GET COLOR VALUES
*
* Returns a pointer to an array containing the current RGB values.
**********************************************************************************/
int* getColour(void);

#endif /* SENSORS_H_ */

