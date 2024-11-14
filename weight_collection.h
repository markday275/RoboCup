/************************************
 *       weight_collection.h        *
 * 
 * Header file for weight detection and color-based play area functions.
 * 
 * Authors:
 *        Mark   Day
 *        Jack   Wilson
 *        Kieran James
 *************************************/

#ifndef WEIGHT_COLLECTION_H_
#define WEIGHT_COLLECTION_H_

#include <Servo.h>      
#include <Wire.h>                  
#include "Arduino.h"
#include "sensors.h"  


/**********************************************************************************
* SET WEIGHT DETECTION
*
* Sets detection flags for weight at center, left, and right sensors.
**********************************************************************************/
void setWeightDetection(bool center, bool left, bool right);


/**********************************************************************************
* WEIGHT SCAN
*
* Scans for weight based on TOF sensor readings and updates detection flags.
**********************************************************************************/
void weight_scan(void);


/**********************************************************************************
* GET WEIGHT DETECTION
*
* Returns true if weight is detected by any sensor.
**********************************************************************************/
bool GetWeightDetection(void);


/**********************************************************************************
* GET WEIGHT DETECTION LEFT
*
* Returns true if weight is detected on the left.
**********************************************************************************/
bool GetWeightDetectionLeft(void);


/**********************************************************************************
* GET WEIGHT DETECTION RIGHT
*
* Returns true if weight is detected on the right.
**********************************************************************************/
bool GetWeightDetectionRight(void);


/**********************************************************************************
* GET WEIGHT DETECTION CENTER
*
* Returns true if weight is detected at the center.
**********************************************************************************/
bool GetWeightDetectionCenter(void);


//**********************************************************************************
// Play Area Functions
//**********************************************************************************

/**********************************************************************************
* SET PLAY COLOR
*
* Sets the color values for the play area using the color sensor readings.
**********************************************************************************/
void setPlayColour(void);


/**********************************************************************************
* IS PLAY AREA
*
* Checks if the current color matches the play area color within the error margin.
**********************************************************************************/
int isPlay(void);

#endif /* WEIGHT_COLLECTION_H_ */
