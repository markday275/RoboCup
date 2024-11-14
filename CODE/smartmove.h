/************************************
 *          smartmove.h             *
 * 
 * Header file for robot movement based on sensor inputs, 
 * including obstacle avoidance and goal-oriented movement.
 * 
 * Authors:
 *        Mark   Day
 *        Jack   Wilson
 *        Kieran James
 *************************************/

#ifndef SMARTMOVE_H_
#define SMARTMOVE_H_

#include "Arduino.h"


/**********************************************************************************
* CALCULATE POSITION
*
* Updates the current position (X, Y) of the robot based on odometer and heading.
**********************************************************************************/
void CalcPos(void);


/**********************************************************************************
* RESET HOME POSITION
*
* Resets the robot's current position to the origin (0,0).
**********************************************************************************/
void resetHomePos(void);


/**********************************************************************************
* GET X POSITION
*
* Returns the current X position of the robot.
**********************************************************************************/
int GetXPos(void);


/**********************************************************************************
* GET Y POSITION
*
* Returns the current Y position of the robot.
**********************************************************************************/
int GetYPos(void);


//**********************************************************************************
// Obstacle Detection Functions
//**********************************************************************************

/**********************************************************************************
* CHECK OBSTACLES
*
* Updates the obstacle detection flags based on Time-of-Flight sensor values.
**********************************************************************************/
void CheckObstacles(void);


//**********************************************************************************
// Movement Functions
//**********************************************************************************

/**********************************************************************************
* MOVE FORWARD AVOIDANCE
*
* Moves the robot forward while avoiding obstacles based on sensor readings.
**********************************************************************************/
void MoveForwardAvoid(int speed);


/**********************************************************************************
* MOVE TO TARGET POSITION
*
* Moves the robot to a specified X, Y position at a given speed. Adjusts direction
* based on obstacle avoidance and heading difference.
**********************************************************************************/
void MoveTo(int xPos, int yPos, int speed);


/**********************************************************************************
* MOVE FORWARD A SPECIFIC DISTANCE
*
* Moves the robot forward a set distance in millimeters, returning 1 when goal is
* reached and 0 otherwise.
**********************************************************************************/
int MoveForwardsDistance(int distance, int speed);


//**********************************************************************************
// Obstacle Handling Protocols
//**********************************************************************************

/**********************************************************************************
* CHECK RAMP OBSTACLE
*
* Detects and handles ramp-like obstacles. If detected, performs a backing up and
* turning maneuver to avoid the ramp.
**********************************************************************************/
uint8_t checkRamp(void);


/**********************************************************************************
* CHECK ANGLE
*
* Detects if the robot is on an inclined surface and adjusts movement if needed.
* Returns a flag indicating if weight adjustment is required.
**********************************************************************************/
uint8_t checkAngle(void);

#endif /* SMARTMOVE_H_ */

