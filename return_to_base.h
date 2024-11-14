/************************************
 *         return_to_base.h         *
 * 
 * header for returning robot home
 * 
 * Arthors 
 *        Mark   Day
 *        Jack   Wilson
 *        Kieran James
 *************************************/


#ifndef RETURN_TO_BASE_H_
#define RETURN_TO_BASE_H_

#include <stdint.h>

/**********************************************************************************
* Return to base
*
* uses the moveto function from smartmove
* assumes base is at x,y = 0,0
**********************************************************************************/
void return_to_base();


/**********************************************************************************
* Calculate direction home
*
* is called by the state machine
**********************************************************************************/
void CalcHome(void);


/**********************************************************************************
* returns distance home
*
* In mm -> int32 
**********************************************************************************/
int32_t GetDisHome(void);


/**********************************************************************************
* sets the colour of home base
*
* needs to be done while at home 
* only call once 
**********************************************************************************/
void setHomeColour(void);


/**********************************************************************************
* Checks if at home
*
* compares current colour to stored home colour + error
**********************************************************************************/
int isHome(void);


#endif /* RETURN_TO_BASE_H_ */