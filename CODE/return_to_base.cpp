/************************************
 *        return_to_base.cpp        *
 * 
 * functions for returning robot home
 * 
 * Arthors 
 *        Mark   Day
 *        Jack   Wilson
 *        Kieran James
 *************************************/


#include "return_to_base.h"
#include "Arduino.h"

#include "motors.h"
#include "sensors.h"
#include "smartmove.h"


//**********************************************************************************
// Home Globals
//**********************************************************************************
#define HOME_ERROR 10
int homeColour[3];
float g_HeadingHome;
int32_t g_DistanceHome;



//**********************************************************************************
// Return to home base
//**********************************************************************************
void return_to_base(void){
  MoveTo(0, 0, 10);
}


//**********************************************************************************
// Calculate Home direction and distance
//**********************************************************************************
void CalcHome(void)
{
  int XPos = GetXPos();
  int YPos = GetYPos();

  g_DistanceHome = sqrt(XPos * XPos + YPos * YPos);

  if ((YPos > 0) & (XPos > 0)) {
    g_HeadingHome = atan((-YPos / -XPos) + PI) + PI;
  } else if ((YPos > 0) & (XPos < 0)) {
    g_HeadingHome = atan((-YPos / -XPos) + PI);
  } else if  ((YPos < 0) & (XPos > 0)) {
    g_HeadingHome = atan((-YPos / -XPos)) + PI;
  } else {
      g_HeadingHome = atan2(-YPos, -XPos);
  }
}


int32_t GetDisHome(void)
{
  return g_DistanceHome;
}


//**********************************************************************************
// base colour detection
//**********************************************************************************
void setHomeColour(void)
{
  read_colour();
  int* colour = getColour(); 
  memcpy(homeColour, colour, 3 * sizeof(int)); 
}

int isHome(void)
{
  int* colour = getColour();

  return((abs(colour[0]-homeColour[0]) <= HOME_ERROR )
          && (abs(colour[1]-homeColour[1]) <= HOME_ERROR)
          && (abs(colour[2]-homeColour[2]) <= HOME_ERROR));
}


