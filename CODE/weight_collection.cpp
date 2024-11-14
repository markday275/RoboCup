/************************************
 *       weight_collection.cpp      *
 * 
 * 
 * Arthors 
 *        Mark   Day
 *        Jack   Wilson
 *        Kieran James
 *************************************/


#include "weight_collection.h"
#include "Arduino.h"
#include "sensors.h"


//**********************************************************************************
// WEIGHT GLOBALS
//**********************************************************************************
#define MAX_TOF_DIST               1200
#define ERROR_THRESHOLD            200 
#define CENTRE_ERROR               150
#define DIST_TOF_ULTRA             115

bool isWeightDetected = false;
bool isWeightDetectedLeft = false;
bool isWeightDetectedRight = false;
bool isWeightDetectedCenter = false;


//**********************************************************************************
// Colour Sensor Globals
//**********************************************************************************
int playColour[3] = {108,73,64};
#define COLOUR_ERROR 20


//**********************************************************************************
// SET WEIGHTS BOOLS
//**********************************************************************************
void setWeightDetection(bool center, bool left, bool right) {
    isWeightDetected = center || left || right;
    isWeightDetectedCenter = center;
    isWeightDetectedLeft = left;
    isWeightDetectedRight = right;
}

//**********************************************************************************
// WEIGHT DETECTION
//**********************************************************************************
void weight_scan() {
  int tofbottomleft = TofbottomLeftValue();
  int tofbottomright = TofbottomRightValue();
  int tofCentre = TofcentreValue();
  setWeightDetection(false, false, false);

  if ((tofbottomleft > MAX_TOF_DIST) && (tofbottomright > MAX_TOF_DIST)) {
    setWeightDetection(false, false, false);
    return;
  }

  if ((((tofbottomleft + tofbottomright) / 2) < max(tofbottomleft, tofbottomright) + CENTRE_ERROR) 
        && (((tofbottomleft + tofbottomright)/2) > max(tofbottomleft, tofbottomright) - CENTRE_ERROR) 
        && (((tofbottomleft + tofbottomright)/2) + ERROR_THRESHOLD < tofCentre)) {
    setWeightDetection(true, false, false);
    // Weight detected by both bottom tof
  } else if (tofbottomleft + ERROR_THRESHOLD < tofCentre) {
    setWeightDetection(false, true, false);
    // Weight detected by left tof
  } else if (tofbottomright + ERROR_THRESHOLD < tofCentre) {
    setWeightDetection(false, false, true);
    // Weight detected by right tof
  }
}


bool GetWeightDetection(void)
{
  return isWeightDetected;
}


bool GetWeightDetectionLeft(void)
{
  return isWeightDetectedLeft;
}


bool GetWeightDetectionRight(void)
{
  return isWeightDetectedRight;
}


bool GetWeightDetectionCenter(void)
{
  return isWeightDetectedCenter;
}


//**********************************************************************************
// PLAYING FIELD
//**********************************************************************************
void setPlayColour(void)
{
  read_colour();
  int* colour = getColour(); 
  memcpy(playColour, colour, 3 * sizeof(int)); 
}


int isPlay(void)
{
  int* colour = getColour();
  return((abs(colour[0]-playColour[0]) <= COLOUR_ERROR )
          && (abs(colour[1]-playColour[1]) <= COLOUR_ERROR)
          && (abs(colour[2]-playColour[2]) <= COLOUR_ERROR));
}

