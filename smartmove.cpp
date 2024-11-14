/************************************
 *          smartmove.cpp           *
 * 
 * Moves robot based on sensors
 * 
 * Arthors 
 *        Mark   Day
 *        Jack   Wilson
 *        Kieran James
 *************************************/


#include "smartmove.h"
#include "return_to_base.h"
#include "weight_collection.h"
#include "sensors.h"
#include "motors.h"

//**********************************************************************************
// Avoidance Globals
//**********************************************************************************
#define DEADBAND_ANGLE 15 * 3.141592654  / 180 
#define STUCK_THRESHOLD 200
#define MOVED_THRESHOLD 10
#define TURN_BACK_DELAY 500

int movegoal;
bool goalflag;


//**********************************************************************************
// Obstacles Globals
//**********************************************************************************
#define OBSTACLE_THRESHOLD 250
#define OBSTACLE_LEFT_THRESHOLD 150 
#define OBSTACLE_RIGHT_THRESHOLD 150

bool g_obstacle = false;
bool g_obstacleRight = false;
bool g_obstacleLeft = false;
uint32_t g_ObstacleFreeTime = 0;
enum ObstacleAvoidance_t {
  NO_OBSTACLE,
  OBSTACLE,
  LEFT_AVOID,
  RIGHT_AVOID,
};
enum ObstacleAvoidance_t g_ObstacleAvoidance = NO_OBSTACLE;


//**********************************************************************************
//Postion Globals in mm
//**********************************************************************************
int32_t g_XPos = 0;
int32_t g_YPos = 0;
int32_t g_LastOdom = 0;
float g_lastHeading = 0;


//**********************************************************************************
// Position x,y in mm
//**********************************************************************************
void CalcPos(void)
{
  int32_t currentOdom = GetOdom();
  float heading = GetHeadingRad();
  g_XPos += (currentOdom - g_LastOdom) * cos(g_lastHeading);
  g_YPos += (currentOdom - g_LastOdom) * sin(g_lastHeading);
  g_LastOdom = currentOdom;
  g_lastHeading = heading;
}


void resetHomePos(void)
{
  g_XPos = g_YPos = 0;
}


int GetXPos(void)
{
  return g_XPos;
}


int GetYPos(void)
{
  return g_YPos;
}


//**********************************************************************************
// OBSTACLE
//**********************************************************************************
void CheckObstacles(void)
{
  g_obstacle = (TofcentreValue() < OBSTACLE_THRESHOLD);
  g_obstacleLeft = (TofleftValue() < OBSTACLE_LEFT_THRESHOLD);
  g_obstacleRight = (TofrightValue() < OBSTACLE_RIGHT_THRESHOLD);

  if ((g_obstacle || g_obstacleLeft || g_obstacleRight) && (g_ObstacleAvoidance == NO_OBSTACLE)){
    g_ObstacleAvoidance = OBSTACLE;
  }
}


//**********************************************************************************
// MOVE RANDOM INTO FREE SPACE
//**********************************************************************************
void MoveForwardAvoid(int speed)
{
  int centre = TofcentreValue();
  int left = TofleftValue();
  int right = TofrightValue(); 

  if ((centre < 250) | (left < 150) | (right < 150)) {
    if (centre < 50){
      Serial.println("POLE");
      MoveBackward(10);
      delay(500);
      TurnLeftForTime(10, 900);
    } else if (left < STUCK_THRESHOLD && centre < STUCK_THRESHOLD && right < STUCK_THRESHOLD) {
      TurnLeftForTime(10, 1800); 
      Serial.println("STUCK");
    } else if (left < right) {
      TurnRight(speed);
    } else {
      TurnLeft(speed);
    }
  } else {
    MoveForward(speed);
  }
}


//**********************************************************************************
// MOVE TO 
//**********************************************************************************
void MoveTo(int xPos, int yPos, int speed)
{
  int deltaX = (xPos - g_XPos);
  int deltaY = (yPos - g_YPos);
  float Heading = atan2f(deltaY, deltaX);
  float currentHeading = GetHeadingRad();

  float headingDifference = Heading - currentHeading;
  if (headingDifference > PI) headingDifference -= 2 * PI;
  if (headingDifference < -PI) headingDifference += 2 * PI;

  CheckObstacles();

  if (TofcentreValue() < 50){
    MoveBackward(10);
    delay(500);
    TurnLeftForTime(10, 900);
  }

  switch (g_ObstacleAvoidance)
  {
  case NO_OBSTACLE:
    if (headingDifference < -DEADBAND_ANGLE) {
      TurnLeft(speed);
    } else if (headingDifference > DEADBAND_ANGLE) {
      TurnRight(speed);
    } else {
      Serial.println("No Obstacle heading striaght");
      MoveForwardAvoid(speed);
    }
    break;


  case OBSTACLE:
    Serial.println("obstacle");
    if (TofleftValue() > TofrightValue()) {
        g_ObstacleAvoidance = RIGHT_AVOID;
      } else {
        g_ObstacleAvoidance = LEFT_AVOID;
      }
    break;


  case LEFT_AVOID:
    if (g_obstacle) {
        TurnRight(speed);
        Serial.println("turn right");
      } else {
        MoveForwardAvoid(speed);
        if (!g_obstacleLeft){
          if (g_ObstacleFreeTime == 0){
            g_ObstacleFreeTime = millis();
          } else if (millis() > (g_ObstacleFreeTime + TURN_BACK_DELAY)){
            Serial.println("free left");
            g_ObstacleAvoidance = NO_OBSTACLE;
            g_ObstacleFreeTime = 0;
          }
        }
      }
    break;


  case RIGHT_AVOID:
    if (g_obstacle) {
        TurnLeft(speed);
        Serial.println("turn left");
      } else {
        MoveForwardAvoid(speed);
        if (!g_obstacleRight) {
          if (g_ObstacleFreeTime == 0){
            g_ObstacleFreeTime = millis();
          } else if (millis() > (g_ObstacleFreeTime + TURN_BACK_DELAY)){
            Serial.println("free right");
            g_ObstacleAvoidance = NO_OBSTACLE;    
            g_ObstacleFreeTime = 0;
          }
        }
      }
    break;
  }
}



//**********************************************************************************
// FORWARD FOR DISTANCE mm
//**********************************************************************************
int MoveForwardsDistance(int distance, int speed)
{
if (!goalflag) {
  movegoal = GetOdom() + distance;
  goalflag = true;
  return 0;
} else {
  if (movegoal > GetOdom()) {
      MoveForward(speed);
      return 0;
    } else {
      goalflag = false;
      return 1;
    }
  }
}


//**********************************************************************************
// Obstacle protocal 
//**********************************************************************************
uint8_t checkRamp(void)
{
  if (TofbottomLeftValue() < 100 && TofbottomRightValue() < 100){
    MoveBackward(10);
    set_motor();
    delay(500);
    updateTof();
    if (TofleftValue() > TofrightValue()){
      TurnLeftForTime(10, 1200);
    } else {
      TurnRightForTime(10, 1200);
    }
    IMUloop();
    updateTof();
    Serial.println("RAMP");
    return 1;
  } else {
    return 0;
  }
}

uint8_t checkAngle(void)
{
  uint8_t weight_flag = 0;
  if (GetAngleofAttack() > 4 && GetDisHome() > 500 ) {
    Serial.println("ANGLE");
    MoveBackward(10);
    set_motor();
    delay(500);
    updateTof();
    if (TofleftValue() > TofrightValue()){
      TurnLeftForTime(10, 1200);
    } else {
      TurnRightForTime(10, 1200);
    }
    IMUloop();
    updateTof();
  }
  return weight_flag;
}