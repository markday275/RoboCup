/************************************
 *        finite_state_machine.cpp       *
 * 
 * Finite state machine only 
 * 
 * Arthors 
 *        Mark   Day
 *        Jack   Wilson
 *        Kieran James
 *************************************/


#include "finite_state_machine.h"
#include "weight_collection.h"
#include "return_to_base.h"
#include "sensors.h"
#include "motors.h"
#include "smartmove.h"
#include "Arduino.h"

//**********************************************************************************
// States 
//**********************************************************************************
enum states {
  START,
  HOME,
  SEARCHING,
  WEIGHTDETECTION,
  WAITFORIR,
  CHECKINDUCTIVE,
  TARGETCOLLECTION,
  DUMMYREJECTION,
  RETURNHOME,
  OPPONENTBASE,
  RAMP,
};
enum states g_CurrentState = START;


//**********************************************************************************
// Program timer
//**********************************************************************************
#define RUN_TIME 1000 * 80
u_int StartTime;


//**********************************************************************************
// Weight Detection and Collection globals 
//**********************************************************************************
#define MAX_WEIGHTS             3
#define TIMER_RETURN_WEIGHT     2
#define DETECTION_TIMEOUT       7000
#define COLLECTION_TIMEOUT      5000
int num_weights = 0;
static unsigned long collection_watchdog = 0;
static unsigned long move_timer = 0;
static unsigned long Detectiontimer;
// Collection Sequence Waits
#define COLLECTION_STAGE_1      50 
#define COLLECTION_STAGE_2      500 + COLLECTION_STAGE_1
#define COLLECTION_STAGE_3      500 + COLLECTION_STAGE_2
#define COLLECTION_STAGE_FINAL  1000 + COLLECTION_STAGE_3
// Rejection Sequence Waits
#define REJECTION_STAGE_1       50 
#define REJECTION_STAGE_2       500 + REJECTION_STAGE_1
#define REJECTION_STAGE_3       1000 + REJECTION_STAGE_2
#define REJECTION_STAGE_FINAL   1000 + REJECTION_STAGE_3

//**********************************************************************************
// Home Globals
//**********************************************************************************
#define NOT_CLOSE_HOME 800
#define LIFTED_ANGLE   4
uint8_t g_playColourflag = 0;

//**********************************************************************************
// Ramp Globals
//**********************************************************************************
#define RAMPANGLE 6
#define RAMPTOFS  200

//**********************************************************************************
// RunStateMachine
//**********************************************************************************
void runStateMachine(void) 
{
  // Special Condictions 
  if (millis() > (RUN_TIME + StartTime) && (num_weights >= TIMER_RETURN_WEIGHT) && (g_CurrentState == SEARCHING)) {
    g_CurrentState = RETURNHOME;
  } 
  if (num_weights == MAX_WEIGHTS) {
    g_CurrentState = RETURNHOME;
    frontGateHalfClose();
  } 
  if (isHome() && g_CurrentState != START){
    g_CurrentState = HOME;
  }
  if (!g_playColourflag && GetDisHome() > NOT_CLOSE_HOME) {
    setPlayColour();
    g_playColourflag = 1;
  } 
  if (!isHome() && !isPlay() && (GetDisHome() > NOT_CLOSE_HOME) && g_playColourflag && (GetAngleofAttack() < LIFTED_ANGLE)) {
    g_CurrentState = OPPONENTBASE;
  }
  if (((GetAngleofAttack() > RAMPANGLE || ((TofbottomLeftValue() < RAMPTOFS) && (TofbottomRightValue() < RAMPTOFS))) 
      && g_CurrentState != START) 
      && GetDisHome() > NOT_CLOSE_HOME){
    g_CurrentState = RAMP;
  }


  // State Machine
  Serial.println(g_CurrentState);
  switch (g_CurrentState) {

    case START: {
    // waits for start button push
      frontGateCentre();
      BackGateClose();
      if (digitalRead(START_BUTTON)) {
        StartTime = millis();
        g_CurrentState = SEARCHING;
      }
      break;
    }


    case SEARCHING: {
    // Main function. Moves unless it sees a weight
      MoveForwardAvoid(10);
      if (get_infrared()) {
        g_CurrentState = CHECKINDUCTIVE;
      } else if (GetWeightDetection()) {
        g_CurrentState = WEIGHTDETECTION;
        Detectiontimer = millis();
      } 
      frontGateCentre();
      BackGateClose();
      break;
    }


    case WEIGHTDETECTION: {
      if (millis() > Detectiontimer + DETECTION_TIMEOUT) {
        Serial.print("Weight Detect timeout");
        g_CurrentState = RAMP;
        return;
      }

      if (GetWeightDetectionLeft()) {
        Serial.println("left");
        if (TofbottomLeftValue() < 200){
          TurnLeftForTime(6, 100);
          collection_watchdog = millis();
          g_CurrentState = WAITFORIR;
        } else if (TofbottomLeftValue() < 400) {
          Serial.println("longturnleft");
          TurnLeftForTime(6, 50);
        }
        MoveForwardAvoid(7);

      } else if (GetWeightDetectionRight()) {
        Serial.println("right");
        if (TofbottomRightValue() < 200){
          TurnRightForTime(6, 100);
          collection_watchdog = millis();
          g_CurrentState = WAITFORIR;
        } else if (TofbottomRightValue() < 400) {
          Serial.println("longturnright");
          TurnRightForTime(6, 50);
        }
        MoveForwardAvoid(7);

      } else if (GetWeightDetectionCenter()) {
        Serial.println("centre");
        MoveForwardAvoid(7);
        if ((TofbottomLeftValue() + TofbottomRightValue())/2 < 300) {
          Serial.println("DRIVE IN");
          collection_watchdog = millis();
          g_CurrentState = WAITFORIR;
        }
      } else {
        g_CurrentState = SEARCHING;
      }
      break;
    }


    case WAITFORIR: {
      // Weight has been seen and robot driving into it.
      if (!get_infrared()) {
        MoveForwardAvoid(7);
        if (millis() > collection_watchdog + COLLECTION_TIMEOUT) {
          Serial.print("Wait IR timeout");
          MoveBackward(10);
          set_motor();
          delay(400);
          TurnLeftForTime(10, 900);
          g_CurrentState = SEARCHING;
          return;
        }

      } else {
        Serial.println("IR DETECTION");
        delay(50);
        MoveForward(0);
        g_CurrentState = CHECKINDUCTIVE;
      }
      break;
    }


    case CHECKINDUCTIVE: {
      delay(100);
      read_inductive();
      move_timer = millis();
      if (get_inductive())
      {
        g_CurrentState = TARGETCOLLECTION;
        Serial.println("Target!");
      } else {
        g_CurrentState = DUMMYREJECTION;
        Serial.println("Dummy!");
      }
      break;
    }


    case TARGETCOLLECTION: {
      uint32_t new_time = millis();
      //Collection Sequence 
      if (new_time  > move_timer + COLLECTION_STAGE_FINAL) {       
        frontGateCentre();
        Serial.println("Collection Completed");
        num_weights += 1;
        g_CurrentState = SEARCHING;
      } else if (new_time  > move_timer + COLLECTION_STAGE_3) {   
        MoveForward(0);
        frontGateClose();
        Serial.println("Collection Stage 3 Completed");
      } else if (new_time  > move_timer + COLLECTION_STAGE_2) {   
        MoveForward(7);
        Serial.println("Collection Stage 2 Completed");
      } else if (new_time  > move_timer + COLLECTION_STAGE_1) {    
        frontGateHalfOpen();
        Serial.println("Collection Stage 1 Completed");
      }
      break;
    }


    case DUMMYREJECTION: {
      uint32_t new_time = millis();
      if (new_time  > move_timer + REJECTION_STAGE_FINAL) {          
        frontGateCentre();
        g_CurrentState = SEARCHING;
        Serial.println("DUMMY REJECTED");
      } else if (new_time  > move_timer + REJECTION_STAGE_3) {   
        MoveForward(0);
        frontGateOpen();
      } else if (new_time > move_timer + REJECTION_STAGE_2) {     
        MoveForward(7);
      } else if (new_time > move_timer + REJECTION_STAGE_1) {      
        frontGateHalfClose();
      }
      break;
    }


    case RETURNHOME: {
      if (isHome()){
        g_CurrentState = HOME;
        MoveForward(0);
      } else {
        Serial.println("Going Home");
        MoveTo(0,0,10);
        frontGateHalfClose();

        //Find weights on way back
        if (get_infrared() && num_weights < MAX_WEIGHTS) {
          g_CurrentState = CHECKINDUCTIVE;
        }
        if (GetWeightDetection() && num_weights < MAX_WEIGHTS) {
          g_CurrentState = WEIGHTDETECTION;
          Detectiontimer = millis();
        } 
      }
      break;
    }


    case HOME: {
      if (num_weights != 0 && millis() > (RUN_TIME + StartTime)) {
        // DO NOT USE THIS. Did not work during competition needs a closed loop alternative 
        MoveForward(10);
        set_motor();
        delay(1000);
        TurnLeftForTime(10, 1600);
        BackGateOpen();
        set_motor();
        delay(200);
        frontGateClose();
        for (size_t i = 0; i < 10; i++)
        {
          MoveBackward(2);
          set_motor();
          delay(300);
          
          MoveBackward(4);
          set_motor();
          delay(100);
          
          MoveBackward(6);
          set_motor();
          delay(100);
          
          MoveBackward(8);
          set_motor();
          delay(100);

          MoveBackward(10);
          set_motor();
          delay(400);

          MoveForward(10);
          set_motor();
          delay(300);

          MoveForward(8);
          set_motor();
          delay(100);

          MoveForward(6);
          set_motor();
          delay(100);

          MoveForward(4);
          set_motor();
          delay(100);

          MoveForward(2);
          set_motor();
          delay(300);

          MoveForward(0);
          set_motor();
          delay(100);

          TurnLeftForTime(7, 100);
        }
        num_weights = 0;
        g_CurrentState = SEARCHING;
        MoveForward(0);

      } else {
        g_CurrentState = SEARCHING;
        MoveForwardAvoid(10);
      }
      break;
    }


    case OPPONENTBASE: {
      MoveBackward(10);
      set_motor();
      delay(500);
      if (TofleftValue() > TofrightValue()){
        TurnLeftForTime(10, 1000);
      } else {
        TurnRightForTime(10, 1000);
      }
      g_CurrentState = SEARCHING;
      break;
    }


    case RAMP: {
      MoveBackward(10);
      set_motor();
      delay(500);
      if (TofleftValue() > TofrightValue()){
        TurnLeftForTime(10, 1000);
      } else {
        TurnRightForTime(10, 1000);
      }
      g_CurrentState = SEARCHING;
      break;
    }
  }
}