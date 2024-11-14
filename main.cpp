
/********************************************************************************
 *                                   ROBOCUP                              
 *        
 *  main.cpp only runs the task schedular
 *  Refer to finite_state_machine for functionality   
 *  
 *  Template by: Logan Chatfield, Ben Fortune, Lachlan McKenzie, Jake Campbell
 * 
 *  Written By:
 *            Mark   Day
 *            Jack   Wilson
 *            Kieran James
 ******************************************************************************/

#include <Wire.h>                   
#include <TaskScheduler.h>          

// Custom headers
#include "motors.h"
#include "sensors.h"
#include "weight_collection.h"
#include "return_to_base.h" 
#include "finite_state_machine.h"
#include "smartmove.h"

//**********************************************************************************
// Local Definitions
//**********************************************************************************

// Task period Definitions
#define IR_READ_TASK_PERIOD                 100
#define INDUCTIVE_READ_TASK_PERIOD          100
#define COLOUR_READ_TASK_PERIOD             500
#define SET_MOTOR_TASK_PERIOD               50
#define WEIGHT_SCAN_TASK_PERIOD             50

#define PRINT_TOF_SERIAL_TASK_PERIOD        100
#define RUN_FINITE_STATE_TASK_PERIOD        80
#define PRINT_ENCODER_SERIAL_TASK_PERIOD    40
#define IMU_TASK_PERIOD                     40 
#define POS_TASK_PERIOD                     100
#define HOME_VECTOR_TASK_PERIOD             100
#define ODOMETRY_TASK_PERIOD                40
#define PRINT_WEIGHT_SCAN_TASK_PERIOD       50
#define UPDATE_TOF_TASK_PERIOD              50


// Task execution amount definitions
#define IR_READ_TASK_NUM_EXECUTE           -1
#define INDUCTIVE_READ_TASK_NUM_EXECUTE    -1
#define COLOUR_READ_TASK_NUM_EXECUTE       -1
#define SET_MOTOR_TASK_NUM_EXECUTE         -1
#define WEIGHT_SCAN_TASK_NUM_EXECUTE       -1

#define PRINT_TOF_SERIAL_NUM_EXECUTE       -1
#define RUN_FINITE_STATE_NUM_EXECUTE       -1
#define PRINT_ENCODER_SERIAL_NUM_EXECUTE   -1
#define IMU_NUM_EXECUTE                    -1
#define POS_NUM_EXECUTE                    -1
#define HOME_VECTOR_NUM_EXECUTE            -1
#define ODOMETRY_NUM_EXECUTE               -1
#define PRINT_WEIGHT_SCAN_NUM_EXECUTE      -1
#define UPDATE_TOF_NUM_EXECUTE             -1


// Pin deffinitions
#define IO_POWER  49


// Serial deffinitions
#define BAUD_RATE 115200 


//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
void robot_init();
void task_init();


//**********************************************************************************
// Task Scheduler and Tasks
//**********************************************************************************

// Tasks for reading sensors 
Task tRead_infrared(      IR_READ_TASK_PERIOD,                IR_READ_TASK_NUM_EXECUTE,             &read_infrared);
Task tRead_inductive(     INDUCTIVE_READ_TASK_PERIOD,         INDUCTIVE_READ_TASK_NUM_EXECUTE,      &read_inductive);
Task tRead_colour(        COLOUR_READ_TASK_PERIOD,            COLOUR_READ_TASK_NUM_EXECUTE,         &read_colour);
Task tPrintTofserial(     PRINT_TOF_SERIAL_TASK_PERIOD,       PRINT_TOF_SERIAL_NUM_EXECUTE,         &PrintTofSerial);
Task tPrintEncoderSerial( PRINT_ENCODER_SERIAL_TASK_PERIOD,   PRINT_ENCODER_SERIAL_NUM_EXECUTE,     &printEncoderSerial);
Task tImuRefresh(         IMU_TASK_PERIOD,                    IMU_NUM_EXECUTE,                      &IMUloop);
Task tUpdateTof(          UPDATE_TOF_TASK_PERIOD,             UPDATE_TOF_NUM_EXECUTE,               &updateTof);

// Task to set the motor speeds and direction
Task tSet_motor(          SET_MOTOR_TASK_PERIOD,              SET_MOTOR_TASK_NUM_EXECUTE,           &set_motor);
Task tOdomReading(        ODOMETRY_TASK_PERIOD,               ODOMETRY_NUM_EXECUTE,                 &OdomReading);
Task tCalcPos(            POS_TASK_PERIOD,                    POS_NUM_EXECUTE,                      &CalcPos);
Task tCalcHome(           HOME_VECTOR_TASK_PERIOD,            HOME_VECTOR_NUM_EXECUTE,              &CalcHome);

// Tasks to scan for weights and collection upon detection
Task tWeightScan(         PRINT_WEIGHT_SCAN_TASK_PERIOD,      PRINT_WEIGHT_SCAN_NUM_EXECUTE,        &weight_scan);

Task trunStateMachine(    RUN_FINITE_STATE_TASK_PERIOD,       RUN_FINITE_STATE_NUM_EXECUTE,         &runStateMachine);
Scheduler taskManager;


//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  pin_init();
  Wire.begin();
  motor_init();
  TofSensorInit();
  encoderSetUp();
  IMUsetup();
  task_init();
  robot_init();
  colourSetup();
  init_infrared();
  setHomeColour();
}


//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work) 
// Set as high or low
//**********************************************************************************
void pin_init(){
    
    Serial.println("Pins have been initialised \n"); 

    pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
    digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
    pinMode(START_BUTTON, INPUT);
}


//**********************************************************************************
// Set default robot state
//**********************************************************************************
void robot_init() {
    Serial.println("Robot is ready \n");
}


//**********************************************************************************
// Initialise the tasks for the scheduler
//**********************************************************************************
void task_init() {  
  taskManager.init();     
  taskManager.addTask(tRead_infrared);
  taskManager.addTask(tRead_inductive);
  taskManager.addTask(tRead_colour);
  taskManager.addTask(tSet_motor); 
  taskManager.addTask(tPrintTofserial);
  taskManager.addTask(trunStateMachine);
  taskManager.addTask(tPrintEncoderSerial);
  taskManager.addTask(tImuRefresh);
  taskManager.addTask(tOdomReading);
  taskManager.addTask(tCalcPos);
  taskManager.addTask(tCalcHome);
  taskManager.addTask(tWeightScan);
  taskManager.addTask(tUpdateTof);

  tRead_infrared.enable();
  tRead_inductive.enable();
  tRead_colour.enable();
  tSet_motor.enable();
  tPrintTofserial.enable();
  trunStateMachine.enable();
  tImuRefresh.enable();
  tOdomReading.enable();
  tCalcPos.enable();
  tCalcHome.enable();
  tWeightScan.enable();
  tUpdateTof.enable();


 Serial.println("Tasks have been initialised \n");
}



//**********************************************************************************
// MAIN LOOP
//**********************************************************************************
void loop() {
  taskManager.execute();
}
