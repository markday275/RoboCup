/************************************
 *           sensors.cpp            *
 * 
 * all inputs into the robot
 * 
 * Arthors 
 *        Mark   Day
 *        Jack   Wilson
 *        Kieran James
 *************************************/


#include "sensors.h"

#include <stdbool.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <utility/imumaths.h>
#include <Adafruit_TCS34725.h>


//**********************************************************************************
// IR Globals
//**********************************************************************************
#define infrared_pin 21
static bool infrared_value = 0;


//**********************************************************************************
// Inductive Globals
//**********************************************************************************
#define inductive_pin 20
static bool inductive_value = 0;


//**********************************************************************************
// Time of Flights Globals
//**********************************************************************************
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35

const byte SX1509_ADDRESS = 0x3F;

const uint8_t sensorCountL0 = 2;
const uint8_t sensorCountL1 = 4;
const uint8_t xshutPinsL0[8] = {7,2,5,6,7};
const uint8_t xshutPinsL1[8] = {0,1,3,4};

SX1509 io; 
VL53L0X sensorsL0[sensorCountL0];
VL53L1X sensorsL1[sensorCountL1];

int Tofarray[6] = {0};


//**********************************************************************************
// IMU Globals
//**********************************************************************************
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
float g_headingDeg;
float g_angleOfAttack;


//**********************************************************************************
// COLOUR Globals
//**********************************************************************************
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
TwoWire *theWire = &Wire1;


//**********************************************************************************
//colour array
//**********************************************************************************
int g_currentcolour[3];


//**********************************************************************************
// Time of flight 
//**********************************************************************************
void TofSensorInit(void){
  io.begin(SX1509_ADDRESS);
  Wire.begin();
  Wire.setClock(400000);
  for (uint8_t i = 0; i < 8; i++)
  {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }
  for (uint8_t i = 0; i < sensorCountL1; i++)
  {
    io.digitalWrite(xshutPinsL1[i], HIGH);
    delay(10);
    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L1 ");
      Serial.println(i);
      while (1);
    }
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);
    sensorsL1[i].startContinuous(50);
  }
  for (uint8_t i = 0; i < sensorCountL0; i++)
  {
    io.digitalWrite(xshutPinsL0[i], HIGH);
    delay(10);
    sensorsL0[i].setTimeout(500);
    if (!sensorsL0[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L0 ");
      Serial.println(i);
      while (1);
    }
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);
    sensorsL0[i].startContinuous(50);
  }
}


void PrintTofSerial(void) 
{
  for (uint8_t i = 0; i < sensorCountL0; i++)
  {
    Serial.print(Tofarray[i+4]);
    Serial.print('\t');
  }
  for (uint8_t i = 0; i < sensorCountL1; i++)
  {
    Serial.print(Tofarray[i]);
    Serial.print('\t');

  }
  Serial.println();
}


void updateTof(void)
{
  for (uint8_t i = 0; i < sensorCountL0; i++)
  {
    Tofarray[i+4] = sensorsL0[i].readRangeContinuousMillimeters(); 
    if (sensorsL0[i].timeoutOccurred()) { Serial.print(" L0 TIMEOUT"); }
  }
  for (uint8_t i = 0; i < sensorCountL1; i++)
  {
    Tofarray[i] = sensorsL1[i].read(false);
    if (sensorsL1[i].timeoutOccurred()) { Serial.print(" L1 TIMEOUT"); }
  }
}


int TofcentreValue(void)
{
  return min(Tofarray[1], Tofarray[0]);
}


int TofLeftCentreValue (void) 
{
  return Tofarray[1];
}


int TofRightCentreValue (void)
{
  return Tofarray[0];
}


int TofleftValue (void) 
{
  return Tofarray[2];
}


int TofrightValue (void) 
{
  return Tofarray[3];
}


int TofbottomLeftValue (void)
{
  return Tofarray[5];
}


int TofbottomRightValue (void)
{
  return Tofarray[4];
}


//**********************************************************************************
// IMU
//**********************************************************************************
void IMUsetup (void) 
{
  Serial.println("Orientation Sensor Test"); Serial.println("");
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(500);
}

void IMUloop(void)
{
  sensors_event_t orientationData; 
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  g_headingDeg = orientationData.orientation.x;
  g_angleOfAttack = orientationData.orientation.pitch;
}


float GetHeadingDeg(void)
{
  return g_headingDeg;
}


float GetHeadingRad(void)
{
  return (g_headingDeg * PI / 180);
}


float GetAngleofAttack(void)
{
  return(g_angleOfAttack);
}


//**********************************************************************************
// COLOUR
//**********************************************************************************
void colourSetup(void)
{
  Serial.println("Color View Test!");
  if (tcs.begin(TCS34725_ADDRESS , &Wire1)) 
  {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}


void read_colour(void)
{
  uint16_t clear, red, green, blue;
  tcs.setInterrupt(false);  
  delay(60);  
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true); 
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  g_currentcolour[0] = r; g_currentcolour[1] = g; g_currentcolour[2] = b;
}


int* getColour(void)
{
  return g_currentcolour;
}


//**********************************************************************************
// INFRARED SENSOR
//**********************************************************************************
void init_infrared(void)
{
  pinMode(infrared_pin, INPUT);
}


void read_infrared(void) 
{
  //IR sensor is inverted hence the ! not
  infrared_value = !digitalRead(infrared_pin);
}


bool get_infrared(void)
{
  return infrared_value;
}


//**********************************************************************************
// INDUCTIVE SENSOR
//**********************************************************************************
void init_inductive(void)
{
  pinMode(inductive_pin, INPUT);
}


void read_inductive(void) 
{
  //Inductive sensor is inverted hence the ! not
  inductive_value = !digitalRead(inductive_pin);
}


bool get_inductive(void)
{
  return inductive_value;
}

