#ifndef BNO055_h
#define BNO055_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include "helpers.h"

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

//********
// much of this code was adapted from this example provided by Adafruit
// https://github.com/adafruit/Adafruit_BNO055/blob/master/examples/restore_offsets/restore_offsets.ino
//********

extern LiquidCrystal lcd;

class BNO055{
public:
  BNO055();
  void initialize();
  void displaySensorDetails();
  void displayCalStatus();
  void displaySensorStatus();

  void calibrate();
  void reset(int);

  float getX();
  float getY();
  float getZ();


private:
  void loadCalibration();
  float offset;

  Adafruit_BNO055 bno;

};

#endif
