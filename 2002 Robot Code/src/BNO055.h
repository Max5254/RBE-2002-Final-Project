#ifndef BNO055_h
#define BNO055_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

class BNO055{
public:
  BNO055();
  void initialize();
  void displaySensorDetails();
  void displayCalStatus();
  void displaySensorStatus();

  void calibrate();

  float getX();
  float getY();
  float getZ();



private:
  void loadCalibration();


  Adafruit_BNO055 bno;

};

#endif
