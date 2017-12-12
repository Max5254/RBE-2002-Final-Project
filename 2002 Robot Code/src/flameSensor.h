#ifndef flameSensor_h
#define flameSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "helpers.h"
#include "Drive.h"

#define NO_VALUE 100

// much of the communication is taken from the example code provided by the retailer
// https://www.dfrobot.com/wiki/index.php/Positioning_ir_camera

class flameSensor{
public:
  flameSensor();
  void initialize();

  int getX1();
  int getY1();
  int getActive();
  void get();
  double getHAngle();
  double getVAngle();
  bool checkFlame(double x, double y, double t);
  double getCandleX();
  double getCandleY();
  double getCandleZ();

  double  bestDist, bestVAngle;


private:
  double bestX, bestXt, bestY, bestYt, bestZ, cameraVerticalTheta;

void Write_2bytes(byte,byte);

int IRsensorAddress = 0xB0;
//int IRsensorAddress = 0x58;
int slaveAddress;
boolean ledState = false;
byte data_buf[16];
int i;

int Ix[4];
int Iy[4];
int s;

};

#endif
