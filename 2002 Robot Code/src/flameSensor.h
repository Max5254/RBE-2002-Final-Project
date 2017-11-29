#ifndef flameSensor_h
#define flameSensor_h

#include <Arduino.h>
#include <Wire.h>


class flameSensor{
public:
  flameSensor();
  void initialize();

  int getX1();
  int getY1();


private:

  void get();
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