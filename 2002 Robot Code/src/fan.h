#ifndef fan_H
#define fan_H

#include <Arduino.h>
#include <Servo.h>

class Fan{
public:
  Fan();
  void initialize(int);
  void setFan(bool);

private:
  Servo fanMotor;


};

#endif
