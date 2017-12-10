#ifndef fan_H
#define fan_H

#include <Arduino.h>
#include <Servo.h>

class Fan{
public:
  Fan();
  void initialize(int,int);
  void setFan(bool);
  void setAngle(int);

private:
  Servo fanMotor;
  Servo tiltMotor;



};

#endif
