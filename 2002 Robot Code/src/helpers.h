#ifndef Helpers_h
#define Helpers_h

#include <Arduino.h>

double servoToFrc(int input);
int frcToServo(double input);
bool booleanDelay(bool latch, unsigned int delay);
double scaleValues(double x, double in_min, double in_max, double out_min, double out_max);

enum armState{
  ManualUp,
  ManualDown,
  Off,
  PID
};

enum intakeState{
  IN,
  OUT,
  OFF
};


#endif
