#include "helpers.h"

double servoToFrc(int input) { //Converts from a 0 to 180 scale to a -1 to 1 scale
  return ((input - 90.0) / 90.0);
}

int frcToServo(double input){ //Converts from -1 to 1 scale to a 0 180 sclase
    return 90 + (input * 90.0);
}

unsigned int lastLatched;
//return true only when a bool has been true for "delay" amount of time
bool booleanDelay(bool latch, unsigned int delay){
  if(!latch){
    lastLatched = millis();
    return false;
  } else {
    return millis() - lastLatched > delay;
  }
}

double scaleValues(double x, double in_min, double in_max, double out_min, double out_max) //modified version of map()
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float wrap(float theta){
  // //Wrap theta
  if(theta > 180)
    theta -= 360;
  if(theta < -180)
    theta += 360;
  return theta;
}
