#include "helpers.h"

double servoToFrc(int input) { //Converts from a 0 to 180 scale to a -1 to 1 scale
  return ((input - 90.0) / 90.0);
}

int frcToServo(double input){ //Converts from -1 to 1 scale to a 0 180 sclase
    return 90 + (input * 90.0);
}

long lastLatched;
//return true only when a bool has been true for "delay" amount of time
bool booleanDelay(bool latch, int delay){
  if(!latch){
    lastLatched = millis();
    return false;
  } else {
    return millis() - lastLatched > delay;
  }
}

long lastLatchedInverse;
// returns true when a bool is true or has been in the past delay microseconds
bool inverseBooleanDelay(bool latch, int delay){
  if(latch){
    lastLatchedInverse = millis();
    return true;
  } else {
    return millis() - lastLatchedInverse < delay;
  }
}

// arduino map function modified for doubles
double scaleValues(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// //Wrap theta between -180 and 180
float wrap(float theta){
  if(theta > 180)
    theta -= 360;
  if(theta < -180)
    theta += 360;
  return theta;
}
