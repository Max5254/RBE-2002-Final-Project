#include "fan.h"

Fan::Fan(){}


void Fan::initialize(int fanPort, int tiltPort){
  fanMotor.attach(fanPort);
  tiltMotor.attach(tiltPort);
  fanMotor.write(10);
  // delay(2000);
  // fanMotor.write(0);
}

void Fan::setFan(bool on){
  if(on){
    fanMotor.write(180);
  } else {
    fanMotor.write(10);
  }
}

void Fan::setAngle(int angle){
  angle = map(angle,0,210,180,0);
  tiltMotor.write(angle);
}
