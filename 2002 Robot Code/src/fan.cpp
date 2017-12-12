#include "fan.h"

Fan::Fan(){}

// init the fan ESC and tilt servo
void Fan::initialize(int fanPort, int tiltPort){
  fanMotor.attach(fanPort);
  tiltMotor.attach(tiltPort);
  fanMotor.write(10); // value to arm the ESC
}

// turn the fan on or off
void Fan::setFan(bool on){
  if(on){
    fanMotor.write(180);
  } else {
    fanMotor.write(10);
  }
}

// sets the servo angle of the fan (0 deg is horrizontal to the ground)
void Fan::setAngle(int angle){
  angle = map(angle,0,210,180,0); // map between servo input and servo rotation amount
  tiltMotor.write(angle);
}
