#include "fan.h"

Fan::Fan(){}


void Fan::initialize(int fanPort){
  fanMotor.attach(fanPort);
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
