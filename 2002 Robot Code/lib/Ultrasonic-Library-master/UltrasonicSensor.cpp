/*
  ultrasonicSensor.cpp - Sensor class for ultrasonic library
  Takes 5 samples and averages them together for a better reading.
 */

#include "Arduino.h"
#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(int pingPin) {
  pinMode(pingPin, OUTPUT);
  _pingPin = pingPin;
}
void UltrasonicSensor::giveValue(unsigned long timeDifference){   
   recentTimeDifference=timeDifference;
}
void UltrasonicSensor::pulsePin(){
	//Serial.println("pulsing");
	digitalWrite(_pingPin, LOW);
    digitalWrite(_pingPin, HIGH);
	delayMicroseconds(250);
    digitalWrite(_pingPin, LOW);
}

float UltrasonicSensor::distance() {
  noInterrupts();
  dist = ((562.6 * ((recentTimeDifference-700) / 1000000.0)) * 12.0);
  //dist = recentTimeDifference; //error testing 

  interrupts();

  return dist;
}
