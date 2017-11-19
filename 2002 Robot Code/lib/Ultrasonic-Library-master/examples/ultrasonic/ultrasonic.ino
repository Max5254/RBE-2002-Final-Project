//Mandatory Includes
#include <TimerOne.h>
#include <UltrasonicSensor.h>
#include <UltrasonicSensorArray.h>
#define numOfUltrasonics 3 // not necessary for less than 5 sensors


// first, create an Ultrasonic Sensor Array (USA) to receive inputs on pin 2
UltrasonicSensorArray usa(2);
// And then create each ultrasonic sensor (with their output pins)
UltrasonicSensor left(13);
UltrasonicSensor right(12);
UltrasonicSensor front(11);

void setup() {
  // setup serial communication for print statements
  Serial.begin(115200);
  // Add each sensor to the UltrasonicSensorArray
  usa.addSensor(&left);
  usa.addSensor(&right);
  usa.addSensor(&front);
  // Initalize the USA timer and input interrupts
  usa.begin();
}

void loop() {
  // each sensor is update asynchonously by the UltrasonicSesnorArray
  // so we don't have to do any work in loop
  delay(100);
  // clear the screen
  for(int j=0; j<11; j++)
    Serial.println("");
  // print each distance
  Serial.println(left.distance());
  Serial.println(right.distance());
  Serial.println(front.distance());
}
