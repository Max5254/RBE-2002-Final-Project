#include "wallSensors.h"

wallSensors::wallSensors(){}

// initiallize the distance sensors
void wallSensors::initialize(int _frontTriggerPort, int _frontEchoPort, int _rightTriggerPort, int _rightEchoPort){
  frontEchoPort = _frontEchoPort;
  frontTriggerPort = _frontTriggerPort;
  rightEchoPort = _rightEchoPort;
  rightTriggerPort = _rightTriggerPort;

  initHC_SR04(frontTriggerPort,frontEchoPort);
  initHC_SR04(rightTriggerPort,rightEchoPort);
  frontTimer, rightTimer, loopStartTime = millis();
}

// attach the pins for the ultrasonic sensor
void wallSensors::initHC_SR04(int trigPin, int echoPin){
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

// return the last right value
double wallSensors::getRight(){
  return rightDistance;
}
// return the last front value
double wallSensors::getFront(){
  return frontDistance;
}

// ping the sensors periodically based on input times
void wallSensors::periodicPing(int frontFreq, int rightFreq){
  loopStartTime = millis();
  if(millis() - frontTimer >= frontFreq){ // if read to ping, do it
    frontTimer = millis();
    frontDistance = getHC_SR04(frontTriggerPort,frontEchoPort);
  }
  if(millis() - rightTimer >= rightFreq){ // if read to ping, do it
    rightTimer = millis();
    rightDistance = getHC_SR04(rightTriggerPort, rightEchoPort);
  }
  loopDelay = millis() - loopStartTime; // fing out how long those pings took so it can be compensated later
}

int wallSensors::getLoopDelay(){ // returns how long the pings took
  return loopDelay;
}

// NOT USED
// gets the analog reading from a sharp and converets to distance
double wallSensors::getSharp(int port){
  int value = analogRead(port);
  return (value > 450 || value < 180) ? 999.0 : 16757 * pow(analogRead(port), -1.53);
}

// triggers the ultrasonic and measures it's response
double wallSensors::getHC_SR04(int trigPin, int echoPin){
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH, 5000);
// Calculating the distance
distance = duration*0.034/2.0;
return (distance > 0 ? distance : 2600) / 2.54;  // scale to inches (if timeout make really big as flag)
}
