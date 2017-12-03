#include "wallSensors.h"

wallSensors::wallSensors(){}

void wallSensors::initialize(int _frontTriggerPort, int _frontEchoPort, int _rightTriggerPort, int _rightEchoPort){
  frontEchoPort = _frontEchoPort;
  frontTriggerPort = _frontTriggerPort;
  rightEchoPort = _rightEchoPort;
  rightTriggerPort = _rightTriggerPort;

  initHC_SR04(frontTriggerPort,frontEchoPort);
  initHC_SR04(rightTriggerPort,rightEchoPort);
  frontTimer, rightTimer, loopStartTime = millis();

}

void wallSensors::initHC_SR04(int trigPin, int echoPin){
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

double wallSensors::getRight(){
  return rightDistance;
}

double wallSensors::getFront(){
  return frontDistance;
}

void wallSensors::periodicPing(int frontFreq, int rightFreq){
  loopStartTime = millis();
  if(millis() - frontTimer >= frontFreq){
    frontTimer = millis();
    frontDistance = getHC_SR04(frontTriggerPort,frontEchoPort);
  }

  if(millis() - rightTimer >= rightFreq){
    rightTimer = millis();
    rightDistance = getHC_SR04(rightTriggerPort, rightEchoPort);
  }
  loopDelay = millis() - loopStartTime;
}

int wallSensors::getLoopDelay(){
  return loopDelay;
}

double wallSensors::getSharp(int port){
  int value = analogRead(port);
  return (value > 450 || value < 180) ? 999.0 : 16757 * pow(analogRead(port), -1.53);
}

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
return (distance > 0 ? distance : 2600) / 2.54;  // scale to inches
}
