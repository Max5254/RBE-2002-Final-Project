#include "wallSensors.h"

wallSensors::wallSensors(){}

void wallSensors::initialize(int _frontTriggerPort, int _frontEchoPort, int _rightPort){
  frontEchoPort = _frontEchoPort;
  rightPort = _rightPort;
  frontTriggerPort = _frontTriggerPort;

  initHC_SR04(frontTriggerPort,frontEchoPort);
}

void wallSensors::initHC_SR04(int trigPin, int echoPin){
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

double wallSensors::getRight(){
  return getSharp(rightPort);
}

double wallSensors::getFront(){
  return getHC_SR04(frontTriggerPort,frontEchoPort);
}

double wallSensors::getFront(int freq){
  return getHC_SR04(frontTriggerPort,frontEchoPort);
}

double wallSensors::getSharp(int port){
  return 16757 * pow(analogRead(port), -1.53);
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
distance= duration*0.034/2;
return distance / 2.54;  // scale to inches
}
