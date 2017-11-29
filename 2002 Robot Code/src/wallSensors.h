#ifndef wallSensors_h
#define wallSensors_h

#include "Arduino.h"

class wallSensors{
public:
  wallSensors();
  void initialize(int,int,int);

  double getRight();
  double getFront();
  double getFront(int);


private:
  int frontTriggerPort, frontEchoPort, rightPort;

  long duration;
  int distance;

  void initHC_SR04(int,int);
  double getSharp(int);
  double getHC_SR04(int,int);



};

#endif
