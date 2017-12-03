#ifndef wallSensors_h
#define wallSensors_h

#include "Arduino.h"

class wallSensors{
public:
  wallSensors();
  void initialize(int,int,int,int);

  double getRight();
  double getFront();
  void periodicPing(int,int);
  int getLoopDelay();


private:
  int frontTriggerPort, frontEchoPort, rightTriggerPort, rightEchoPort;

  long duration, loopStartTime, loopDelay, frontTimer, rightTimer;
  int distance;
  double frontDistance, rightDistance;


  void initHC_SR04(int,int);
  double getSharp(int);
  double getHC_SR04(int,int);



};

#endif
