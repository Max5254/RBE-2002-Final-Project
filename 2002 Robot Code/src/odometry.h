#ifndef odometry_h
#define odometry_h

#include "Arduino.h"
#include "Encoder.h"

class Odom {
public:
  Odom(double,double,double);
  void setScale(double,double);
  void track();
  void resetEncoders();

  void reset(double,double,double);
  long getAverageEncoder();
  double getX();
  double getY();
  double getTheta();

private:
  //Encoders
  const int rightEncoderA = 18;
  const int rightEncoderB = 19;
  const int leftEncoderA = 2;
  const int leftEncoderB = 3;
  Encoder leftEncoder;
  Encoder rightEncoder;

  double x, y, theta;
  double driveScale, turnScale;
  long lastLeft, lastRight, leftTicks, rightTicks;
  float leftIn, rightIn, avgIn;

};

#endif
