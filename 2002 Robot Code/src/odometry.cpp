#include "odometry.h"



Odom::Odom(double startX, double startY, double startTheta) :
leftEncoder(leftEncoderA,leftEncoderB),
rightEncoder(rightEncoderA,rightEncoderB)
{
  x = startX;
  y = startY;
  theta = startTheta;
}

//set scales to convert from tics to inches
void Odom::setScale(double _driveScale, double _turnScale){
  driveScale = _driveScale;
  turnScale = _turnScale;
}

void Odom::track(){
  //Get delta
  long leftCount = leftEncoder.read();
  long rightCount = rightEncoder.read();

  leftTicks = leftCount - lastLeft;
  rightTicks = rightCount - lastRight;

  //Save last values
  lastLeft = leftCount;
  lastRight = rightCount;
  //scale to inches
  leftIn = leftTicks * driveScale;
  rightIn = rightTicks * driveScale;
  //get average
  avgIn = (leftIn + rightIn) / 2.0;
  //Get theta
  // theta += (leftTicks - rightTicks) / turnScale;
  theta = IMU.getX();

  // //Wrap theta
  // if(theta > 180)
  // theta -= 360;
  // if(theta < -180)
  // theta += 360;

  //add new componets
  y += avgIn * cos((theta * 3.14) / 180);
  x += avgIn * sin((theta * 3.14) / 180);
}

void Odom::resetEncoders(){
  leftEncoder.write(0);
  rightEncoder.write(0);
}

//Reset to new location
void Odom::reset(double newX,double newY, double newTheta){
  x = newX;
  y = newY;
  theta = newTheta;
  lastLeft = 0;
  lastRight = 0;
  resetEncoders();
}

long Odom::getAverageEncoder(){
  return ((leftEncoder.read() + rightEncoder.read()) / 2) * driveScale;
}

float Odom::getLeftEncoder(){
  return leftEncoder.read() * driveScale;
}

float Odom::getRightEncoder(){
  return rightEncoder.read() * driveScale;
}

double Odom::getX(){return x;}
double Odom::getY(){return y;}
double Odom::getTheta(){return theta;}

double Odom::distToPoint(double x2, double y2){
  return sqrt(pow(x - x2, 2) + pow(y - y2, 2));
}
