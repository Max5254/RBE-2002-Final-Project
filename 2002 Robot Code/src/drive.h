#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "PID_v1.h"
#include <Servo.h>
#include "Odometry.h"
#include <LiquidCrystal.h>

extern LiquidCrystal lcd; //reference lcd object declaired in main

class Drive {
public:

  Drive();
  void initialize(int,int);
  void arcadeDrive(double,double);
  bool turnToAngle(double,bool);
  bool driveDistance(double,double,bool);
  void odometry();
  void reset(double,double,double);
  double getX();
  double getY();
  double getTheta();

private:
  Odom odom;
  Servo leftDrive;
  Servo rightDrive;


  int frcToServo(double);
  bool booleanDelay(bool, unsigned int);
  unsigned int lastLatched;


  //drive forward constants
  double driveSlewRate = 0.01;
  double driveNegativeSlewRate = 0.1;
  double upSlew,downSlew;
  long driveStartingPoint;
  bool driveStarting = true;
  double driveTolerance = 0.75;
  double driveInput, driveOutputDesired, driveOutput, driveSetpoint;
  double Kp_drive = 0.12, Ki_drive = 0.006, Kd_drive = 0.01; // i .004
  PID drivePID;

  //drive straight constants
  double straightInput, straightOutputDesired, straightOutput, straightSetpoint;
  double Kp_straight = 0.015, Ki_straight = 0, Kd_straight = 0.0;
  PID straightPID;

  //turning constants
  double turnSlewRate = 0.01;
  double turnNegativeSlewRate = 0.5;
  double turnTolerance = 1.5;
  double turnInput, turnOutputDesired, turnOutput, turnSetpoint;
  double Kp_turn = 0.014, Ki_turn = 0.02, Kd_turn = 0.003; //old p=0.015, old d=0.001-->0.002
  PID turnPID;
};

#endif
