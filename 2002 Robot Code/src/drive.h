#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "PID_v1.h"
#include <Servo.h>
#include "Odometry.h"
#include "BNO055.h"
#include "wallSensors.h"
#include "helpers.h"
#include <LiquidCrystal.h>

extern LiquidCrystal lcd; //reference lcd object declaired in main
extern BNO055 IMU;
extern wallSensors walls;

class Drive {
public:

  Drive();
  void initialize(int,int);
  void arcadeDrive(double,double);
  bool turnToAngle(double,bool);
  bool driveDistance(double,double,bool);
  void driveStraight(double,double,bool);
  void odometry();
  void reset(double,double,double);
  double angleDiff(double angle1, double angle2);
  double getX();
  double getY();
  double getTheta();
  void navigation(bool,double);
  float getLeftEncoder();
  float getRightEncoder();

private:
  Odom odom;
  Servo leftDrive;
  Servo rightDrive;

  enum navigationStates {
  FOLLOWING_WALL,
  TURNING_LEFT,
  PID_FORWARD,
  PID_FORWARD_LONG,
  TURNING_RIGHT,
  STOPPING,
  PID_WALL
};

double PIDWallDistance;

int navAngle;

navigationStates navStates = FOLLOWING_WALL;


  double fixAngle(double);
  int frcToServo(double);
  bool booleanDelay(bool, unsigned int);
  unsigned int lastLatched;


  //drive forward constants
  double driveSlewRate = 0.01;
  double driveNegativeSlewRate = 0.1;
  double upSlew,downSlew;
  long driveStartingPoint;
  bool driveStarting = true;
  double driveTolerance = 1.5;
  double driveInput, driveOutputDesired, driveOutput, driveSetpoint;
  double Kp_drive = 0.3, Ki_drive = 0.006, Kd_drive = 0.01; // i .004  .12
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
  double Kp_turn = 0.01, Ki_turn = 0.005, Kd_turn = 0.0039, Kf_turn = 0.21; //old .045,.017,.004
  PID turnPID;

  double wallError;
  double Kp_wall = 4;
};

#endif
