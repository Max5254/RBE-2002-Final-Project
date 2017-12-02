#include "drive.h"

Drive::Drive() :
drivePID(&driveInput,&driveOutputDesired,&driveSetpoint,Kp_drive,Ki_drive,Kd_drive,DIRECT),
straightPID(&straightInput,&straightOutput,&straightSetpoint,Kp_straight,Ki_straight,Kd_straight,DIRECT),
turnPID(&turnInput,&turnOutputDesired,&turnSetpoint,Kp_turn,Ki_turn,Kd_turn,DIRECT),
odom(0,0,0)
{ }

void Drive::initialize(int leftDrivePort, int rightDrivePort){
  //Init PIDs with output limits, mode, and integration ranges
  drivePID.SetOutputLimits(-0.75,0.75);
  drivePID.SetMode(AUTOMATIC);
  drivePID.setIRange(0.5);
  straightPID.SetOutputLimits(-0.5,0.5);
  straightPID.SetMode(AUTOMATIC);
  turnPID.SetOutputLimits(-1, 1);
  turnPID.SetMode(AUTOMATIC);
  turnPID.setIRange(10);

  leftDrive.attach(leftDrivePort, 1000, 2000);
  rightDrive.attach(rightDrivePort, 1000, 2000);

  odom.setScale(0.024, 5.4);  //drive scale , turn scale to map encoder tics to inches and degrees

  odom.reset(0,0,0);  //init robot location to x=0 y =0 theta=0
  navAngle = IMU.getX();
}

/* drive forward a set distance using PID
*  @param setpoint the target distance to drive in inches
*  @param angle    the target angle to hold to while driving
*  @param enabled  turns the loop on or off
*  @return true when within target encoder range for certain time
*/
bool Drive::driveDistance(double setpoint, double angle, bool enabled){
  if(driveStarting){ //reset if new setpoint
    driveStartingPoint = odom.getAverageEncoder();
    driveOutput = 0;
    drivePID.flush();
    straightPID.flush();
    driveStarting = false;
    Serial.print("driving distance: ");
    Serial.print(driveStartingPoint);
    Serial.println(angle);
  }


  driveInput = odom.getAverageEncoder() - driveStartingPoint;
  driveSetpoint = setpoint;
  drivePID.Compute();

  straightInput = IMU.getX();
  straightSetpoint = angle;
  double error = (angle-straightInput);
  if(straightInput > 135 && abs(angle) == 180){
    error += 360;
  }
  if(straightInput < -135 && angle == 0){
    error -= 360;
  }
  straightPID.Compute(error);

  // if (drivePID.getError() > 0) { //invert slew gains if setpoint is backwards
  //   upSlew = driveSlewRate;
  //   downSlew = driveNegativeSlewRate;
  // } else {
  //   downSlew = driveSlewRate;
  //   upSlew = driveNegativeSlewRate;
  // }
  //
  // if(enabled){
  //   if (driveOutputDesired > driveOutput){ //ramp the motor values up and down to avoid jerky motion and wheel slip
  //     driveOutput += driveOutputDesired - driveOutput > upSlew ? upSlew : driveOutputDesired - driveOutput;
  //   } else {
  //     driveOutput -= driveOutput - driveOutputDesired > downSlew ? downSlew : driveOutput - driveOutputDesired;
  //   }
  // } else {
  //   driveOutput = 0;
  // }

  arcadeDrive(driveOutputDesired , straightOutput);

  // Serial.print(abs(drivePID.getError()));
  // Serial.print(" ");
  // Serial.println(driveTolerance);

  //end condition met if in target rangle for 0.5 seconds
  bool done = booleanDelay(abs(drivePID.getError()) < driveTolerance , 500);

  if(done){ //if done reset for next setpoint
    driveStarting = true;
    driveStartingPoint = odom.getAverageEncoder();
    driveOutput = 0;
  }
  return done;
}

void Drive::driveStraight(double speed, double angle, bool enabled){
  straightInput = IMU.getX();
  straightSetpoint = angle;
  double error = (angle-straightInput);
  if(straightInput > 135 && abs(angle) == 180){
    error += 360;
  }
  if(straightInput < -135 && angle == 0){
    error -= 360;
  }

  straightPID.Compute(error);

  // Serial.println(straightPID.getError());


  arcadeDrive(speed, straightOutput);
}

/* turn to specific angle using PID
*  @param angle    the target angle to turn
*  @param enabled  if the loop is running
*  @return true when in range half a second
*/
bool Drive::turnToAngle(double angle, bool enabled){
  turnInput = IMU.getX();
  turnSetpoint = angle;
  double error = (angle-turnInput);
  if(turnInput > 135 && abs(angle) == 180){
    error += 360;
  }
  if(turnInput < -135 && angle == 0){
    error -= 360;
  }
  turnPID.Compute(error);

  lcd.setCursor(0, 1);
  lcd.print(fixAngle(IMU.getX()));

  // if (turnPID.getError() > 0) { //invert slew rates if moving backwards
  //   upSlew = turnSlewRate;
  //   downSlew = turnNegativeSlewRate;
  // } else {
  //   downSlew = turnSlewRate;
  //   upSlew = turnNegativeSlewRate;
  // }
  //
  // if(enabled){
  //   if (turnOutputDesired > turnOutput){ //ramp up and down speed of motors
  //     turnOutput += turnOutputDesired - turnOutput > upSlew ? upSlew : turnOutputDesired - turnOutput;
  //   } else {
  //     turnOutput -= turnOutput - turnOutputDesired > downSlew ? downSlew : turnOutput - turnOutputDesired;
  //   }
  // } else {
  //   turnOutput = 0;
  // }

  arcadeDrive(0, turnOutputDesired);

  //return true if in target range for 0.5 secons
  return booleanDelay(abs(turnPID.getError()) < turnTolerance, 500);
}

void Drive::navigation(bool enabled, double wallDistanceSetpoint){
  if(enabled){
    switch (navStates) {
      case FOLLOWING_WALL:
      wallError = walls.getRight() - wallDistanceSetpoint;
      driveStraight(1, navAngle + wallError * Kp_wall, enabled);
      if(walls.getFront() < 8){
        navStates = TURNING_LEFT;
        navAngle = wrap(navAngle - 90);
      }
      if(walls.getRight() > 900){
        navStates = PID_FORWARD;
      }
      break;

      case TURNING_LEFT:
      if(turnToAngle(navAngle, enabled)){
        navStates = FOLLOWING_WALL;
      }
      break;

      case TURNING_RIGHT:
      if(turnToAngle(navAngle, enabled)){
        if(walls.getFront() > 25){
          navStates = PID_FORWARD_LONG;
        } else {
          navStates = PID_FORWARD;
        }
      }
      break;

      case PID_FORWARD:
      if(driveDistance(8, navAngle, enabled)){
        if(walls.getFront() < 25){
          navStates = FOLLOWING_WALL;
        } else {
          navStates = TURNING_RIGHT;
          navAngle = wrap(navAngle + 90);
        }

      }
      break;

      case PID_FORWARD_LONG:
      if(driveDistance(20, navAngle, enabled)){
        if(walls.getRight() > 900){
          navStates = TURNING_RIGHT;
          navAngle = wrap(navAngle + 90);
        } else {
          navStates = FOLLOWING_WALL;
        }
      }
      break;


      case STOPPING:
      arcadeDrive(0, 0);
      break;
    }
  } else {
    arcadeDrive(0, 0);
  }
}

double Drive::fixAngle(double angle){
  //Fix angle
  while(angle>180){
    angle-=360;
  }
  while(angle<=-180){
    angle+=360;
  }

  return angle;
}

/* control drive motors
*  @param throttle  the forward/backwards speed
*  @param turn      the turning speed (+ is right)
*/
void Drive::arcadeDrive(double throttle, double turn){
  leftDrive.write(frcToServo(throttle - turn));
  rightDrive.write(frcToServo(-turn - throttle));
}

//Converts from -1 to 1 scale to a 0 180 sclase
int Drive::frcToServo(double input){
  return 90 + (input * 90.0);
}

/* returns true when input has been true for certian time
*  @param latch  the value of the condition you want to
*  @param delay  the time the input must be true to return true
*  @return true when in range for specified time
*/bool Drive::booleanDelay(bool latch, unsigned int delay){
if(!latch){
  lastLatched = millis();
  return false;
} else {
  return millis() - lastLatched > delay;
}
}

//run odometry funciton (odom is private to drive)
void Drive::odometry(){
  odom.track();
}
//reset odom data to new location
void Drive::reset(double newX,double newY, double newTheta){
  odom.reset(newX,newY,newTheta);
}

float Drive::getLeftEncoder(){ return odom.getLeftEncoder();}
float Drive::getRightEncoder(){ return odom.getRightEncoder();}


double Drive::getX(){return odom.getX();}
double Drive::getY(){return odom.getY();}
double Drive::getTheta(){return odom.getTheta();}
