#include "drive.h"

Drive::Drive() :
drivePID(&driveInput,&driveOutputDesired,&driveSetpoint,Kp_drive,Ki_drive,Kd_drive,DIRECT),
straightPID(&straightInput,&straightOutput,&straightSetpoint,Kp_straight,Ki_straight,Kd_straight,DIRECT),
turnPID(&turnInput,&turnOutputDesired,&turnSetpoint,Kp_turn,Ki_turn,Kd_turn,Kf_turn,DIRECT),
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

  straightPID.Compute(angleDiff(angle, IMU.getX()));

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

/* drive straight using a PID
*  @param speed    the speed to run at
*  @param angle    the target angle to drive at
*  @param enabled  if the loop is running
*/
void Drive::driveStraight(double speed, double angle, bool enabled){
  straightPID.Compute(angleDiff(angle, IMU.getX()));

  Serial.println(straightPID.getError());

  arcadeDrive(speed, straightOutput);
}

/* turn to specific angle using PID
*  @param angle    the target angle to turn
*  @param enabled  if the loop is running
*  @return true when in range half a second
*/
bool Drive::turnToAngle(double angle, bool enabled){
  turnPID.Compute(angleDiff(angle, IMU.getX()));

  lcd.setCursor(0, 1);
  lcd.print(IMU.getX());
  lcd.setCursor(8, 1);
  lcd.print(angleDiff(angle, IMU.getX()));

  arcadeDrive(0, turnOutputDesired);

  //return true if in target range for 0.5 secons
  return booleanDelay(abs(turnPID.getError()) < turnTolerance, 500);
}

void Drive::navigation(bool enabled, double wallDistanceSetpoint, bool taskComplete){
  if(enabled){
    // print the current state on the LCD
    lcd.setCursor(14, 0);
    lcd.print(navStates);

    if(analogRead(A2) > 900){ // if you ever see a black line back up!!!!
      navStates = PID_BACKWARDS;
    }

    switch (navStates) {
      case FOLLOWING_WALL:  // drive forward trying to keep wall at constant distance
      lastSawWall = millis();
      // find error from right wall and run P loop to change angle setpoint away from wall
      // if within specific range to front wall, slow down
      wallError = walls.getRight() - wallDistanceSetpoint;
      driveStraight(walls.getFront() < 15 ? 0.6 : 1, navAngle + wallError * Kp_wall, enabled);
      if(walls.getFront() < 8){ // if wall is in front, turn left
        navStates = TURNING_LEFT;
        navAngle = wrap(navAngle - 90);
        notHomeAnymore = true;
      }
      if(walls.getRight() > 21){ // if no more wall on right drive forward away from walls
        navStates = PID_FORWARD;
        notHomeAnymore = true;
      }
      if(abs(odom.getX()) < 8 && abs(odom.getY()) < 8 && notHomeAnymore && taskComplete){ // if you're done and back home, stop
        navStates = STOPPING;
      }
      break;

      case TURNING_LEFT: // PID 90deg to left
      if(turnToAngle(navAngle, enabled)){
        if(hitGap){ // if you came here from hitting a gap in the wall drive forward untill you see a wall again
          navStates = DRIVE_UNTIL_WALL;
          hitGap = false;
        } else { // if not coming from gap start follwing a wall
          navStates = FOLLOWING_WALL;
        }
      }
      break;

      case TURNING_RIGHT: // PID 90deg to right
      if(turnToAngle(navAngle, enabled)){
        if(walls.getFront() > 27){ //old 20 --> 25 // if no wall in front of you PID forward
          navStates = PID_FORWARD_LONG;
        } else { // if there is a wall in front of ou figure out what that distance is and PID slightly less
          PIDWallDistance = walls.getFront() - 7;
          navStates = PID_WALL;
        }
        // navStates = PID_FORWARD_LONG;
      }
      break;

      case PID_FORWARD: // PID forward to get away from a wall
      if(driveDistance(7, navAngle, enabled)){
        navStates = TURNING_RIGHT;
        navAngle = wrap(navAngle + 90);
      }
      break;

      case PID_BACKWARDS: // PID backwards away from a gap in the wall
      if(driveDistance(-4, navAngle, enabled)){
        navStates = TURNING_LEFT;
        navAngle = wrap(navAngle - 90);
        hitGap = true;
      }
      break;

      case DRIVE_UNTIL_WALL:  // when coming off a gap, drive at your heading until a wall is encountered
      driveStraight(0.75, navAngle, enabled);
      if(walls.getFront() < 8){ // if something infront of you turn left
        navStates = TURNING_LEFT;
        navAngle = wrap(navAngle - 90);
      } else if(walls.getRight() < 12){ // if not and a walls on your right follow it
        navStates = FOLLOWING_WALL;
      }
      break;

      case PID_WALL: // PID forward to a distance slighty leess than that object
      if(driveDistance(PIDWallDistance, navAngle, enabled)){
        if (walls.getRight() > 10 && millis() - lastSawWall > 1000L * 15){ // if you're lost and you have been for 15 secs, try to stop doing that
          navStates = TURNING_LEFT;
          navAngle = wrap(navAngle - 90);
        } else if (walls.getRight() > 10){ // if there isn't a wall on the right turn right
          navStates = TURNING_RIGHT;
          navAngle = wrap(navAngle + 90);
        } else{ // otherwise follow that wall on your right
          navStates = FOLLOWING_WALL;
        }
      }
      break;

      case PID_FORWARD_LONG: // PID forward for a longer distance,
      if(driveDistance(18, navAngle, enabled)){ //was 19
        if (walls.getRight() <= 10){ // if there's a wall on your right follow it
          navStates = FOLLOWING_WALL;
        } else if (walls.getFront() < 12 && millis() - lastSawWall > 1000L * 15){ // if you're lost and you have been for 15 secs, try to stop doing that
          navStates = TURNING_LEFT;
          navAngle = wrap(navAngle - 90);
        } else { // otherwise turn right
          navStates = TURNING_RIGHT;
          navAngle = wrap(navAngle + 90);
        }
      }
      break;

      case STOPPING: // good job (hopefully), you're done robot
      arcadeDrive(0, 0);
      break;
    }
  } else { // if disabled turn off drive
    arcadeDrive(0, 0);
  }
}


/* control drive motors
*  @param throttle  the forward/backwards speed
*  @param turn      the turning speed (+ is right)
*/
void Drive::arcadeDrive(double throttle, double turn){
  leftDrive.write(frcToServo(throttle - turn));
  rightDrive.write(frcToServo(-turn - throttle));
}

/* returns true when input has been true for certian time
*  @param latch  the value of the condition you want to
*  @param delay  the time the input must be true to return true
*  @return true when in range for specified time
*/
bool Drive::booleanDelay(bool latch, unsigned int delay){
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

// finds the difference between two angles to feading into heading of PID loops
double Drive::angleDiff(double angle1, double angle2){
  return fmod(angle1 - angle2 + 540, 360) - 180;
}

// getter methods for debugging/printing
float Drive::getLeftEncoder(){ return odom.getLeftEncoder();}
float Drive::getRightEncoder(){ return odom.getRightEncoder();}
double Drive::getX(){return odom.getX();}
double Drive::getY(){return odom.getY();}
double Drive::getTheta(){return odom.getTheta();}
