#include "flameSensor.h"

#define THETA_RANGE 4

flameSensor::flameSensor(){}

// much of the communication is taken from the example code provided by the retailer
// https://www.dfrobot.com/wiki/index.php/Positioning_ir_camera

// initialize the IR camera
void flameSensor::initialize(){
  bestX = NO_VALUE;
  bestY = NO_VALUE;
  bestXt = THETA_RANGE;
  bestYt = THETA_RANGE;
 slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
 Wire.begin();
 // IR sensor initialize
 Write_2bytes(0x30,0x01); delay(10);
 Write_2bytes(0x30,0x08); delay(10);
 Write_2bytes(0x06,0x90); delay(10);
 Write_2bytes(0x08,0xC0); delay(10);
 Write_2bytes(0x1A,0x40); delay(10);
 Write_2bytes(0x33,0x33); delay(10);
 delay(100);
}

// send two bytes to the camera
void flameSensor::Write_2bytes(byte d1, byte d2)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1); Wire.write(d2);
    Wire.endTransmission();
}

// request data from the camera and store the response in the arrays for x and y
void flameSensor::get(){
  //IR sensor read
   Wire.beginTransmission(slaveAddress);
   Wire.write(0x36);
   Wire.endTransmission();

   Wire.requestFrom(slaveAddress, 16);        // Request the 2 byte heading (MSB comes first)
   for (i=0;i<16;i++) { data_buf[i]=0; }
   i=0;
   while(Wire.available() && i < 16) {
       data_buf[i] = Wire.read();
       i++;
   }

   Ix[0] = data_buf[1];
   Iy[0] = data_buf[2];
   s   = data_buf[3];
   Ix[0] += (s & 0x30) <<4;
   Iy[0] += (s & 0xC0) <<2;

   Ix[1] = data_buf[4];
   Iy[1] = data_buf[5];
   s   = data_buf[6];
   Ix[1] += (s & 0x30) <<4;
   Iy[1] += (s & 0xC0) <<2;

   Ix[2] = data_buf[7];
   Iy[2] = data_buf[8];
   s   = data_buf[9];
   Ix[2] += (s & 0x30) <<4;
   Iy[2] += (s & 0xC0) <<2;

   Ix[3] = data_buf[10];
   Iy[3] = data_buf[11];
   s   = data_buf[12];
   Ix[3] += (s & 0x30) <<4;
   Iy[3] += (s & 0xC0) <<2;
}

// gets the active index of which reading is currently returning an actual reading
int flameSensor::getActive(){
  for (int i = 0; i < 4; i++){
    if(Ix[i] != 1023){
      return i;
    }
  }
  return 0;
}

// returns true once the robot has driven by the flame in two axis such that it knows where it is on the field
// records odom coordinates and calculates height when driving by candle at close to 90deg angle
bool flameSensor::checkFlame(double x, double y, double t){
  if (abs(fmod(t+360,180)) < abs(bestXt)){
    bestX = x;
    bestDist = abs(bestY - y);
    bestVAngle = getVAngle();
    bestZ = 8 + abs(bestY - y)*tan((getVAngle() * 3.14) / 180);
    bestXt = fmod(t+360,180);
  }
  if (abs(fmod(t+360,180)-90) < abs(bestYt)){
    bestY = y;
    bestDist = abs(bestX - x);
    bestVAngle = getVAngle();
    bestZ = 8 + abs(bestX - x)*tan((getVAngle() * 3.14) / 180);
    bestYt = fmod(t+360,180-90);
  }
  return abs(bestXt) < THETA_RANGE && abs(bestYt) < THETA_RANGE;
}

// returns the X coordinate of candle
double flameSensor::getCandleX(){
  return bestX;
}
// returns the Y coordinate of candle
double flameSensor::getCandleY(){
  return bestY;
}
// returns the Z coordinate of candle
double flameSensor::getCandleZ(){
    return bestZ;
}
// returns the X coordinate of IR camera reading
int flameSensor::getX1(){
  get();
  return int(Ix[getActive()]);
}
// returns the Y coordinate of IR camera reading
int flameSensor::getY1(){
  get();
  return int(Iy[getActive()]);
}
// returns the horrizontal angle offset of the IR camera to the flame
double flameSensor::getHAngle(){
  return (getX1()-576)*-0.0396; //determined experimentally with data points
}
// returns the vertical angle offset of the IR camera to the flame
double flameSensor::getVAngle(){
  return (getY1()*0.0408)-8.81; //determined experimentally with data points
}
