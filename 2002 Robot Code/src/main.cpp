#include <Arduino.h>
#include <LiquidCrystal.h>
#include <TimerOne.h>
#include "drive.h"
#include "helpers.h"
#include "BNO055.h"
#include "wallSensors.h"
#include "flameSensor.h"
#include "fan.h"

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

LiquidCrystal lcd(40,41,42,43,44,45);

////////
// IO //
////////
#define LED_PIN 23
#define NUM_PIXELS 4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

//Motors
const int leftDrivePort = 11;
const int rightDrivePort = 10;
const int armPivotPort = 8;
const int fanPort = 9;
//Digital IO
const int startPort = 22;
const int fanButtonPort = 24;
const int frontEchoPort = 13;
const int frontTriggerPort = 12;
const int rightEchoPort = 26;
const int rightTriggerPort = 25;

//Analog Input
const int lineSensorPort = A2;
// const int sharpPort = A3;


bool enabled = false;

// Drive drive;
BNO055 IMU;
wallSensors walls;
Drive drive;
flameSensor flame;
Fan fan;

//color constants
uint32_t RED = strip.Color(255, 0, 0);
uint32_t GREEN = strip.Color(0, 255, 0);
uint32_t BLUE = strip.Color(0, 0, 255);
uint32_t YELLOW = strip.Color(255, 200, 0);
uint32_t PURPLE = strip.Color(200, 0, 200);
uint32_t ORANGE = strip.Color(255, 100, 0);
uint32_t NO_COLOR = strip.Color(0, 0, 0);

void setLEDs(uint32_t color){ //set all 4 LEDs to specific color
  strip.setPixelColor(0,color);
  strip.setPixelColor(1,color);
  strip.setPixelColor(2,color);
  strip.setPixelColor(3,color);
  strip.show();
}


///////////
// SETUP //
///////////
void setup() {
  Serial.begin(9600);
  lcd.begin(16,2);
  lcd.setCursor(0, 0 );
  lcd.print("hi");

  pinMode(startPort, INPUT_PULLUP);
  pinMode(fanButtonPort, INPUT_PULLUP);

  //Neopixel Init
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  setLEDs(RED);
  walls.initialize(frontTriggerPort, frontEchoPort, rightTriggerPort, rightEchoPort);
  flame.initialize();
  fan.initialize(fanPort);
  Serial.println("1");
  setLEDs(ORANGE);
  IMU.initialize();
  IMU.reset(0);

  Serial.println("2");
  drive.initialize(leftDrivePort,rightDrivePort); // must be after IMU


  setLEDs(GREEN);
}



// void printOdomToLCD(){
//   lcd.setCursor(0, 1);
//   lcd.print(drive.getX());
//   lcd.setCursor(6, 1);
//   lcd.print(drive.getY());
//   lcd.setCursor(12, 1);
//   lcd.print(drive.getTheta());
// }
float scale(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool getStart(){
  return !digitalRead(startPort);
}

bool getFanButton(){
  return !digitalRead(fanButtonPort);
}

void logCameraCalibration(){
  Serial.print(IMU.getX());
  Serial.print(", ");
  Serial.print((flame.getX1()-576)*-0.0396);
  Serial.print(", ");
  Serial.println(flame.getX1());
  Serial.println(drive.angleDiff(25.3, 180.1));
  Serial.println(drive.angleDiff(25.3, 180.1));

}

int state = 3;
int numStates = 7;
bool lastPressed, lastFan = false;

void printThings(){
  if(getStart() && !lastPressed){ state++; }

  lastPressed = getStart();

  if(state > numStates){ state = 1; }

  lcd.clear();
  lcd.setCursor(0, 0);
  switch (state) {
    case 1: // front sensor
    // setLEDs(GREEN);
    lcd.print("Front Wall (in)");
    lcd.setCursor(0, 1);
    lcd.print(walls.getFront());
    break;
    case 2: // right sensor
    // setLEDs(BLUE);
    lcd.print("Right Wall (in)");
    lcd.setCursor(0, 1);
    lcd.print(walls.getRight());
    break;
    case 3: // IMU
    // setLEDs(YELLOW);
    lcd.print("IMU Heading");
    lcd.setCursor(0, 1);
    lcd.print(IMU.getX());
    break;
    case 4: // IR
    // setLEDs(RED);
    lcd.print("IR Reading");
    lcd.setCursor(0, 1);
    lcd.print(flame.getX1());
    lcd.setCursor(7, 1);
    lcd.print(flame.getY1());
    break;
    case 5: // Line
    // setLEDs(PURPLE);
    lcd.print("Line Sensor");
    lcd.setCursor(0, 1);
    lcd.print(analogRead(lineSensorPort));
    break;
    case 6: // odom
    lcd.print("Odometry (x y z)");
    lcd.setCursor(0, 1);
    lcd.print(drive.getX());
    lcd.setCursor(6, 1);
    lcd.print(drive.getY());
    lcd.setCursor(12, 1);
    lcd.print(drive.getTheta());
    break;
    case 7: // odom
    lcd.print("Candle Pos");
    lcd.setCursor(0, 1);
    lcd.print(flame.getCandleX());
    lcd.setCursor(6, 1);
    lcd.print(flame.getCandleY());
    lcd.setCursor(12, 1);
    lcd.print(drive.getTheta() - flame.getHAngle());
    break;
  }
}

bool seesCandle = false;

///////////////
// MAIN LOOP //
///////////////
void loop() {

  // Serial.println(analogRead(A0));
  // Serial.println(IMU.getX());
  // lcd.setCursor(0, 0 );
  // lcd.print("hello");
  // Serial.println("hello");
  drive.odometry();
  walls.periodicPing(100,100);



  // Serial.println(flame.getX1());
  printThings();
  // fan.setFan(getFanButton() && false);



  // if(getFanButton()){
  //   while(!drive.turnToAngle(90, true));
  //   // drive.arcadeDrive(0, 1);
  // } else {
  //   drive.arcadeDrive(0, 0);
  // }



  if(getFanButton() && !lastFan) {
    enabled = !enabled;

  }

  // Serial.println(walls.getLoopDelay());
  lastFan = getFanButton();
  seesCandle = flame.getX1() < 700 && flame.checkFlame(drive.getX(), drive.getY(), drive.getTheta() - flame.getHAngle());

  fan.setFan(seesCandle);
  if(seesCandle){
    setLEDs(RED);
  }

  drive.navigation(enabled && !seesCandle, 6.5);
  // drive.driveStraight(1, 180, true);
  // Serial.println(drive.getRightEncoder());



  // Serial.println(analogRead(A1));

  delay(50 - walls.getLoopDelay());
}
