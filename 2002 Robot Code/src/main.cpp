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
#define NUM_PIXELS 3
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

bool enabled = false;
bool sawCandle = false;

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

void setLEDs(uint32_t color){ //set all 3 LEDs to specific color
  strip.setPixelColor(0,color);
  strip.setPixelColor(1,color);
  strip.setPixelColor(2,color);
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

  // init both buttons
  pinMode(startPort, INPUT_PULLUP);
  pinMode(fanButtonPort, INPUT_PULLUP);

  //Neopixel Init
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // init the subsystems
  setLEDs(RED);
  walls.initialize(frontTriggerPort, frontEchoPort, rightTriggerPort, rightEchoPort);
  flame.initialize();
  fan.setAngle(105);
  fan.initialize(fanPort,armPivotPort);
  setLEDs(ORANGE);
  IMU.initialize();
  IMU.reset(0);
  drive.initialize(leftDrivePort,rightDrivePort); // must be after IMU

  setLEDs(PURPLE);
}

// get if buttons have been pressed
bool getStart(){
  return !digitalRead(startPort);
}

bool getFanButton(){
  return !digitalRead(fanButtonPort);
}

// for debuging of camerage angle calibrations
void logCameraCalibration(){
  Serial.print(IMU.getX());
  Serial.print(", ");
  Serial.print((flame.getX1()-576)*-0.0396);
  Serial.print(", ");
  Serial.println(flame.getX1());
  Serial.println(drive.angleDiff(25.3, 180.1));
  Serial.println(drive.angleDiff(25.3, 180.1));

}

// states for the LCD display system
int state = 3;
int numStates = 7;
bool lastPressed, lastFan = false;

void printThings(){

  // increment state on single button press
  if(getStart() && !lastPressed){ state++; }
  lastPressed = getStart();
  if(state > numStates){ state = 1; }

  lcd.clear();
  lcd.setCursor(0, 0);
  switch (state) {
    case 1: // front sensor
    lcd.print("Front Wall (in)");
    lcd.setCursor(0, 1);
    lcd.print(walls.getFront());
    break;
    case 2: // right sensor
    lcd.print("Right Wall (in)");
    lcd.setCursor(0, 1);
    lcd.print(walls.getRight());
    break;
    case 3: // IMU
    lcd.print("IMU Heading");
    lcd.setCursor(0, 1);
    lcd.print(IMU.getX());
    break;
    case 4: // IR
    lcd.print("IR Reading");
    // lcd.print(flame.getVAngle());
    // lcd.setCursor(7, 0);
    // lcd.print(8+30*tan((flame.getVAngle() * 3.14) / 180));
    lcd.setCursor(0, 1);
    lcd.print(flame.getX1());
    lcd.setCursor(7, 1);
    lcd.print(flame.getY1());
    lcd.setCursor(12, 1);
    lcd.print(flame.getCandleZ(),2);
    break;
    case 5: // Line sensor
    lcd.print("Line Sensor");
    lcd.setCursor(0, 1);
    lcd.print(analogRead(lineSensorPort));
    break;
    case 6: // odom
    lcd.print("Odometry (x y T)"); // fudged a bit here to follow the desired conventions
    lcd.setCursor(0, 1);
    lcd.print(drive.getY());
    lcd.setCursor(6, 1);
    lcd.print(-1*drive.getX());
    lcd.setCursor(12, 1);
    lcd.print(drive.getTheta());
    break;
    case 7: // Candle position
    lcd.print("Candle (x y z)");
    // lcd.print(flame.bestDist,2);
    // lcd.setCursor(7, 0);
    // lcd.print(flame.bestVAngle,2);
    lcd.setCursor(0, 1);
    lcd.print(flame.getCandleX(),1);
    lcd.setCursor(6, 1);
    lcd.print(flame.getCandleY(),1);
    lcd.setCursor(12, 1);
    lcd.print(flame.getCandleZ(),2);
    break;
  }
}

bool seesCandle = false;
int turningToFlameAngle;


///////////////
// MAIN LOOP //
///////////////
void loop() {

  // run position tracking and distance sensors
  drive.odometry();
  walls.periodicPing(100,100);

  printThings(); // LCD controller

  //// testing for tuning turning PID
  // if(getFanButton()){
  //   while(!drive.turnToAngle(90, true));
  //   // drive.arcadeDrive(0, 1);
  // } else {
  //   drive.arcadeDrive(0, 0);
  // }

  // start/stop robot
  if(getFanButton() && !lastFan) {
    enabled = !enabled;
  }
  lastFan = getFanButton();


  seesCandle = flame.getX1() < 1023 && (sawCandle || flame.checkFlame(drive.getY(), -1*drive.getX(), drive.getTheta() - flame.getHAngle()));

  bool seesCandleLag = inverseBooleanDelay(seesCandle,2000); // return true if sees candle and after it looses sight of it for delay time (ms)

  fan.setFan(seesCandleLag);

  if(seesCandleLag){ // if candle has been in sight for the last delay time from inverseBooleanDelay()
    if(seesCandle){ //if candle is in sight aim at it with with drive and servo
      turningToFlameAngle = drive.getTheta() - flame.getHAngle();
      fan.setAngle(90+flame.getVAngle());
    }
    // turn to candle, display you've seen it again, and switch LCD to report it's position
    drive.turnToAngle(turningToFlameAngle, true);
    setLEDs(RED);
    sawCandle = true;
    state = 7;
  }
  else{ // normally driving around the field
    drive.navigation(enabled && !seesCandleLag, 6.5, sawCandle);
  }

  if (sawCandle && !seesCandleLag){  // change LEDs to Green once flame is blown out
    setLEDs(GREEN);
  } else if (flame.getCandleX() == NO_VALUE ^ flame.getCandleY() == NO_VALUE){ // change LEDs to Blue if haven't found candle
    setLEDs(BLUE);
  }

  delay(50 - walls.getLoopDelay()); // delay for time compensated for the time it took to ping the ultrasonics
}
