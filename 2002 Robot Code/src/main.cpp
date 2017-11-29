#include <Arduino.h>
#include <LiquidCrystal.h>
#include <TimerOne.h>
// #include <Servo.h>
#include "drive.h"
#include "helpers.h"
#include "BNO055.h"
#include "wallSensors.h"

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
const int armPivotPort = 9;
const int fanPort = 8;
//Digital IO
const int startPort = 22;
const int ultrasonicEchoPort = 12;
const int ultrasonicTriggerPort = 13;

//Analog Input
const int lineSensorPort = A0;
const int sharpPort = A1;


bool enabled = false;
bool lastPressed = true;

// Drive drive;
BNO055 IMU;
wallSensors walls;
Drive drive;

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

  //Neopixel Init
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

walls.initialize(ultrasonicTriggerPort, ultrasonicEchoPort, sharpPort);
// Serial.println("1");
IMU.initialize();
// Serial.println("2");

drive.initialize(leftDrivePort,rightDrivePort);

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



///////////////
// MAIN LOOP //
///////////////
void loop() {

Serial.println(analogRead(A0));
// Serial.println(IMU.getX());
lcd.setCursor(0, 0 );
lcd.print("hello");
// Serial.println("hello");

// drive.arcadeDrive(0,0);

  delay(50);
}
