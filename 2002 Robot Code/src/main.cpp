#include <Arduino.h>
#include <LiquidCrystal.h>
#include <TimerOne.h>
#include <UltrasonicSensor.h>
#include <UltrasonicSensorArray.h>
#include <Servo.h>
#include "drive.h"
#include "helpers.h"
#include "BNO055.h"

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

LiquidCrystal lcd(40,41,42,43,44,45);


#define LED_PIN 24
#define NUM_PIXELS 4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, 24, NEO_GRB + NEO_KHZ800);

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

int lastRadiation, currentRadiation = 0;

////////
// IO //
////////

// first, create an Ultrasonic Sensor Array (USA) to receive inputs on pin 20
UltrasonicSensorArray usa(20);
// And then create each ultrasonic sensor (with their output pins)
UltrasonicSensor leftUltrasonic(24);
UltrasonicSensor rightUltrasonic(23);
UltrasonicSensor frontUltrasonic(22);

//Motors
const int leftDrivePort = 11;
const int rightDrivePort = 10;
const int armPivotPort = 9;
const int fanPort = 12;
//Digital IO
const int startPort = 13;
//Analog Input
const int lineSensorPort = A0;

bool enabled = false;
bool lastPressed = true;

Drive drive;
BNO055 IMU;


///////////
// SETUP //
///////////
void setup() {
  Serial.begin(9600);
  lcd.begin(16,2);

  //Neopixel Init
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Add each sensor to the UltrasonicSensorArray
usa.addSensor(&leftUltrasonic);
usa.addSensor(&rightUltrasonic);
usa.addSensor(&frontUltrasonic);
// Initalize the USA timer and input interrupts
usa.begin();

IMU.initialize();

drive.initialize(leftDrivePort,rightDrivePort);


  setLEDs(GREEN);
}



void printOdomToLCD(){
  lcd.setCursor(0, 1);
  lcd.print(drive.getX());
  lcd.setCursor(6, 1);
  lcd.print(drive.getY());
  lcd.setCursor(12, 1);
  lcd.print(drive.getTheta());
}

///////////////
// MAIN LOOP //
///////////////
void loop() {

  Serial.println(leftUltrasonic.distance());
Serial.println(rightUltrasonic.distance());
Serial.println(frontUltrasonic.distance());

  delay(20);
}
