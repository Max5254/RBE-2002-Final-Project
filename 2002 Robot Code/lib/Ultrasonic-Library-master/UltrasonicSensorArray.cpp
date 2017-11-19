#include "Arduino.h"
#include "UltrasonicSensorArray.h"
#include "UltrasonicSensor.h"
#include "TimerOne.h"

UltrasonicSensorArray *sensorPointer; //global variable containing pointer to sensorArray object that has been created
void UltrasonicSensorArray::sensorStaticHandler(){
  sensorPointer->ultrasonicISR();  //handles static function call from echo interupt to ultrasonic ISR
}
void UltrasonicSensorArray::timerStaticHandler() {
	sensorPointer->pulseNext();  //handles static function calls from Timer1 to pulse next ultrasonic sensor
}

UltrasonicSensorArray::UltrasonicSensorArray(int inputPin) {
  _echoPin=inputPin;
  currentSensor=0;
  sensorsEntered=0;
  sensorPointer = this;
  pinMode(_echoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(_echoPin),
	  UltrasonicSensorArray::sensorStaticHandler, FALLING);//attaches echoPin interupt to ultrasonicIRS's static handler

}

 void UltrasonicSensorArray::addSensor(UltrasonicSensor *newSensor){
  sensors[sensorsEntered]= newSensor;
  sensorsEntered++;
}
 void UltrasonicSensorArray::begin(){
  currentStartTime = micros();
  sensors[0]->pulsePin();  //pulses first pin
  Timer1.initialize(25000); //initializes pings to occur every 10ms functioning as both an upper bound on ping length and the frequency of pings
  Timer1.attachInterrupt(UltrasonicSensorArray::timerStaticHandler); //

}

void UltrasonicSensorArray::ultrasonicISR(){	
  currentEndTime=micros();
  sensors[currentSensor]->giveValue(currentEndTime-currentStartTime);  
}

void UltrasonicSensorArray::pulseNext() {
    currentSensor++;
      if (currentSensor >= sensorsEntered) {
	    currentSensor = 0;
      }
    currentStartTime = micros();
	sensors[currentSensor]->pulsePin();
}



