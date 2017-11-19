/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) Griffin Tabor                          */
/*                                   2015                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     ultrasonicSensorArray.h                                      */
/*    Author:     Griffin Tabor                                                */
/*    Created:    9 Nov 2015                                                   */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00  9 Nov 2015 - Initial release                          */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*-----------------------------------------------------------------------------*/

#ifndef ultrasonicSensorArray_h
#define ultrasonicSensorArray_h

#include "Arduino.h"
#include "UltrasonicSensor.h"


class UltrasonicSensorArray {
public:
  UltrasonicSensorArray(int echoPin);  
  void addSensor(UltrasonicSensor *newSensor);
  void begin();


private:
  int _echoPin;
  volatile int currentSensor;
  int sensorsEntered;  
  volatile unsigned long currentStartTime;
  volatile unsigned long currentEndTime;
  void ultrasonicISR();
  void pulseNext();
  static void sensorStaticHandler();
  static void timerStaticHandler();

  
#ifndef NUMOFULTRASONICS
    #define NUMOFULTRASONICS 5
#endif  
  UltrasonicSensor* sensors[NUMOFULTRASONICS];
};

#endif

