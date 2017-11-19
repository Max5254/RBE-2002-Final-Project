/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) Griffin Tabor                          */
/*                                   2015                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     ultrasonicSensor.h                                           */
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

#ifndef ultrasonicSensor_h
#define ultrasonicSensor_h

#include "Arduino.h"

class UltrasonicSensor {
public:
  UltrasonicSensor(int pingPin);
  void giveValue(long unsigned timeDifference);
  void pulsePin();
  float distance();

private:
  volatile unsigned long recentTimeDifference;
  int _pingPin;
  float dist;
};

#endif

