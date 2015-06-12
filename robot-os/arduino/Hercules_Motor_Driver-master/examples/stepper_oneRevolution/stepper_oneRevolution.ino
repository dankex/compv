
/*
 Stepper Motor Control - one revolution

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.

 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.


 Created 11 Mar. 2007
 Modified 30 Nov. 2009
 by Tom Igoe

 Modify by Loovee, to fit Seeed Hercules
 2014-6-27

 */

#include "Hercules_Stepper.h"

const int stepsPerRevolution = 200;             // change this to fit the number of steps per revolution

stepper_4wd myStepper(stepsPerRevolution);

void setup() 
{
    // set the speed at 100 rpm, start speed 60
    myStepper.setSpeed(60, 100);
    // initialize the serial port:
    Serial.begin(9600);
}

void loop() 
{
    // step one revolution  in one direction:
    Serial.println("clockwise");
    myStepper.step(stepsPerRevolution);
    delay(500);

    // step one revolution in the other direction:
    Serial.println("counterclockwise");
    myStepper.step(-stepsPerRevolution);
    delay(500);
}

