/*
 * MotorKnob
 *
 * A stepper motor follows the turns of a potentiometer
 * (or other sensor) on analog input 0.
 *
 * fork from Arduino Origin examples
 *
 * Modify by Loovee, to fit Seeed Hercules
 * 2014-6-27
 */

#include "Hercules_Stepper.h"

// change this to the number of steps on your motor
#define STEPS 200

// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to

stepper_4wd stepper(STEPS);

// the previous reading from the analog input
int previous = 0;

void setup()
{
    // set the speed of the motor to 30 RPMs
    stepper.setSpeed(30, 100);                // start speed: 30, max speed: 100
}

void loop()
{
    // get the sensor value
    int val = analogRead(0);

    // move a number of steps equal to the change in the
    // sensor reading
    stepper.step(val - previous);

    // remember the previous value of the sensor
    previous = val;
}