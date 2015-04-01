/*
 * Demo of Hercules control DC Motor
 *
 * Loovee
 * 2014-6-27
 *
 * This demo will show you how to control a DC motor
 * you can connect 2 DC motor here, both of they will go 
 * a direction for 3s the turn reverse for 3s
 *
 * This demo use the following function:
 *
 * void setSpeedDir(int ispeed, unsigned char dir);
 * - ispeed - speed, range 0-100, that mean pwm in %
 * - dir - DIRF(0x00) or DIRR(0x01)
 */

#include <Hercules.h>

void setup()
{
    MOTOR.begin();                      // initialize
}

void loop()
{
    MOTOR.setSpeedDir(80, DIRF);        // pwm: 80%, direction DIRF
    delay(3000);        
    MOTOR.setSpeedDir(80, DIRR);        // pwm: 80%, direction DIRR
    delay(3000);
}



