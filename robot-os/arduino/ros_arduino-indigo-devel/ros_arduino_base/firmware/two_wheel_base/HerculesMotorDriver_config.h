#ifndef _HERCULES_MOTOR_DRIVER_CONFIG_H_
#define _HERCULES_MOTOR_DRIVER_CONFIG_H_

#include "motor_driver_config.h"
//#include <Hercules.h>

void setupMotors()
{
  // Initalize Motors
  MOTOR.begin();
}
void commandLeftMotor(int16_t cmd)
{
  if(cmd >= 0) {
    MOTOR.setSpeedDir1((int) cmd, DIRF);
  } else {
    MOTOR.setSpeedDir1((int) abs(cmd), DIRR);
  }
}
void commandRightMotor(int16_t cmd)
{
  if(cmd >= 0) {
    MOTOR.setSpeedDir2((int) cmd, DIRF);
  } else {
    MOTOR.setSpeedDir2((int) abs(cmd), DIRR);
  }
}

#endif // _HERCULES_MOTOR_DRIVER_CONFIG_H_

