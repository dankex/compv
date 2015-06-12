#ifndef _MOTOR_DRIVER_CONFIG_H_
#define _MOTOR_DRIVER_CONFIG_H_

void setupMotors();
void commandLeftMotor(int16_t cmd);
void commandRightMotor(int16_t cmd);

#include "HerculesMotorDriver_config.h"

#endif  // _MOTOR_DRIVER_CONFIG_H_
