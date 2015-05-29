#include "motordriver_4wd.h"
#include <seeed_pwm.h>

void setup()
{
    MOTOR.init();
}

void loop()
{
    MOTOR.setSpeedDir(80, DIRF);
    delay(3000);
    MOTOR.setSpeedDir(80, DIRR);
    delay(3000);
}
/*********************************************************************************************************
 * END FILE
 *********************************************************************************************************/


