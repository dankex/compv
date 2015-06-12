#ifndef _SEEED_PWM_H_
#define _SEEED_PWM_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#define TIMER1COUNT 65536    // Timer1 is 16 bit

class seeed_pwm 
{
    private:
    unsigned int pwmPeriod;
    unsigned char clockSelectBits;
    
    public:
    
    void init();
    void setPwm(char pin, int duty, long freq);             // Hz
    void disablePwm(char pin);                              // pin = 9 or 10
    void setFreq(long freq);                                // Hz
    void setPwmDuty(char pin, int duty);                    // duty: 0-100 %
};

extern seeed_pwm PWM;
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

