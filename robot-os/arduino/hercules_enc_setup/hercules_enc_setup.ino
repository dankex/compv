//MECHANICAPE.NL FIRMWARE FOR SEEEDSTUDIO HERCULES 4WD CONTROLLER
//Copyright By Rein Velt / http://mechanicape.nl        
//May be used under the terms and conditions of Create Commons CC-BY-SA
//=====================================================================
// The device is controlled by serial commands. The command is send 
// to the controllerboard. The controllerboard performs the 
// requested action and sends a response message
// back to the requestor.
// The commands, descriptions and responses are:
//  CMD     DESCRIPTION                     RESPONSE
//  M0      Motor OFF                      -returns M0,speedL,speedR\n
//  M1      Motor Forward                  -returns M1,speedL,speedR\n
//  M2      Motor Reverse                  -returns M2,speedL,speedR\n
//  M3      Motor Turn Left (ccw)          -returns M3,speedL,speedR\n
//  M4      Motor Turn Right (cw)          -returns M4,speedL,speedR\n
//  MT      Motor Test (test motors)       -returns MT,speedL,speedR\n
//  S0..S9  Speed (0=stop,.., 9=fast)      -returns S*,speedL,speedR\n
//  B?      Query battery voltage          -returns B?,millivolts\n
//
//  DLR     Drive two wheels at speeds L/R -return D,cntL,cntR\n
//
//  the comma is used to separate commands when more are following
//  the newline is used to terminate one or more commands
//  eg:
//      M1\n 
//      S5,M1,M1,S3,M2,M3,M0\n
//====================================================================

#include <avr/wdt.h>
#include <Hercules.h>
// pin ctrl
#define PINRX           0           // receive serial data
#define PINTX           1           // send serial data
#define PININT0         2           // interrupt 0
#define PININT1         3           // interrupt 1
#define PINCS           6           // all motors cs
#define PINM1F          4           // motor 1 forward
#define PINM1R          5           // motor 1 reverse
#define PINM2F          7           // motor 2 forward
#define PINM2R          8           // motor 2 reverse
#define PINPWMA         9           // PWM channel A (motor A speed)
#define PINPWMB         10          // PWM channel B (motor B speed)
#define PINVS           A6          // voltage/battery sensor

//global vars
volatile int leftFeedback = 0;
volatile int rightFeedback = 0;

int motorSpeed;

void setup()
{
  MOTOR.begin();
  pinMode(PINVS, INPUT);

  // initialize interrupts
  attachInterrupt(0, leftInterruptHandler, CHANGE);
  attachInterrupt(1, rightInterruptHandler, CHANGE);

  // initialize serial port
  Serial.begin(19200);
  Serial.println("//HERCULES 4WD ROBOTFIRMWARE V1.0");
  Serial.print("//Battery=");
  Serial.print(getBatteryVoltage());
  Serial.print("mV");
  Serial.println();
  
  // initialize watch dog timer and set it to 2 seconds
  wdt_enable(WDTO_2S);
  
  // speed test
  speedTest();
}

void speedTest() {
  resetFeedback();
  
  motorSetSpeed(50);
  motorForward();
  
  // run for 10 secs
  delay(2*1000);
  
  motorStop();
  
  printData();
  resetFeedback();  
}

void loop()
{
  wdt_reset();  //reset watch dog timer
  //printData();
}

void printData() {
  Serial.print("L=");
  Serial.print(leftFeedback);
  Serial.print(" R=");
  Serial.println(rightFeedback);
}

/**** INTERRUPT HANDLERS ****/

void leftInterruptHandler()
{
  leftFeedback++;
}

void rightInterruptHandler()
{
  rightFeedback++;
}

/**** MOTOR CONTROLLER ****/

void resetFeedback()
{
  leftFeedback=0;
  rightFeedback=0;
}

void motorSetSpeed(int power)
{
  motorSpeed=power;
}

void motorStop()
{
  MOTOR.setStop1();
  MOTOR.setStop2();
}

void motorForward()
{
  MOTOR.setSpeedDir(motorSpeed, DIRF);
}

/**** SENSOR HANDLING ****/

int getBatteryVoltage()
{
  int value=analogRead(PINVS);
  int millivolt=round((float)(value)/0.037479); //guess
  return millivolt;
}



