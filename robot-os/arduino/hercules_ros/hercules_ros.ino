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
volatile int motorSpeed = 0;
char received[7];
int testEnc = 12345;

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
  Serial.println("OK");

  // initialize watch dog timer and set it to 2 seconds
  wdt_enable(WDTO_2S);
}

void loop()
{
//  testEnc++;
  wdt_reset();  //reset watch dog timer
  processSerialData(); //read data from serial and send it to the motors
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

void motorReverse()
{
  MOTOR.setSpeedDir(motorSpeed, DIRR);
}

void motorTurnRight()
{
  MOTOR.setSpeedDir1(motorSpeed-(motorSpeed >> 1), DIRF);
  MOTOR.setSpeedDir2(motorSpeed, DIRF);
}

void motorTurnLeft()
{
  MOTOR.setSpeedDir1(motorSpeed, DIRF);
  MOTOR.setSpeedDir2(motorSpeed- (motorSpeed >>1), DIRF);
}

/**** MESSAGE HANDLING ****/

void processCmdM(int valueByte) {
      //incoming data is correct and conform specs  
      motorSetSpeed(motorSpeed);      
      switch (valueByte)
      {
        //control the motors
      case 48+0: 
        motorStop(); 
        break;       //M0\n
      case 48+1: 
        motorForward(); 
        break;       //M1\n
      case 48+2: 
        motorReverse(); 
        break;       //M2\n
      case 48+3: 
        motorTurnLeft(); 
        break;       //M3\n
      case 48+4: 
        motorTurnRight(); 
        break;       //M4\n
      case 84:
        motorTest(); //MT\n
        break;
      default: 
        motorStop();  
        break;       // fail safe
      } //end-switch

      //confirm the serial request by sending a response
      Serial.print('M');
      Serial.print(char(valueByte));
      Serial.print(char(44));
      Serial.print(leftFeedback, DEC);
      Serial.print(char(44));
      Serial.print(rightFeedback, DEC);
      Serial.println();
}

void processCmdS(int valueByte) {
      //incoming data is correct and conform specs        
      if (valueByte>47 && valueByte<59)
      {
        int motorNewSpeed=(valueByte-48)*10;
        motorSetSpeed(motorNewSpeed);
      }
      //confirm the serial request by sending a response
      Serial.print('S');
      Serial.println(char(valueByte));
}

void processCmdB() {
      int millivolt=getBatteryVoltage();
      //confirm the serial request by sending a response
      Serial.print('B');
//      Serial.print(char(valueByte));
      Serial.print(char(','));
      Serial.print(millivolt);
      Serial.print(char(','));
      Serial.println();

      Serial.read();
      Serial.read();
}

void processCmdD() {
      int left, right;

      noInterrupts();
      left = leftFeedback;
      leftFeedback = 0;

      right = rightFeedback;
      rightFeedback = 0;
      interrupts();

      // output odometry
      Serial.print('D');
      Serial.print(char(','));
      Serial.print(left, DEC);
      Serial.print(char(','));
      Serial.print(right, DEC);
      Serial.print(char(','));
      Serial.println();

      // drive the wheels
      int speedL = 0;
      int dirL = 0;
      int speedR = 0;
      int dirR = 0;
      speedL = Serial.parseInt();
      speedR = Serial.parseInt();
      dirL = Serial.parseInt();
      dirR = Serial.parseInt();
/*      
      if (data[1]>47 && data[1]<58) {
          speedL = (data[1]-48) * 10;
      }        
      if (data[2]>47 && data[2]<58)
          speedL += data[2]-48;

      if (data[3]>47 && data[3]<58) {
          speedR = (data[3]-48) * 10;
      }        
      if (data[4]>47 && data[4]<58)
          speedR += data[4]-48;
          
      if (data[5]>47 && data[5]<58)
          dirL = data[5]-48;
      if (data[6]>47 && data[6]<58)
          dirR = data[6]-48;
*/          
//      speedL = 20;
//      speedR = 10;
      MOTOR.setSpeedDir1(speedL, dirL ? DIRR : DIRF);
      MOTOR.setSpeedDir2(speedR, dirR ? DIRR : DIRF);
}
void processSerialData()
{
  while (Serial.available()>0) {

  int commandByte = Serial.read();
  switch (commandByte) {

  // A message has 3 bytes

  // If the first byte is M (Motor) or S (Speed)
  //   the second byte is a number (0|1|2|3|4|..|9) 
  //   the third byte is a comma or a newline (,|\n)

  // If the first byte is D (Drive)
  //   the second byte is the left speed 0 - 99
  //   the third byte is the right speed 0 - 99

    case'D':
        processCmdD();
        break;
    case'B':
        processCmdB();
        break;
    default :
//        continue;
        break;
 /*
  else if (commandByte=='M' && (third==10 || third==44)) { // M
    processCmdM(second);
  } else if (commandByte=='S' && (third==10 || third==44)) { // S
    processCmdS(second);
*/
  }
  delay(25); // this delay can be tweaked or omitted to adjust accuracy. keep it low!
  }
}

/**** SENSOR HANDLING ****/

int getBatteryVoltage()
{
  int value=analogRead(PINVS);
  int millivolt=round((float)(value)/0.037479); //guess
  return millivolt;
}

/**** SYSTEM TEST ****/

void motorTest()
{
  for (int i=0;i<255;i++)
  {
    wdt_reset();
    motorSetSpeed(i);
    motorTurnRight();
    delay(50) ;
  }
  for (int i=255;i>-1;i--)
  {
    wdt_reset();
    motorSetSpeed(i);
    motorTurnRight();
    delay(50) ;
  }
  motorStop();
}


