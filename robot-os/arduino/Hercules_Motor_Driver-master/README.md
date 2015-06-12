Hercules Dual 15A 6-20V Motor Controller
---------------------------------------------------------
[![Hercules Dual 15A 6-20V Motor Controller](http://www.seeedstudio.com/depot/images/product/Hercules%20controller.jpg)](http://www.seeedstudio.com/depot/Hercules-Dual-15A-620V-Motor-Controller-p-1515.html?cPath=91_92)



<br>
Hercules Dual 15A 6-20V Motor Controller is a high current motor drive control board. including micro controller processor, motor drive circuit, charging circuit and protection circuit. It features a complete solution to power supply, control and drive.
 
Compared with L298, its half bridge motor controller IR2104 and N-MOSFET take prominent advantage to drive heavy load device with output current even up to 15A. Hercules Dual motor controller is wide power supply designed, so the universal 7.4~11.1V lipo battery in the remote car and model airplane can be applied as well. Fuse will protect the board while overloaded, the correspondent LED show the protection status.
 
This motor controller is Arduino compatible. Best of all, it is focusing on high- expansion. The reserved standard Grove ports bring you a convenient access to abundant modules on the platform of motor car, such as Servo and Encoder, thus it embraces multi-functions that you can build up versatile masterpieces.

***Features***

- Full bridge drive circuit based on MOSFET supports two independent channels， each channel up to 15A
- LED shows fuse protection status
- Several Grove ports, conveniently connect with servo, encoder and sensors
- Atmege328 controller, Arduino compatible

<br>

#Usage

##How to program
There is Atmega328P on borad, but without USB to Serial IC, you need a extend USB to Serial device, if you can't find such device, you can refer to [here](http://www.seeedstudio.com/depot/UartSBee-V5-p-1752.html)

RX, TX, DTR and GND need to connect to the USB to Serial device.

Then you can upload code to Hercules just like you programming an Arduino. And the board choose *Arduino Duemilanove w/Atmega328*


##Control a DC motor
First you should connect a motor to M1 or M2 of Hercules. just as following:

![](http://www.seeedstudio.com/wiki/images/b/b3/IMG_0204-1-.JPG)

Then you can try the demo code in examples - DC_Motor, You will find you motor will turn a direction for 3s, then turn reverse for 3s, and loop like this.

***Thre are some funcion to control dc motor:***

    void begin();                                                   // initialize
    
    void setRun1();                                                 // motor1 run
    void setRun2();                                                 // motor2 run
    
    void setStop1();                                                // motor1 stop
    void setStop2();                                                // motor2 stop
    
    void setSpeed1(int ispeed);                                     // set speed of motor1, 0-100
    void setSpeed2(int ispeed);                                     // set speed of motor2, 0-100
    
    void setDir1(unsigned char dir);                                // set direction of motor1
    void setDir2(unsigned char dir);                                // set direction of motor2
    
    void setSpeedDir(int ispeed, unsigned char dir);                // set both motor speed
    void setSpeedDir1(int ispeed, unsigned char dir);               // set speed & direction of motor1
    void setSpeedDir2(int ispeed, unsigned char dir);               // set speed & direction of motor2

##Control a Stepper
Hercules can also drive stepper, expecially suit for high-power stepper.

Connect you stepper to Hercules:

- stepper A1 -> M1+
- stepper A2 -> M1-
- stepper B1 -> M2+
- stepper B2 -> M2-

Then try the demo code for stepper in examples foler. It's just like the Arduino origin stepper demo.

***Thre are some funcion to control stepper:***

    // constructors:
    stepper_4wd(int number_of_steps);

    // speed setter method:
    void setSpeed(long rpm_start, long rpm_max);

    // move untill it's done
    void step(int number_of_steps);
    
    // move untill it's donw or fun return 1
    void step(int steps_to_move,  int (*fun)());

In the second step function, you can get out of this function although the step hadn't moved over. For an example, you want your stepper move 10000 steps, and when a button is pressed, the stepper stop running. You can use like this:

	const int pinBtn = 2;
	
	int isBtn()
	{
		return digitalRead(pinBtn);
	}

	step(10000, &isBtn);





<br>

----------
***Some important tips***

If you download zip of this library, when you extract it to your libraries folder of Arduino, you should delete the "-master" in the folder name, or it may curse some compile error.

----

This software is written by loovee ([luweicong@seeedstudio.com](luweicong@seeedstudio.com "luweicong@seeedstudio.com")) for seeed studio<br>
and is licensed under [The MIT License](http://opensource.org/licenses/mit-license.php). Check License.txt for more information.<br>

Contributing to this software is warmly welcomed. You can do this basically by<br>
[forking](https://help.github.com/articles/fork-a-repo), committing modifications and then [pulling requests](https://help.github.com/articles/using-pull-requests) (follow the links above<br>
for operating guide). Adding change log and your contact into file header is encouraged.<br>
Thanks for your contribution.

Seeed Studio is an open hardware facilitation company based in Shenzhen, China. <br>
Benefiting from local manufacture power and convenient global logistic system, <br>
we integrate resources to serve new era of innovation. Seeed also works with <br>
global distributors and partners to push open hardware movement.<br>




[![Analytics](https://ga-beacon.appspot.com/UA-46589105-3/CAN_BUS_Shield)](https://github.com/igrigorik/ga-beacon)
