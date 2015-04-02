#include <iostream>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "Hercules.h"

using namespace std;

#define CMD_DRIVE "D%c%c"

#define MAX_LEVEL	100
#define MAX_SPEED	10.0		// 10 m/s

Hercules::Hercules()
{
}

Hercules::~Hercules() {
	if (mSerialPtr) {
		delete mSerialPtr;
	}
}

void Hercules::reconnect() {
	mSerialPtr->Open(SerialPort::BAUD_19200,
				SerialPort::CHAR_SIZE_8,
				SerialPort::PARITY_DEFAULT,
				SerialPort::STOP_BITS_DEFAULT,
				SerialPort::FLOW_CONTROL_DEFAULT);
}

void Hercules::connect(const string &port) {
	mPort = port;
	mSerialPtr = new SerialPort(port);
}

void Hercules::configureLimits(double max_speed, double max_accel) {

}

void Hercules::controlSpeed(double speed_left, double speed_right,
		double accel_left, double accel_right) {
	int signL = speed_left < 0 ? 0x80 : 0;
	int signR = speed_right < 0 ? 0x80 : 0;

	speed_left = fabs(speed_left);
	speed_right = fabs(speed_right);

	int lspeed = speedToLevel(speed_left);
	int rspeed = speedToLevel(speed_right);

	sendDriveCmd(lspeed | signL, rspeed | signR);
}

void Hercules::sendDriveCmd(int leftSpeed, int rightSpeed) {
	char buf[10];
	snprintf(buf, sizeof(buf), CMD_DRIVE, leftSpeed, rightSpeed);
	mSerialPtr->Write(buf);
}

int Hercules::speedToLevel(double speed) {
	return speed * MAX_LEVEL / MAX_SPEED;
}
