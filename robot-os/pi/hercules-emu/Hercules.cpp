/*
 * Hercules.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: danke
 */

#include <cstdio>
#include <sstream>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include "Hercules.h"

#define READ_TIME 100

using namespace std;

static int gDebugCount;

Hercules::Hercules(const string &port)
  : mSerial(port)
{
}

Hercules::~Hercules() {
}

int Hercules::run() {
	setup();
	while (true) {
		if (!loop())
			break;
	}
	return 0;
}

void Hercules::setup() {
	mSerial.Open(SerialPort::BAUD_19200,
			SerialPort::CHAR_SIZE_8,
			SerialPort::PARITY_DEFAULT,
			SerialPort::STOP_BITS_DEFAULT,
			SerialPort::FLOW_CONTROL_DEFAULT);
}

int Hercules::stringToNum(string &str) {
	istringstream iss(str);
	int num;
	iss >> num;
	return num;
}

void Hercules::cmdD() {
	string leftSpeed = mSerial.ReadLine(READ_TIME, ',');
	string rightSpeed = mSerial.ReadLine(READ_TIME, ',');
	string leftDir = mSerial.ReadLine(READ_TIME, ',');
	string rightDir = mSerial.ReadLine(READ_TIME, ',');

	printf("%d DEBUG: L=%s R=%s LD=%s RD=%s\n", 
		gDebugCount++, leftSpeed.c_str(), rightSpeed.c_str(), leftDir.c_str(), rightDir.c_str());

	int lspeed = stringToNum(leftSpeed);
	int rspeed = stringToNum(rightSpeed);

	ostringstream oss;
	oss << "D" << lspeed << "," << rspeed << ",\n";
	mSerial.Write(oss.str());

	printf("%d DEBUG: Output: %s\n", gDebugCount++, oss.str().c_str());
}

void Hercules::cmdB() {
	string dummy1 = mSerial.ReadLine(READ_TIME, ',');
	string dummy2 = mSerial.ReadLine(READ_TIME, ',');
}

bool Hercules::loop() {
	try {
		// Print serial port
		while (mSerial.IsDataAvailable()) {
			char newChar = mSerial.ReadByte(READ_TIME);
			switch (newChar) {
				case 'D':
					cmdD();
					break;
				case 'B':
					cmdB();
					break;
			}
		}
	} catch (exception e) {
		printf("**** EXCEPTION: %s\n", e.what());
	}
	return true;
}

