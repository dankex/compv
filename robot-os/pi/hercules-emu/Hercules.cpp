/*
 * Hercules.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: danke
 */

#include <iostream>
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

void Hercules::cmdD() {
	string leftSpeed = mSerial.ReadLine(READ_TIME, ',');
	string rightSpeed = mSerial.ReadLine(READ_TIME, ',');
	string leftDir = mSerial.ReadLine(READ_TIME, ',');
	string rightDir = mSerial.ReadLine(READ_TIME, ',');

	cout << gDebugCount++ << " DEBUG: " << "L=" << leftSpeed << " R=" << rightSpeed <<
			" LD=" << leftDir << " RD=" << rightDir << endl;

	ostringstream oss;
	oss << "D" << leftSpeed << rightSpeed << "\n";
	mSerial.Write(oss.str());

	cout << gDebugCount++ << " DEBUG: " << "Output: " << oss.str() << endl;
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
		cout << "**** EXCEPTION: " << e.what() << endl;
	}
	return true;
}

