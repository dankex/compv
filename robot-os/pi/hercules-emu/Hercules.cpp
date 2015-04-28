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

using namespace std;

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
	string leftSpeed = mSerial.ReadLine(100, ',');
	string rightSpeed = mSerial.ReadLine(100, ',');
	string leftDir = mSerial.ReadLine(100, ',');
	string rightDir = mSerial.ReadLine(100, ',');

	cout << "DEBUG: " << "L=" << leftSpeed << " R=" << rightSpeed <<
			" LD=" << leftDir << " RD=" << rightDir << endl;

	ostringstream oss;
	oss << "D" << leftSpeed << rightSpeed << "\n";
	mSerial.Write(oss.str());

	cout << "DEBUG: " << "Output: " << oss.str() << endl;
}

void Hercules::cmdB() {
	string dummy1 = mSerial.ReadLine(100, ',');
	string dummy2 = mSerial.ReadLine(100, ',');
}

bool Hercules::loop() {
	try {
		// Print serial port
		while (mSerial.IsDataAvailable()) {
			char newChar = mSerial.ReadByte(100);
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
		cout << e.what() << endl;
	}
	return true;
}

