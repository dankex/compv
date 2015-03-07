/*
 * Hercules.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: danke
 */

#include <iostream>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include "Hercules.h"

using namespace std;

#define KEY_DEFAULT_FORWARD			'F'
#define KEY_DEFAULT_LEFT_TURN		'L'
#define KEY_DEFAULT_RIGHT_TURN		'R'

#define CMD_FORWARD "M1,"
#define CMD_LEFT_TURN "M3,"
#define CMD_RIGHT_TURN "M4,"
#define CMD_SET_SPEED "S"
#define CMD_ENDCMD	","

Hercules::Hercules(const string &port)
  : mSerial(port)
  , mForward(KEY_DEFAULT_FORWARD)
  , mLeftTurn(KEY_DEFAULT_LEFT_TURN)
  , mRightTurn(KEY_DEFAULT_RIGHT_TURN)
{
}

Hercules::~Hercules() {
}

int Hercules::run() {
	setup();
	while (1) {
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

bool Hercules::loop() {
	try {
		// Print serial port
		while (mSerial.IsDataAvailable()) {
			char newChar = mSerial.ReadByte(10000);
			cout << newChar;
		}

		// Process keyboard commands
		if (mKeyboard.isHit()) {
			bool esc;
			int ch = mKeyboard.getChar(esc);

			cout << "[" << ch << ":" << esc << "] ("
					<< (int) ch << ")" << endl;

			if (!esc) {
				if (ch == mForward || ch == 'b')
					do_forward();
				else if (ch == mLeftTurn)
					do_left_turn();
				else if (ch == mRightTurn)
					do_right_turn();
				else if (ch >= '0' && ch <= '9') {
					do_set_speed(ch);
				} else {
					// error case
					mKeyboard.clear();
				}
			} else {
				// Remote left
				if (ch == 53) {
					do_set_speed('5');
					do_left_turn();
				}
				// Remote right
				else if (ch == 54) {
					do_set_speed('5');
					do_right_turn();
				} else {
					// error case
					mKeyboard.clear();
				}
			}
		} else {
			// no keyboard hit
			usleep(100 * 1000);
		}
	} catch (exception e) {
		cout << e.what() << endl;
		// ignore
		mKeyboard.clear();
	}
	return true;
}

void Hercules::do_forward() {
	mSerial.Write(CMD_FORWARD);
}

void Hercules::do_left_turn() {
	mSerial.Write(CMD_LEFT_TURN);
}

void Hercules::do_right_turn() {
	mSerial.Write(CMD_RIGHT_TURN);
}

void Hercules::do_set_speed(char num) {
	mSerial.Write(CMD_SET_SPEED);
	mSerial.WriteByte(num);
	mSerial.Write(CMD_ENDCMD);
}
