/*
 * Hercules.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: danke
 */

#include <iostream>
#include "Hercules.h"
#include "getch.h"

using namespace std;

#define KEY_DEFAULT_FORWARD			'F'
#define KEY_DEFAULT_LEFT_TURN		'L'
#define KEY_DEFAULT_RIGHT_TURN		'R'

#define CMD_FORWARD "M1,"
#define CMD_LEFT_TURN "M3,"
#define CMD_RIGHT_TURN "M4,"

Hercules::Hercules()
  : mForward(KEY_DEFAULT_FORWARD)
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
	mSerialStream.Open("/dev/ttyUSB0");
	mSerialStream.SetBaudRate(SerialStreamBuf::BAUD_19200);
	mSerialStream.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 );
	mSerialStream.SetNumOfStopBits(1);
	mSerialStream.SetParity(SerialStreamBuf::PARITY_ODD);
	mSerialStream.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;

	while (mSerialStream.IsDataAvailable()) {
	}
}

bool Hercules::loop() {
	char ch;

	ch = getch();

	cout << "new char: " << ch << " (" << (int) ch << ")" << endl;

	if (ch == mForward)
		do_forward();
	else if (ch == mLeftTurn)
		do_left_turn();
	else if (ch == mRightTurn)
		do_right_turn();

	return true;
}

void Hercules::do_forward() {
	mSerialStream << CMD_FORWARD;
}

void Hercules::do_left_turn() {
	mSerialStream << CMD_LEFT_TURN;
}

void Hercules::do_right_turn() {
	mSerialStream << CMD_RIGHT_TURN;
}
