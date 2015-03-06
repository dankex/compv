/*
 * Hercules.h
 *
 *  Created on: Mar 5, 2015
 *      Author: danke
 */

#ifndef HERCULES_H_
#define HERCULES_H_

#include <string>
#include "Keyboard.h"
#include "libserial/SerialStream.h"

using namespace std;
using namespace LibSerial;

class Hercules {
public:
	Hercules(const string &port);
	virtual ~Hercules();
	int run();

protected:
	void setup();
	bool loop();

private:
	SerialPort mSerial;
	Keyboard mKeyboard;

	char mForward;
	char mLeftTurn;
	char mRightTurn;

	void do_forward();
	void do_left_turn();
	void do_right_turn();
	void do_set_speed(char num);
};

#endif /* HERCULES_H_ */
