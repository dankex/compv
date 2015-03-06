/*
 * Hercules.h
 *
 *  Created on: Mar 5, 2015
 *      Author: danke
 */

#ifndef HERCULES_H_
#define HERCULES_H_

#include "libserial/SerialStream.h"

using namespace LibSerial ;

class Hercules {
public:
	Hercules();
	virtual ~Hercules();
	int run();

protected:
	void setup();
	bool loop();

private:
	char mForward;
	char mLeftTurn;
	char mRightTurn;

	SerialStream mSerialStream;

	void do_forward();
	void do_left_turn();
	void do_right_turn();
};

#endif /* HERCULES_H_ */
