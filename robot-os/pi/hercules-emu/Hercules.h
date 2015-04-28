/*
 * Hercules.h
 *
 *  Created on: Mar 5, 2015
 *      Author: danke
 */

#ifndef HERCULES_H_
#define HERCULES_H_

#include <string>
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

	void cmdD();
	void cmdB();
};

#endif /* HERCULES_H_ */
