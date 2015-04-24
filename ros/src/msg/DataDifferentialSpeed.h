/*
 * DataEncoders.h
 *
 *  Created on: Apr 17, 2015
 *      Author: 
 */

#ifndef DATADIFFERENTIALSPEED_H_
#define DATADIFFERENTIALSPEED_H_

#include "Message.h"

class DataDifferentialSpeed : public Message {
private:
	double mLeftSpeed, mRightSpeed;

public:
	DataDifferentialSpeed(double left, double right);
	virtual ~DataDifferentialSpeed();

        double getLeftSpeed();
        double getRightSpeed();
};

#endif /* DATADIFFERENTIAL_H_ */
