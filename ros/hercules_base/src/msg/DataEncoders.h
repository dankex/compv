/*
 * DataEncoders.h
 *
 *  Created on: Apr 17, 2015
 *      Author: danke
 */

#ifndef DATAENCODERS_H_
#define DATAENCODERS_H_

#include "Message.h"

class DataEncoders : public Message {
private:
	int left, right;
	double encoderToTravel(int enc);

public:
	DataEncoders(int left, int right);
	virtual ~DataEncoders();

	double getTravel(int encoderId);

	static DataEncoders* parse(const string &msg);
};

#endif /* DATAENCODERS_H_ */
