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
public:
	DataEncoders();
	virtual ~DataEncoders();

	double getTravel(int encoderId);
};

#endif /* DATAENCODERS_H_ */
