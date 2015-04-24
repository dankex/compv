/*
 * DataEncoders.cpp
 *
 *  Created on: Apr 17, 2015
 *      Author: danke
 */

#include <sstream>

#include "DataEncoders.h"

#define ENC_TO_TRAVEL 0.01f

DataEncoders::DataEncoders(int left, int right)
: Message("DataEncoders")
, left(left)
, right(right) {
}

DataEncoders::~DataEncoders() {

}

double DataEncoders::getTravel(int encoderId) {
	switch (encoderId) {
	case 0:
		return left;
	case 1:
		return right;
	default:
		return 0;
	}
}

double DataEncoders::encoderToTravel(int enc) {
	return (double)enc * ENC_TO_TRAVEL;
}

DataEncoders* DataEncoders::parse(const string &msg) {
	// Pass the line to a subclass of Message based on header
	istringstream iss(msg);

	string msgType;
	getline(iss, msgType, DELIMITER);

	string strLeftEncoder, strRightEncoder;
	getline(iss, strLeftEncoder, DELIMITER);
	getline(iss, strRightEncoder, DELIMITER);

	istringstream issLeft(strLeftEncoder);
	int leftEncoder;
	issLeft >> leftEncoder;

	istringstream issRight(strRightEncoder);
	int rightEncoder;
	issRight >> rightEncoder;

	return new DataEncoders(leftEncoder, rightEncoder);
}
