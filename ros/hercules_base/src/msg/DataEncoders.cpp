/*
 * DataEncoders.cpp
 *
 *  Created on: Apr 17, 2015
 *      Author: danke
 */

#include <sstream>

#include <ros/console.h>
#include "DataEncoders.h"

#define ENC_TO_TRAVEL 0.01f

DataEncoders::DataEncoders(int left, int right, bpt::ptime timeStamp)
: Message("DataEncoders")
, mLeftEncoder(left)
, mRightEncoder(right)
, mLeftDir(true)
, mRightDir(true)
, mTimeStamp(timeStamp) {
}

DataEncoders::~DataEncoders() {

}

double DataEncoders::getTravel(int encoderId) {
	switch (encoderId) {
	case 0:
		//Todo : position
		return encoderToTravel(mLeftEncoder) * (mLeftDir ? -1 : 1);
	case 1:
		return encoderToTravel(mRightEncoder) * (mRightDir ? -1 : 1);
	default:
		return 0;
	}
}

double DataEncoders::encoderToTravel(int enc) {
	return (double)enc * ENC_TO_TRAVEL;
}

void DataEncoders::setDir(bool left, bool right) {
	mLeftDir = left;
	mRightDir = right;
}

bpt::ptime DataEncoders::getTimeStamp() {
	return mTimeStamp;
}

DataEncoders* DataEncoders::parse(const string &msg) {
	// Pass the line to a subclass of Message based on header
	istringstream iss(msg);

	char msgType;
	msgType = iss.get();

	string strLeftEncoder, strRightEncoder;
	getline(iss, strLeftEncoder, DELIMITER);
	getline(iss, strRightEncoder, DELIMITER);

	istringstream issLeft(strLeftEncoder);
	int leftEncoder;
	issLeft >> leftEncoder;

	istringstream issRight(strRightEncoder);
	int rightEncoder;
	issRight >> rightEncoder;

	ROS_INFO("DataEncoders::parse leftEncoder=%d, rightEncoder=%d",
			leftEncoder, rightEncoder);
	bpt::ptime timeStamp(bpt::microsec_clock::local_time());

	return new DataEncoders(leftEncoder, rightEncoder, timeStamp);
}

void DataEncoders::dump() {
	ROS_INFO_STREAM("DataEncoders: left=" << mLeftEncoder
			<< " right=" << mRightEncoder << " time=" << mTimeStamp);
}
