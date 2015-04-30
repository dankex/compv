/*
 * DataEncoders.cpp
 *
 *  Created on: Apr 17, 2015
 *      Author: 
 */

#include <sstream>

#include <ros/console.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "DataDifferentialSpeed.h"

namespace bpt = boost::posix_time;

#define ENC_TO_TRAVEL 0.01f

DataDifferentialSpeed::DataDifferentialSpeed(double left, double right)
: Message("DataDifferentialSpeed")
, mLeftSpeed(left)
, mRightSpeed(right) {
}

DataDifferentialSpeed::~DataDifferentialSpeed() {

}

double DataDifferentialSpeed::getLeftSpeed() {
        return mLeftSpeed;
}

double DataDifferentialSpeed::getRightSpeed() {
	return mRightSpeed;
}

void DataDifferentialSpeed::dump() {
	ROS_INFO_STREAM("DataDifferentialSpeed: leftSpeed=" << mLeftSpeed
			<< " rightSpeed=" << mLeftSpeed);
}
