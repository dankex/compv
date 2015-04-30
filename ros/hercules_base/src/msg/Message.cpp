/*
 * Message.cpp
 *
 *  Created on: Apr 17, 2015
 *      Author: danke
 */

#include <sstream>
#include <ros/console.h>

#include "Message.h"

Message::Message(const string &type)
: mType(type) {
}

Message::~Message() {
}

bool Message::isType(const string &type) {
	return !mType.compare(type);
}

Message* Message::parse(const string &msg) {
	return 0;
}

// The message type is the first character
const string Message::getMsgType(const string &msg) {
	// Pass the line to a subclass of Message based on header
	return msg.substr(0, 1);
}

void Message::dump() {
	ROS_INFO_STREAM("Message: type=" << mType);
}
