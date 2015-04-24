/*
 * Message.cpp
 *
 *  Created on: Apr 17, 2015
 *      Author: danke
 */

#include <sstream>

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

const string Message::getMsgType(const string &msg) {
	// Pass the line to a subclass of Message based on header
	istringstream iss(msg);

	string msgType;
	getline(iss, msgType, DELIMITER);

	return msgType;
}
