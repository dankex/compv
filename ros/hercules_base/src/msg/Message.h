/*
 * Message.h
 *
 *  Created on: Apr 17, 2015
 *      Author: danke
 */

#ifndef MESSAGE_H_
#define MESSAGE_H_

#include <string>

#define DELIMITER ','

using namespace std;

class Message {
protected:
	string mType;
public:
	Message(const string &type);
	virtual ~Message();

	bool isType(const string &type);

	static const string getMsgType(const string &msg);
	static Message* parse(const string &msg);
};

#endif /* MESSAGE_H_ */
