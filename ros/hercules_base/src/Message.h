/*
 * Message.h
 *
 *  Created on: Apr 17, 2015
 *      Author: danke
 */

#ifndef MESSAGE_H_
#define MESSAGE_H_

class Message {
public:
	Message();
	virtual ~Message();

	Message* getUpdate(int timeoutMs);
};

#endif /* MESSAGE_H_ */
