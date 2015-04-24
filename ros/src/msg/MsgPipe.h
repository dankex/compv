/*
 * MsgPipe.h
 *
 *  Created on: Apr 17, 2015
 *      Author: danke
 */

#ifndef MSGPIPE_H_
#define MSGPIPE_H_

#include "../libserial/SerialPort.h"
#include "Message.h"
#include "DataEncoders.h"

#define EOL '\n'

class MsgPipe {
public:
	MsgPipe();
	virtual ~MsgPipe();

	void bind(SerialPort *port);
	Message *readMessage();

private:
	SerialPort *mPort;
};

#endif /* MSGPIPE_H_ */
