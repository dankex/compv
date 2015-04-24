/*
 * MsgQueue.h
 *
 *  Created on: Apr 17, 2015
 *      Author: danke
 */

#ifndef MSGQUEUE_H_
#define MSGQUEUE_H_

#include <queue>
#include "Message.h"

#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace bpt = boost::posix_time;

template <class E, int MAX>
class MsgQueue {
	class MsgQ {
	public:
		std::queue<Message*> queue;
		boost::mutex mutex;
		boost::interprocess::interprocess_semaphore sema;

		MsgQ()
		: sema(0)
		{
		}
	};
private:
	MsgQ mQueue[MAX];

public:
	MsgQueue() {
	}

	virtual ~MsgQueue() {
	}

	void enqueueMessage(E type, Message *msg) {
		MsgQ &mq(mQueue[type]);

		mq.mutex.lock();
		mq.queue.push(msg);
		mq.mutex.unlock();
		mq.sema.post();
	}

	Message* waitForMessage(E type, double timeout) {
		bpt::ptime toTime(bpt::microsec_clock::local_time());
		toTime += bpt::time_duration(0, 0,
				floor(timeout),
				timeout - floor(timeout));

		MsgQ &mq(mQueue[type]);

		if (mq.sema.timed_wait(toTime)) {
			mq.mutex.lock();
			Message* msg = mq.queue.front();
			mq.queue.pop();
			mq.mutex.unlock();

			return msg;
		} else {
			return 0;
		}
	}
};

#endif /* MSGQUEUE_H_ */
