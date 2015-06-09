#include <iostream>
#include <string>
#include <cstdlib>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>
#include "Hercules.h"

using namespace std;

//#define CMD_DRIVE "D%c%c"
#define CMD_DRIVE "D%d,%d,%d,%d,\n"
#define CMD_BATTERY "B,,\n"

#define MAX_LEVEL	50 //100
#define MAX_SPEED	1		// 10 m/s

#define MAX_ENCODERDATA 10

Hercules::Hercules()
{
	mLeftEncoder = 0;
	mRightEncoder = 0;
	countLoop = 0;
}

Hercules::~Hercules() {
	if (mSerialPtr) {
		delete mSerialPtr;
	}
}

void Hercules::reconnect() {
	mSerialPtr->Open(SerialPort::BAUD_19200,
			SerialPort::CHAR_SIZE_8,
			SerialPort::PARITY_DEFAULT,
			SerialPort::STOP_BITS_DEFAULT,
			SerialPort::FLOW_CONTROL_DEFAULT);
}

void Hercules::connect(const string &port) {
	mPort = port;
	mSerialPtr = new SerialPort(port);
	mMsgPipe.bind(mSerialPtr);
	mThread = new boost::thread(boost::bind(&Hercules::reader, this));
	reconnect();
}

void Hercules::reader() {
	long value = 0;
	while (true) {
		try {
			Message *msg = mMsgPipe.readMessage();
			if (msg) {
				processMsg(msg);
				enqueue(msg);

				// Debug
				msg->dump();
			}

			usleep(100);
		} catch (exception e) {

		}
	}
}

void Hercules::configureLimits(double max_speed, double max_accel) {

}

void Hercules::controlSpeed(double speed_left, double speed_right,
		double accel_left, double accel_right) {
	int signL = speed_left < 0 ? 0x80 : 0;
	int signR = speed_right < 0 ? 0x80 : 0;

	speed_left = fabs(speed_left);
	speed_right = fabs(speed_right);

	int lspeed = speedToLevel(speed_left);
	int rspeed = speedToLevel(speed_right);
	//        ROS_DEBUG("Hercules int lspeed=%d, rspeed=%d", lspeed, rspeed);
	sendDriveCmd(lspeed, signL, rspeed, signR);
	if ((countLoop++ % 10) == 0) {
		sendGetBatteryCmd();
	}
}

void Hercules::sendDriveCmd(int leftSpeed, int leftDir, int rightSpeed, int rightDir) {
	char buf[20];
	snprintf(buf, sizeof(buf), CMD_DRIVE, leftSpeed, rightSpeed, leftDir, rightDir);
	mSerialPtr->Write(buf);
	mLeftDir[1] = mLeftDir[0]; // previous
	mRightDir[1] = mRightDir[0]; // previous
	mLeftDir[0] = leftDir;
	mRightDir[0] = rightDir;

	ROS_DEBUG("Hercules sendDriveCmd:%s \n", buf);
}

void Hercules::sendGetBatteryCmd() {
	char buf[20];
	snprintf(buf, sizeof(buf), CMD_BATTERY);
	mSerialPtr->Write(buf);
	ROS_DEBUG("Hercules sendGetBatteryCmd:%s \n", buf);
}

int Hercules::speedToLevel(double speed) {
	int level = speed * MAX_LEVEL / MAX_SPEED;
	if (level == MAX_LEVEL)
		level--;
	return level;
}

DataDifferentialSpeed* Hercules::getDifferentialSpeed() {
	std::vector<DataEncoders>::iterator it;

	mMutexEncoderData.lock();
	if(mEncoderData.empty()) {
		ROS_DEBUG("EncoderData empty");
		mMutexEncoderData.unlock();
		return 0;
	}

	bpt::ptime fromTime(bpt::microsec_clock::local_time());
	fromTime -= bpt::time_duration(0,0,1,0);

	int left = 0;
	int right = 0;
	int i = 0;
	for (it=mEncoderData.begin(); it!=mEncoderData.end(); i++) {
		if(mEncoderData[i].getTimeStamp() > fromTime) {
			left += mEncoderData[i].getTravel(0);
			right += mEncoderData[i].getTravel(1);
			it++;
		} else {
			it = mEncoderData.erase(it);
		}
	}
	int num = mEncoderData.size();
	if(num < 2) {
		ROS_DEBUG("EncoderData outdated");
		mMutexEncoderData.unlock();
		return 0;
	}
	ROS_DEBUG("EncoderData num=%d", num);
	bpt::ptime frontTimeStamp = mEncoderData[0].getTimeStamp();
	bpt::ptime backTimeStamp = mEncoderData[num-1].getTimeStamp();
	bpt::time_duration td = backTimeStamp - frontTimeStamp;
	mMutexEncoderData.unlock();
	if(td.total_milliseconds()==0.0){
		ROS_DEBUG("EncoderData timeduration =0");
		return 0;
	}

//	double leftSpeed =(double) left/td.total_milliseconds()/1000; // m/sec
//	double rightSpeed =(double) right/td.total_milliseconds()/1000;
	ROS_DEBUG("EncoderData Traveled left=%lf, right=%lf", left, right);
	return new DataDifferentialSpeed(left, right);
}

Message* Hercules::requestData(Channel channel, double timeout) {
	if (channel == DIFFERENTIALSPEED) {
		ROS_DEBUG("DifferentialSpeed requestData");
//		return (Message*) getDifferentialSpeed();
                return 0;
	} else if (channel == ODOMETRY) {
		return mQueue.waitForMessage(channel, timeout);
	} else {
               return 0;
	}
}

void Hercules::enqueue(Message *msg) {
	if (msg->isType("DataEncoders")) {
		mQueue.enqueueMessage(ODOMETRY, msg);

		mMutexEncoderData.lock();

		DataEncoders dataEncoder(*(DataEncoders *) msg);
		if (mEncoderData.size() < MAX_ENCODERDATA) {
			mEncoderData.push_back(dataEncoder);
			ROS_DEBUG("EncoderData pushback");
		} else {
			mEncoderData.erase(mEncoderData.begin());
			mEncoderData.push_back(dataEncoder);
			ROS_DEBUG("EncoderData erase and pushback");
		}

		mMutexEncoderData.unlock();
	}
}

void Hercules::processMsg(Message *msg) {
	if (msg->isType("DataEncoders")) {
		DataEncoders *enc = (DataEncoders*) msg;
		enc->setDir(mLeftDir[1], mRightDir[1]);
		//enc->setOrigin(mLeftEncoder, mRightEncoder);

		// update origin
		enc->getData(mLeftEncoder, mRightEncoder);
	}
}
