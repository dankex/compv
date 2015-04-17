#ifndef HERCULES_H_
#define HERCULES_H_

#include <string>

//#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#include "libserial/SerialStream.h"

using namespace std;
using namespace LibSerial;

class Hercules {
public:
	Hercules();
	virtual ~Hercules();

	void connect(const std::string &port);
	void reconnect();

	void configureLimits(double max_speed, double max_accel);
	void controlSpeed(double speed_left, double speed_right,
			double accel_left, double accel_right);
        void reader();
        int processData(long *num);
private:
	string mPort;
	SerialPort *mSerialPtr;
        boost::thread* mThread;
        long mLeftEncoder;
        long mRightEncoder;
        int mVoltage;
        int countLoop;
       

protected:
	void sendDriveCmd(int left, int right, int dirL, int dirR);
	void sendGetBatteryCmd();
	int speedToLevel(double speed);
};

#endif /* HERCULES_H_ */
