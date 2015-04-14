#ifndef HERCULES_H_
#define HERCULES_H_

#include <string>
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

private:
	string mPort;
	SerialPort *mSerialPtr;

protected:
	void sendDriveCmd(int left, int right, int dirL, int dirR);
	int speedToLevel(double speed);
};

#endif /* HERCULES_H_ */
