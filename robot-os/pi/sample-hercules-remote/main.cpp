/*
 * project: sample-hercules-remote
 *
 * Remote control the hercules robotic platform.
 *
 * main.cpp
 */

#include "Hercules.h"
#include <string>

using namespace std;

int main(int argc, char **argv) {
	string serialTty = (argc > 1 ? argv[1] : "/dev/ttyUSB0");
	Hercules h(serialTty);
	h.run();
	return 0;
}
