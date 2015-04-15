#include <iostream>
#include <string>
#include "cpputils.hpp"
#include "subprograms.hpp"

using namespace std;

struct Subprogram {
	string name;
	int (*pMain)(int argc, char **argv);
};

Subprogram gPrograms[] = {
		{"Hello World", HelloWorld::main}
};

int main(int argc, char **argv) {
	int nProgs = SizeOfArray(gPrograms);
	cout << "Select a program to run" << endl << endl;
	for (int i = 0; i < nProgs; i++) {
		cout << i + 1 << ". " << gPrograms[i].name << endl;
	}

	int num;

	while (true) {
		cout << endl << "Which program? ";
		cin >> num;
		if (num >= 1 && num <= nProgs)
			break;
	}

	num--;

	cout << "Execute program " << gPrograms[num].name << endl;
	return gPrograms[num].pMain(argc, argv);
}
