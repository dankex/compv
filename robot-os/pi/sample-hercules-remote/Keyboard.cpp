/*
 * Keyboard.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: danke
 */

#include "Keyboard.h"
#include <ncurses.h>

#define KEY_ESC 27

Keyboard::Keyboard() {
	initscr();
	cbreak();
}

Keyboard::~Keyboard() {
}

bool Keyboard::isHit() {
	int ch = getch();

	if (ch != ERR) {
		ungetch(ch);
		return true;
	} else {
		return false;
	}
}

int Keyboard::getChar(bool &esc) {
	esc = false;
	char first = getch();
	if (first != KEY_ESC)
		return first;

	esc = true;
	int second = getch();
	int third = getch();
	int fourth = getch();

	return third;
}

void Keyboard::clear() {
	while (isHit()) {
                bool dummy;
		getChar(dummy);
	}
}
