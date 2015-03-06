/*
 * Keyboard.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: danke
 */

#include "Keyboard.h"
#include <ncurses.h>

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

char Keyboard::getChar() {
	return getch();
}
