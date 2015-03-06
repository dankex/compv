/*
 * Keyboard.h
 *
 *  Created on: Mar 6, 2015
 *      Author: danke
 */

#ifndef KEYBOARD_H_
#define KEYBOARD_H_

class Keyboard {
public:
	Keyboard();
	virtual ~Keyboard();

	bool isHit();
	int getChar(bool &esc);
};

#endif /* KEYBOARD_H_ */
