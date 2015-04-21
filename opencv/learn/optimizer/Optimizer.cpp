/*
 * Optimizer.cpp
 *
 *  Created on: Apr 21, 2015
 *      Author: danke
 */

#include "Optimizer.h"

Optimizer::Optimizer() {

}

Optimizer::~Optimizer() {

}

void Optimizer::setup(int dim, ObjFunc func) {
	mDim = dim;
	mObjFunc = func;
}
