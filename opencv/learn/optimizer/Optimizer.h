/*
 * Optimizer.h
 *
 *  Created on: Apr 21, 2015
 *      Author: danke
 */

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include <vector>
#include <map>

using namespace std;

class Optimizer {
public:
	typedef double (*ObjFunc)(double *, int);

	class Result {
		double error;
		vector<double> globalBest; // should contain DIM elements!!
	};

	Optimizer();
	virtual ~Optimizer();

	void setup(int dim, ObjFunc func);

	virtual double solve(vector<double> &solution) = 0;

	ObjFunc getObjFunc() { return mObjFunc; }

protected:
	int mDim;
	ObjFunc mObjFunc;
};

#endif /* OPTIMIZER_H_ */
