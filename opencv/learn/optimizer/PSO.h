/*
 * PSO.h
 *
 *  Created on: Apr 21, 2015
 *      Author: danke
 */

#ifndef OPT_PSO_H_
#define OPT_PSO_H_

#include "Optimizer.h"

#include "pso/pso.h"

class PSO : public Optimizer {
protected:
	pso_settings_t mSettings;

public:
	PSO();
	virtual ~PSO();

	virtual double solve(vector<double> &solution);

	pso_settings_t* getSettings();
};

#endif /* OPT_PSO_H_ */
