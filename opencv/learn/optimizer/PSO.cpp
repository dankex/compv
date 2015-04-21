/*
 * PSO.cpp
 *
 *  Created on: Apr 21, 2015
 *      Author: danke
 */

#include "PSO.h"
#include "cstdlib"

double pso_obj_fun(double *vars, int dim, void *ctx) {
	PSO *pso = (PSO*)ctx;
	return (*pso->getObjFunc())(vars, dim);
}

PSO::PSO() {
	pso_set_default_settings(&mSettings);
}

PSO::~PSO() {
}

double PSO::solve(vector<double> &solution) {
	pso_result_t sol;
	// allocate memory for the best position buffer
	sol.gbest = (double*)malloc(mSettings.dim * sizeof(double));

    // run optimization algorithm
    pso_solve(pso_obj_fun, NULL, &sol, &mSettings);

    // Free memory
    free(sol.gbest);

	return 0;
}
