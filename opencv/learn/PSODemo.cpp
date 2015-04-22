/*
 * PSODemo.cpp
 *
 *  Created on: Apr 21, 2015
 *      Author: danke
 */

#include <iostream>
#include "optimizer/PSO.h"
#include "math.h"

namespace PSODemo {

double pso_sphere(double *vec, int dim) {
	double sum = 0;
	int i;
	for (i=0; i<dim; i++)
		sum += pow(vec[i], (double) 2.f);

	return sum;
}

const double xMin = -100, xMax = 100;
const double target[] = {1, 2, 3, 4, 6};
const int polyDim = sizeof(target) / sizeof(target[0]);

double poly(double x, const double *vec, int dim) {
	double ans = 0;
	double xx = 1;
	for (int i = 0; i < dim; i++) {
		ans += vec[i] * xx;
		xx = xx * x;
	}

	return ans;
}

double pso_polynomial(double *vec, int dim) {
	const int N = 100;
	double xStep = (xMax - xMin) / N;

	double err = 0;
	for (double x = xMin; x < xMax; x += xStep) {
		double err_step = poly(x, vec, dim) - poly(x, target, dim);
		err += err_step * err_step;
	}

	return err;
}

// BENCHMARK FUNCTION SETTINGS
void pso_set_sphere_settings(pso_settings_t *settings) {
	settings->x_lo = -100;
	settings->x_hi = 100;
	settings->goal = 1e-5;
}

// BENCHMARK FUNCTION SETTINGS
void pso_set_polynomial_settings(pso_settings_t *settings) {
	settings->dim = polyDim;
	settings->x_lo = -100;
	settings->x_hi = 100;
	settings->goal = 1e-5;
}

void test_sphere() {
	PSO pso;

	pso_settings_t &settings(*pso.getSettings());

	pso_set_sphere_settings(&settings);
	pso.setup(settings.dim, pso_sphere);

	// set PSO settings manually
	settings.size = 30;
	settings.nhood_strategy = PSO_NHOOD_RING;
	settings.nhood_size = 10;
	settings.w_strategy = PSO_W_LIN_DEC;

	vector<double> sol;
	double err;
	err = pso.solve(sol);

	cout << "Error = " << err << endl;
}

void test_poly() {
	PSO pso;

	pso_settings_t &settings(*pso.getSettings());

	pso_set_polynomial_settings(&settings);
	pso.setup(settings.dim, pso_polynomial);

	// set PSO settings manually
	settings.nhood_strategy = PSO_NHOOD_RING;
	settings.nhood_size = 10;
	settings.w_strategy = PSO_W_LIN_DEC;

	vector<double> sol;
	double err;
	err = pso.solve(sol);

	cout << "Error = " << err << endl;

	cout << "Params: ";
	for (vector<double>::iterator it = sol.begin(); it != sol.end(); ++it) {
		cout << *it << " ";
	}
	cout << endl;
}

int main( int argc, char** argv ) {
	test_sphere();
	test_poly();
	return 0;
}

} // NS
