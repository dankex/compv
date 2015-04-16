/*
 * PolynomialFit.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: danke
 */

#include "opencv2/highgui/highgui.hpp"
#include <string>

using namespace cv;
using namespace std;

namespace PolynomialFit {

int main(int argc, char **argv) {
	string windowName("PolynomialFit");

	cvNamedWindow( windowName.c_str(), CV_WINDOW_AUTOSIZE );

	cvWaitKey(0);
	cvDestroyWindow( windowName.c_str() );

	return 0;
}

} // namespace PolynomialFit
