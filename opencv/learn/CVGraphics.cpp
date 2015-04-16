/*
 * CVGraphics.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: danke
 */

#include "opencv2/highgui/highgui.hpp"
#include <string>

using namespace cv;
using namespace std;

namespace CVGraphics {

#define WIN_WIDTH		600

void myPolygon( Mat &img, double w )
{
	int lineType = 8;

	/** Create some points */
	Point rook_points[1][20];
	rook_points[0][0] = Point( w/4.0, 7*w/8.0 );
	rook_points[0][1] = Point( 3*w/4.0, 7*w/8.0 );
	rook_points[0][2] = Point( 3*w/4.0, 13*w/16.0 );
	rook_points[0][3] = Point( 11*w/16.0, 13*w/16.0 );
	rook_points[0][4] = Point( 19*w/32.0, 3*w/8.0 );
	rook_points[0][5] = Point( 3*w/4.0, 3*w/8.0 );
	rook_points[0][6] = Point( 3*w/4.0, w/8.0 );
	rook_points[0][7] = Point( 26*w/40.0, w/8.0 );
	rook_points[0][8] = Point( 26*w/40.0, w/4.0 );
	rook_points[0][9] = Point( 22*w/40.0, w/4.0 );
	rook_points[0][10] = Point( 22*w/40.0, w/8.0 );
	rook_points[0][11] = Point( 18*w/40.0, w/8.0 );
	rook_points[0][12] = Point( 18*w/40.0, w/4.0 );
	rook_points[0][13] = Point( 14*w/40.0, w/4.0 );
	rook_points[0][14] = Point( 14*w/40.0, w/8.0 );
	rook_points[0][15] = Point( w/4.0, w/8.0 );
	rook_points[0][16] = Point( w/4.0, 3*w/8.0 );
	rook_points[0][17] = Point( 13*w/32.0, 3*w/8.0 );
	rook_points[0][18] = Point( 5*w/16.0, 13*w/16.0 );
	rook_points[0][19] = Point( w/4.0, 13*w/16.0) ;

	const Point* ppt[1] = { rook_points[0] };
	int npt[] = { 20 };

	fillPoly(img,
			ppt,
			npt,
			1,
			Scalar( 255, 255, 255 ),
			lineType );
}

void showGraphics(const char *winName) {
	Mat image = Mat::zeros( WIN_WIDTH, WIN_WIDTH, CV_8UC3 );

	myPolygon(image, WIN_WIDTH);
	imshow(winName, image);
}

int main(int argc, char **argv) {
	const char windowName[] = "PolynomialFit";

	cvNamedWindow( windowName, 0 );
	cvResizeWindow( windowName, WIN_WIDTH, WIN_WIDTH);

	showGraphics(windowName);

	cvWaitKey(0);
	cvDestroyWindow( windowName );

	return 0;
}

}

