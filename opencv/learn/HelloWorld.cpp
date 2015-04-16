#include "opencv2/highgui/highgui.hpp"
#include <string>

using namespace std;

namespace HelloWorld {

#define WINDOW_NAME "HelloWorld"

int main( int argc, char** argv ) {
	IplImage* img = cvLoadImage( argc >= 2 ? argv[1] : "images/test.jpg" );
	cvNamedWindow( WINDOW_NAME, CV_WINDOW_AUTOSIZE );
	cvShowImage( WINDOW_NAME, img );
	cvWaitKey(0);
	cvReleaseImage( &img );
	cvDestroyWindow( WINDOW_NAME );

	return 0;
}

}
