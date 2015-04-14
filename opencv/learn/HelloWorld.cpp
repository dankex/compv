#include "opencv2/highgui/highgui.hpp"

namespace HelloWorld {

int main( int argc, char** argv ) {
	IplImage* img = cvLoadImage( argc >= 2 ? argv[1] : "images/test.jpg" );
	cvNamedWindow( "Example1", CV_WINDOW_AUTOSIZE );
	cvShowImage( "Example1", img );
	cvWaitKey(0);
	cvReleaseImage( &img );
	cvDestroyWindow( "Example1" );
}

}
