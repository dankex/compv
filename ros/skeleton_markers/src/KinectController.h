
#ifndef KINECT_CONTROLLER_H_
#define KINECT_CONTROLLER_H_

#include <XnCppWrapper.h>

class KinectController
{
	public:
  	KinectController();
    int init(const char* path, bool recording=false);
    int shutdown();
    
    xn::UserGenerator&  getUserGenerator();
    xn::DepthGenerator& getDepthGenerator();
		xn::Context&        getContext();
    
  public:
		static xn::Context g_Context;
		static xn::DepthGenerator g_DepthGenerator;
		static xn::UserGenerator g_UserGenerator;

		static XnBool g_bNeedPose;
		static XnChar g_strPose[20];
};

#endif 
