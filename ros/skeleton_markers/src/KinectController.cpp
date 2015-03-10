#include <XnOpenNI.h>
#include <XnCodecIDs.h>

#include "KinectController.h"

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

xn::Context KinectController::g_Context;
xn::DepthGenerator KinectController::g_DepthGenerator;
xn::UserGenerator KinectController::g_UserGenerator;

XnBool KinectController::g_bNeedPose = FALSE;
XnChar KinectController::g_strPose[20] = "";

KinectController::KinectController()
{
}

// Accessors
xn::UserGenerator& KinectController::getUserGenerator()
{
	return g_UserGenerator;
}

xn::DepthGenerator& KinectController::getDepthGenerator()
{
	return g_DepthGenerator;
}

xn::Context& KinectController::getContext()
{
	return g_Context;
}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("New User %d\n", nId);
	// New user found
	if (KinectController::g_bNeedPose)
	{
		KinectController::g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(KinectController::g_strPose, nId);
	}
	else
	{
		KinectController::g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("Lost user %d\n", nId);
}

// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	printf("Pose %s detected for user %d\n", strPose, nId);
	KinectController::g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	KinectController::g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
	printf("Calibration started for user %d\n", nId);
}

// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
	if (bSuccess)
	{
		// Calibration succeeded
		printf("Calibration complete, start tracking user %d\n", nId);
		KinectController::g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else
	{
		// Calibration failed
		printf("Calibration failed for user %d\n", nId);
		if (KinectController::g_bNeedPose)
		{
			KinectController::g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(KinectController::g_strPose, nId);
		}
		else
		{
			KinectController::g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}


int KinectController::init(const char* path, bool recording)
{
	XnStatus nRetVal = XN_STATUS_OK;

	if (recording)
	{
		nRetVal = g_Context.Init();
		CHECK_RC(nRetVal, "Init");
		nRetVal = g_Context.OpenFileRecording(path);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("Can't open recording %s: %s\n", path, xnGetStatusString(nRetVal));
			return 1;
		}
	}
	else
	{
		nRetVal = g_Context.InitFromXmlFile(path);
		CHECK_RC(nRetVal, "InitFromXml");
	}

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(nRetVal, "Find depth generator");
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		printf("Supplied user generator doesn't support skeleton\n");
		return 1;
	}
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			printf("Pose required, but not supported\n");
			return 1;
		}
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");
  
  return 0;
}

int KinectController::shutdown()
{
	g_Context.Release();
  return 0;
}
