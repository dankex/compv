// This code is a modification of Taylor Veltrop's kinect_teleop.cpp
// file found in his Veltrobot package at:
//
// http://www.ros.org/wiki/veltrop-ros-pkg

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <skeleton_markers/Skeleton.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <GL/glut.h>
#include <string>
#include "KinectController.h"
#include "KinectDisplay.h"

using std::string;

#ifndef PI
#define PI 3.14159265359
#endif
#ifndef HALFPI
#define HALFPI 1.57079632679
#endif
#ifndef QUARTPI
#define QUARTPI 0.785398163397
#endif

namespace skeleton_tracker
{
	class SkeletonTracker
  {
    public:
      std::string fixed_frame;

      SkeletonTracker()
      {    
      }
      
      void init()
      {
        ros::NodeHandle n;
	    int rate;
	    n.param("tracking_rate", rate, 1);
	    n.param("fixed_frame", fixed_frame, std::string("openni_depth_frame"));
        skeleton_pub_ = n.advertise<skeleton_markers::Skeleton>("/skeleton", rate);
      }
      
      void publishTransform(KinectController &kinect_controller, XnUserID const &user, XnSkeletonJoint const &joint, string const &frame_id, string const &child_frame_id, skeleton_markers::Skeleton &skeleton)
      {

      	xn::UserGenerator& UserGenerator = kinect_controller.getUserGenerator();
        static tf::TransformBroadcaster br;

        XnSkeletonJointPosition joint_position;
        UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
        double x = joint_position.position.X / 1000.0;
        double y = joint_position.position.Y / 1000.0;
        double z = joint_position.position.Z / 1000.0;

        XnSkeletonJointOrientation joint_orientation;
        UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

        XnFloat* m = joint_orientation.orientation.elements;
        KDL::Rotation rotation(m[0], m[1], m[2],
			       m[3], m[4], m[5],
			       m[6], m[7], m[8]);
        double qx, qy, qz, qw;
        rotation.GetQuaternion(qx, qy, qz, qw);

		geometry_msgs::Vector3 position;
		geometry_msgs::Quaternion orientation;

		position.x = x;
		position.y = y;
		position.z = z;

		orientation.x = qx;
		orientation.y = qy;
		orientation.z = qz;
		orientation.w = qw;

		skeleton.name.push_back(child_frame_id);
		skeleton.position.push_back(position);
		skeleton.orientation.push_back(orientation);
		skeleton.confidence.push_back(joint_position.fConfidence);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, z));
        transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
      }
        
    void processKinect(KinectController &kinect_controller)
      {
        XnUserID users[15];
        XnUInt16 users_count = 15;
        xn::UserGenerator& UserGenerator = kinect_controller.getUserGenerator();
        UserGenerator.GetUsers(users, users_count);
	    skeleton_markers::Skeleton g_skel;

        for (int i = 0; i < users_count; ++i)
        {
          XnUserID user = users[i];
          if (!UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;
            
          publishTransform(kinect_controller, user, XN_SKEL_HEAD,           fixed_frame, "head", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_NECK,           fixed_frame, "neck", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_TORSO,          fixed_frame, "torso", g_skel);

          publishTransform(kinect_controller, user, XN_SKEL_LEFT_SHOULDER,  fixed_frame, "left_shoulder", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_ELBOW,     fixed_frame, "left_elbow", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_HAND,      fixed_frame, "left_hand", g_skel);

          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_SHOULDER, fixed_frame, "right_shoulder", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_ELBOW,    fixed_frame, "right_elbow", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_HAND,     fixed_frame, "right_hand", g_skel);

          publishTransform(kinect_controller, user, XN_SKEL_LEFT_HIP,       fixed_frame, "left_hip", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_KNEE,      fixed_frame, "left_knee", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_FOOT,      fixed_frame, "left_foot", g_skel);

          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_HIP,      fixed_frame, "right_hip", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_KNEE,     fixed_frame, "right_knee", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_FOOT,     fixed_frame, "right_foot", g_skel);

	      g_skel.user_id = user;
	      g_skel.header.stamp = ros::Time::now();
	      g_skel.header.frame_id = fixed_frame;
          skeleton_pub_.publish(g_skel);
	      break;	// only read first user
        }
      }
         
  private:
    ros::Publisher  skeleton_pub_;    
  };

}


#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480
KinectController g_kinect_controller;
skeleton_tracker::SkeletonTracker g_skeleton_tracker;

void glutIdle (void)
{
	glutPostRedisplay();
}

void glutDisplay (void)
{
	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
  
  g_kinect_controller.getContext().WaitAndUpdateAll();
  g_skeleton_tracker.processKinect(g_kinect_controller);
  g_kinect_controller.getDepthGenerator().GetMetaData(depthMD);
  g_kinect_controller.getUserGenerator().GetUserPixels(0, sceneMD);  
	
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
  
  glDisable(GL_TEXTURE_2D);
  
  kinect_display_drawDepthMapGL(depthMD, sceneMD);
	kinect_display_drawSkeletonGL(g_kinect_controller.getUserGenerator(),
                                g_kinect_controller.getDepthGenerator());  
  
	glutSwapBuffers();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
    exit(1);
    break;
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "skeleton_tracker");
  ros::NodeHandle np("~");

  string filepath;
  bool is_a_recording;
  np.getParam("load_filepath", filepath); 
  np.param<bool>("load_recording", is_a_recording, false);      

  g_skeleton_tracker.init();
  g_kinect_controller.init(filepath.c_str(), is_a_recording);
  
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
  glutCreateWindow ("NITE Skeleton Tracker");

  glutKeyboardFunc(glutKeyboard);
  glutDisplayFunc(glutDisplay);
  glutIdleFunc(glutIdle);

  glDisable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);

  glEnableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);  
  
  glutMainLoop();
  
  g_kinect_controller.shutdown();
  
  return 0;
}


