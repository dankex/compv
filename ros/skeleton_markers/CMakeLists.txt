cmake_minimum_required(VERSION 2.8.3)
project(skeleton_markers)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
	geometry_msgs
	std_msgs
	visualization_msgs
	message_generation)
	
# Find Orocos KDL
find_package(orocos_kdl REQUIRED)

# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

# Find Glut
find_package(GLUT REQUIRED)

# Find OpenGL
find_package(OpenGL REQUIRED) 

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
  ${OpenNI_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${OPENGL_INCLUDE_DIR}
)

include_directories(${catkin_INCLUDE_DIRS}
                    ${OpenNI_INCLUDE_DIRS}
                    ${GLUT_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${OPENGL_INCLUDE_DIR}
)

add_message_files (
  FILES
  Skeleton.msg
)

generate_messages(
  	DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS 
	rospy
	roscpp
	message_runtime
	openni_camera
	openni_tracker
	tf
	visualization_msgs
	std_msgs
	geometry_msgs
)

add_executable(skeleton_tracker
  src/skeleton_tracker.cpp
  src/KinectController.cpp
  src/KinectDisplay.cpp)

target_link_libraries(skeleton_tracker
  ${catkin_LIBRARIES}
  ${OpenNI_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}  
  ${orocos_kdl_LIBRARIES})

#install(TARGETS skeleton_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

message("DEBUG variable catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message("DEBUG variable OpenNI_INCLUDE_DIRS: ${OpenNI_INCLUDE_DIRS}")
message("DEBUG variable GLUT_INCLUDE_DIRS: ${GLUT_INCLUDE_DIRS}")
message("DEBUG variable orocos_kdl_INCLUDE_DIRS: ${orocos_kdl_INCLUDE_DIRS}")

