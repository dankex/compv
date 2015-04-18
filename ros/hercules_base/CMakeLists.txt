cmake_minimum_required(VERSION 2.8.3)
project(hercules_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface hercules_msgs
  diagnostic_updater roslaunch roslint roscpp sensor_msgs)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES libserial
    CATKIN_DEPENDS diagnostic_updater hardware_interface hercules_msgs roscpp sensor_msgs
    DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(libserial
   src/libserial/PosixSignalDispatcher.cpp
   src/libserial/SerialPort.cpp
   src/libserial/SerialStreamBuf.cc
   src/libserial/SerialStream.cc
)

add_library(libmsg
   src/msg/Message.cpp
   src/msg/MsgPipe.cpp
   src/msg/DataEncoders.cpp
)

add_executable(hercules_node
  src/hercules_base.cpp
  src/hercules_hardware.cpp
#  src/hercules_diagnostics.cpp
  src/Hercules.cpp
  src/hercules_wilson_wrapper.cpp)
target_link_libraries(hercules_node libserial libmsg ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(hercules_node hercules_msgs_gencpp)

roslint_cpp(
  src/hercules_base.cpp
  src/hercules_hardware.cpp
#  src/hercules_diagnostics.cpp
  include/hercules_base/hercules_diagnostics.h
  include/hercules_base/hercules_hardware.h
)

roslaunch_add_file_check(launch)

install(TARGETS hercules_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
