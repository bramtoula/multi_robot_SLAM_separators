cmake_minimum_required(VERSION 2.8.3)
project(multi_robot_separators)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  rtabmap_ros
  tf 
  tf_conversions 
  tf2_ros 
  eigen_conversions
  cv_bridge
  image_transport
  sensor_msgs
  roslaunch
)

roslaunch_add_file_check(launch)


FIND_PACKAGE(RTABMap REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(GTSAM REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)

# Find Qt5 first
FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Svg QUIET)
IF(NOT Qt5_FOUND)
  FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtSvg)
ENDIF(NOT Qt5_FOUND)

# fix libproj.so not found on Xenial
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  KeyPointVec.msg
  KeyPoint3DVec.msg
  Descriptors.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  EstTransform.srv
  GetFeatsAndDesc.srv
  FindMatches.srv
  ReceiveSeparators.srv
  PoseEstimates.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  rtabmap_ros
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_robot_separators
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs message_runtime rtabmap_ros tf tf_conversions tf2_ros eigen_conversions cv_bridge image_transport sensor_msgs std_srvs
  DEPENDS system_lib
)

include_directories(
  ${RTABMap_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(${GTSAM_LIBRARY_DIRS})

add_library(multi_robot_separators 
  src/myRegistration.cpp src/myRegistrationVis.cpp src/MsgConversion.cpp src/logger.cpp
)
target_link_libraries(multi_robot_separators ${catkin_LIBRARIES} ${RTABMap_LIBRARIES} ${OpenCV_LIBRARIES} ${QT_LIBRARIES} ${PCL_LIBRARIES} gtsam)
add_dependencies(multi_robot_separators multi_robot_separators_gencpp)

add_executable(stereo_cam_geometric_tools src/stereoCamGeometricTools.cpp src/myRegistration.cpp src/myRegistrationVis.cpp src/MsgConversion.cpp)
target_link_libraries(stereo_cam_geometric_tools ${catkin_LIBRARIES} ${RTABMap_LIBRARIES} ${OpenCV_LIBRARIES} ${QT_LIBRARIES} ${PCL_LIBRARIES} gtsam multi_robot_separators)
add_dependencies(stereo_cam_geometric_tools ${catkin_EXPORTED_TARGETS} multi_robot_separators_gencpp)

add_executable(factor_graph src/factorGraph.cpp)
target_link_libraries(factor_graph ${catkin_LIBRARIES} gtsam multi_robot_separators)
add_dependencies(factor_graph ${catkin_EXPORTED_TARGETS} multi_robot_separators_gencpp)

add_executable(communication src/communication.cpp src/logger.cpp)
target_link_libraries(communication ${catkin_LIBRARIES} multi_robot_separators)
add_dependencies(communication ${catkin_EXPORTED_TARGETS} multi_robot_separators_gencpp)