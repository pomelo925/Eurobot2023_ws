cmake_minimum_required(VERSION 3.0.2)
project(vive_tracker_ros_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs tf
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   /home/jing/ViveTracker/libsurvive
   /home/jing/ViveTracker/libsurvive/include
   /home/jing/ViveTracker/libsurvive/include/libsurvive
   /home/jing/ViveTracker/libsurvive/redist
)

set(LIBS /home/jing/ViveTracker/libsurvive/lib/libsurvive.so)

# Node for vive tracker
add_executable(vive_tracker_ros_node src/vive_tracker_node.cpp)
target_link_libraries(vive_tracker_ros_node ${LIBS} ${catkin_LIBRARIES})
