/**
 *
 * @file imu_ros_driver_node.cpp
 * @brief
 *
 * @code{.unparsed}
 *      _____
 *     /  /::\       ___           ___
 *    /  /:/\:\     /  /\         /  /\
 *   /  /:/  \:\   /  /:/        /  /:/
 *  /__/:/ \__\:| /__/::\       /  /:/
 *  \  \:\ /  /:/ \__\/\:\__   /  /::\
 *   \  \:\  /:/     \  \:\/\ /__/:/\:\
 *    \  \:\/:/       \__\::/ \__\/  \:\
 *     \  \::/        /__/:/       \  \:\
 *      \__\/         \__\/         \__\/
 * @endcode
 *
 * @author jamie.chen (jamie@gmail.com)
 * @version 0.1
 * @date 2022-03-20
 *
 */

#include "imu_ros_driver/imu_ros_driver.h"

using namespace imu;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpu9250");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Imu]: Initializing node");
    Imu imu(nh, nh_local);
    ros::spin();
    // ros::spinOnce();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Imu]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Imu]: Unexpected error");
  }

  return 0;
}
