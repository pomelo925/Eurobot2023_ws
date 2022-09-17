/**
 *
 * @file imu_ros_driver.h
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
 * @author jamie.chen (cjm108033229@gapp.nthu.edu.tw)
 * @version 0.1
 * @date 2022-03-21
 *
 */

#pragma once

#include <cmath>
#include <random>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <sensor_msgs/Imu.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

/* Device address/Identifier for MPU6050*/
const int I2C_ADDR = 0x68; 

const int PWR_MGMT_1 = 0x6B;

const int gyro_x_H = 0x43;
const int gyro_y_H = 0x45;
const int gyro_z_H = 0x47;
const int accel_x_H = 0x3B;
const int accel_y_H = 0x3D;
const int accel_z_H = 0x3F;

int fd;

namespace imu
{
class Imu
{
public:
  Imu(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

private:
  void initialize()
  {
    std_srvs::Empty empt;

    // mpu6050 initialize, connect to device.
    fd = wiringPiI2CSetup(I2C_ADDR);
    if (fd == -1) 
    {
      ROS_FATAL_STREAM("[Imu]: no i2c devicd found?");
      // return false;
    }
    // Device starts in sleep mode so wake it up.
    wiringPiI2CWriteReg16(fd, PWR_MGMT_1, 0);

    updateParams(empt.request, empt.response);
    // return true;
  }

  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void timerCallback(const ros::TimerEvent& e);

  // float read_raw_data(int fd, int addr);

  void calibrateImu();
  void updateImuData();

  void publish();

  /* ros node */
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceServer params_srv_;
  ros::Timer timer_;

  /* ros inter-node */
  ros::Publisher imu_pub_;

  sensor_msgs::Imu imu_data;

  /* ros param */
  bool p_active_;
  bool p_publish_imu_;

  double p_frequency_;
  double p_cov_Gz_;
  double p_cov_ax_;
  double p_cov_ay_;

  /* for mpu6050 calibration */
  bool p_do_calib_;
  bool p_calibrate_mode_;
  int p_sampling_time_;
  float p_offset_Gz_;
  float p_offset_ax_;
  float p_offset_ay_;

  std::string p_imu_frame_;
  std::string p_imu_topic_;
};
}  // namespace imu
