/**
 *
 * @file imu_ros_driver.cpp
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
 * @author jamie.chen (jamie.chen@gmail.com)
 * @version 0.1
 * @date 2022-03-21
 *
 */

#include "imu_ros_driver/imu_ros_driver.h"

using namespace std;
using namespace imu;

/*for unit correctness*/
const float dps_to_rps = 3.14159265 / 180 ;
const float la_rescale = 16384.0 / 9.807;

int sampling_num = 0;

Imu::Imu(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
  : nh_(nh), nh_local_(nh_local)
{
  timer_ = nh_.createTimer(ros::Duration(1.0), &Imu::timerCallback, this, false, false);
  initialize();
}

float read_raw_data(int fd, int addr) 
{
  int high = wiringPiI2CReadReg8(fd, addr);
  int low = wiringPiI2CReadReg8(fd, addr+1);
  int val = (high << 8) + low;
  return float((val >= 0x8000) ? -((65535 - val) + 1) : val);
}

bool Imu::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  bool get_param_ok = true;
  bool prev_active = p_active_;

  /* get param */
  get_param_ok = nh_local_.param<bool>("active", p_active_, true);
  get_param_ok = nh_local_.param<bool>("publish_imu", p_publish_imu_, true);

  get_param_ok = nh_local_.param<double>("frequency", p_frequency_, 1000);

  get_param_ok = nh_local_.param<bool>("do_calib", p_do_calib_, false);
  get_param_ok = nh_local_.param<bool>("calibrate_mode", p_calibrate_mode_, false);
  get_param_ok = nh_local_.param<int> ("sampling_time", p_sampling_time_, 1000);
  get_param_ok = nh_local_.param<float>("offset_Gz", p_offset_Gz_, 0.0);
  get_param_ok = nh_local_.param<float>("offset_ax", p_offset_ax_, 0.0);
  get_param_ok = nh_local_.param<float>("offset_ay", p_offset_ay_, 0.0);

  get_param_ok = nh_local_.param<double>("cov_Gz", p_cov_Gz_, 1e-9);
  get_param_ok = nh_local_.param<double>("cov_ax", p_cov_ax_, 1e-9);
  get_param_ok = nh_local_.param<double>("cov_ay", p_cov_ay_, 1e-9);

  get_param_ok = nh_local_.param<string>("imu_frame", p_imu_frame_, "mpu9250_frame");
  get_param_ok = nh_local_.param<string>("imu_topic_", p_imu_topic_, "raw");

  /* check param */
  if (get_param_ok)
  {
    ROS_INFO_STREAM("[Imu]: "
                    << "param set ok");
  }
  else
  {
    ROS_WARN_STREAM("[Imu]: "
                    << "param set fail");
  }

  /* ros node param */
  timer_.setPeriod(ros::Duration(1 / p_frequency_), false);

  if (p_active_ != prev_active)
  {
    if (p_active_)
    {
      imu_pub_ = nh_.advertise<sensor_msgs::Imu>(p_imu_topic_, 10);
      timer_.start();
      ROS_INFO_STREAM("[Imu]: "
                    << "timer start!");
    }
    else
    {
      imu_pub_.shutdown();
      timer_.stop();
      ROS_WARN_STREAM("[Imu]: "
                    << "timer set fail!");
    }
  }

  // clang-format off
  imu_data.orientation_covariance = {0, 0, 0,
                                     0, 0, 0,
                                     0, 0, 0};

                                         // x    y    z
  imu_data.angular_velocity_covariance = {0, 0, 0,
                                          0, 0, 0,
                                          0, 0, p_cov_Gz_};

                                            // x       y       z
  imu_data.linear_acceleration_covariance = {p_cov_ax_, 0,         0,
                                             0,         p_cov_ay_, 0,
                                             0,         0,         0};
  // clang-format on

  return true;
}

void Imu::calibrateImu()
/*for do_calib*/
{
  // ROS_INFO("do_calib mode start!");

  p_offset_Gz_ = 0;
  p_offset_ax_ = 0;
  p_offset_ay_ = 0;

  /*imu_calib does not calibrate gyro velocity, need data processing */
  imu_data.angular_velocity.x = (read_raw_data(fd, gyro_x_H) / 131 * dps_to_rps);
  imu_data.angular_velocity.y = (read_raw_data(fd, gyro_y_H) / 131 * dps_to_rps);
  imu_data.angular_velocity.z = (read_raw_data(fd, gyro_z_H) / 131 * dps_to_rps);

  /*imu_calib calibrate ax, ay, az*/
  imu_data.linear_acceleration.x = (read_raw_data(fd, accel_x_H) / la_rescale);
  imu_data.linear_acceleration.y = (read_raw_data(fd, accel_y_H) / la_rescale);
  if(p_do_calib_)
    imu_data.linear_acceleration.z = (read_raw_data(fd, accel_z_H) / la_rescale);
}

void Imu::updateImuData()
{
  // Read gyroscope values.
  // At default sensitivity of 250deg/s we need to scale by 131, the unit here is deg/s
  // And!! Imu msg docs say rotational velocity shoould be in rad/s so need to scale by *pi / 180
  imu_data.angular_velocity.x = (read_raw_data(fd, gyro_x_H) / 131 * dps_to_rps);
  imu_data.angular_velocity.y = (read_raw_data(fd, gyro_y_H) / 131 * dps_to_rps);
  imu_data.angular_velocity.z = (read_raw_data(fd, gyro_z_H) / 131 * dps_to_rps);

    // Read accelerometer values.
    // At default sensitivity of 2g we need to scale by 16384.
    // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
    // But! Imu msg docs say acceleration should be in m/s^2 so need to *9.807
  imu_data.linear_acceleration.x = (read_raw_data(fd, accel_x_H) / la_rescale);
  imu_data.linear_acceleration.y = (read_raw_data(fd, accel_y_H) / la_rescale);
  imu_data.linear_acceleration.z = (read_raw_data(fd, accel_z_H) / la_rescale);

}

void Imu::timerCallback(const ros::TimerEvent& e)
{
  updateImuData();
  publish();
}

void Imu::publish()
{
  ros::Time now = ros::Time::now();

  /* imu_data */

  imu_data.header.stamp = now;
  imu_data.header.frame_id = p_imu_frame_;
  if (p_publish_imu_)
    imu_pub_.publish(imu_data);
}