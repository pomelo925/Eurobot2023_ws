#include "ros/ros.h"
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Float64MultiArray.h>

void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    ROS_INFO("%ld", msg->data.size());
    ROS_INFO("%f", msg->data[0]);
    ROS_INFO("%f", msg->data.at(0));
    for (int i = 0; i < msg->data.size(); i++)
    {
        if (i < msg->data.size() / 2)
        {
        }
        else
        {
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_array");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("arr", 10, arrayCallback);
    while (ros::ok())
    {
        /* code */
        ros::spin();
    }
}