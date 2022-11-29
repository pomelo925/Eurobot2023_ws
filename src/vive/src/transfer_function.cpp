#include <vector>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/transform_datatypes.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transfer_test");
    ros::NodeHandle nh;
    while (ros::ok())
    {
        /* code */
        ros::spin();
    }
}