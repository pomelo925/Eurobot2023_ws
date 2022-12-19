// for ros
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
// ros message
#include <std_srvs/Empty.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
// for cpp
#include <iostream>
#include <cmath>
#include <string>
using namespace std;

enum class Survive
{
    LH1,
    LH2,
    LH_origin,
    Tracker
};

class EulerPose
{
public:
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

class Vive
{
public:
    Vive(ros::NodeHandle nh, ros::NodeHandle nh_local);
    ~Vive();
    void initialize();
    bool initializeParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;
    ros::Publisher origin_pub_;
    ros::Publisher lh_1_pub_;
    ros::Publisher lh_2_pub_;
    ros::Publisher tracker_pub_;
    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent &e);

    tf::TransformBroadcaster lh_broadcaster;
    tf::TransformListener listener;

    EulerPose lookup_tf(std::string target_frame, std::string source_frame);
    EulerPose calculate_the_origin();
    void publish_tf(std::string target_frame, std::string source_frame, EulerPose pose_);
    void publish_pose(EulerPose pose_, Survive survive);

    EulerPose lh1_origin;
    EulerPose lh2_origin;
    EulerPose lh_world_origin;
    EulerPose tracker_pose;

    bool check_lighthouse_tf;
    bool check_survive_tf;

    // param
    bool p_active_;
    int control_frequency_;

    // survive tree frame
    std::string survive_world_frame_;
    std::string survive_tracker_frame_;
    std::string survive_lh1_frame_;
    std::string survive_lh2_frame_;

    // main tf tree frame
    std::string robot_tracker_frame_;
    std::string robot_tracker_parent_frame_;
    std::string robot_tracker_root_frame_;
    std::string lh_world_frame_;
    std::string lh_frame_prefix_;
    std::string lh_origin_frame_prefix_;
    std::string lh_parent_frame_;

    double lh1_x_;
    double lh1_y_;
    double lh1_z_;
    double lh1_theta_;

    double lh2_x_;
    double lh2_y_;
    double lh2_z_;
    double lh2_theta_;
};