#include "lighthouse_tf.h"

/* publish transform from map_frame to lighthouse_frame (lighthouse world origin)*/

Vive::Vive(ros::NodeHandle nh, ros::NodeHandle nh_local)
{
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty e;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("/params", &Vive::initializeParams, this);
    initializeParams(e.request, e.response);
    initialize();
}

Vive::~Vive()
{
    nh_local_.deleteParam("/active");
    nh_local_.deleteParam("/control_frequency");
    nh_local_.deleteParam("/lh1_x");
    nh_local_.deleteParam("/lh1_y");
    nh_local_.deleteParam("/lh1_z");
    nh_local_.deleteParam("/lh1_theta");
    nh_local_.deleteParam("/lh2_x");
    nh_local_.deleteParam("/lh2_y");
    nh_local_.deleteParam("/lh2_z");
    nh_local_.deleteParam("/lh2_theta");
}

void Vive::initialize()
{
    check_survive_tf = false;
    check_lighthouse_tf = false;
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &Vive::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer_.start();
}

bool Vive::initializeParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    bool get_param_ok = true;
    bool prev_active = p_active_;

    get_param_ok = nh_local_.param<bool>("active", p_active_, true);
    get_param_ok = nh_local_.param<int>("control_frequency", control_frequency_, 100);

    // survive tree frame
    get_param_ok = nh_local_.param<string>("survive_world_frame", survive_world_frame_, "lh_world");
    get_param_ok = nh_local_.param<string>("survive_tracker_frame", survive_tracker_frame_, "LHR-94135636");
    get_param_ok = nh_local_.param<string>("survive_lh1_frame", survive_lh1_frame_, "LHB-D4EEE18");
    get_param_ok = nh_local_.param<string>("survive_lh2_frame", survive_lh2_frame_, "LHB-400B1A3E");

    // main tree frame
    get_param_ok = nh_local_.param<string>("robot_tracker_frame", robot_tracker_frame_, "tracker_frame");
    get_param_ok = nh_local_.param<string>("robot_tracker_parent_frame", robot_tracker_parent_frame_, "lighthouse_frame");
    get_param_ok = nh_local_.param<string>("robot_tracker_root_frame", robot_tracker_root_frame_, "map");
    get_param_ok = nh_local_.param<string>("lh_world_frame", lh_world_frame_, "lighthouse_frame");
    get_param_ok = nh_local_.param<string>("lh_frame_prefix", lh_frame_prefix_, "lh_frame");
    get_param_ok = nh_local_.param<string>("lh_origin_frame_prefix", lh_origin_frame_prefix_, "lh_origin_frame");
    get_param_ok = nh_local_.param<string>("lh_parent_frame", lh_parent_frame_, "map");

    get_param_ok = nh_local_.param<double>("lh1_x", lh1_x_, 0.2);
    get_param_ok = nh_local_.param<double>("lh1_y", lh1_y_, 0.2);
    get_param_ok = nh_local_.param<double>("lh1_z", lh1_z_, 0.17);
    get_param_ok = nh_local_.param<double>("lh1_theta", lh1_theta_, 45);

    get_param_ok = nh_local_.param<double>("lh2_x", lh2_x_, 2.8);
    get_param_ok = nh_local_.param<double>("lh2_y", lh2_y_, 0.2);
    get_param_ok = nh_local_.param<double>("lh2_z", lh2_z_, 0.17);
    get_param_ok = nh_local_.param<double>("lh2_theta", lh2_theta_, 135);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            origin_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("lighthouse_origin", 10);
            lh_1_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("lh1_pose", 10);
            lh_2_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("lh2_pose", 10);
            tracker_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tracker_pose", 10);
        }
        else
        {
            origin_pub_.shutdown();
        }
    }
    if (get_param_ok)
    {
        ROS_INFO("[lighthouse_localization] : set param ok");
    }
    else
    {
        ROS_WARN_STREAM("[lighthouse_localization] : "
                        << "set param failed");
    }
    return true;
}

EulerPose Vive::lookup_tf(std::string target, std::string source)
{
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(source, target, ros::Time(0), transform);
    }
    catch (const tf::TransformException &ex)
    {
        try
        {
            listener.lookupTransform(source, target, ros::Time(0), transform);
        }
        catch (const tf::TransformException &ex)
        {
            std::cout << "lookup_TF => "
                      << "target : " << target << " | "
                      << "source : " << source << endl;
            ROS_WARN_STREAM(ex.what());
        }
    }
    EulerPose trans;
    trans.x = transform.getOrigin().getX();
    trans.y = transform.getOrigin().getY();
    trans.z = transform.getOrigin().getZ();
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

    // std::cout << "lookup_TF => "
    //           << "target : " << target << " | "
    //           << "source : " << source << endl;
    // ROS_INFO("%f %f %f %f   =>  %f %f %f\n", transform.getRotation().getX(), transform.getRotation().getY(),
    //          transform.getRotation().getZ(), transform.getRotation().getW(), roll, pitch, yaw);
    trans.roll = roll;
    trans.pitch = pitch;
    trans.yaw = yaw;

    return trans;
}

EulerPose Vive::calculate_the_origin()
{
    tf::StampedTransform origin_1, origin_2;
    EulerPose calculate_origin;
    try
    {
        listener.lookupTransform(lh_parent_frame_, lh_origin_frame_prefix_ + std::to_string(1), ros::Time(0), origin_1);
        listener.lookupTransform(lh_parent_frame_, lh_origin_frame_prefix_ + std::to_string(2), ros::Time(0), origin_2);
    }
    catch (const tf::TransformException &ex)
    {
        try
        {
            listener.lookupTransform(lh_parent_frame_, lh_origin_frame_prefix_ + std::to_string(1), ros::Time(0), origin_1);
            listener.lookupTransform(lh_parent_frame_, lh_origin_frame_prefix_ + std::to_string(2), ros::Time(0), origin_2);
        }
        catch (const tf::TransformException &ex)
        {
            std::cout << "calculated the origin => lookup tf " << endl;
            // std::cout << "lookup_TF => "
            //           << "target : " << target << " | "
            //           << "source : " << source << endl;
            ROS_WARN_STREAM(ex.what());
        }
    }
    ROS_INFO("=======================================================================");
    // print
    double roll, pitch, yaw;
    tf::Matrix3x3(origin_1.getRotation()).getRPY(roll, pitch, yaw);
    ROS_INFO("origin1 => %f %f %f %f %f %f", origin_1.getOrigin().getX(), origin_1.getOrigin().getY(), origin_1.getOrigin().getZ(), roll, pitch, yaw);
    tf::Matrix3x3(origin_2.getRotation()).getRPY(roll, pitch, yaw);
    ROS_INFO("origin2 => %f %f %f %f %f %f", origin_2.getOrigin().getX(), origin_2.getOrigin().getY(), origin_2.getOrigin().getZ(), roll, pitch, yaw);

    calculate_origin.x = (origin_1.getOrigin().getX() + origin_2.getOrigin().getX()) / 2;
    calculate_origin.y = (origin_1.getOrigin().getY() + origin_2.getOrigin().getY()) / 2;
    calculate_origin.z = (origin_1.getOrigin().getZ() + origin_2.getOrigin().getZ()) / 2;

    /* slerp */
    tf::Quaternion o_1, o_2, o_;
    o_1 = origin_1.getRotation();
    o_2 = origin_2.getRotation();
    o_ = o_1.slerp(o_2, 0.5);
    tf::Matrix3x3(o_).getRPY(roll, pitch, yaw);

    calculate_origin.roll = roll;
    calculate_origin.pitch = pitch;
    calculate_origin.yaw = yaw;

    ROS_INFO("calculated origin %f %f %f %f %f %f", calculate_origin.x, calculate_origin.y, calculate_origin.z, roll, pitch, yaw);
    ROS_INFO("=======================================================================");

    return calculate_origin;
}

void Vive::publish_pose(EulerPose pose_, Survive survive)
{
    ros::Publisher puber;
    switch (survive)
    {
    case Survive::LH1:
        puber = lh_1_pub_;
        break;
    case Survive::LH2:
        puber = lh_2_pub_;
        break;
    case Survive::LH_origin:
        puber = origin_pub_;
        break;
    case Survive::Tracker:
        puber = tracker_pub_;
        break;
    }

    geometry_msgs::PoseStamped p_;
    p_.header.frame_id = "map";
    p_.header.stamp = ros::Time::now();
    p_.pose.position.x = pose_.x;
    p_.pose.position.y = pose_.y;
    p_.pose.position.z = pose_.z;
    tf2::Quaternion q;
    q.setRPY(pose_.roll, pose_.pitch, pose_.yaw);
    p_.pose.orientation.x = q.getX();
    p_.pose.orientation.y = q.getY();
    p_.pose.orientation.z = q.getZ();
    p_.pose.orientation.w = q.getW();
    puber.publish(p_);
}

void Vive::publish_tf(std::string target_frame, std::string source_frame, EulerPose pose_)
{
    geometry_msgs::TransformStamped trans_;
    trans_.header.frame_id = source_frame;
    trans_.child_frame_id = target_frame;
    trans_.header.stamp = ros::Time::now();
    trans_.transform.translation.x = pose_.x;
    trans_.transform.translation.y = pose_.y;
    trans_.transform.translation.z = pose_.z;
    tf2::Quaternion q;
    q.setRPY(pose_.roll, pose_.pitch, pose_.yaw);
    trans_.transform.rotation.x = q.getX();
    trans_.transform.rotation.y = q.getY();
    trans_.transform.rotation.z = q.getZ();
    trans_.transform.rotation.w = q.getW();
    lh_broadcaster.sendTransform(trans_);
}

void Vive::timerCallback(const ros::TimerEvent &e)
{
    if (!check_survive_tf)
    {
        ROS_WARN_STREAM("[lighthouse_localization] : "
                        << " wait for survive tf tree");

        bool ok_1 = false, ok_2 = false;
        std::string *error_msg1, *error_msg2;
        ok_1 = listener.canTransform(survive_lh1_frame_, survive_world_frame_, ros::Time(0), error_msg1);
        ok_2 = listener.canTransform(survive_lh2_frame_, survive_world_frame_, ros::Time(0), error_msg2);

        if (ok_1 && ok_2)
        {
            check_survive_tf = true;
        }
    }
    else
    {
        // get transform from survivie tree : lh_world and publish to main tf tree
        EulerPose survivie_tf;
        // look up (target_frame , source_frame) // publish tf (target_frame , source_frame)
        survivie_tf = lookup_tf(survive_world_frame_, survive_lh1_frame_);
        publish_tf(lh_origin_frame_prefix_ + std::to_string(1), lh_frame_prefix_ + std::to_string(1), survivie_tf);
        survivie_tf = lookup_tf(survive_world_frame_, survive_lh2_frame_);
        publish_tf(lh_origin_frame_prefix_ + std::to_string(2), lh_frame_prefix_ + std::to_string(2), survivie_tf);

        lh_world_origin = calculate_the_origin();
        publish_tf(lh_world_frame_, lh_parent_frame_, lh_world_origin);

        // publish tf : lighthouse->tracker
        if (!check_lighthouse_tf)
        {
            ROS_WARN_STREAM("[lighthouse_localization] : "
                            << "wait for tf from map to lighthouse");
            bool ok_1 = false, ok_2 = false;
            std::string *error_msg1, *error_msg2;
            ok_1 = listener.canTransform(lh_world_frame_, lh_parent_frame_, ros::Time(0), error_msg1);
            ok_2 = listener.canTransform(survive_tracker_frame_, survive_world_frame_, ros::Time(0), error_msg2);
            if (ok_1 && ok_2)
            {
                check_lighthouse_tf = true;
            }
        }
        else
        {
            tracker_pose = lookup_tf(survive_tracker_frame_, survive_world_frame_);
            /*maybe do the offset...*/
            publish_tf(robot_tracker_frame_, robot_tracker_parent_frame_, tracker_pose);
            // view
            EulerPose t_ = lookup_tf(robot_tracker_frame_, robot_tracker_root_frame_);
            publish_pose(t_, Survive::Tracker);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lighthouse_tf");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");
    Vive vive(nh, nh_local);

    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}