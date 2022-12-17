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
    tf::TransformListener listener;
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
            tracker_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tracker_base", 10);
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
        ROS_WARN_STREAM("[lighthouse_localization] : " << "set param failed");
    }
    return true;
}

EulerPose Vive::lookup_tf(std::string target, std::string source)
{
    // geometry_msgs::TransformStamped transform;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(target, source, ros::Time(0), transform);
    }
    catch(const tf::TransformException& ex)
    {
        try
        {
            listener.lookupTransform(target, source, ros::Time(0), transform);
        }
        catch(const tf::TransformException& ex)
        {
            std::cout << "lookup_TF => " << "target : " << target << " | " << "source : " << source << endl;
            ROS_WARN_STREAM(ex.what());
        }
    }
    EulerPose trans;
    trans.x = transform.getOrigin().getX();
    trans.y = transform.getOrigin().getY();
    trans.z = transform.getOrigin().getZ();

    geometry_msgs::Quaternion qq;
    qq.x = transform.getRotation().getX();
    qq.y = transform.getRotation().getY();
    qq.z = transform.getRotation().getZ();
    qq.w = transform.getRotation().getW();
    tf::Quaternion q;
    tf::quaternionMsgToTF(qq, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    trans.roll = roll;
    trans.pitch = pitch;
    trans.yaw = yaw;

    return trans;
}

EulerPose Vive::calculate_the_origin(EulerPose origin1, EulerPose origin2)
{
    // calculate the origin // take average
    EulerPose calculated_origin;
    calculated_origin.x = (origin1.x + origin2.x) / 2;
    calculated_origin.y = (origin1.y + origin2.y) / 2;
    calculated_origin.z = (origin1.z + origin2.z) / 2;

    tf2::Quaternion origin1_, origin2_;
    origin1_.setRPY(origin1.roll, origin1.pitch, origin1.yaw);
    origin2_.setRPY(origin2.roll, origin2.pitch, origin2.yaw);
    origin1_ = origin1_.normalize();
    origin2_ = origin2_.normalize();
    tf2::Quaternion origin_ = origin1_.slerp(origin2_, 0.5);

    geometry_msgs::Quaternion qq;
    qq.x = origin_.getX();
    qq.y = origin_.getY();
    qq.z = origin_.getZ();
    qq.w = origin_.getW();

    tf::Quaternion q;
    tf::quaternionMsgToTF(qq, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    calculated_origin.roll = roll;
    calculated_origin.pitch = pitch;
    calculated_origin.yaw = yaw;
    // for showing in rviz
    publish_pose(calculated_origin, Survive::LH_origin);
    return calculated_origin;
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

void Vive::publish_initial_tf(std::string target_frame, std::string source_frame, Survive survive)
{
    EulerPose p_;
    switch (survive)
    {
        case Survive::LH1:
            p_.x = lh1_x_;
            p_.y = lh1_y_;
            p_.z = lh1_z_;
            p_.yaw = lh1_theta_ * M_PI / 180;
            p_.roll = 0.0;
            p_.pitch = 0.0;
            break;
        case Survive::LH2:
            p_.x = lh2_x_;
            p_.y = lh2_y_;
            p_.z = lh2_z_;
            p_.yaw = lh2_theta_ * M_PI / 180;
            p_.roll = 0.0;
            p_.pitch = 0.0;
            break;
    }
    // ROS_INFO("initial TF : pose[%f, %f, %f, %f, %f, %f]\n", p_.x, p_.y, p_.z, p_.roll, p_.pitch, p_.yaw);
    publish_tf(target_frame, source_frame, p_);
}

void Vive::timerCallback(const ros::TimerEvent &e)
{
    publish_initial_tf(lh_frame_prefix_ + std::to_string(2), lh_parent_frame_, Survive::LH2);
    publish_initial_tf(lh_frame_prefix_ + std::to_string(1), lh_parent_frame_, Survive::LH1);
    if (!check_survive_tf)
    {
        ROS_WARN_STREAM("[lighthouse_localization] : " << " wait for survive tf tree");    

        bool ok_1 = false, ok_2 = false, ok_3 = false, ok_4 = false;
        std::string* error_msg1, *error_msg2, *error_msg3, *error_msg4;
        ok_1 = listener.canTransform(survive_lh1_frame_, survive_world_frame_, ros::Time(0), error_msg1);
        ok_2 = listener.canTransform(survive_lh2_frame_, survive_world_frame_, ros::Time(0), error_msg2);
        ok_3 = listener.canTransform(lh_frame_prefix_ + std::to_string(1), lh_parent_frame_, ros::Time(0), error_msg3);
        ok_4 = listener.canTransform(lh_frame_prefix_ + std::to_string(2), lh_parent_frame_, ros::Time(0), error_msg4);

        if (ok_1 && ok_2 && ok_3 && ok_4)
        {
            check_survive_tf = true;
        }
    }
    else
    {
        // get transform from survivie tree : lh_world
        // and publish to main tf tree
        EulerPose survivie_tf;
        // look up (target_frame , source_frame)
        survivie_tf = lookup_tf(survive_world_frame_, survive_lh1_frame_);
        publish_tf(lh_origin_frame_prefix_ + std::to_string(1), lh_frame_prefix_ + std::to_string(1), survivie_tf);
        survivie_tf = lookup_tf(survive_world_frame_, survive_lh2_frame_);
        publish_tf(lh_origin_frame_prefix_ + std::to_string(2), lh_frame_prefix_ + std::to_string(2), survivie_tf);

        // // publish tf : map->lightthouse
        lh1_origin = lookup_tf(lh_origin_frame_prefix_ + std::to_string(1), lh_parent_frame_);
        lh2_origin = lookup_tf(lh_origin_frame_prefix_ + std::to_string(2), lh_parent_frame_);
        lh_world_origin = calculate_the_origin(lh1_origin, lh2_origin);
        publish_tf(lh_world_frame_, lh_parent_frame_, lh_world_origin);

        // // publish tf : lighthouse->tracker
        if (!check_lighthouse_tf)
        {
            ROS_WARN_STREAM("[lighthouse_localization] : " << "wait for tf from map to lighthouse");
            bool ok_ = false;
            ok_ = listener.canTransform(lh_world_frame_, lh_parent_frame_, ros::Time(0));
            if (ok_)
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