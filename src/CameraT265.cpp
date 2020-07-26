#include "Camera.hpp"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include <mutex_t265.h>

T265Camera::T265Camera(ros::NodeHandle& nodeHandle, ThreadSafeDeque& odomBuffer)
:
Camera(1000, odomBuffer)
,sharedMemIndex_(0)
{
    cfg_.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    auto sensor = getSensors().front();
    sensor.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, 0);
    sensor.set_option(RS2_OPTION_ENABLE_POSE_JUMPING, 0);

    publishTransform();

    odomPub_ = nodeHandle.advertise<nav_msgs::Odometry>("odom", 1);

    start();
}

T265Camera::~T265Camera() { wait(); }

void T265Camera::cameraThread() {
    cout << "T265Camera started" << endl;
    try {
        while (ros::ok()) {
            try {
                auto frames = waitForFrames();
                auto pose = frames.first_or_default(RS2_STREAM_POSE);
                processPoseFrame(pose);
            } catch (const rs2::error & e) {
                cout << "restarting T265..." << endl;
                restartPipe();
                cout << "restarted" << endl;
            }
        }
    } catch (const rs2::error& e) {
        cerr << "t265: RealSense error calling " <<
                e.get_failed_function() << "(" << e.get_failed_args() << "):\n" << e.what() << endl;
    } catch (const exception& e) {
        cerr << "t265: " << e.what() << endl;
    }
    cout << "T265Camera terminated" << endl;
}

void T265Camera::publishTransform() {
    typedef struct transformMsg {
        const char *frame_id, *child_frame_id;
        double translation_x, translation_y, translation_z;
        double rotation_x, rotation_y, rotation_z, rotation_w;
    } transformMsg_t;

    transformMsg_t messages[] = {
            {"t265_link", "base_link", -0.11, 0, 0, 0, 0, 0, 1},
            {"t265_link", "d435_link", 0, 0, 0.055, 0, 0, 0, 1},
            {"d435_link", "d435_depth_optical_frame", 0, 0, 0, -0.5, 0.5, -0.5, 0.5},
            {"d435_link", "d435_color_frame", -0.000368016, 0.0146025, 9.40469e-05, 0.00265565,
                    -0.00162789, 0.00487547, 0.999983},
            {"d435_color_frame", "d435_color_optical_frame", 0, 0, 0, -0.5, 0.5, -0.5, 0.5}
    };

    ros::Time t = ros::Time::now();
    vector<geometry_msgs::TransformStamped> msgs;

    for (uint i = 0; i < sizeof(messages)/sizeof(transformMsg_t); i++) {
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = t;
        msg.header.frame_id = messages[i].frame_id;
        msg.child_frame_id = messages[i].child_frame_id;
        msg.transform.translation.x = messages[i].translation_x;
        msg.transform.translation.y = messages[i].translation_y;
        msg.transform.translation.z = messages[i].translation_z;
        msg.transform.rotation.x = messages[i].rotation_x;
        msg.transform.rotation.y = messages[i].rotation_y;
        msg.transform.rotation.z = messages[i].rotation_z;
        msg.transform.rotation.w = messages[i].rotation_w;
        msgs.push_back(msg);
    }

    static tf2_ros::StaticTransformBroadcaster staticTransformBroadcaster;
    staticTransformBroadcaster.sendTransform(msgs);
}

void T265Camera::buildOdomFrame(rs2::frame& f, const ros::Time& t,
        nav_msgs::Odometry& odom_msg, ulong sequence, bool doBroadcast) {

    auto pose = f.as<rs2::pose_frame>().get_pose_data();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = -pose.translation.z;
    pose_msg.pose.position.y = -pose.translation.x;
    pose_msg.pose.position.z = pose.translation.y;
    pose_msg.pose.orientation.x = -pose.rotation.z;
    pose_msg.pose.orientation.y = -pose.rotation.x;
    pose_msg.pose.orientation.z = pose.rotation.y;
    pose_msg.pose.orientation.w = pose.rotation.w;

    if (doBroadcast) {
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = t;
        msg.header.frame_id = "world";
        msg.child_frame_id = "t265_link";
        msg.transform.translation.x = pose_msg.pose.position.x;
        msg.transform.translation.y = pose_msg.pose.position.y;
        msg.transform.translation.z = pose_msg.pose.position.z;
        msg.transform.rotation.x = pose_msg.pose.orientation.x;
        msg.transform.rotation.y = pose_msg.pose.orientation.y;
        msg.transform.rotation.z = pose_msg.pose.orientation.z;
        msg.transform.rotation.w = pose_msg.pose.orientation.w;
        br.sendTransform(msg);
    }

    double cov_pose(0.01 * pow(10, 3-(int)pose.tracker_confidence));
    double cov_twist(0.01 * pow(10, 1-(int)pose.tracker_confidence));

    geometry_msgs::Vector3Stamped v_msg;
    v_msg.vector.x = -pose.velocity.z;
    v_msg.vector.y = -pose.velocity.x;
    v_msg.vector.z = pose.velocity.y;
    tf::Vector3 tfv;
    tf::vector3MsgToTF(v_msg.vector,tfv);
    tf::Quaternion q(-pose_msg.pose.orientation.x,-pose_msg.pose.orientation.y,
            -pose_msg.pose.orientation.z,pose_msg.pose.orientation.w);
    tfv=tf::quatRotate(q,tfv);
    tf::vector3TFToMsg(tfv,v_msg.vector);

    geometry_msgs::Vector3Stamped om_msg;
    om_msg.vector.x = -pose.angular_velocity.z;
    om_msg.vector.y = -pose.angular_velocity.x;
    om_msg.vector.z = pose.angular_velocity.y;
    tf::vector3MsgToTF(om_msg.vector,tfv);
    tfv=tf::quatRotate(q,tfv);
    tf::vector3TFToMsg(tfv,om_msg.vector);

    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "t265_link";
    odom_msg.header.stamp = t;
    odom_msg.header.seq = sequence;
    odom_msg.pose.pose = pose_msg.pose;
    odom_msg.pose.covariance = {
            cov_pose, 0, 0, 0, 0, 0,
            0, cov_pose, 0, 0, 0, 0,
            0, 0, cov_pose, 0, 0, 0,
            0, 0, 0, cov_twist, 0, 0,
            0, 0, 0, 0, cov_twist, 0,
            0, 0, 0, 0, 0, cov_twist};
    odom_msg.twist.twist.linear = v_msg.vector;
    odom_msg.twist.twist.angular = om_msg.vector;
    odom_msg.twist.covariance = {
            cov_pose, 0, 0, 0, 0, 0,
            0, cov_pose, 0, 0, 0, 0,
            0, 0, cov_pose, 0, 0, 0,
            0, 0, 0, cov_twist, 0, 0,
            0, 0, 0, 0, cov_twist, 0,
            0, 0, 0, 0, 0, cov_twist};
}

void T265Camera::processPoseFrame(frame &pose) {
    ros::Time ts = getTimeStamp(pose);
    odomBuffer_.update(ts.toNSec(), pose);

    nav_msgs::Odometry odom_msg;
    buildOdomFrame(pose, ts, odom_msg, sequence_);

    odomPub_.publish(odom_msg);
    sequence_++;

    float linearVelocity = odom_msg.twist.twist.linear.x;
    float angularVelocity = odom_msg.twist.twist.angular.z;
    t265_set_data(sharedMemIndex_++, linearVelocity, angularVelocity);
}
