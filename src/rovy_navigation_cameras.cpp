#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <librealsense2/rs.hpp>
#include <iostream>
#include <thread>

#include <rovy_navigation_cameras/rovy_navigation_cameras.h>
#include "ThreadSafeDeque.hpp"

using namespace std;

static ThreadSafeDeque odomBuffer;

// external variables
std::function<void()> color_fn;
std::function<void()> depth_fn;
bool colorArrived;

void rovy_test_function() {
    printf("Hello Rovy! You made it.\n");
}

void t265PublishTransform() {
    typedef struct transformMsg {
        const char *frame_id, *child_frame_id;
        double translation_x, translation_y, translation_z;
        double rotation_x, rotation_y, rotation_z, rotation_w;
    } transformMsg_t;

    transformMsg_t messages[] = {
            {"t265_pose_frame", "t265_link", 0, 0, 0, 0, 0, 0, 1},
            {"t265_link", "d435_link", 0, 0, 0, 0, 0, 0, 1},
            {"d435_link", "d435_depth_frame", 0, 0, 0, 0, 0, 0, 1},
            {"d435_depth_frame", "d435_depth_optical_frame", 0, 0, 0, -0.5, 0.5, -0.5, 0.5},
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

void t265BuildOdomFrame(rs2::frameset& frames, const ros::Time& t,
        nav_msgs::Odometry& odom_msg, const ulong sequence) {

    auto f = frames.first_or_default(RS2_STREAM_POSE);
    auto pose = f.as<rs2::pose_frame>().get_pose_data();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = -pose.translation.z;
    pose_msg.pose.position.y = -pose.translation.x;
    pose_msg.pose.position.z = pose.translation.y;
    pose_msg.pose.orientation.x = -pose.rotation.z;
    pose_msg.pose.orientation.y = -pose.rotation.x;
    pose_msg.pose.orientation.z = pose.rotation.y;
    pose_msg.pose.orientation.w = pose.rotation.w;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = "world";
    msg.child_frame_id = "t265_pose_frame";
    msg.transform.translation.x = pose_msg.pose.position.x;
    msg.transform.translation.y = pose_msg.pose.position.y;
    msg.transform.translation.z = pose_msg.pose.position.z;
    msg.transform.rotation.x = pose_msg.pose.orientation.x;
    msg.transform.rotation.y = pose_msg.pose.orientation.y;
    msg.transform.rotation.z = pose_msg.pose.orientation.z;
    msg.transform.rotation.w = pose_msg.pose.orientation.w;
    br.sendTransform(msg);

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
    odom_msg.child_frame_id = "t265_pose_frame";
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
    odom_msg.twist.covariance ={
            cov_pose, 0, 0, 0, 0, 0,
            0, cov_pose, 0, 0, 0, 0,
            0, 0, cov_pose, 0, 0, 0,
            0, 0, 0, cov_twist, 0, 0,
            0, 0, 0, 0, cov_twist, 0,
            0, 0, 0, 0, 0, cov_twist};
}

uint d435BuildImageFrame(const rs2::video_frame& frame, const ros::Time& t, sensor_msgs::ImagePtr& img_msg,
        cv::Mat& matImg, const int type, const string& encoding, const ulong sequence, const string& frameId) {
    uint width = frame.get_width();
    uint height = frame.get_height();
    int bpp = frame.get_bytes_per_pixel();

    if (matImg.empty()) {
        matImg = cv::Mat(height, width, type, cv::Scalar(0, 0, 0));
    }
    matImg.data = (uint8_t*)frame.get_data();

    img_msg = cv_bridge::CvImage(std_msgs::Header(), encoding, matImg).toImageMsg();
    img_msg->width = width;
    img_msg->height = height;
    img_msg->is_bigendian = false;
    img_msg->step = width * bpp;
    img_msg->header.frame_id = frameId;
    img_msg->header.stamp = t;
    img_msg->header.seq = sequence;

    return width;
}

void d435TriggerFrame(bool *d435TriggerFrameTerminate) {
    ros::Rate rate(1);

    while (!*d435TriggerFrameTerminate) {
        colorArrived = false;
        if (color_fn) color_fn();
        if (depth_fn) depth_fn();

        rate.sleep();
    }
}

void d435UpdateCamInfo(rs2::video_frame& frame, sensor_msgs::CameraInfo& camInfo, const string& frameId) {
    const rs2::video_stream_profile& video_profile = frame.get_profile().as<rs2::video_stream_profile>();
    auto intrinsic = video_profile.get_intrinsics();

    camInfo.width = intrinsic.width;
    camInfo.height = intrinsic.height;
    camInfo.header.frame_id = frameId;

    camInfo.K.at(0) = intrinsic.fx;
    camInfo.K.at(2) = intrinsic.ppx;
    camInfo.K.at(4) = intrinsic.fy;
    camInfo.K.at(5) = intrinsic.ppy;
    camInfo.K.at(8) = 1;

    camInfo.P.at(0) = camInfo.K.at(0);
    camInfo.P.at(1) = 0;
    camInfo.P.at(2) = camInfo.K.at(2);
    camInfo.P.at(3) = 0;
    camInfo.P.at(4) = 0;
    camInfo.P.at(5) = camInfo.K.at(4);
    camInfo.P.at(6) = camInfo.K.at(5);
    camInfo.P.at(7) = 0;
    camInfo.P.at(8) = 0;
    camInfo.P.at(9) = 0;
    camInfo.P.at(10) = 1;
    camInfo.P.at(11) = 0;

    camInfo.distortion_model = "plumb_bob";

    // set R (rotation matrix) values to identity matrix
    camInfo.R.at(0) = 1.0;
    camInfo.R.at(1) = 0.0;
    camInfo.R.at(2) = 0.0;
    camInfo.R.at(3) = 0.0;
    camInfo.R.at(4) = 1.0;
    camInfo.R.at(5) = 0.0;
    camInfo.R.at(6) = 0.0;
    camInfo.R.at(7) = 0.0;
    camInfo.R.at(8) = 1.0;

    camInfo.D.resize(5);
    for (int i = 0; i < 5; i++) {
        camInfo.D.at(i) = intrinsic.coeffs[i];
    }

    camInfo.P.at(3) = 0;     // Tx
    camInfo.P.at(7) = 0;     // Ty
}

void t265Thread(ros::NodeHandle& nodeHandle) {
    try {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        auto pipe = make_shared<rs2::pipeline>();

        auto sensor = cfg.resolve(*pipe).get_device().query_sensors().front();
        sensor.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, 0);
        sensor.set_option(RS2_OPTION_ENABLE_POSE_JUMPING, 0);

        pipe->start(cfg);
        cout << "t265Thread: Pipe started" << endl;

        t265PublishTransform();

        ros::NodeHandle n;
        ros::Publisher odom_pub = nodeHandle.advertise<nav_msgs::Odometry>("odom", 1);

        ulong sequence = 0;

        while (ros::ok()) {
            try {
                auto frames = pipe->wait_for_frames(1000);
                auto frame = frames.first_or_default(RS2_STREAM_POSE);

                ros::Time t(frame.get_timestamp() / 1000.0);

                if (odom_pub.getNumSubscribers() > 0) {
                    nav_msgs::Odometry odom_msg;
                    t265BuildOdomFrame(frames, t, odom_msg, sequence);

                    odom_pub.publish(odom_msg);
                    sequence++;
                }

//                cout << "T265: " << t << endl;
                odomBuffer.update(t.toNSec(), frame);

            } catch (const rs2::error & e) {
                cout << "restarting T265..." << endl;
                pipe->stop();
                pipe = make_shared<rs2::pipeline>();
                pipe->start(cfg);
                cout << "restarted" << endl;
            }
        }
    } catch (const rs2::error & e) {
        cerr << "t265Thread: RealSense error calling " <<
                e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << endl;
    } catch (const exception& e) {
        cerr << "t265Thread: " << e.what() << endl;
    }
}

void d435Thread(ros::NodeHandle& nodeHandle) {
    bool d435TriggerFrameTerminate = false;
    thread triggerFrame;

    try {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_ANY, 60);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_ANY, 60);
        rs2::pipeline pipe;
        pipe.start(cfg);
        cout << "d435Thread: Pipe started" << endl;

        image_transport::ImageTransport it(nodeHandle);
        image_transport::Publisher color_pub = it.advertise("color", 1);
        image_transport::Publisher depth_pub = it.advertise("depth", 1);

        ros::Publisher camInfo_pub = nodeHandle.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

        cv::Mat matColorImg, matDepthImg;
        sensor_msgs::CameraInfo camInfo;
        string colorFrameId("d435_color_optical_frame");
        string depthFrameId("d435_depth_optical_frame");

        rs2::align align_to_color(RS2_STREAM_COLOR);

        ulong sequence = 0;
        sleep(1);

        triggerFrame = thread(d435TriggerFrame, &d435TriggerFrameTerminate);

        while (ros::ok()) {
            rs2::frameset frames_drop;
            while (pipe.poll_for_frames(&frames_drop));

            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();

            ros::Time t(frames.get_timestamp() / 1000.0);

            sensor_msgs::ImagePtr colorImg_msg;
            uint width = d435BuildImageFrame(color, t, colorImg_msg, matColorImg,
                    CV_8UC3, sensor_msgs::image_encodings::RGB8, sequence, colorFrameId);

            sensor_msgs::ImagePtr depthImg_msg;
            d435BuildImageFrame(depth, t, depthImg_msg, matDepthImg, CV_16UC1,
                    sensor_msgs::image_encodings::TYPE_16UC1, sequence, depthFrameId);

            if (camInfo.width != width) {
                d435UpdateCamInfo(color, camInfo, colorFrameId);
            }
            camInfo.header.stamp = t;
            camInfo.header.seq = sequence;

            frame* t265Frame = odomBuffer.getClosest(colorImg_msg->header.stamp.toNSec());
            if (!t265Frame) {
                continue;
            }

//            cout << "D435: " << colorImg_msg->header.stamp << ", size: " << frames.size() << endl;
            ros::Time t2(t265Frame->get_timestamp() / 1000.0);
            cout << "matched:" << t2 << endl;

//            color_pub.publish((sensor_msgs::ImageConstPtr)colorImg_msg);
//            depth_pub.publish((sensor_msgs::ImageConstPtr)depthImg_msg);
//            camInfo_pub.publish(camInfo);

            sequence++;
        }
    } catch (const rs2::error & e) {
        cerr << "d435Thread: RealSense error calling " <<
                e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << endl;
    } catch (const exception& e) {
        cerr << "d435Thread: " << e.what() << endl;
    }

    d435TriggerFrameTerminate = true;
    triggerFrame.join();
}

int main(int argc, char **argv) try {
    ros::init(argc, argv, "rovy_navigation_cameras");
    ros::NodeHandle nodeHandle;

    thread first(t265Thread, ref(nodeHandle));
    thread second(d435Thread, ref(nodeHandle));

    first.join();
    second.join();

    return EXIT_SUCCESS;
} catch (const rs2::error & e) {
    cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << endl;
    return EXIT_FAILURE;
} catch (const exception& e) {
    cerr << e.what() << endl;
    return EXIT_FAILURE;
}
