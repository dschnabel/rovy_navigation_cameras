/*
 * Camera.hpp
 *
 *  Created on: Jun. 24, 2020
 *      Author: daniel
 */

#ifndef SRC_CAMERA_HPP_
#define SRC_CAMERA_HPP_

#include <thread>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <librealsense2/rs.hpp>

#include "ThreadSafeDeque.hpp"

#define D435_PUBLISH_COLOR_DEPTH 0
#define D435_RATE_HZ 2

#define FLOOR_CALIBRATION_MODE 0
#define FLOOR_MATRIX_WIDTH   640
#define FLOOR_MATRIX_HEIGHT   145
#define FLOOR_UNDEFINED_DISTANCE 0

using namespace std;

typedef std::function<void(
        const nav_msgs::OdometryConstPtr& odomMsg,
        const sensor_msgs::ImageConstPtr& imageMsg,
        const sensor_msgs::ImageConstPtr& depthMsg,
        const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)> RtabmapCallback;

class Camera {
protected:
    Camera(int timeout, ThreadSafeDeque& odomBuffer);
    virtual ~Camera() {};

    void start();
    void wait();

    vector<rs2::sensor> getSensors();
    rs2::frameset waitForFrames(int timeoutMS = -1);
    void clearFrames();
    ros::Time getTimeStamp(rs2::frame& frame);
    void restartPipe();

    rs2::config cfg_;
    ulong sequence_;
    ThreadSafeDeque& odomBuffer_;
private:
    static void dispatch(void *arg);
    virtual void cameraThread() = 0;

    const int timeoutMS_;
    shared_ptr<rs2::pipeline> pipe_;

    thread cameraThread_;
};

class T265Camera: public Camera {
public:
    T265Camera(ros::NodeHandle& nodeHandle, ThreadSafeDeque& odomBuffer);
    ~T265Camera();
    static void buildOdomFrame(rs2::frame& f, const ros::Time& t, nav_msgs::Odometry& odom_msg, ulong sequence);
private:
    void cameraThread();
    void publishTransform();

    ros::Publisher odomPub_;
};

class D435Camera: public Camera {
public:
    D435Camera(ros::NodeHandle& nodeHandle, ThreadSafeDeque& odomBuffer, RtabmapCallback& callback);
    ~D435Camera();
private:
    void cameraThread();
    uint buildImageFrame(const rs2::video_frame& frame, const ros::Time& t, sensor_msgs::ImagePtr& img_msg,
            cv::Mat& matImg, const int type, const string& encoding, const string& frameId);
    void updateCamInfo(rs2::video_frame& frame);
    void updateScanInfo();
    double magnitudeOfRay(const cv::Point3d& ray);
    double angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2);
    void processScan(ros::Time& ts);
    void calibrateFloor(const int x, const int y, const int distance);
    bool isFloor(const int x, const int y, const int distance);

    cv::Mat matColorImg_, matDepthImg_;
    sensor_msgs::CameraInfoPtr camInfo_;
    sensor_msgs::LaserScanPtr scan_;
    image_geometry::PinholeCameraModel camModel_;
    string colorFrameId_, depthFrameId_;
    rs2::align alignToColor_;
    ros::Rate rateHz_;
    RtabmapCallback& rtabmapCallback_;

    bool scanOnlyRound_;
    thread scanThread_;
    ulong scanSequence_;

#if D435_PUBLISH_COLOR_DEPTH
    image_transport::ImageTransport imageTransport_;
    image_transport::Publisher colorPub_;
    image_transport::Publisher depthPub_;
    ros::Publisher camInfoPub_;
#endif
    ros::Publisher scanPub_;

    pair<int,int> floorMatrix_[FLOOR_MATRIX_WIDTH][FLOOR_MATRIX_HEIGHT];
};

#endif /* SRC_CAMERA_HPP_ */
