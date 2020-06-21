/*
 * rovy_navigation_cameras.h
 *
 *  Created on: Jun. 12, 2020
 *      Author: daniel
 */

#ifndef INCLUDE_ROVY_NAVIGATION_CAMERAS_H_
#define INCLUDE_ROVY_NAVIGATION_CAMERAS_H_

#include <functional>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>

int rovy_start_cameras(ros::NodeHandle& nodeHandle, std::function<void(
        const nav_msgs::OdometryConstPtr& odomMsg,
        const sensor_msgs::ImageConstPtr& imageMsg,
        const sensor_msgs::ImageConstPtr& depthMsg,
        const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)> callback);

#endif /* INCLUDE_ROVY_NAVIGATION_CAMERAS_H_ */
