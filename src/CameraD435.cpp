#include "Camera.hpp"

// external variables
std::function<void()> color_fn;
std::function<void()> depth_fn;
bool colorArrived;

D435Camera::D435Camera(ros::NodeHandle& nodeHandle, ThreadSafeDeque& odomBuffer, RtabmapCallback& callback)
:
Camera(5000, odomBuffer)
,camInfo_(new sensor_msgs::CameraInfo())
,scan_(new sensor_msgs::LaserScan())
,colorFrameId_("d435_color_optical_frame")
,depthFrameId_("d435_depth_optical_frame")
,alignToColor_(RS2_STREAM_COLOR)
,rateHz_(RATE_HZ)
,rtabmapCallback_(callback)
,scanOnlyRound_(true)
#if PUBLISH_COLOR_DEPTH
,imageTransport_(nodeHandle)
,colorPub_(imageTransport_.advertise("color", 1))
,depthPub_(imageTransport_.advertise("depth", 1))
,camInfoPub_(nodeHandle.advertise<sensor_msgs::CameraInfo>("camera_info", 1))
#endif
,scanPub_(nodeHandle.advertise<sensor_msgs::LaserScan>("scan", 1))
{
    cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_ANY, 60);
    cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_ANY, 60);

    for (auto sensor : getSensors()) {
        if (strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME), "RGB Camera") == 0) {
//            rs2::option_range range = sensor.get_option_range(RS2_OPTION_EXPOSURE);
//            cout<<"min:"<<range.min<<",max:"<<range.max<<",def:"<<range.def<<",step:"<<range.min<<endl;
            sensor.set_option(RS2_OPTION_EXPOSURE, 50);
        }
    }

    start();
}

D435Camera::~D435Camera() { wait(); }

void D435Camera::cameraThread() {
    cout << "D435Camera started" << endl;
    try {
        while (ros::ok()) {
            rateHz_.sleep();

            clearFrames();
            colorArrived = false;
            if (color_fn) color_fn();
            if (depth_fn) depth_fn();

            scanOnlyRound_ = !scanOnlyRound_;

            rs2::frameset frames;
            do {
                frames = waitForFrames();
            } while (!colorArrived);

            ros::Time ts = getTimeStamp(frames);

            frame* odomFrame = NULL;
            if (!scanOnlyRound_) {
                // only get odom frame if this is a full round
                odomFrame = odomBuffer_.getClosest(ts.toNSec());
                if (!odomFrame) {
                    continue;
                }
            }

            frames = alignToColor_.process(frames);

            auto depth = frames.get_depth_frame();
            sensor_msgs::ImagePtr depthImg_msg;
            uint width = buildImageFrame(depth, ts, depthImg_msg, matDepthImg_, CV_16UC1,
                    sensor_msgs::image_encodings::TYPE_16UC1, depthFrameId_);

            if (camInfo_->width != width) {
                updateCamInfo(depth);
                updateScanInfo();
            }

            scan_->header.stamp = ts;
            scan_->header.seq = sequence_;

            // process scan
            convertDepthToScan();
            scanPub_.publish(scan_);
            if (scanOnlyRound_) continue;

            auto color = frames.get_color_frame();
            sensor_msgs::ImagePtr colorImg_msg;
            buildImageFrame(color, ts, colorImg_msg, matColorImg_,
                    CV_8UC3, sensor_msgs::image_encodings::RGB8, colorFrameId_);

            camInfo_->header.stamp = ts;
            camInfo_->header.seq = sequence_;

            if (rtabmapCallback_ && odomFrame) {
                nav_msgs::Odometry odom_msg;
                T265Camera::buildOdomFrame(*odomFrame, ts, odom_msg, sequence_);

                const nav_msgs::OdometryConstPtr odomMsgPtr(new nav_msgs::Odometry(odom_msg));

                rtabmapCallback_(odomMsgPtr, colorImg_msg, depthImg_msg, camInfo_);
            }

            if (odomFrame) {
                delete odomFrame;
            }

            // publish topics
#if PUBLISH_COLOR_DEPTH
            colorPub_.publish((sensor_msgs::ImageConstPtr)colorImg_msg);
            depthPub_.publish((sensor_msgs::ImageConstPtr)depthImg_msg);
            camInfoPub_.publish(camInfo_);
#endif

            sequence_++;
        }
    } catch (const rs2::error& e) {
        cerr << "d435: RealSense error calling " <<
                e.get_failed_function() << "(" << e.get_failed_args() << "):\n" << e.what() << endl;
    } catch (const exception& e) {
        cerr << "d435: " << e.what() << endl;
    }

    cout << "D435Camera terminated" << endl;
}

uint D435Camera::buildImageFrame(const rs2::video_frame& frame, const ros::Time& t, sensor_msgs::ImagePtr& img_msg,
        cv::Mat& matImg, const int type, const string& encoding, const string& frameId) {
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
    img_msg->header.seq = sequence_;

    return width;
}

void D435Camera::updateCamInfo(rs2::video_frame& frame) {
    const rs2::video_stream_profile& video_profile = frame.get_profile().as<rs2::video_stream_profile>();
    auto intrinsic = video_profile.get_intrinsics();

    camInfo_->width = intrinsic.width;
    camInfo_->height = intrinsic.height;
    camInfo_->header.frame_id = colorFrameId_;

    camInfo_->K.at(0) = intrinsic.fx;
    camInfo_->K.at(2) = intrinsic.ppx;
    camInfo_->K.at(4) = intrinsic.fy;
    camInfo_->K.at(5) = intrinsic.ppy;
    camInfo_->K.at(8) = 1;

    camInfo_->P.at(0) = camInfo_->K.at(0);
    camInfo_->P.at(1) = 0;
    camInfo_->P.at(2) = camInfo_->K.at(2);
    camInfo_->P.at(3) = 0;
    camInfo_->P.at(4) = 0;
    camInfo_->P.at(5) = camInfo_->K.at(4);
    camInfo_->P.at(6) = camInfo_->K.at(5);
    camInfo_->P.at(7) = 0;
    camInfo_->P.at(8) = 0;
    camInfo_->P.at(9) = 0;
    camInfo_->P.at(10) = 1;
    camInfo_->P.at(11) = 0;

    camInfo_->distortion_model = "plumb_bob";

    // set R (rotation matrix) values to identity matrix
    camInfo_->R.at(0) = 1.0;
    camInfo_->R.at(1) = 0.0;
    camInfo_->R.at(2) = 0.0;
    camInfo_->R.at(3) = 0.0;
    camInfo_->R.at(4) = 1.0;
    camInfo_->R.at(5) = 0.0;
    camInfo_->R.at(6) = 0.0;
    camInfo_->R.at(7) = 0.0;
    camInfo_->R.at(8) = 1.0;

    camInfo_->D.resize(5);
    for (int i = 0; i < 5; i++) {
        camInfo_->D.at(i) = intrinsic.coeffs[i];
    }

    camInfo_->P.at(3) = 0;     // Tx
    camInfo_->P.at(7) = 0;     // Ty
}

void D435Camera::updateScanInfo() {

    camModel_.fromCameraInfo(*camInfo_);

    // Calculate angle_min and angle_max by measuring angles between the left ray, right ray, and optical center ray
    cv::Point2d raw_pixel_left(0, camModel_.cy());
    cv::Point2d rect_pixel_left = camModel_.rectifyPoint(raw_pixel_left);
    cv::Point3d left_ray = camModel_.projectPixelTo3dRay(rect_pixel_left);

    cv::Point2d raw_pixel_right(matDepthImg_.cols-1, camModel_.cy());
    cv::Point2d rect_pixel_right = camModel_.rectifyPoint(raw_pixel_right);
    cv::Point3d right_ray = camModel_.projectPixelTo3dRay(rect_pixel_right);

    cv::Point2d raw_pixel_center(camModel_.cx(), camModel_.cy());
    cv::Point2d rect_pixel_center = camModel_.rectifyPoint(raw_pixel_center);
    cv::Point3d center_ray = camModel_.projectPixelTo3dRay(rect_pixel_center);

    const double angle_max = angleBetweenRays(left_ray, center_ray);
    const double angle_min = -angleBetweenRays(center_ray, right_ray);

    scan_->header = camInfo_->header;
    scan_->header.frame_id = "d435_scan";
    scan_->angle_min = angle_min;
    scan_->angle_max = angle_max;
    scan_->angle_increment = (scan_->angle_max - scan_->angle_min) / (matDepthImg_.cols-1);
    scan_->time_increment = 0.0;
    scan_->scan_time = 1.0/(double)RATE_HZ;
    scan_->range_min = 0.0;
    scan_->range_max = 3.0;
}

double D435Camera::angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2) {
  const double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
  const double magnitude1 = magnitudeOfRay(ray1);
  const double magnitude2 = magnitudeOfRay(ray2);;
  return acos(dot_product / (magnitude1 * magnitude2));
}

double D435Camera::magnitudeOfRay(const cv::Point3d& ray) {
  return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
}

void D435Camera::convertDepthToScan() {
    cv::Size depth_size = matDepthImg_.size();
    int width = depth_size.width;
    uint16_t distance[width] = {0};
    const float center_x = camModel_.cx();
    const float constant_x = 1.0 / camModel_.fx();

    scan_->ranges.assign(width, std::numeric_limits<float>::quiet_NaN());

    for (int u = 0; u < width; u++) {
        for (int v = depth_size.height/2; v < depth_size.height/2+1; v++) {
            uint16_t depth_i = matDepthImg_.at<uint16_t>(v, u);
            if (depth_i > 0) {
                if (distance[u] > 0) {
                    if (distance[u] > depth_i) {
                        distance[u] = depth_i;
                    }
                } else {
                    distance[u] = depth_i;
                }
            }
        }
        if (distance[u] > 0) {
            double x = (u - center_x) * distance[u] * constant_x;
            double z = distance[u];
            scan_->ranges[width-u-1] = hypot(x, z) / 1000.0;
        }
    }
}
