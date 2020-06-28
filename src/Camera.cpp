#include "Camera.hpp"

Camera::Camera(int timeoutMS, ThreadSafeDeque& odomBuffer)
:
sequence_(0)
,odomBuffer_(odomBuffer)
,timeoutMS_(timeoutMS)
,pipe_(make_shared<rs2::pipeline>())
{}

void Camera::start() {
    pipe_->start(cfg_);
    cameraThread_ = thread(dispatch, this);
}

void Camera::wait() {
    if (cameraThread_.joinable()) {
        cameraThread_.join();
    }
}

void Camera::dispatch(void *arg) {
    static_cast<Camera*>(arg)->cameraThread();
}

vector<rs2::sensor> Camera::getSensors() {
    return cfg_.resolve(*pipe_).get_device().query_sensors();
}

rs2::frameset Camera::waitForFrames(int timeoutMS) {
    if (timeoutMS == -1) timeoutMS = timeoutMS_;
    return pipe_->wait_for_frames(timeoutMS);
}

void Camera::clearFrames() {
    rs2::frameset frames_drop;
    while (pipe_->poll_for_frames(&frames_drop));
}

ros::Time Camera::getTimeStamp(rs2::frame& frame) {
    return ros::Time(frame.get_timestamp() / 1000.0);
}

void Camera::restartPipe() {
    pipe_->stop();
    pipe_ = make_shared<rs2::pipeline>();
    pipe_->start(cfg_);
}
