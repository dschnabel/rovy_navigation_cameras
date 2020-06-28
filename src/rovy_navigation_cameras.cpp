#include <rovy_navigation_cameras/rovy_navigation_cameras.h>
#include "Camera.hpp"

int rovy_start_cameras(ros::NodeHandle& nodeHandle, RtabmapCallback callback) {

    try {
        ThreadSafeDeque odomBuffer;

        T265Camera t265(nodeHandle, odomBuffer);
        D435Camera d435(nodeHandle, odomBuffer, callback);

        return 0;
    } catch (const rs2::error & e) {
        cerr << "RealSense error calling " <<
                e.get_failed_function() << "(" << e.get_failed_args() << "):\n" << e.what() << endl;
        return -1;
    } catch (const exception& e) {
        cerr << e.what() << endl;
        return -1;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rovy_navigation_cameras");
    ros::NodeHandle nodeHandle;
    RtabmapCallback rtabmapCallback;

    return rovy_start_cameras(nodeHandle, rtabmapCallback);
}
