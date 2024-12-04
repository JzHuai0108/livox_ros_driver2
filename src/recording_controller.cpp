
#include <ros/ros.h>
#include <livox_ros_driver2/StartStop.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_control_client");
    ros::NodeHandle nh;

    if (argc < 2) {
        std::cout << "Use: " << argv[0] << " full/path/to/output.bag" << std::endl;
        return 1;
    }
    std::string outputbag = argv[1];

    ros::ServiceClient client = nh.serviceClient<livox_ros_driver2::StartStop>("start_stop_recording");
    livox_ros_driver2::StartStop srv;

    // Start recording
    srv.request.start = true;
    srv.request.filename = outputbag;
    if (client.call(srv)) {
        ROS_INFO_STREAM("Response: " << srv.response.message);
    } else {
        ROS_ERROR("Failed to call service.");
    }

    ros::Duration(5.0).sleep(); // Simulate recording time

    // Stop recording
    srv.request.start = false;
    if (client.call(srv)) {
        ROS_INFO_STREAM("Response: " << srv.response.message);
    } else {
        ROS_ERROR("Failed to call service.");
    }

    return 0;
}
