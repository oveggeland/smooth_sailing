#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>

#include "yaml-cpp/yaml.h"

#include "smooth_sailing/IceNav.h"

using namespace std;

int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "rosbag_navigation_node");
    ros::NodeHandle nh;
    ROS_INFO("Initialized navigation node");

    // Load config
    std::string config_file;
    if (nh.getParam("config_file", config_file)) {
        ROS_INFO("Using config file: %s", config_file.c_str());
    } else {
        ROS_WARN("Failed to get parameter 'config_file'");
        exit(1);
    }

    const YAML::Node config = YAML::LoadFile(config_file);

    // Initialize the navigation 
    IceNav ice_nav = IceNav(config);
    
    // Open bag
    std::string bagPath = config["workspace"].as<std::string>() + "raw.bag";    
    rosbag::Bag bag(bagPath.c_str());  // BagMode is Read by default

    // Iterate over bag file and build factors
    double t0 = 0.0;
    double prev_ts = 0.0;
    double current_ts = 0.0;
    double max_time_interval = config["max_time_interval"].as<double>();

    string msg_type;
    for(rosbag::MessageInstance const m: rosbag::View(bag)){
        // Validate time stamps
        current_ts = m.getTime().toSec();
        if (t0 == 0.0){
            t0 = current_ts;
        }
        
        if (current_ts - t0 >= max_time_interval){
            ROS_INFO("Max time reached, stop new messages");
            break;
        }
        else if (current_ts <= prev_ts){
            ROS_WARN("Old message, skipping measurement");
            continue;
        }
        prev_ts = current_ts;

        // Sort by message type
        msg_type = m.getDataType();
        if (msg_type == "sensor_msgs/Imu"){
            ice_nav.newImuMsg(m.instantiate<sensor_msgs::Imu>());
        }

        else if (msg_type == "sensor_msgs/NavSatFix"){
            ice_nav.newGNSSMsg(m.instantiate<sensor_msgs::NavSatFix>());
        }
    }

    // Close up shop
    bag.close();
    ice_nav.finish();

    return 0;
}