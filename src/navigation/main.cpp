#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>

#include "yaml-cpp/yaml.h"
#include <filesystem>
#include "smooth_sailing/IceNav.h"

using namespace std;

int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;

    string workspace;
    if (!nh.getParam("/ws", workspace)){
        cout << "Error: No workspace provided" << endl;
        exit(1);
    }

    // Load config
    std::string config_file;
    if (nh.getParam("nav_config", config_file)) {
        ROS_INFO("Using config file: %s", config_file.c_str());
    } else {
        ROS_WARN("Failed to get parameter 'config_file'");
        exit(1);
    }

    const YAML::Node config = YAML::LoadFile(config_file);

    // Initialize the navigation 
    IceNav ice_nav = IceNav(config);
    
    // Open bag 
    rosbag::Bag bag(workspace + "cooked.bag");  // BagMode is Read by default

    // Iterate over bag file and build factors
    double t0 = 0.0;
    double prev_ts = 0.0;
    double current_ts = 0.0;

    double max_time_interval;
    nh.getParam("max_time_interval", max_time_interval);

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
        string msg_type = m.getDataType();
        if (msg_type == "sensor_msgs/Imu"){
            ice_nav.newImuMsg(m.instantiate<sensor_msgs::Imu>());
        }

        else if (msg_type == "sensor_msgs/NavSatFix"){
            ice_nav.newGNSSMsg(m.instantiate<sensor_msgs::NavSatFix>());
        }
    }

    // Close up shop
    bag.close();
    // Create a navigation folder to save data in

    std::string nav_path = std::filesystem::path(workspace) / "navigation";
    std::filesystem::create_directory(nav_path);
    ice_nav.finish(nav_path);

    return 0;
}