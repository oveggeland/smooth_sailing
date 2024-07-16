#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>

#include "yaml-cpp/yaml.h"

#include "gtsam_nav/IceNav.h"

using namespace std;

int main(int argc, char **argv)
{    
    // Initialize node
    ros::init(argc, argv, "rosbag_nav_node");

    // Navigation object
    const YAML::Node config = YAML::LoadFile("/home/oskar/navigation/src/gtsam_nav/cfg/params.yaml");
    cout << "init" << endl;
    IceNav ice_nav = IceNav(config);
    cout << "icenav is init" << endl;
    
    // Open bag
    rosbag::Bag bag(config["in_file"].as<std::string>().c_str());  // BagMode is Read by default

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