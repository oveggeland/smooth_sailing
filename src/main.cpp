#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>

#include "yaml-cpp/yaml.h"

#include "gtsam_nav/graph.h"
#include <iomanip>

using namespace std;


int main(int argc, char **argv)
{    
    // Initialize node
    ros::init(argc, argv, "navigation");

    // Try to read yaml files
    const YAML::Node config = YAML::LoadFile("/home/oskar/navigation/src/gtsam_nav/cfg/params.yaml");

    // Graph handle
    GraphHandle graph_handle = GraphHandle(config);

    // Open bag
    rosbag::Bag bag(config["in_file"].as<std::string>().c_str());  // BagMode is Read by default

    // Iterate over bag file and build factors
    double prev_ts = 0.0;
    double current_ts = 0.0;

    string msg_type;
    for(rosbag::MessageInstance const m: rosbag::View(bag)){
        // Validate time stamps
        current_ts = m.getTime().toSec();
        if (current_ts <= prev_ts){
            ROS_WARN("Old message, skipping measurement");
            continue;
        }
        prev_ts = current_ts;

        // Sort by message type
        msg_type = m.getDataType();
        if (msg_type == "sensor_msgs/Imu"){
            graph_handle.newImuMsg(m.instantiate<sensor_msgs::Imu>());
        }

        else if (msg_type == "sensor_msgs/NavSatFix"){
            graph_handle.newGNSSMsg(m.instantiate<sensor_msgs::NavSatFix>());
        }

        // Limit trajectory size (Change this as appropriate)
        if (graph_handle.getStateCount() > 120){
            ROS_INFO("Maximum state count reached, stop reading rosbag");
            break;
        }
    }

    // Close bag and delete graph handle (why not)
    bag.close();
    
    //Save trajectory
    ROS_INFO_STREAM("Finished bag file, writing navigation results to file: " << config["out_file"].as<std::string>());
    ofstream outputFile(config["out_file"].as<std::string>().c_str());
    graph_handle.writeResults(outputFile);

    return 0;
}