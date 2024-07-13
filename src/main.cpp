#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>

#include "gtsam_nav/graph.h"
#include <iomanip>

using namespace std;


int main(int argc, char **argv)
{    
    // Initialize node
    ros::init(argc, argv, "navigation");

    // Set precision on cout
    std::cout << std::fixed;
    std::cout << std::setprecision(2);

    // Graph handle
    GraphHandle* gh = new GraphHandle();

    // Open bag
    rosbag::Bag bag("/home/oskar/navigation/src/gtsam_nav/data/cooked.bag");  // BagMode is Read by default

    // Iterate over bag file and build factors
    double prev_ts = 0.0;
    double current_ts = 0.0;

    string msg_type;
    for(rosbag::MessageInstance const m: rosbag::View(bag)){
        // Validate time stamps
        current_ts = m.getTime().toSec();
        if (current_ts <= prev_ts){
            cout << "Old message, sensor_msgs::NavSatFix::ConstPtr msgskipping measurement" << endl;
            continue;
        }
        prev_ts = current_ts;

        // Sort by message type
        msg_type = m.getDataType();
        if (msg_type == "sensor_msgs/Imu"){
            gh->newImuMsg(m.instantiate<sensor_msgs::Imu>());
        }

        else if (msg_type == "sensor_msgs/NavSatFix"){
            gh->newGNSSMsg(m.instantiate<sensor_msgs::NavSatFix>());
        }

        // Limit trajectory size (Change this as appropriate)
        if (gh->getStateCount() > 120){
            break;
        }
    }

    // Close bag and delete graph handle (why not)
    bag.close();

    
    //Save trajectory
    ofstream outputFile("/home/oskar/navigation/src/gtsam_nav/data/traj.txt");
    gh->writeResults(outputFile);

    delete gh;

    return 0;
}