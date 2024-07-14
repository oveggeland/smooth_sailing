#ifndef GRAPH_H
#define GRAPH_H


#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

#include "gtsam_nav/gnss.h"
#include "gtsam_nav/imu.h"
#include "gtsam_nav/lidar.h"

#include "yaml-cpp/yaml.h"

using namespace gtsam;
using namespace std;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::G;  // GNSS OFFSET (north, east)

class GraphHandle{
    public:
        // Constructor
        GraphHandle(const YAML::Node &config);

        // Sensor specific entry points
        void newImuMsg(sensor_msgs::Imu::ConstPtr msg);
        void newGNSSMsg(sensor_msgs::NavSatFix::ConstPtr msg);

        // Get/set
        int getStateCount(){return state_count;};

        // Write
        void writeResults(ofstream& f);

    private:
        // Sensor handlers
        IMUHandle imu_handle;
        GNSSHandle gnss_handle;

        // Init stuff
        void initializePlanarPosition(Vector2 p, double ts);
        void initializeOrientation(Rot3 q0, double ts);
        void initializePlanarVelocity(Vector2 v, double ts);

        void newCorrection(double ts);

        // Gnss stuff
        Vector2 prev_xy_gnss;
        double prev_ts_gnss;

        // State stuff
        NavState prev_state;
        NavState prop_state;
        imuBias::ConstantBias prev_bias;
        Point2 prev_gnss_bias;

        // Factor graph class
        NonlinearFactorGraph graph;    
        Values initial_values; // Keep track of initial values for optimization

        int state_count; // Counts the number of states

        void initializeFactorGraph();

        double ts_head; // Keep track of timestamp
        double ts_init;

        // Init stuff
        bool init=false;
        void updateInit();
    
        bool rp_init=false; // Rool pitch from IMU
        bool y_init=true; // Yaw (NOT IMPLEMENTED)
        bool xy_init=false; // XY from GPS
        bool v_xy_init=false; // Velocity from GPS (Two consequtive measurements)
        bool z_init=true; // Height (NOT IMPLEMENTED)
        bool v_z_init=true; // Velocity in z direction (init to 0)

        // Initial states
        Point3 prior_pos = Point3(0, 0, 0);
        Rot3 prior_rot = Rot3();
        Vector3 prior_vel = Vector3(0, 0, 0);

        imuBias::ConstantBias prior_imu_bias = imuBias::ConstantBias();  // assume zero initial bias
        Point2 prior_gnss_bias = Point2(0, 0); 
};

#endif