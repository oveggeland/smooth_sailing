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


using namespace gtsam;
using namespace std;

class GraphHandle{
    public:
        GraphHandle();

        // Get/Set functionality
        bool isInit();

        void initializePlanarPosition(Vector2 p);
        void initializeOrientation(Rot3 q0);
        void initializePlanarVelocity(Vector2 v);

    private:
        bool init;
        void updateInit();

        bool rp_init=false; // Rool pitch from IMU
        bool y_init=true; // Yaw (NOT IMPLEMENTED)
        bool xy_init=false; // XY from GPS
        bool v_xy_init=false; // Velocity from GPS (Two consequtive measurements)
        bool z_init=true; // Height (NOT IMPLEMENTED)
        bool v_z_init=true; // Velocity in z direction (init to 0)

        // Initial states
        Point3 prior_pos;
        Rot3 prior_rot;
        Vector3 prior_vel;

        imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias
};

#endif