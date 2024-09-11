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
#include <gtsam/navigation/AttitudeFactor.h>

#include "yaml-cpp/yaml.h"

#include "smooth_sailing/common.h"
#include "smooth_sailing/eulerFactor.h"
#include "smooth_sailing/rollPitchFactor.h"
#include "smooth_sailing/altitudeFactor.h"
#include "smooth_sailing/gnssFactor.h"

using namespace gtsam;
using namespace std;


class GraphHandle{
    public:
        // Constructor
        GraphHandle(){};
        GraphHandle(const YAML::Node &config);

        // Factor handle
        template <class T>
        void addFactor(T factor){graph.add(factor);};

        // Values
        void addNewValues(const NavState state, const imuBias::ConstantBias bias, const int correction_count);
        void optimizeAndUpdateValues(bool verbose=false);
        void fromValues(NavState &state, imuBias::ConstantBias &bias, const int correction_count);

        // Write
        void writeResults(int correction_count, vector<double> & correction_stamps_, const std::string& out_file);

        // Initializers
        void initialize();

        void initializePlanarPosition(Vector2 p);
        void initializeRotation(Rot3 R0);
        void setInitialAttitude(Vector3 nZ){nZ_ = nZ;};

    private:
        YAML::Node config;

        // Factor graph class
        NonlinearFactorGraph graph;  
        Values values_; // Best guess for all states

        // Priors
        Vector3 nZ_;
        Point3 prior_pos = Point3(0, 0, 0);
        Rot3 prior_rot = Rot3();
        Vector3 prior_vel = Vector3(0, 0, 0);

        imuBias::ConstantBias prior_imu_bias = imuBias::ConstantBias();  // assume zero initial bias
        Point2 prior_gnss_bias = Point2(0, 0); 
};

#endif