#ifndef ICENAV_H
#define ICENAV_H

#include "smooth_sailing/gnss.h"
#include "smooth_sailing/imu.h"
#include "smooth_sailing/altitudeFactor.h"
#include "smooth_sailing/graph.h"

#include "yaml-cpp/yaml.h"

using namespace gtsam;
using namespace std;

class IceNav{
    public:
        // Constructor
        IceNav(const YAML::Node &config);

        // Sensor specific entry points
        void newImuMsg(p_imu_msg msg);
        void newGNSSMsg(p_gnss_msg msg);

        // Write
        void finish();

    private:
        void checkInit(double ts);
        void newCorrection(double ts);
        

        // Control parameters
        bool gnss_init_ = false;
        bool imu_init_ = false;
        bool is_init_ = false;

        int correction_count_;
        vector<double> correction_stamps_;

        // Handle for factor graph
        GraphHandle graph_handle;
        Values values_; // Keep track of best guesses

        // Current estimates
        double ts_head_;
        NavState state_;
        imuBias::ConstantBias bias_;

        // Handle for different sensors
        IMUHandle imu_handle;
        GNSSHandle gnss_handle;

        // Virtual height
        int virtual_height_interval_;
        double virtual_height_sigma_;

    
};

#endif