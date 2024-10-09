#ifndef ICENAV_H
#define ICENAV_H

#include "smooth_sailing/gnss.h"
#include "smooth_sailing/imu.h"
#include "smooth_sailing/lidar.h"
#include "smooth_sailing/altitudeFactor.h"
#include "smooth_sailing/vectorNormFactor.h"
#include "smooth_sailing/angleNormFactor.h"

#include "gtsam/navigation/GPSFactor.h"
#include "gtsam/navigation/AttitudeFactor.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/slam/BetweenFactor.h"

#include "gtsam_unstable/slam/GaussMarkov1stOrderFactor.h"


#include "yaml-cpp/yaml.h"

#include <filesystem>

using namespace gtsam;
using namespace std;

class IceNav{
    public:
        // Constructor
        IceNav(const YAML::Node &config);

        // Sensor specific entry points
        void newImuMsg(p_imu_msg msg);
        void newGNSSMsg(p_gnss_msg msg);
        void newLidarMsg(sensor_msgs::PointCloud2::ConstPtr msg);

        // Write
        void finish(const std::string& outdir);

    private:
        const YAML::Node config_;

        NonlinearFactorGraph graph_;  // Factor graph class
        Values values_;     // Best guess for all states

        void initialize(double ts, Pose3 initial_pose);
        void predictAndUpdate();

        void newCorrection(double ts);
        void newAltitudeConstraint(double ts);

        void writeToFile(const std::string& out_file);
        void writeInfoYaml(const std::string& out_file);
        void writeHeightMeasurements(const std::string& out_file);

        ///// General control /////
        bool is_init_ = false;

        int correction_count_ = 0; // TODO: Remove this?
        vector<double> correction_stamps_;

        int prior_count_ = 0;
        int optimize_interval_;

        ///// Sensors /////
        IMUHandle imu_handle_;
        GNSSHandle gnss_handle_;
        LidarHandle lidar_handle_;


        ///// Altitude constraints /////
        int virtual_height_interval_;
        double virtual_height_sigma_;
        double t_last_height_factor_ = 0.0;
        bool useLeveredHeight_;

        bool altitude_estimate_gm_;
        double altitude_gm_tau_;
        double altitude_gm_sigma_; 


        ///// GNSS /////
        int gnss_seq_ = 0;
        int gnss_sample_interval_;
        double gnss_ts_prev_;
};

#endif