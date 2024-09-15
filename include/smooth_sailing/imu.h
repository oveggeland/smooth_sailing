#ifndef IMU_H
#define IMU_H

#include <gtsam/base/Vector.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "smooth_sailing/common.h"

#include "yaml-cpp/yaml.h"

typedef sensor_msgs::Imu::ConstPtr p_imu_msg;

using namespace std;
using namespace gtsam;

Vector3 getAcc(p_imu_msg msg);
Vector3 getRate(p_imu_msg msg);

class IMUHandle{
    public:
        // Constructor
        IMUHandle(){};
        IMUHandle(const YAML::Node &config);

        void resetIntegration(double ts, imuBias::ConstantBias bias=imuBias::ConstantBias());

        bool integrate(p_imu_msg msg); // Returns true when max_intergration_time is reached
        CombinedImuFactor finishIntegration(double ts_correction, int correction_count);

        NavState predict(NavState prev_state, imuBias::ConstantBias prev_bias);

        Rot3 getRotPrior();
        void init(p_imu_msg);
        Unit3 getNz();

        bool isInit(){return is_init_;};

    private:
        boost::shared_ptr<gtsam::PreintegrationCombinedParams> getPreintegrationParams();


        // Control
        bool is_init_ = false;
        Vector3 prev_acc_;
        Vector3 prev_rate_;

        double ts_tail_;
        double ts_head_; // Head of integration measurement 

        std::shared_ptr<PreintegrationType> preintegrated;

        // Parameters
        double gravity_norm_;

        double accel_noise_sigma;
        double gyro_noise_sigma;
        double accel_bias_rw_sigma;
        double gyro_bias_rw_sigma;

        double acc_scale_;

        bool split_integration_;
        double max_integration_interval_;
};

#endif