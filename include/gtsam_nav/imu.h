#ifndef IMU_H
#define IMU_H

#include <gtsam/base/Vector.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include "sensor_msgs/Imu.h"
#include "gtsam_nav/common.h"

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace gtsam;

Vector3 getAcc(sensor_msgs::Imu::ConstPtr msg);
Vector3 getRate(sensor_msgs::Imu::ConstPtr msg);


class IMUHandle{
    public:
        // Constructor
        IMUHandle(){};
        IMUHandle(const YAML::Node &config);


        void integrateMeasurement(double dt); // Use last measurement available
        void integrateMeasurement(sensor_msgs::Imu::ConstPtr msg, double dt);
        NavState predict(NavState state, imuBias::ConstantBias bias);
        void resetIntegrationAndSetBias(imuBias::ConstantBias bias);

        CombinedImuFactor getIMUFactor(Key xi, Key vi, Key bi, Key xj, Key vj, Key bj);

        Rot3 getInitialOrientation(sensor_msgs::Imu::ConstPtr msg);

        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> getPreintegrationParams();

    private:
        // Parameters
        double gravity_norm_;
        double initial_heading;

        double accel_noise_sigma;
        double gyro_noise_sigma;
        double accel_bias_rw_sigma;
        double gyro_bias_rw_sigma;

        // Pre-integration
        Vector3 prev_acc;
        Vector3 prev_rate;

        std::shared_ptr<PreintegrationType> preintegrated;
};

#endif