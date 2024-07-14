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

        Rot3 getInitialOrientation(sensor_msgs::Imu::ConstPtr msg);

        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> getParams();

    private:
        double gravity_norm_;
        double initial_heading;

        double accel_noise_sigma;
        double gyro_noise_sigma;
        double accel_bias_rw_sigma;
        double gyro_bias_rw_sigma;
};

#endif