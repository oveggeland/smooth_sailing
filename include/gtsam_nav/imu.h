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

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParams();


Vector3 getAcc(sensor_msgs::Imu::ConstPtr msg);
Vector3 getRate(sensor_msgs::Imu::ConstPtr msg);


class IMUHandle{
    public:
        // Constructor
        IMUHandle();
        IMUHandle(const YAML::Node &config);

        // Initialize orientation from accelerometer measurement
        Rot3 getInitialOrientation(sensor_msgs::Imu::ConstPtr msg);

    private:
        Vector3 gravity_;
        double initial_heading;
};

#endif