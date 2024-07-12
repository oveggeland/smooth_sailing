#ifndef IMU_H
#define IMU_H

#include <gtsam/base/Vector.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include "sensor_msgs/Imu.h"

using namespace std;
using namespace gtsam;

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParams();


Vector3 getAcc(sensor_msgs::Imu::ConstPtr msg);
Vector3 getRate(sensor_msgs::Imu::ConstPtr msg);


class IMUHandle{
    public:
        // Constructor
        IMUHandle();

        // Initialize orientation from accelerometer measurement
        Rot3 getOrientation(sensor_msgs::Imu::ConstPtr msg);

    private:
        Vector3 gravity_;

        double ts_prev; // Track IMU timestamps for preintegration
        Vector3 acc_prev;
        Vector3 rate_prev;

        shared_ptr<PreintegrationType> preintegrated;
};

#endif