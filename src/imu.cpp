#include "gtsam_nav/imu.h"

// Extract accelerometer vector from Imu message
Vector3 getAcc(sensor_msgs::Imu::ConstPtr msg){
    return Vector3(
        msg->linear_acceleration.x, 
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
}

// Extract gyroscope angular velocity vector from Imu message
Vector3 getRate(sensor_msgs::Imu::ConstPtr msg){
    return Vector3(
        msg->angular_velocity.x, 
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );
}

IMUHandle::IMUHandle(const YAML::Node &config){
    gravity_norm_ = config["gravity_norm"].as<double>();
    initial_heading = DEG2RAD*config["initial_heading"].as<double>();

    accel_noise_sigma = config["imu_accel_noise_sigma"].as<double>();
    gyro_noise_sigma = config["imu_gyro_noise_sigma"].as<double>();
    accel_bias_rw_sigma = config["imu_accel_bias_rw_sigma"].as<double>();
    gyro_bias_rw_sigma = config["imu_gyro_bias_rw_sigma"].as<double>();
}


// Estimate a initial pose of the IMU based on a measurement
Rot3 IMUHandle::getInitialOrientation(sensor_msgs::Imu::ConstPtr msg){
    // Allign acceleration and gravity for roll and pitch
    Unit3 acc(getAcc(msg));
    Unit3 g(0, 0, -1);

    Rot3 R0 = Rot3::AlignPair(acc.cross(g), g, acc);
    
    // Align heading to fixed initial value
    Rot3 R_align_heading = Rot3::Ypr(initial_heading - R0.ypr()[0], 0, 0);
    R0 = R_align_heading.compose(R0);

    return R0;
}

boost::shared_ptr<PreintegratedCombinedMeasurements::Params> IMUHandle::getParams() {
  // We use the sensor specs to build the noise model for the IMU factor.
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * 1e-8;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_init =
      I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(gravity_norm_);
  
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_init;

  return p;
}