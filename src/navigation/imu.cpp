#include "smooth_sailing/imu.h"

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
    // Import parameters from yaml
    gravity_norm_ = config["gravity_norm"].as<double>();

    accel_noise_sigma = config["imu_accel_noise_sigma"].as<double>();
    gyro_noise_sigma = config["imu_gyro_noise_sigma"].as<double>();
    accel_bias_rw_sigma = config["imu_accel_bias_rw_sigma"].as<double>();
    gyro_bias_rw_sigma = config["imu_gyro_bias_rw_sigma"].as<double>();
    
    max_integration_interval_ = config["imu_max_integration_interval"].as<double>();

    // Pre-integration
    auto p = getPreintegrationParams();

    preintegrated = std::make_shared<PreintegratedCombinedMeasurements>(p);
    assert(preintegrated);
}

void IMUHandle::resetIntegration(double ts, imuBias::ConstantBias bias){
    ts_tail_ = ts;
    ts_head_ = ts;
    preintegrated->resetIntegrationAndSetBias(bias);
}

/**
 * Used after initialization of the navigation system. Every new IMU measurement should be integrated in waiting for a new correction.
 */
bool IMUHandle::integrate(p_imu_msg msg){
    double ts = msg->header.stamp.toSec();
    double dt = ts - ts_head_;

    prev_acc_ = getAcc(msg);
    prev_rate_ = getRate(msg);
    ts_head_ = ts;

    if (dt > 0.02){
        ROS_WARN_STREAM("DT = " << dt << " > 0.02, skipping integration");
    }
    else{
        preintegrated->integrateMeasurement(prev_acc_, prev_rate_, dt);
    }

    if (ts - ts_tail_ > max_integration_interval_){
        ROS_INFO("Max integration time reached");
        // This means it is time to suit up
        return true;
    }
    return false;
}

/**
 * When correction occurs, we should finish integration by extrapolating the last imu measurements, and return a IMU factor
 */
CombinedImuFactor IMUHandle::finishIntegration(double ts_correction, int correction_count){
    double dt = ts_correction - ts_head_;
    assert(dt >= 0 && dt < 0.02);

    if (dt > 0.02){
        cout << "DT = " << dt << " > 0.02, skipping integration" << endl;
    }
    else if (dt != 0.0){
        preintegrated->integrateMeasurement(prev_acc_, prev_rate_, dt);
    }

    auto preint_imu = dynamic_cast<const PreintegratedCombinedMeasurements&>(*preintegrated);
    return CombinedImuFactor(
        X(correction_count-1), V(correction_count-1), 
        X(correction_count), V(correction_count), 
        B(correction_count-1), B(correction_count), 
        preint_imu);
}

NavState IMUHandle::predict(NavState prev_state, imuBias::ConstantBias prev_bias){
    return preintegrated->predict(prev_state, prev_bias);
}


void IMUHandle::init(p_imu_msg msg){
    prev_acc_ = getAcc(msg);
    prev_rate_ = getRate(msg);
    is_init_ = true;
}

Unit3 IMUHandle::getNz(){
    return Unit3(-prev_acc_);
}

// Estimate attitude from last acceleration measurement
Rot3 IMUHandle::getRotPrior(){
    Unit3 nA = getNz();
    Unit3 nG(0, 0, 1);

    return Rot3::AlignPair(nA.cross(nG), nG, nA);
}


boost::shared_ptr<gtsam::PreintegrationCombinedParams> IMUHandle::getPreintegrationParams() {
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