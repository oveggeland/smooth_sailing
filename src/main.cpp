#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>


#include <sstream>


#include <boost/program_options.hpp>

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include "gtsam_nav/gnss.h"
#include "gtsam_nav/imu.h"
#include "gtsam_nav/graph.h"

#include <cstring>
#include <fstream>
#include <iostream>


#include <cassert>
#include <cmath>   // for HUGE_VAL
#include <iomanip> // for std::setprecision()

using namespace gtsam;
using namespace std;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)



boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams() {
  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * 1e-8;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_init =
      I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
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

int main(int argc, char **argv)
{    
    // Initialize node
    ros::init(argc, argv, "navigation");

    // Graph handle
    GraphHandle gh = GraphHandle();

    // Sensor handles
    GNSSHandle gnss_handle = GNSSHandle(&gh);
    IMUHandle imu_handle = IMUHandle(&gh);

    // Open bag
    rosbag::Bag bag;
    bag.open("/home/oskar/navigation/src/gtsam_nav/data/cooked.bag");  // BagMode is Read by default

    // Initial state
    Vector10 initial_state;
    Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3),
                                         initial_state(4), initial_state(5));
    Point3 prior_point(initial_state.head<3>());
    Pose3 prior_pose(prior_rotation, prior_point);
    Vector3 prior_velocity(initial_state.tail<3>());
    imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias

    // Insert priors to 
    Values initial_values;
    int correction_count = 0;
    initial_values.insert(X(correction_count), prior_pose);
    initial_values.insert(V(correction_count), prior_velocity);
    initial_values.insert(B(correction_count), prior_imu_bias);


    // Assemble prior noise model and add it the graph.`
    auto pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.5, 0.5, 0.01, 0.5, 0.5, 0.5)
        .finished());  // rad,rad,rad,m, m, m
    auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
    auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

    // Add all prior factors (pose, velocity, bias) to the graph.
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    graph->addPrior(X(correction_count), prior_pose, pose_noise_model);
    graph->addPrior(V(correction_count), prior_velocity, velocity_noise_model);
    graph->addPrior(B(correction_count), prior_imu_bias, bias_noise_model);

    // Pre-integration
    auto p = imuParams();
    std::shared_ptr<PreintegrationType> preintegrated =
        std::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);

    assert(preintegrated);


    // Store previous state for imu integration and latest predicted outcome.
    NavState prev_state(prior_pose, prior_velocity);
    NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias = prior_imu_bias;

    ros::Time last_time = ros::Time(0, 0); // Current time

    // Iterate over bag file to and build factor graph
    string msg_type;
    for(rosbag::MessageInstance const m: rosbag::View(bag)){
        msg_type = m.getDataType();

        ros::Time msg_time = m.getTime();
        ros::Duration dt = msg_time - last_time;
    
        if (msg_type == "sensor_msgs/Imu"){
            sensor_msgs::Imu::ConstPtr p_imu_msg = m.instantiate<sensor_msgs::Imu>();
            imu_handle.newMsg(p_imu_msg);

            /*
            sensor_msgs::Imu::ConstPtr msg = m.instantiate<sensor_msgs::Imu>();
            Vector6 imu;
            
            imu[0] = msg->linear_acceleration.x;

            // Adding the IMU preintegration.
            preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt.toSec());
            */
        }

        else if (msg_type == "sensor_msgs/NavSatFix"){
            gnss_handle.newMsg(m.instantiate<sensor_msgs::NavSatFix>());
        }

        else{
            continue;
        }

        // Common stuff
        last_time = msg_time;
    }

    /*
    if (type == 0) {  // IMU measurement
        Vector6 imu;
      for (int i = 0; i < 5; ++i) {
        getline(file, value, ',');
        imu(i) = stof(value.c_str());
      }
      getline(file, value, '\n');
      imu(5) = stof(value.c_str());

      // Adding the IMU preintegration.
      preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
    */

    bag.close();



    return 0;
}