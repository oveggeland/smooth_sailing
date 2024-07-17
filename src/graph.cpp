#include "gtsam_nav/graph.h"


// Constructor
GraphHandle::GraphHandle(const YAML::Node& config){
    this->config = config;
}

/*
HERE COMES ALL IMPLEMENTATION RELATED TO FACTOR GRAPH INITIALIZATION
*/
void GraphHandle::initialize(){
    graph = NonlinearFactorGraph(); // Initialize factor graph

    Vector3 initial_pos_sigma = Vector3::Map(config["initial_pos_sigma"].as<std::vector<double>>().data(), 3);
    Vector3 initial_vel_sigma = Vector3::Map(config["initial_vel_sigma"].as<std::vector<double>>().data(), 3);
    
    Vector3 initial_euler_sigma = Vector::Map(config["initial_euler_sigma"].as<std::vector<double>>().data(), 3);
    double initial_acc_bias_sigma = config["initial_acc_bias_sigma"].as<double>();
    double initial_gyro_bias_sigma = config["initial_gyro_bias_sigma"].as<double>();

   // Assemble prior noise model and add it the graph.`
    auto pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << initial_euler_sigma, initial_pos_sigma).finished());
    auto velocity_noise_model = noiseModel::Diagonal::Sigmas(initial_vel_sigma);
    auto bias_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) <<   initial_acc_bias_sigma, initial_acc_bias_sigma, initial_acc_bias_sigma, 
                        initial_gyro_bias_sigma, initial_gyro_bias_sigma, initial_gyro_bias_sigma).finished()); 
    Pose3 prior_pose(prior_rot, prior_pos);

    values_.insert(X(0), prior_pose);
    values_.insert(V(0), prior_vel);
    values_.insert(B(0), prior_imu_bias);

    graph.addPrior(X(0), prior_pose, pose_noise_model);
    graph.addPrior(V(0), prior_vel, velocity_noise_model);
    graph.addPrior(B(0), prior_imu_bias, bias_noise_model);
    cout << "Adding priors at " << 0 << endl;
}

void GraphHandle::addNewValues(const NavState state, const imuBias::ConstantBias bias, const int correction_count){
    values_.insert(X(correction_count), state.pose());
    values_.insert(V(correction_count), state.velocity());
    values_.insert(B(correction_count), bias);
}

void GraphHandle::optimizeAndUpdateValues(bool verbose){
    LevenbergMarquardtParams p;
    if (verbose) p.setVerbosityLM("SUMMARY");

    LevenbergMarquardtOptimizer optimizer(graph, values_, p);
    values_ = optimizer.optimize();
}


void GraphHandle::fromValues(NavState &state, imuBias::ConstantBias &bias, const int correction_count){
    state = NavState(values_.at<Pose3>(X(correction_count)),
                            values_.at<Vector3>(V(correction_count)));
    bias = values_.at<imuBias::ConstantBias>(B(correction_count));
}


void GraphHandle::initializePlanarPosition(Point2 xy){
    prior_pos.head<2>() = xy;
}

void GraphHandle::initializeRotation(Rot3 R0){
    prior_rot = R0;
}

void GraphHandle::writeResults(int correction_count_){
    ofstream f("/home/oskar/navigation/src/gtsam_nav/data/traj.txt");

    f << "x, y, z, vx, vy, vz, r, p, y, bax, bay, baz, bgx, bgy, bgz" << endl;
    Pose3 pose;
    Vector3 x;
    Vector3 ypr;
    Vector3 v;
    Vector6 b;

    f << fixed;
    for (int i = 0; i < correction_count_; i++){ // TODO
        pose = values_.at<Pose3>(X(i));
        x = pose.translation();
        ypr = pose.rotation().ypr();
        v = values_.at<Vector3>(V(i));
        b = values_.at<imuBias::ConstantBias>(B(i)).vector();

        f << x[0] << ", " << x[1] << ", " << x[2] << ", ";
        f << v[0] << ", " << v[1] << ", " << v[2] << ", ";
        f << ypr[2] << ", " << ypr[1] << ", " << ypr[0] << ", ";
        f << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << ", " << b[4] << ", " << b[5] << endl;
    }
}