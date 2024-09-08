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

    // Initial values
    values_.insert(X(0), Pose3(prior_rot, prior_pos));
    values_.insert(V(0), prior_vel);
    values_.insert(B(0), prior_imu_bias);

    // Prior on bias
    auto bias_noise_model = noiseModel::Diagonal::Sigmas(Vector6::Map(config["initial_imu_bias_sigma"].as<std::vector<double>>().data(), 6)); 
    graph.addPrior(B(0), prior_imu_bias, bias_noise_model);


    // Velocity prior
    auto velocity_noise_model = noiseModel::Diagonal::Sigmas(Vector3::Map(config["initial_velocity_sigma"].as<std::vector<double>>().data(), 3)); 
    graph.addPrior(V(0), prior_vel, velocity_noise_model);

    // Prior on position
    auto gnss_factor = GPSFactor(X(0), prior_pos, noiseModel::Diagonal::Sigmas(
        Vector3(config["gnss_sigma"].as<double>(), 
                config["gnss_sigma"].as<double>(), 
                config["virtual_height_sigma"].as<double>()
        )
    ));
    graph.add(gnss_factor);

    // Prior on attitude. Documentation is very confusing here, regarding what should be the nav/body frame
    auto attitudeFactor = Pose3AttitudeFactor(X(0), Unit3(0, 0, 1), 
        noiseModel::Isotropic::Sigma(2, config["initial_attitude_sigma"].as<double>()), 
        Unit3(-nZ_)
    );
    graph.add(attitudeFactor);
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

void GraphHandle::writeResults(int correction_count_, vector<double> & correction_stamps_){
    ofstream f("/home/oskar/navigation/src/gtsam_nav/data/traj.txt");

    f << "t, x, y, z, vx, vy, vz, r, p, y, bax, bay, baz, bgx, bgy, bgz" << endl;
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

        f << correction_stamps_[i] << ", ";
        f << x[0] << ", " << x[1] << ", " << x[2] << ", ";
        f << v[0] << ", " << v[1] << ", " << v[2] << ", ";
        f << ypr[2] << ", " << ypr[1] << ", " << ypr[0] << ", ";
        f << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << ", " << b[4] << ", " << b[5] << endl;
    }
}