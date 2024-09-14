#include "smooth_sailing/IceNav.h"

// Constructor
IceNav::IceNav(const YAML::Node& config): config_(config){
    imu_handle_ = IMUHandle(config_);
    gnss_handle_ = GNSSHandle(config_);
    lidar_handle_ = LidarHandle(config_);

    virtual_height_interval_ = config_["virtual_height_interval"].as<int>();
    virtual_height_sigma_ = config_["virtual_height_sigma"].as<double>();

    optimize_interval_ = config_["optimize_interval"].as<int>();
}

// Entry point for new IMU measurements
void IceNav::newImuMsg(p_imu_msg msg){
    if (is_init_){
        imu_handle_.integrate(msg);
    }
    else{
        cout << "Init IMU" << endl;
        imu_handle_.init(msg);
    }
}

// Entry point for new GNSS measurements
void IceNav::newGNSSMsg(p_gnss_msg msg){
    double ts = msg->header.stamp.toSec();
    Point2 xy = gnss_handle_.getMeasurement(msg);

    if (is_init_){
        auto correction_factor = GPSFactor(X(correction_count_), (Vector(3) << xy, 0).finished(), noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 1)));
        graph_.add(correction_factor);
        cout << "Add GNSS factor at " << correction_count_ << endl;

        newCorrection(ts);
    }
    else if (imu_handle_.isInit()){
        // estimate initial pose (height is zero, rotation from nZ_, xy is measured here.)
        Vector3 t = (Vector(3) << xy, 0).finished();
        Rot3 R = imu_handle_.getRotPrior();

        initialize(ts, Pose3(R, t));

        // After initialization, we can add GNSS factor
        auto correction_factor = GPSFactor(X(correction_count_), (Vector(3) << xy, 0).finished(), noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 1)));
        graph_.add(correction_factor);
        correction_count_ = 1;
    }
    else{
        cout << "Waiting for IMU to initialize rotation" << endl;
    }
}

void IceNav::newLidarMsg(sensor_msgs::PointCloud2::ConstPtr msg){
    return; // Disable this for now
    if (!is_init_)
        return;

    if (!lidar_handle_.isInit()){
        lidar_handle_.init(msg, correction_count_);
        newCorrection(msg->header.stamp.toSec());
    }
    else if (lidar_handle_.newFrame(msg)){
        auto lidarFactor = lidar_handle_.getOdometryFactor(correction_count_);
        // graph_.add(lidarFactor);
    }
}


void IceNav::predictAndUpdate(){
    // Predict next state based on imu pre-integration
    Pose3 pose0 = values_.at<Pose3>(X(correction_count_-1));
    Vector3 v0 = values_.at<Vector3>(V(correction_count_-1));
    imuBias::ConstantBias bias0 = values_.at<imuBias::ConstantBias>(B(correction_count_-1));
    NavState state_pred = imu_handle_.predict(NavState(pose0, v0), bias0);
    
    // Insert predictions
    values_.insert(X(correction_count_), state_pred.pose());
    values_.insert(V(correction_count_), state_pred.velocity());
    values_.insert(B(correction_count_), bias0);

    // Perform optimization and update values
    if (correction_count_ % optimize_interval_ == 0){
        LevenbergMarquardtOptimizer optimizer(graph_, values_);
        values_ = optimizer.optimize();
    }
}

void IceNav::newCorrection(double ts){
    correction_stamps_.push_back(ts);

    // Add IMU factor between states
    auto imu_factor = imu_handle_.finishIntegration(ts, correction_count_);
    graph_.add(imu_factor);
    cout << "Add IMU factor between " << correction_count_ - 1 << " and " << correction_count_ << endl;

    // TODO: Add LiDAR factor?


    // Propogate to get new initial values
    predictAndUpdate();

    // Reset IMU integration
    imuBias::ConstantBias bias_est_ = values_.at<imuBias::ConstantBias>(B(correction_count_));
    imu_handle_.resetIntegration(ts, bias_est_);

    // New correction is finished
    correction_count_ ++;
}

void IceNav::initialize(double ts, Pose3 initial_pose){
    correction_stamps_.push_back(ts);
    graph_ = NonlinearFactorGraph();

    // Initial values
    values_.insert(X(0), initial_pose);
    values_.insert(V(0), Point3());
    values_.insert(B(0), imuBias::ConstantBias());

    // Prior on bias
    auto bias_noise_model = noiseModel::Diagonal::Sigmas(Vector6::Map(config_["initial_imu_bias_sigma"].as<std::vector<double>>().data(), 6)); 
    graph_.addPrior(B(0), imuBias::ConstantBias(), bias_noise_model);

    // Prior on velocity 
    auto velocity_noise_model = noiseModel::Diagonal::Sigmas(Vector3::Map(config_["initial_velocity_sigma"].as<std::vector<double>>().data(), 3)); 
    graph_.addPrior(V(0), Vector3(), velocity_noise_model);

    // Prior on attitude. Documentation is very confusing here, regarding what should be the nav/body frame
    auto attitudeFactor = Pose3AttitudeFactor(X(0), Unit3(0, 0, 1), 
        noiseModel::Isotropic::Sigma(2, config_["initial_attitude_sigma"].as<double>()), 
        imu_handle_.getNz()
    );
    graph_.add(attitudeFactor);

    // Reset IMU 
    imu_handle_.resetIntegration(ts, imuBias::ConstantBias());

    is_init_ = true;
}



void IceNav::writeToFile(const std::string& out_file){
    ofstream f(out_file);

    f << "ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz" << endl << fixed; 

    for (int i = 0; i < correction_count_; i++){ // TODO
        Pose3 pose = values_.at<Pose3>(X(i));
        Vector3 x = pose.translation();
        Vector3 ypr = pose.rotation().ypr();
        Vector3 v = values_.at<Vector3>(V(i));
        Vector6 b = values_.at<imuBias::ConstantBias>(B(i)).vector();

        f << correction_stamps_[i] << ",";
        f << x[0] << "," << x[1] << "," << x[2] << ",";
        f << v[0] << "," << v[1] << "," << v[2] << ",";
        f << ypr[2] << "," << ypr[1] << "," << ypr[0] << ",";
        f << b[0] << "," << b[1] << "," << b[2] << "," << b[3] << "," << b[4] << "," << b[5] << endl;
    }
}


void IceNav::writeInfoYaml(const std::string& out_file){
    YAML::Node nav_info;

    // Add data to the node
    double x0, y0;
    gnss_handle_.getOffset(x0, y0);
    nav_info["x0"] = x0;
    nav_info["y0"] = y0;
    nav_info["t0"] = correction_stamps_[0];

    // Convert the node to a YAML string
    YAML::Emitter emitter;
    emitter << nav_info;

    // Write to a file
    std::ofstream fout(out_file);
    fout << emitter.c_str();
    fout.close();
}


void IceNav::finish(const std::string& outdir){
    // Perform last update 
    LevenbergMarquardtOptimizer optimizer(graph_, values_);
    values_ = optimizer.optimize();

    // Results to file
    writeToFile(std::filesystem::path(outdir) / "nav.csv");
    gnss_handle_.writeToFile(std::filesystem::path(outdir) / "gnss.csv");

    // Write nav info yaml
    writeInfoYaml(std::filesystem::path(outdir) / "nav_info.yaml");
}