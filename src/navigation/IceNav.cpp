#include "smooth_sailing/IceNav.h"

// Constructor
IceNav::IceNav(const YAML::Node& config): config_(config){
    imu_handle_ = IMUHandle(config_);
    gnss_handle_ = GNSSHandle(config_);

    virtual_height_interval_ = config_["virtual_height_interval"].as<double>();
    virtual_height_sigma_ = config_["virtual_height_sigma"].as<double>();

    optimize_interval_ = config_["optimize_interval"].as<int>();

    useLeveredHeight_ = config_["virtual_height_levered"].as<bool>();

    altitude_estimate_gm_ = config_["altitude_estimate_gm"].as<bool>();
    altitude_gm_tau_ = config_["altitude_gm_tau"].as<double>();
    altitude_gm_sigma_ = config_["altitude_gm_sigma"].as<double>();

    gnss_sample_interval_ = config_["gnss_sample_interval"].as<int>();

    gnss_estimate_bias_ = config_["gnss_estimate_bias"].as<bool>();
    gnss_bias_gm_ = config_["gnss_bias_gm"].as<bool>();
    gnss_bias_sigma_ = config_["gnss_bias_sigma"].as<double>();
    gnss_bias_tau_ = config_["gnss_bias_tau"].as<double>();
}

// Entry point for new IMU measurements
void IceNav::newImuMsg(p_imu_msg msg){
    if (is_init_){
        if (imu_handle_.integrate(msg))
            newCorrection(msg->header.stamp.toSec());
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

    if (is_init_ && (gnss_seq_ % gnss_sample_interval_ == 0)){
        cout << "Add GNSS factor at " << correction_count_ << endl;
        auto gnss_factor = gnss_handle_.getCorrectionFactor(xy, correction_count_, gnss_estimate_bias_);
        graph_.add(gnss_factor);

        // Here, we can add a gauss-markov or 
        if (gnss_estimate_bias_){
            double dt = ts - gnss_ts_prev_;
            cout << "Add bias node between " << gnssLastBiasKey_ << "and " << G(correction_count_);
            if (gnss_bias_gm_){
                auto gnss_bias_factor = GaussMarkov1stOrderFactor<Point2>(gnssLastBiasKey_, G(correction_count_), dt, Vector2(gnss_bias_tau_, gnss_bias_tau_), noiseModel::Isotropic::Sigma(2, gnss_bias_sigma_));
                graph_.add(gnss_bias_factor);
            }
            else{
                // Defaults to wiener process
                auto gnss_bias_factor = BetweenFactor(gnssLastBiasKey_, G(correction_count_), Point2(), noiseModel::Isotropic::Sigma(2, gnss_bias_sigma_*(dt)));
                graph_.add(gnss_bias_factor);
            }

            gnssLastBiasKey_ = G(correction_count_);
        }

        newCorrection(ts);
    }
    else if (!is_init_ && imu_handle_.isInit()){
        // estimate initial pose (height is zero, rotation from nZ_, xy is measured here.)
        Vector3 t = (Vector(3) << xy, 0).finished();
        Rot3 R = imu_handle_.getRotPrior();

        initialize(ts, Pose3(R, t));

        // After initialization, we can add GNSS factor at node 0
        auto gnss_factor = gnss_handle_.getCorrectionFactor(xy, 0, gnss_estimate_bias_); // This could potentially return a bias gnss factor
        graph_.add(gnss_factor);

        // Initialize control counters
        correction_count_ = 1;
        gnss_seq_ = 0;
    }

    gnss_ts_prev_ = ts;
    gnss_seq_++;
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
    
    if (gnss_estimate_bias_){
        values_.insert(G(correction_count_), values_.at<Point2>(G(correction_count_-1)));
    }

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

    // Consider height constraints
    if (altitude_estimate_gm_){
        // Gauss markov constraint
        double dt = ts - correction_stamps_[correction_count_-1];
        auto altitudeConstraint = ConstrainedAltitudeFactor(X(correction_count_-1), X(correction_count_), dt,
            altitude_gm_tau_, altitude_gm_sigma_);
        graph_.add(altitudeConstraint);
    }
    else if (useLeveredHeight_){
        // Levered height
        cout << "Add levered height factor" << endl;
        auto altitudeFactor = LeveredAltitudeFactor(X(correction_count_), L(0), noiseModel::Isotropic::Sigma(1, virtual_height_sigma_));
        graph_.add(altitudeFactor);
        t_last_height_factor_ = ts;
    }
    else if (ts - t_last_height_factor_ > virtual_height_interval_){
        // Vanilla measurements
        cout << "Add vanilla height factor" << endl;
        auto altitudeFactor = AltitudeFactor(X(correction_count_), 0, noiseModel::Isotropic::Sigma(1, virtual_height_sigma_));
        graph_.add(altitudeFactor);
        t_last_height_factor_ = ts;
    }

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

    if (gnss_estimate_bias_){
        values_.insert(G(0), Point2());

        graph_.addPrior(G(0), Point2(), noiseModel::Isotropic::Sigma(2, 1e-9));
        gnssLastBiasKey_ = G(0);
        prior_count_++;
    }


    if (useLeveredHeight_){
        values_.insert(L(0), Point3()); // No cheating here...

        // Prior on lever arm (essentially fixing this)
        // auto lever_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);
        // graph_.addPrior(L(0), Point3(0, 0, 20), lever_noise_model);
        // prior_count_ ++;

        // Prior on lever arm norm
        double max_norm = 50;
        auto lever_norm_factor = Point3NormConstraintFactor(L(0), max_norm, noiseModel::Isotropic::Sigma(1, 0.1));
        graph_.add(lever_norm_factor);
        prior_count_ ++;

        // Prior on lever arm angle w.r.t gravity (Must be less than 90 degrees)
        double max_angle = 90;
        auto lever_angle_norm_factor = AngularConstraintFactor(L(0), imu_handle_.getNz(), max_angle * DEG2RAD, noiseModel::Isotropic::Sigma(1, 0.01));
        graph_.add(lever_angle_norm_factor);
        prior_count_ ++;
    }

    // Prior on bias
    auto bias_noise_model = noiseModel::Diagonal::Sigmas(Vector6::Map(config_["initial_imu_bias_sigma"].as<std::vector<double>>().data(), 6)); 
    graph_.addPrior(B(0), imuBias::ConstantBias(), bias_noise_model);
    prior_count_ ++;

    // Prior on velocity 
    auto velocity_noise_model = noiseModel::Diagonal::Sigmas(Vector3::Map(config_["initial_velocity_sigma"].as<std::vector<double>>().data(), 3)); 
    graph_.addPrior(V(0), Vector3(), velocity_noise_model);
    prior_count_ ++;

    // Prior on attitude. Documentation is very confusing here, regarding what should be the nav/body frame
    auto attitudeFactor = Pose3AttitudeFactor(X(0), Unit3(0, 0, 1), 
        noiseModel::Isotropic::Sigma(2, config_["initial_attitude_sigma"].as<double>()), 
        imu_handle_.getNz()
    );
    graph_.add(attitudeFactor);
    prior_count_ ++;

    // Reset IMU 
    imu_handle_.resetIntegration(ts, imuBias::ConstantBias());

    is_init_ = true;
}



void IceNav::writeToFile(const std::string& out_file){
    ofstream f(out_file);

    f << "ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz,gnss_bias_north,gnss_bias_east";
    f << endl << fixed; 

    for (int i = 0; i < correction_count_; i++){
        Pose3 pose = values_.at<Pose3>(X(i));
        Vector3 x = pose.translation();
        Vector3 ypr = pose.rotation().ypr();
        Vector3 v = values_.at<Vector3>(V(i));
        Vector6 b = values_.at<imuBias::ConstantBias>(B(i)).vector();

        f << correction_stamps_[i] << ",";
        f << x[0] << "," << x[1] << "," << x[2] << ",";
        f << v[0] << "," << v[1] << "," << v[2] << ",";
        f << ypr[2] << "," << ypr[1] << "," << ypr[0] << ",";
        f << b[0] << "," << b[1] << "," << b[2] << "," << b[3] << "," << b[4] << "," << b[5];

        if (gnss_estimate_bias_){
            Point2 gnss_bias = values_.at<Point2>(G(i));
            f << "," << gnss_bias[0] << "," << gnss_bias[1];
        }
        else
            f << ",0,0";
        
        f << endl;

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

    if (useLeveredHeight_){
        Point3 lever_arm = values_.at<Point3>(L(0));
        emitter << YAML::Key << "lever_arm";
        emitter << YAML::Value << YAML::Flow << YAML::BeginSeq << lever_arm(0) << lever_arm(1) << lever_arm(2) << YAML::EndSeq;
    }
    // Write to a file
    std::ofstream fout(out_file);
    fout << emitter.c_str();
    fout.close();
}


void IceNav::writeHeightMeasurements(const std::string& out_file){
    ofstream f(out_file);

    f << "ts,altitude" << endl << fixed; 

    double height = 0;
    for (int i = 0; i < correction_count_; i++){ // TODO
        if (useLeveredHeight_)
            height = values_.at<Pose3>(X(i)).rotation().rotate(values_.at<Point3>(L(0)))[2];

        f << correction_stamps_[i] << "," << height << endl;
    }

    f.close();
}


void IceNav::finish(const std::string& outdir){
    // Perform last update 
    LevenbergMarquardtParams p;
    p.setVerbosityLM("SUMMARY");

    // Remove priors
    if (config_["optimize_remove_priors"].as<bool>()){
        for (int i = 0; i < prior_count_; i++){
            graph_.remove(i);
        }
    }

    LevenbergMarquardtOptimizer optimizer(graph_, values_, p);
    values_ = optimizer.optimize();

    // Write stuff to files
    std::filesystem::path outpath(outdir);

    // Results to file
    writeToFile(outpath / "nav.csv");
    writeHeightMeasurements(outpath /"height.csv");
    writeInfoYaml(outpath / "info.yaml");
    gnss_handle_.writeToFile(outpath / "gnss.csv");
}