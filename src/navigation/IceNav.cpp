#include "smooth_sailing/IceNav.h"

// Constructor
IceNav::IceNav(const YAML::Node& config){
    graph_handle = GraphHandle(config);
    imu_handle = IMUHandle(config);
    gnss_handle = GNSSHandle(config);

    virtual_height_interval_ = config["virtual_height_interval"].as<int>();
    virtual_height_sigma_ = config["virtual_height_sigma"].as<double>();

    optimize_interval_ = config["optimize_interval"].as<int>();
}

// Entry point for new IMU measurements
void IceNav::newImuMsg(p_imu_msg msg){
    if (is_init_){
        imu_handle.integrate(msg);
    }
    else{
        Vector3 nZ = getAcc(msg);
        graph_handle.setInitialAttitude(nZ);

        Rot3 R0 = imu_handle.getInitialRotation(msg);
        graph_handle.initializeRotation(R0);

        imu_init_ = true;
        checkInit(msg->header.stamp.toSec());
    }
}

// Entry point for new GNSS measurements
void IceNav::newGNSSMsg(p_gnss_msg msg){
    double ts = msg->header.stamp.toSec();
    Point2 xy = gnss_handle.getMeasurement(msg);

    if (is_init_){
        //auto correction_factor = gnss_handle.getCorrectionFactor(xy, correction_count_);
        auto correction_factor = GPSFactor(X(correction_count_), (Vector(3) << xy, 0).finished(), noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 1)));
        graph_handle.addFactor(correction_factor);
        cout << "Add GNSS factor at " << correction_count_ << endl;

        newCorrection(ts);
    }
    else{
        graph_handle.initializePlanarPosition(xy);

        gnss_init_ = true;
        checkInit(ts);
    }
}

void IceNav::newCorrection(double ts_correction){
    correction_stamps_.push_back(ts_correction);

    // Add IMU factor between states
    auto imu_factor = imu_handle.finishIntegration(ts_correction, correction_count_);
    graph_handle.addFactor(imu_factor);
    cout << "Add IMU factor between " << correction_count_ - 1 << " and " << correction_count_ << endl;

    // Add dummy altitude factor
    if (correction_count_ % virtual_height_interval_ == 0){
        auto altitudeFactor = AltitudeFactor(X(correction_count_), 0, noiseModel::Isotropic::Sigma(1, virtual_height_sigma_));
        //graph_handle.addFactor(altitudeFactor);
        cout << "Adding altitude factor at " << correction_count_ << endl;
    }

    // Propogate to get new initial values
    state_ = imu_handle.predict(state_, bias_);
    
    graph_handle.addNewValues(state_, bias_, correction_count_);

    if (correction_count_ % optimize_interval_ == 0)
        graph_handle.optimizeAndUpdateValues();
    graph_handle.fromValues(state_, bias_, correction_count_);
    imu_handle.resetIntegration(ts_correction, bias_);

    // Control variables
    correction_count_ ++;
    ts_head_ = ts_correction;
}

void IceNav::checkInit(double ts){
    if (gnss_init_ && imu_init_){
        t0_ = ts;
        is_init_ = true;
        ts_head_ = ts;

        correction_stamps_.push_back(ts);
        
        imu_handle.resetIntegration(ts);

        graph_handle.initialize();
        graph_handle.fromValues(state_, bias_, 0);

        correction_count_ = 1;
    }
}


void IceNav::finish(const std::string& outdir){
    graph_handle.optimizeAndUpdateValues(true);
    graph_handle.writeResults(correction_count_, correction_stamps_, std::filesystem::path(outdir) / "nav.csv");

    gnss_handle.writeToFile(std::filesystem::path(outdir) / "gnss.csv");

    // Write nav info yaml
    YAML::Node config;

    // Add data to the node
    double x0, y0;
    gnss_handle.getOffset(x0, y0);
    config["x0"] = x0;
    config["y0"] = y0;
    config["t0"] = t0_;

    // Convert the node to a YAML string
    YAML::Emitter emitter;
    emitter << config;

    // Write to a file
    cout << "Writing to " << std::filesystem::path(outdir) / "nav_info.yaml" << endl;
    std::ofstream fout(std::filesystem::path(outdir) / "nav_info.yaml");
    fout << emitter.c_str();
    fout.close();
}