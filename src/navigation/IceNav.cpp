#include "smooth_sailing/IceNav.h"

// Constructor
IceNav::IceNav(const YAML::Node& config){
    graph_handle = GraphHandle(config);
    imu_handle = IMUHandle(config);
    gnss_handle = GNSSHandle(config);

    virtual_height_interval_ = config["virtual_height_interval"].as<int>();
    virtual_height_sigma_ = config["virtual_height_sigma"].as<double>();
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

    graph_handle.optimizeAndUpdateValues();
    graph_handle.fromValues(state_, bias_, correction_count_);
    imu_handle.resetIntegration(ts_correction, bias_);

    // Control variables
    correction_count_ ++;
    ts_head_ = ts_correction;
}

void IceNav::checkInit(double ts){
    if (gnss_init_ && imu_init_){
        is_init_ = true;
        ts_head_ = ts;

        correction_stamps_.push_back(ts);
        
        imu_handle.resetIntegration(ts);

        graph_handle.initialize();
        graph_handle.fromValues(state_, bias_, 0);

        correction_count_ = 1;
    }
}


void IceNav::finish(){
    graph_handle.optimizeAndUpdateValues(true);
    graph_handle.writeResults(correction_count_, correction_stamps_);
}