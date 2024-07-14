#include <iostream>

#include "gtsam_nav/gnss.h"
using namespace std;


GNSSHandle::GNSSHandle(const YAML::Node &config){
    // Noise
    cout << "Initialize Noise" << endl;
    meas_sigma = config["gnss_meas_sigma"].as<double>();
    bias_rw_sigma = config["gnss_bias_rw_sigma"].as<double>();
    bias_decay = config["gnss_bias_decay"].as<double>();

    bias_noise = noiseModel::Isotropic::Sigma(2, bias_rw_sigma);
    correction_noise = noiseModel::Isotropic::Sigma(2, meas_sigma);

    // Projection parameters
    cout << "initialize projection stuff" << endl;
    crs_source = config["gnss_crs_source"].as<string>();
    crs_target = config["gnss_crs_target"].as<string>();

    C = proj_context_create();
    P = proj_create_crs_to_crs(C, crs_source.c_str(), crs_target.c_str(), NULL);
}


Vector2 GNSSHandle::projectCartesian(sensor_msgs::NavSatFix::ConstPtr msg){
    input_coords = proj_coord(msg->latitude, msg->longitude, 0, 0);
    output_coords = proj_trans(P, PJ_FWD, input_coords);
    
    return Vector2(output_coords.xy.y, output_coords.xy.x);
}


BiasedGNSSFactor GNSSHandle::getCorrectionFactor(Key i, Key j, Vector2 meas){
    return BiasedGNSSFactor(
        i, j, meas, correction_noise
    );
}

BetweenFactor<Vector2> GNSSHandle::getBiasFactor(Vector2 prev_bias, Key i, Key j){
    return BetweenFactor<Vector2>(i, j, -prev_bias*(1-bias_decay), bias_noise);
}
