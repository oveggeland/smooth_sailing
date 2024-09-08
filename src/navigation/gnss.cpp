#include <iostream>

#include "smooth_sailing/gnss.h"
using namespace std;


GNSSHandle::GNSSHandle(const YAML::Node &config){
    // Noise
    meas_sigma = config["gnss_sigma"].as<double>();
    correction_noise = noiseModel::Isotropic::Sigma(2, meas_sigma);

    // Projection parameters
    crs_source = config["gnss_crs_source"].as<string>();
    crs_target = config["gnss_crs_target"].as<string>();

    C = proj_context_create();
    P = proj_create_crs_to_crs(C, crs_source.c_str(), crs_target.c_str(), NULL);
}


GNSSFactor GNSSHandle::getCorrectionFactor(Point2 xy, int correction_count){
    return GNSSFactor(
        X(correction_count), xy, correction_noise
    );
}

Point2 GNSSHandle::getMeasurement(p_gnss_msg msg){
    input_coords = proj_coord(msg->latitude, msg->longitude, 0, 0);
    output_coords = proj_trans(P, PJ_FWD, input_coords);
    
    return Point2(output_coords.xy.y, output_coords.xy.x);
}
