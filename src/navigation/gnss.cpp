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

    x0_ = 0;
    y0_ = 0;
}


boost::shared_ptr<gtsam::NonlinearFactor> GNSSHandle::getCorrectionFactor(Point2 xy, int correction_count){
    return boost::make_shared<GNSSFactor>(X(correction_count), xy, correction_noise);
}

Point2 GNSSHandle::getMeasurement(p_gnss_msg msg){
    input_coords = proj_coord(msg->latitude, msg->longitude, 0, 0);
    output_coords = proj_trans(P, PJ_FWD, input_coords);

    if (x0_ == 0 && y0_ == 0){
        x0_ = output_coords.xy.x;
        y0_ = output_coords.xy.y;
    }

    GnssMeasurement m {
        msg->header.stamp.toSec(),
        output_coords.xy.y,
        output_coords.xy.x
    };
    measurements_.push_back(m);
    
    return Point2(output_coords.xy.y - y0_, output_coords.xy.x - x0_);
}


void GNSSHandle::getOffset(double &x0, double &y0){
    x0 = x0_;
    y0 = y0_;
}




void GNSSHandle::writeToFile(const std::string& out_file){
    ofstream f(out_file);

    f << "ts,north,east" << endl;
    f << fixed;

    for (auto m: measurements_){
        f << m.ts << "," << m.north << "," << m.east << endl;
    }
    
    f.close();
}