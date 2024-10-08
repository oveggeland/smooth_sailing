#ifndef GNSS_H
#define GNSS_H

#include <string.h>
#include <proj.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

#include "yaml-cpp/yaml.h"

#include "sensor_msgs/NavSatFix.h"

#include "smooth_sailing/gnssFactor.h"
#include "smooth_sailing/common.h"

using namespace std;
using namespace gtsam;

typedef sensor_msgs::NavSatFix::ConstPtr p_gnss_msg;

typedef struct{
    _Float64 ts;
    _Float64 north;
    _Float64 east;
} GnssMeasurement;



class GNSSHandle{
    public:
        // Constructors
        GNSSHandle(){};
        GNSSHandle(const YAML::Node &config);

        Point2 getMeasurement(p_gnss_msg);

        boost::shared_ptr<gtsam::NonlinearFactor> getCorrectionFactor(Point2 xy, int correction_count);
        void getOffset(double &x0, double &y0);

        void writeToFile(const std::string& out_file);

    private:
        // Noise parameters
        double meas_sigma;

        noiseModel::Isotropic::shared_ptr correction_noise;

        // Projection parameters
        string crs_source;
        string crs_target;

        PJ_CONTEXT *C;
        PJ *P;

        PJ_COORD input_coords, output_coords; // https://proj.org/development/reference/datatypes.html#c.PJ_COORD

        _Float64 x0_, y0_; // Offsets

        std::vector<GnssMeasurement> measurements_;
};


#endif