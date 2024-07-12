#ifndef GNSS_H
#define GNSS_H

#include <string.h>
#include <proj.h>
#include <gtsam/base/Vector.h>

#include "sensor_msgs/NavSatFix.h"

using namespace std;
using namespace gtsam;

class GNSSHandle{
    public:
        // Constructors
        GNSSHandle(string src="EPSG:4326", string target="EPSG:6052");

        Vector2 projectCartesian(sensor_msgs::NavSatFix::ConstPtr msg); // Retrieve local cartesian coordinates from navsatfix message

    private:
        // Member variables
        string crs_source;
        string crs_target;

        PJ_CONTEXT *C;
        PJ *P;

        PJ_COORD input_coords, output_coords; // https://proj.org/development/reference/datatypes.html#c.PJ_COORD
};


#endif