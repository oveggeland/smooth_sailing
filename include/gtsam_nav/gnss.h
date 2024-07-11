#ifndef GNSS_H
#define GNSS_H

#include <string.h>
#include "sensor_msgs/NavSatFix.h"
#include <proj.h>
#include <iomanip>

#include "graph.h"

using namespace std;

class GNSSHandle{
    public:
        // Constructors
        GNSSHandle(GraphHandle* p_gh, string src="EPSG:4326", string target="EPSG:6052");

        // Public functions
        void newMsg(sensor_msgs::NavSatFix::ConstPtr msg);

    private:
        // Member variables
        GraphHandle* p_gh;

        string crs_source;
        string crs_target;

        PJ_CONTEXT *C;
        PJ *P;

        PJ_COORD input_coords, output_coords; // https://proj.org/development/reference/datatypes.html#c.PJ_COORD


        double ts_prev;
        Vector2 xy_prev;

        Vector2 projectCartesian(sensor_msgs::NavSatFix::ConstPtr msg); // Retrieve local cartesian coordinates from navsatfix message
};


#endif