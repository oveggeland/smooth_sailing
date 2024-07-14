#include <iostream>

#include "gtsam_nav/gnss.h"
using namespace std;


GNSSHandle::GNSSHandle(string src, string target){
    crs_source = src;
    crs_target = target;

    C = proj_context_create();
    P = proj_create_crs_to_crs(C, src.c_str(), target.c_str(), NULL);
}


Vector2 GNSSHandle::projectCartesian(sensor_msgs::NavSatFix::ConstPtr msg){
    input_coords = proj_coord(msg->latitude, msg->longitude, 0, 0);
    output_coords = proj_trans(P, PJ_FWD, input_coords);
    
    return Vector2(output_coords.xy.y, output_coords.xy.x);
}



