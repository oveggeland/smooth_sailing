#include <iostream>

#include "gtsam_nav/gnss.h"
using namespace std;


GNSSHandle::GNSSHandle(GraphHandle* p_gh, string src, string target){
    this->p_gh = p_gh;

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


void GNSSHandle::newMsg(sensor_msgs::NavSatFix::ConstPtr msg){
    Vector2 xy = projectCartesian(msg);
    double ts = msg->header.stamp.toSec();

    if (p_gh->isInit()){
        // Update factor graph
    }

    else{
        // Initialize
        p_gh->initializePlanarPosition(xy);

        if (!xy_prev.isZero()){
            // Last position is non-zero, initialize velocity
            Vector2 d_xy = xy - xy_prev;
            float dt = ts - ts_prev;

            Vector2 v_xy = d_xy / dt;
            p_gh->initializePlanarVelocity(v_xy);
        }
        
        // Keep track of previous measurement for velocity intitialization
        xy_prev = xy;
        ts_prev = ts;
    }

}


