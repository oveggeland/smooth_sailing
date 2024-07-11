#include "gtsam_nav/graph.h"


GraphHandle::GraphHandle(){
    init=false;
}

bool GraphHandle::isInit(){
    return init;
}


void GraphHandle::updateInit(){
    if (rp_init && y_init && xy_init && v_xy_init && z_init && v_z_init){
        init=true;

        cout << "INIT!!!!!!!!!!!!!!!!" << endl;
        cout << "Position: " << prior_pos << endl;
        cout << "Orientation: " << prior_rot.ypr() << endl;
        cout << "Velocity: " << prior_vel << endl << endl;
    }
}

void GraphHandle::initializePlanarPosition(Vector2 p){
    cout << "Init position: " << p << endl;
    prior_pos.head<2>() = p;
    xy_init = true;

    updateInit();
}

void GraphHandle::initializeOrientation(Rot3 R0){
    prior_rot = R0;
    rp_init = true;

    updateInit();
}

void GraphHandle::initializePlanarVelocity(Vector2 v){
    cout << "Init velocity " << v << endl;
    prior_vel.head<2>() = v;
    v_xy_init = true;

    updateInit();
}

