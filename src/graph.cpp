#include "gtsam_nav/graph.h"


GraphHandle::GraphHandle(){
    init=false;
}

bool GraphHandle::isInit(){
    return init;
}


void GraphHandle::initializeFactorGraph(){
    graph = new NonlinearFactorGraph(); // Initialize factor graph
    initial_values.clear(); // Clear initial values (to be sure)

    // Assemble prior noise model and add it the graph.`
    auto pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.02, 0.02, 1.5, 1, 1, 0.5)
        .finished());  // rad,rad,rad,m, m, m
    auto velocity_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(3) << 0.5, 0.5, 0.1)
        .finished());  // m/s
    auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

    Pose3 prior_pose(prior_rot, prior_pos);

    initial_values.insert(X(0), prior_pose);
    initial_values.insert(V(0), prior_vel);
    initial_values.insert(B(0), prior_imu_bias);

    graph->addPrior(X(0), prior_pose, pose_noise_model);
    graph->addPrior(V(0), prior_vel, velocity_noise_model);
    graph->addPrior(B(0), prior_imu_bias, bias_noise_model);

    // Number of states is now 1, since we have initialized the graph
    state_count = 1;
}

void GraphHandle::updateInit(){
    if (rp_init && y_init && xy_init && v_xy_init && z_init && v_z_init){
        init=true;

        initializeFactorGraph();

        cout << "INIT!!!!!!!!!!!!!!!!" << endl;
        cout << "Position: " << prior_pos << endl;
        cout << "Orientation: " << prior_rot.ypr() << endl;
        cout << "Velocity: " << prior_vel << endl;
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

