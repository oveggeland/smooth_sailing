#include "gtsam_nav/graph.h"


// Constructor
GraphHandle::GraphHandle(){
    init=false;
    imu_handle = IMUHandle();
    gnss_handle = GNSSHandle();

    // Initialize imu-preintegration
    auto p = imuParams();
    preintegrated =
        std::make_shared<PreintegratedImuMeasurements>(p);
    assert(preintegrated);

}


// Entry point for new IMU measurements
void GraphHandle::newImuMsg(sensor_msgs::Imu::ConstPtr msg){
    double ts = msg->header.stamp.toSec();

    if (init){
        double dt = ts-ts_head;
        if (dt <= 0){
            cout << "ERROR IMU DT < 0" << endl;
        }        
        
        Vector3 acc = getAcc(msg);
        cout << "IMU integration:" << acc[0] << ", " << acc[1] << ", " << acc[2] << " - " << dt << endl;
        preintegrated->integrateMeasurement(
            getAcc(msg), 
            getRate(msg), 
            dt
        );

        ts_head = ts; // IMPORTANT 
    }
    else{
        Rot3 R0 = imu_handle.getOrientation(msg);
        initializeOrientation(R0, ts);
    }
}


void GraphHandle::newCorrection(double ts){

    
    // Adding IMU factor and GPS factor and optimizing.
    auto preint_imu = dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated);
    ImuFactor imu_factor(X(state_count - 1), V(state_count - 1),
                        X(state_count), V(state_count),
                        B(state_count - 1), preint_imu);
    graph.add(imu_factor);
    cout << "Adding IMU factor between node " << state_count-1 << " and " << state_count << endl;


    // Bias noise betweenfactor
    auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);
    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    graph.add(BetweenFactor<imuBias::ConstantBias>(
        B(state_count - 1), B(state_count), zero_bias,
        bias_noise_model));


    // Propogate to get new initial values
    cout << "State before optimization is: " << prev_state << endl;
    prop_state = preintegrated->predict(prev_state, prev_bias);
    initial_values.insert(X(state_count), prop_state.pose());
    initial_values.insert(V(state_count), prop_state.v());
    initial_values.insert(B(state_count), prev_bias);

    // Optimize
    Values result;

    LevenbergMarquardtOptimizer optimizer(graph, initial_values);
    result = optimizer.optimize();

    // Overwrite the beginning of the preintegration for the next step.
    prev_state = NavState(result.at<Pose3>(X(state_count)),
                            result.at<Vector3>(V(state_count)));
    prev_bias = result.at<imuBias::ConstantBias>(B(state_count));

    preintegrated->resetIntegrationAndSetBias(prev_bias);


    cout << "State after optimization is: " << prev_state << endl << endl;

    state_count ++;
    ts_head = ts;
}


// Entry point for new GNSS measurements
void GraphHandle::newGNSSMsg(sensor_msgs::NavSatFix::ConstPtr msg){
    double ts = msg->header.stamp.toSec();
    Vector2 xy = gnss_handle.projectCartesian(msg);

    if (init){
        // Add GNSS factor to graph
        auto correction_noise = noiseModel::Isotropic::Sigma(3, 1.0);
        GPSFactor gps_factor(X(state_count), Point3(xy[0], xy[1], 0), correction_noise);
        graph.add(gps_factor);

        cout << "Added GNSS factor at state node " << state_count << endl;

        
        newCorrection(ts);
    }
    else{
        initializePlanarPosition(xy, ts);

        if (!prev_xy_gnss.isZero()){
            initializePlanarVelocity((xy - prev_xy_gnss) / (ts - prev_ts_gnss), ts);
        }
    }

    prev_xy_gnss = xy;
    prev_ts_gnss = ts;
}


/*
HERE COMES ALL IMPLEMENTATION RELATED TO FACTOR GRAPH INITIALIZATION
*/
void GraphHandle::initializeFactorGraph(){
    graph = NonlinearFactorGraph(); // Initialize factor graph
    initial_values.clear(); // Clear initial values (to be sure)

    // Assemble prior noise model and add it the graph.`
    auto pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 1.5, 1.5, 1.5, 1, 1, 0.5)
        .finished());  // rad,rad,rad,m, m, m
    auto velocity_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(3) << 0.5, 0.5, 0.1)
        .finished());  // m/s
    auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

    Pose3 prior_pose(prior_rot, prior_pos);

    initial_values.insert(X(0), prior_pose);
    initial_values.insert(V(0), prior_vel);
    initial_values.insert(B(0), prior_imu_bias);

    graph.addPrior(X(0), prior_pose, pose_noise_model);
    graph.addPrior(V(0), prior_vel, velocity_noise_model);
    graph.addPrior(B(0), prior_imu_bias, bias_noise_model);


    // Number of states is now 1, since we have initialized the graph
    prev_state = NavState(prior_pose, prior_vel);
    state_count = 1;
    ts_head = ts_init;
}


void GraphHandle::updateInit(){
    if (rp_init && y_init && xy_init && v_xy_init && z_init && v_z_init){
        init=true;

        initializeFactorGraph();

        cout << "INIT FINISHED!!!!!!!!!!!!!!!!" << endl;
    }
}

void GraphHandle::initializePlanarPosition(Vector2 p, double ts){
    cout << "Init position: " << p << endl;
    prior_pos.head<2>() = p;
    xy_init = true;

    ts_init = ts;

    updateInit();
}

void GraphHandle::initializeOrientation(Rot3 R0, double ts){
    prior_rot = R0;
    rp_init = true;

    ts_init = ts;

    updateInit();
}

void GraphHandle::initializePlanarVelocity(Vector2 v, double ts){
    cout << "Init velocity " << v << endl;
    prior_vel.head<2>() = v;
    v_xy_init = true;

    ts_init = ts;

    updateInit();
}