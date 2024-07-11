#include "gtsam_nav/imu.h"

IMUHandle::IMUHandle(GraphHandle* p_gh){
    this->p_gh = p_gh;
    gravity = Unit3(0, 0, -1); // Direction of gravity used for orientation initialization
}


void IMUHandle::integrateMeasurement(sensor_msgs::Imu::ConstPtr msg){
    // TODO: Implement
}

void IMUHandle::initializeOrientation(sensor_msgs::Imu::ConstPtr msg){
    // Alignment test
    Unit3 acc(
        msg->linear_acceleration.x, 
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );

    // Calculate the rotation that aligns the IMU measurement with gravity
    Rot3 orientation = Rot3::AlignPair(acc.cross(gravity), gravity, acc);

    p_gh->initializeOrientation(orientation);

    if IMU_DEBUG{
        // Print the result
        cout << "Rotation (to align IMU with gravity):\n" << orientation.matrix() << endl;
        cout << "YPR " << orientation.ypr() << endl << endl;
    }
}


void IMUHandle::newMsg(sensor_msgs::Imu::ConstPtr msg){
    if (p_gh->isInit()){
        // Regular stuff
        integrateMeasurement(msg);
    }

    else{
        // Init stuff
        initializeOrientation(msg);
    }
}