#include "smooth_sailing/lidar.h"

LidarHandle::LidarHandle(const YAML::Node &config){
    cout << "Initialize LiDAR handle" << endl;

    local_cloud = PointCloud();
}

PointCloud msgToCloud(sensor_msgs::PointCloud2::ConstPtr msg){
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    // Transfer points from PCL to Open3D
    PointCloud pcd;
    for (const auto& point : pcl_cloud.points) {
        if (point.x > 10)
            pcd.points_.emplace_back(point.x, point.y, point.z);
    }
    return pcd;
}


gtsam::Pose3 PerformICPAndReturnPose3(const open3d::geometry::PointCloud& source_cloud,
                                      const open3d::geometry::PointCloud& target_cloud) {
    // Set ICP parameters
    double threshold = 1; // Distance threshold for ICP convergence
    Eigen::Matrix4d initial_transform = Eigen::Matrix4d::Identity(); // Initial guess

    // Perform ICP registration using Open3D
    auto icp_result = open3d::pipelines::registration::RegistrationICP(
        source_cloud, target_cloud, threshold, initial_transform,
        open3d::pipelines::registration::TransformationEstimationPointToPoint());

    // Extract the transformation matrix from ICP result
    Eigen::Matrix4d T = icp_result.transformation_;

    // Return the resulting Pose3
    return gtsam::Pose3(Rot3(T.block<3, 3>(0, 0)), gtsam::Point3(T.block<3, 1>(0, 3)));
}

bool LidarHandle::newFrame(sensor_msgs::PointCloud2::ConstPtr msg){
    double ts = msg->header.stamp.toSec(); 
    cout << "lidar handle received new msg at " << ts << endl;

    if (ts - t0_ > 1){
        // Time for odometry
        PointCloud pcd1 = msgToCloud(msg);

        // Perform odometry
        deltaT_ = PerformICPAndReturnPose3(pcd1, pcd0_);
        dt_ = ts - t0_;
        cout << deltaT_ << endl;

        // Save newest cloud
        pcd0_ = pcd1;
        t0_ = ts;

        return true;
    }    
    
    return false;
}

void LidarHandle::init(sensor_msgs::PointCloud2::ConstPtr msg, int node_count){
    prev_node_count_ = node_count;
    t0_ = msg->header.stamp.toSec();
    pcd0_ = msgToCloud(msg);
    init_ = true;
}

BetweenFactor<gtsam::Pose3> LidarHandle::getOdometryFactor(int node_count){
    cout << "Lidar Odometry factor between " << prev_node_count_ << " and " << node_count; 
    auto factor = BetweenFactor(X(1), X(1), Pose3(), noiseModel::Isotropic::Sigma(6, 0.1));
    prev_node_count_ = node_count; 
    return factor;
}


/*
The plan is:
Dirty opencv stuff, just take 2 frames and align against the next two frames one seconds later
*/