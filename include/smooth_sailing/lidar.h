#ifndef LIDAR_H
#define LIDAR_H

#include <sensor_msgs/PointCloud2.h>
#include <open3d/geometry/PointCloud.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <open3d/Open3D.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common.h"

#include "yaml-cpp/yaml.h"

using namespace open3d::geometry;
using namespace std;
using namespace gtsam;

class LidarHandle{
    PointCloud local_cloud;
    PointCloud global_cloud;

    double t0_ = 0;
    PointCloud pcd0_;

    Pose3 deltaT_;
    double dt_ = 0; 

    int prev_node_count_;
    bool init_ = false;

public: 
    LidarHandle(){};
    LidarHandle(const YAML::Node &config);

    // Returns true when odometry is ready
    bool newFrame(sensor_msgs::PointCloud2::ConstPtr msg);
    void init(sensor_msgs::PointCloud2::ConstPtr msg, int node_count);

    bool isInit(){return init_;};
    
    BetweenFactor<gtsam::Pose3> getOdometryFactor(int node_count);
};



#endif