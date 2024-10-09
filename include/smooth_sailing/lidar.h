#ifndef LIDAR_H
#define LIDAR_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/sac_segmentation.h>  // For SACSegmentation



#include "smooth_sailing/leverArmFactor.h"
#include "common.h"

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace gtsam;

class LidarHandle{
private:
    // Add options for factor (How to calculate, noise model, etc. )
    Pose3 bTl_;
    double t_last_update_ = 0.0;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr msgToCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

public: 
    LidarHandle(){};
    LidarHandle(const YAML::Node &config);
    
    boost::shared_ptr<gtsam::NonlinearFactor> getLeverArmFactor(sensor_msgs::PointCloud2::ConstPtr msg, bool &success);
};



#endif