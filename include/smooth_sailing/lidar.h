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
    Pose3 bTl_;
    double t_last_update_ = 0.0;

    double measurement_interval_;
    double measurement_sigma_;
    double min_x_distance_;
    int min_inlier_count_;

    pcl::SACSegmentation<pcl::PointXYZ> seg_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr msgToCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vector4 &plane_coeffs);

public: 
    LidarHandle(){};
    LidarHandle(const YAML::Node &config);
    
    boost::shared_ptr<gtsam::NonlinearFactor> getLeverArmFactor(sensor_msgs::PointCloud2::ConstPtr msg, bool &success);
};



#endif