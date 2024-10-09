#include "smooth_sailing/lidar.h"


// Read lidar to body transformation
Pose3 readExt(const std::string& filename){
    YAML::Node config = YAML::LoadFile(filename);

    gtsam::Matrix4 Tcb;
    auto Tcb_node = config["T_cam_imu"];

    gtsam::Matrix4 Tlc;
    auto Tlc_node = config["T_lidar_cam"];
    
    
    // Fill the matrix with the values from the YAML node
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            Tcb(i, j) = Tcb_node[i][j].as<double>();
            Tlc(i, j) = Tlc_node[i][j].as<double>();
        }
    }

    gtsam::Pose3 pose3cb(Tcb);
    gtsam::Pose3 pose3lc(Tlc);

    return pose3cb.inverse().compose(pose3lc.inverse());
}


LidarHandle::LidarHandle(const YAML::Node &config){
    bTl_ = readExt("/home/oskar/smooth_sailing/src/smooth_sailing/cfg/calib/ext_right.yaml");

    measurement_interval_ = config["lidar_measurement_interval"].as<double>();
    measurement_sigma_ =  config["lidar_measurement_sigma"].as<double>();
    min_x_distance_ = config["lidar_measurement_min_x"].as<double>();
    min_inlier_count_ = config["lidar_measurement_min_inliers"].as<int>();


    double ransac_threshold_ = config["lidar_measurement_ransac_threshold"].as<double>();
    double ransac_prob_ = config["lidar_measurement_ransac_prob"].as<double>();

    seg_.setModelType(pcl::SACMODEL_PLANE); // Set the model you want to fit
    seg_.setMethodType(pcl::SAC_RANSAC);    // Use RANSAC to estimate the plane
    seg_.setDistanceThreshold(ransac_threshold_);        // Set a distance threshold for points to be considered inliers)
    seg_.setProbability(ransac_prob_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarHandle::msgToCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert from sensor_msgs/PointCloud2 to PCL point cloud
    pcl::fromROSMsg(*msg, *cloud);

    // Create a new point cloud to store filtered and transformed data
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Get the rotation (R) and translation (t) from the gtsam::Pose3 object
    gtsam::Matrix3 R = bTl_.rotation().matrix();  // 3x3 rotation matrix
    gtsam::Point3 t = bTl_.translation();         // Translation vector

    // Iterate over points, transform and filter points with x > 10
    for (const auto& point : cloud->points) {
        if (point.x < min_x_distance_) {
            continue;
        }

        // Apply the transformation to the point
        Eigen::Vector3d p(point.x, point.y, point.z);  // Original point in Eigen form
        Eigen::Vector3d p_transformed = R * p + Eigen::Vector3d(t.x(), t.y(), t.z());  // Transformed point

        pcl::PointXYZ new_point;
        new_point.x = p_transformed.x();
        new_point.y = p_transformed.y();
        new_point.z = p_transformed.z();
        transformed_cloud->points.push_back(new_point);
    }

    transformed_cloud->width = transformed_cloud->points.size();
    transformed_cloud->height = 1; // Unorganized point cloud
    transformed_cloud->is_dense = true;

    return transformed_cloud;
}

bool LidarHandle::segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vector4 &plane_coeffs){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg_.setInputCloud(cloud);
    seg_.segment(*inliers, *coefficients);

    if (inliers->indices.size() < min_inlier_count_) {
        return false;
    }
    
    for (int i = 0; i < 4; i++)
        plane_coeffs[i] = coefficients->values[i];

    return true;
}

boost::shared_ptr<gtsam::NonlinearFactor> LidarHandle::getLeverArmFactor(sensor_msgs::PointCloud2::ConstPtr msg, bool &success){

    double ts = msg->header.stamp.toSec();
    if (ts - t_last_update_ < measurement_interval_){
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = msgToCloud(msg);
    Vector4 plane_coeffs;
    if (segmentPlane(cloud, plane_coeffs)){
        cout << "Plane model: " << plane_coeffs.transpose() << endl;

        success=true;
        t_last_update_ = ts;
        return boost::make_shared<LeverArmFactor>(L(0), plane_coeffs, noiseModel::Isotropic::Sigma(1, measurement_sigma_));
    };

    return nullptr;
}