#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <map>
#include <fstream>
#include <iomanip>
#include <algorithm>  // For std::clamp
#include <opencv2/opencv.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>


using namespace gtsam;
using namespace std;

Pose3 interpolatePose3(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2, double alpha) {
    // Interpolate translation
    gtsam::Point3 interpolated_position = (1 - alpha) * pose1.translation() + alpha * pose2.translation();
    Rot3 interpolated_rotation = pose1.rotation().slerp(alpha, pose2.rotation());
    
    // Construct and return the interpolated Pose3
    return Pose3(interpolated_rotation, interpolated_position);
}



// Function to query pose for a given time
bool queryPose(const std::map<double, gtsam::Pose3>& poses, double query_time, Pose3& pose_out) {
    auto it = poses.lower_bound(query_time);
    
    if (it == poses.end() || it == poses.begin()) {
        return false; // Out of bounds, no pose available
    }
    
    auto prev_it = std::prev(it);
    double time1 = prev_it->first;
    double time2 = it->first;
    
    if (query_time < time1 || query_time > time2) {
        throw std::out_of_range("THIS SHOLD NEVER HAPPEN RIGHT?");
        return false;
    }
    
    double t = (query_time - time1) / (time2 - time1);
    
    const gtsam::Pose3& pose1 = prev_it->second;
    const gtsam::Pose3& pose2 = it->second;
    
    pose_out = interpolatePose3(pose1, pose2, t);
    return true;
}

#include <yaml-cpp/yaml.h>

// Read lidar to body transformation
Pose3 readTbl(const std::string& filename){
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



std::map<double, Pose3> buildLidarPoseMap(const std::string& filename, Pose3 Tbl){
    std::map<double, Pose3> poseMap;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return poseMap;
    }

    std::string line;
    // Skip the header line
    if (std::getline(file, line)) {
        // Read the file line by line
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string item;
            double ts, x, y, z, roll, pitch, yaw;

            // Extract the columns by comma delimiter
            std::getline(ss, item, ',');
            ts = std::stod(item);

            std::getline(ss, item, ',');
            x = std::stod(item);
            std::getline(ss, item, ',');
            y = std::stod(item);
            std::getline(ss, item, ',');
            z = std::stod(item);

            for (int i = 0; i < 3; ++i) {
                std::getline(ss, item, ','); // Skipping vx, vy and vz
            }

            std::getline(ss, item, ',');
            roll = std::stod(item);
            std::getline(ss, item, ',');
            pitch = std::stod(item);
            std::getline(ss, item, ',');
            yaw = std::stod(item);

            // Get vx
            std::getline(ss, item, '\n');
            Point3 translation(x, y, z);
            Rot3 rotation = Rot3::RzRyRx(roll, pitch, yaw);
            Pose3(rotation, translation);

            poseMap[ts] = Pose3(rotation, translation).compose(Tbl);
        }
    }

    file.close();
    return poseMap;
}



#define POINT_INTERVAL 10.0e-6 // microseconds
#define MIN_X_DISTANCE 10


open3d::core::Tensor IntensityToColorTensor(const std::vector<uint8_t>& intensities) {
    // Convert std::vector<int> to cv::Mat
    cv::Mat intensity_image(intensities.size(), 1, CV_8UC1);
    for (size_t i = 0; i < intensities.size(); ++i) {
        intensity_image.at<uchar>(i, 0) = static_cast<uchar>(std::clamp((int)intensities[i], 0, 255));
    }

    // Apply OpenCV colormap
    cv::Mat color_image;
    cv::applyColorMap(intensity_image, color_image, cv::COLORMAP_JET); // You can choose other colormaps

    // Convert cv::Mat to open3d::core::Tensor
    std::vector<float> colors;
    colors.reserve(color_image.rows * color_image.cols * 3);
    for (int i = 0; i < color_image.rows; ++i) {
        const cv::Vec3b& color = color_image.at<cv::Vec3b>(i, 0);
        colors.push_back(color[2] / 255.0f); // R
        colors.push_back(color[1] / 255.0f); // G
        colors.push_back(color[0] / 255.0f); // B
    }

    // Create Tensor from colors
    open3d::core::Tensor color_tensor(
        colors.data(), 
        {colors.size() / 3, 3},
        open3d::core::Dtype::Float32
    );
    
    return color_tensor;
}



void buildPointCloud(std::map<double, Pose3> lidarPoseMap, const std::string& bag_filename) {
    // Open the bag
    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);

    // Set up a ROSBag view for the topic
    std::string topic = "/livox_lidar_node/pointcloud2";
    rosbag::View view(bag, rosbag::TopicQuery(topic));

    // Set up vectors to save point info (We could probably do this a bit smarter but it does not seem like the allocation is out biggest problem)
    std::vector<_Float64> positions;
    std::vector<uint8_t> intensities;
    std::vector<_Float64> timestamps;
    
    // Iterate over the messages
    for (rosbag::MessageInstance const m : view) {
        // Check if the message is a PointCloud2 message
        sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_msg != nullptr) {
            pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
            pcl::fromROSMsg(*cloud_msg, pcl_cloud);

            _Float64 t0 = cloud_msg->header.stamp.toSec();
            _Float64 frame_interval = 2*pcl_cloud.size()*POINT_INTERVAL;
            _Float64 t1 = t0 + frame_interval; // LiDAR in dual return mode
            
            // Initial and end pose of lidar frame, if not available, skip to next frame
            Pose3 T0, T1;
            if (!queryPose(lidarPoseMap, t0, T0) || !queryPose(lidarPoseMap, t1, T1)){
                continue;
            }

            int cnt = 0;
            for (const auto& point : pcl_cloud.points) {
                if (point.x < MIN_X_DISTANCE){ // Rough outlier rejection
                    cnt ++;
                    continue; // Less than 10 meters away, skip point
                }  

                _Float64 ts_point = t0 + (cnt/2)*POINT_INTERVAL;

                Pose3 T = interpolatePose3(T0, T1, (ts_point - t0) / frame_interval);
                gtsam::Point3 transformed_point = T.transformFrom(gtsam::Point3(point.x, point.y, point.z));

                positions.push_back(transformed_point.x());
                positions.push_back(transformed_point.y());
                positions.push_back(transformed_point.z());

                intensities.push_back(point.intensity);
                timestamps.push_back(ts_point);

                cnt ++;
            }
        }
    }
    bag.close();

    std::unordered_map<std::string, open3d::core::Tensor> tensor_map;

    tensor_map["positions"] = open3d::core::Tensor(positions, {positions.size()/3, 3}, open3d::core::Dtype::Float64);
    tensor_map["intensities"] = open3d::core::Tensor(intensities, {intensities.size(), 1}, open3d::core::Dtype::UInt8);
    tensor_map["timestamps"] = open3d::core::Tensor(timestamps, {timestamps.size(), 1}, open3d::core::Dtype::Float64);
    open3d::t::geometry::PointCloud point_cloud(tensor_map);

    // Save the PointCloud to a file
    std::string filename = "/home/oskar/smooth_sailing/data/ws_right1/raw.pcd";
    open3d::t::io::WritePointCloud(filename, point_cloud);
}


void visualizePointCloud(std::string filename){
    // Read file
    open3d::t::geometry::PointCloud point_cloud;
    open3d::t::io::ReadPointCloud(filename, point_cloud);

    // Visualize pointcloud in c++
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Open3D PointCloud Viewer", 800, 600);

    // Add the PointCloud to the visualizer
    visualizer.AddGeometry(std::make_shared<open3d::geometry::PointCloud>(point_cloud.ToLegacy()));

    // Run the visualizer
    visualizer.Run();

    // Close the visualizer window
    visualizer.DestroyVisualizerWindow();
}



int main(){
    Pose3 Tbl = readTbl("/home/oskar/smooth_sailing/data/ws_right1/calib/ext.yaml");

    std::map<double, Pose3> lidarPoseMap = buildLidarPoseMap("/home/oskar/smooth_sailing/data/ws_right1/nav.txt", Tbl);
    

    buildPointCloud(lidarPoseMap, "/home/oskar/smooth_sailing/data/ws_right1/raw.bag");


    return 0;
}