#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <map>
#include <fstream>
#include <iomanip>

#include <ros/ros.h>
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

#include <filesystem>
namespace fs = std::filesystem;

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


void buildPointCloud(std::map<double, Pose3> lidarPoseMap, const std::string& bag_filename, std::string out_file, double max_interval) {
    // Open the bag
    rosbag::Bag bag(bag_filename);
    double t_start = 0;

    // Set up a ROSBag view for the topic
    std::string topic = "/livox_lidar_node/pointcloud2";
    rosbag::View view(bag, rosbag::TopicQuery(topic));

    // Set up vectors to save point info (We could probably do this a bit smarter but it does not seem like the allocation is out biggest problem)
    std::vector<_Float64> positions;
    std::vector<uint8_t> intensities;
    std::vector<_Float64> timestamps;
    std::vector<_Float32> distances;
    
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

            if (t_start == 0){
                t_start = t0;
            }
            else if (t0 - t_start > max_interval){
                break;
            }
            
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

                _Float32 distance2 = point.x*point.x + point.y*point.y + point.z*point.z;
                _Float64 ts_point = t0 + (cnt/2)*POINT_INTERVAL;

                Pose3 T = interpolatePose3(T0, T1, (ts_point - t0) / frame_interval);
                gtsam::Point3 transformed_point = T.transformFrom(gtsam::Point3(point.x, point.y, point.z));

                positions.push_back(transformed_point.x());
                positions.push_back(transformed_point.y());
                positions.push_back(transformed_point.z());

                intensities.push_back(point.intensity);
                distances.push_back(distance2); // Distance squared, to save computational costs
                timestamps.push_back(ts_point);

                cnt ++;
            }
        }

        if (!ros::ok())
            exit(1);
    }
    bag.close();

    std::unordered_map<std::string, open3d::core::Tensor> tensor_map;

    tensor_map["positions"] = open3d::core::Tensor(positions, {(int)positions.size()/3, 3}, open3d::core::Dtype::Float64);
    tensor_map["intensities"] = open3d::core::Tensor(intensities, {(int)intensities.size(), 1}, open3d::core::Dtype::UInt8);
    tensor_map["timestamps"] = open3d::core::Tensor(timestamps, {(int)timestamps.size(), 1}, open3d::core::Dtype::Float64);
    tensor_map["distances"] = open3d::core::Tensor(distances, {(int)distances.size(), 1}, open3d::core::Dtype::Float32);
    open3d::t::geometry::PointCloud point_cloud(tensor_map);

    // Save the PointCloud to a file
    if (!open3d::t::io::WritePointCloud(out_file, point_cloud))
        cout << "Failed to save cloud at " << out_file << endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh("~");
    
    std::string workspace;
    if (!nh.getParam("/ws", workspace)){
        cout << "Error: No workspace provided" << endl;
        exit(1);
    }
    
    std::string ext_file;
    nh.getParam("/ext_file", ext_file);
    Pose3 Tbl = readTbl(ext_file);

    std::string nav_file = fs::path(workspace) / "navigation" / "nav.csv"; 
    std::map<double, Pose3> lidarPoseMap = buildLidarPoseMap(nav_file, Tbl);
    
    //double t_offset = YAML::LoadFile(fs::path(workspace) / "navigation" / "nav_info.yaml")["t0"].as<double>();
    double max_interval;
    nh.getParam("/max_time_interval", max_interval);
    buildPointCloud(lidarPoseMap, fs::path(workspace) / "cooked.bag", fs::path(workspace) / "raw.ply", max_interval);

    return 0;
}