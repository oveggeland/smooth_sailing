#include <map>
#include <fstream>
#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <open3d/Open3D.h>
#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

#include <yaml-cpp/yaml.h>

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


// Build pose map (timestamp->pose) from navigation file and extrinsic matrix
std::map<double, Pose3> buildPoseMap(const std::string& filename, Pose3 T){
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

            poseMap[ts] = Pose3(rotation, translation).compose(T);
        }
    }

    file.close();
    return poseMap;
}


class mapBuilder{
public:
    mapBuilder(){}
    mapBuilder(const fs::path& ws, const Pose3& T, const YAML::Node& config): ws_(ws), ext_(T), config_(config){
        pose_map_ = buildPoseMap(ws_ / "navigation/nav.csv", ext_);

        cloud_interval_ = config_["cloud_max_interval"].as<double>();
        use_relative_stamps_ = config_["cloud_relative_timestamps"].as<bool>();
        sample_rate_ = config["lidar_sample_rate"].as<double>();
        frame_rate_ = config["lidar_frame_rate"].as<double>();
        return_count_ = config["lidar_return_count"].as<int>();
        min_x_distance_ = config["min_x_distance"].as<double>();

        frame_interval_ = (1 / frame_rate_);
        sample_interval_ = (1 / sample_rate_);

        cloud_path_ = ws_ / "raw_clouds"; 
        fs::create_directory(cloud_path_);
    }


    void addFrame(sensor_msgs::PointCloud2::ConstPtr cloud_msg){
        // Get timestamps
        double t0_frame = cloud_msg->header.stamp.toSec();
        double t1_frame = t0_frame + frame_interval_;

        // Query poses
        Pose3 T0, T1;
        if (!queryPose(pose_map_, t0_frame, T0) || !queryPose(pose_map_, t1_frame, T1)){
            return;
        }

        // Extract cloud
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);
        
        // Iterate through and add points
        int cnt = -1;
        for (const auto& point : cloud.points) {
            cnt ++;
            if (point.x < min_x_distance_){ // Rough outlier rejection
                continue; // Less than 10 meters away, skip point
            }  

            // Get point stamp and pose
            double ts_point = t0_frame + (cnt/2)*sample_interval_;
            Pose3 T = interpolatePose3(T0, T1, (ts_point - t0_frame) / frame_interval_);

            // Transform to world frame
            gtsam::Point3 transformed_point = T.transformFrom(gtsam::Point3(point.x, point.y, point.z));
            positions_.push_back(transformed_point.x());
            positions_.push_back(transformed_point.y());
            positions_.push_back(transformed_point.z());

            // Save auxillary attributes (Intensity, distance2, and timestamps)
            intensities_.push_back(point.intensity);
            distances_.push_back(point.x*point.x + point.y*point.y + point.z*point.z); // Distance squared, to save computational costs
            if (use_relative_stamps_)
                timestamps_.push_back(ts_point - t0_);
            else
                timestamps_.push_back(ts_point);
        }
    }


    void cloudTimeout(){
        if (!positions_.empty()){
            std::unordered_map<std::string, open3d::core::Tensor> tensor_map;

            tensor_map["positions"] = open3d::core::Tensor(positions_, {(int)positions_.size()/3, 3}, open3d::core::Dtype::Float64);
            tensor_map["intensities"] = open3d::core::Tensor(intensities_, {(int)intensities_.size(), 1}, open3d::core::Dtype::UInt8);
            tensor_map["timestamps"] = open3d::core::Tensor(timestamps_, {(int)timestamps_.size(), 1}, open3d::core::Dtype::Float64);
            tensor_map["distances"] = open3d::core::Tensor(distances_, {(int)distances_.size(), 1}, open3d::core::Dtype::Float32);
            open3d::t::geometry::PointCloud point_cloud(tensor_map);

            // Save the PointCloud to file
            string out_file = cloud_path_ / (std::to_string(cloud_cnt_) + "raw.ply");
            cout << "Saving cloud to " << out_file << endl;;
            if (!open3d::t::io::WritePointCloud(out_file, point_cloud))
                cout << "Failed to save cloud at " << out_file << endl;


            // Empty vectors (But keep capacity fixed)
            positions_.clear();
            intensities_.clear();
            timestamps_.clear();
            distances_.clear();
        }
        cloud_cnt_ ++;
    }

    void buildMaps(){
        rosbag::Bag bag(ws_ / "cooked.bag");
        rosbag::View view(bag, rosbag::TopicQuery(config_["lidar_topic"].as<string>()));

        t0_ = view.getBeginTime().toSec();

        for (rosbag::MessageInstance const m : view) {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg != nullptr) {
                double t0_frame = cloud_msg->header.stamp.toSec();

                if (t0_map_ == 0.0){
                    t0_map_ = t0_frame;
                }
                else if (t0_frame - t0_map_ > cloud_interval_){
                    cloudTimeout(); // Save cloud and reset
                    t0_map_ = 0.0;
                }
                
                addFrame(cloud_msg);
            }

            if (!ros::ok())
                exit(1);
        }
        bag.close();
    }


private:
    const fs::path ws_;
    fs::path cloud_path_;

    const Pose3 ext_; // Extrinsic matrix
    const YAML::Node config_;

    double sample_rate_;
    double frame_rate_;
    double sample_interval_;
    double frame_interval_;

    double min_x_distance_;
    int return_count_;


    std::map<double, Pose3> pose_map_;

    double cloud_interval_;
    int cloud_cnt_ = 0;
    bool use_relative_stamps_;
    double t0_ = 0.0;

    double t0_map_ = 0.0;

    vector<_Float64> positions_;
    vector<uint8_t> intensities_;
    vector<_Float64> timestamps_;
    vector<_Float32> distances_;
};


// Entry point to node
int main(int argc, char** argv){
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh("~");
    
    // Find workspace
    std::string workspace;
    if (!nh.getParam("/ws", workspace)){
        cout << "Error: No workspace provided" << endl;
        exit(1);
    }

    // Find extrinsics
    std::string ext_file;
    nh.getParam("/ext_file", ext_file);

    // Find extrinsics
    std::string config;
    nh.getParam("/map_config", config);

    // Build map object
    mapBuilder mb(workspace, readTbl(ext_file), YAML::LoadFile(config));
    mb.buildMaps();

    //buildPointCloud(lidarPoseMap, fs::path(workspace) / "cooked.bag", fs::path(workspace) / "raw.ply", max_interval);
    return 0;
}