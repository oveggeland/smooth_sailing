#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <map>
#include <fstream>
#include <iomanip>

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



std::map<double, Pose3> buildPoseMap(const std::string& filename){
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

            poseMap[ts] = Pose3(rotation, translation);
        }
    }

    file.close();
    return poseMap;
}



#define POINT_INTERVAL 10.0e-6 // microseconds
#define MIN_SQUARE_DISTANCE 100



void buildPointCloud(std::map<double, Pose3> poseMap, Pose3 Tbl, const std::string& bag_filename) {
    // Open the bag
    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);

    // Define the topic of interest
    std::string topic = "/livox_lidar_node/pointcloud2";


    
    // Set up a ROSBag view for the topic
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    
    int total_point_count = 0;
    std::vector<std::vector<_Float32>> position_vectors;
    position_vectors.reserve(view.size());
    //std::vector<std::vector<uint8_t>> intensity_tensors;
    //std::vector<open3d::core::Tensor> timestamp_tensors;

    std::vector<_Float32> transformed_points;
    transformed_points.reserve(3*3000*100000);

    cout << std::fixed << std::setprecision(20); // Set precision

    // Iterate over the messages
    for (rosbag::MessageInstance const m : view) {
        // Check if the message is a PointCloud2 message
        sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_msg != nullptr) {
            double t0 = cloud_msg->header.stamp.toSec();

            pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
            pcl::fromROSMsg(*cloud_msg, pcl_cloud);

            cout << "New point cloud at " << t0 << endl;

            int cnt = 0;

            // Allocate memory for current frame
            // std::vector<_Float32> transformed_points;
            // transformed_points.reserve(3*pcl_cloud.points.size());

            // std::vector<uint8_t> intensities;   
            // intensities.reserve(pcl_cloud.points.size());
            
            // std::vector<_Float64> timestamps;
            // timestamps.reserve(pcl_cloud.points.size());

            for (const auto& point : pcl_cloud.points) {
                if (point.x == 0.0 && point.y == 0.0 && point.z == 0.0) {
                    cnt ++;
                    continue; // Skip this point
                }
                if (point.x*point.x + point.y*point.y + point.z*point.z < MIN_SQUARE_DISTANCE){
                    cnt ++;
                    continue; // Less than 10 meters away, skip point
                }  
                
                double ts_point = t0 + (cnt/2)*POINT_INTERVAL;
                
                Pose3 T;
                if (queryPose(poseMap, ts_point, T)){
                    //T = T.compose(Tbl);
                    gtsam::Point3 gtsam_point(point.x, point.y, point.z);
                    gtsam::Point3 transformed_point = T.transformFrom(gtsam_point);

                    transformed_points.push_back(point.x);
                    transformed_points.push_back(point.y); 
                    transformed_points.push_back(point.z); 

                    //intensities.push_back(point.intensity);
                    //timestamps.push_back(ts_point);
                }
                cnt ++;
            }

            // if (transformed_points.size() != 0){
            //     //transformed_points.shrink_to_fit();
            //     position_vectors.push_back(transformed_points);

            //     total_point_count += transformed_points.size();
            // }
        }
    }


    position_vectors.shrink_to_fit();

    std::vector<_Float64> all_positions;
    all_positions.reserve(3*total_point_count);

    for (int i = 0; i < position_vectors.size(); i++){
        all_positions.insert(all_positions.end(), position_vectors[i].begin(), position_vectors[i].end());
    }


    // point_cloud.Translate(-point_cloud.GetCenter());

    // // Save the PointCloud to a file
    // std::string filename = "/home/oskar/smooth_sailing/data/ws_right1/raw.pcd";
    // open3d::t::io::WritePointCloud(filename, point_cloud);

    // // Visualize pointcloud in c++
    // open3d::visualization::Visualizer visualizer;
    // visualizer.CreateVisualizerWindow("Open3D PointCloud Viewer", 800, 600);

    // // Add the PointCloud to the visualizer
    // visualizer.AddGeometry(std::make_shared<open3d::geometry::PointCloud>(point_cloud.ToLegacy()));

    // // Run the visualizer
    // visualizer.Run();

    // // Close the visualizer window
    // visualizer.DestroyVisualizerWindow();

    // Close the bag    
    bag.close();
}





int main(){
    cout << std::fixed << std::setprecision(20); // Set precision

    Pose3 Tbl = readTbl("/home/oskar/smooth_sailing/data/ws_right1/calib/ext.yaml");

    std::map<double, Pose3> poseMap = buildPoseMap("/home/oskar/smooth_sailing/data/ws_right1/nav.txt");
    

    buildPointCloud(poseMap, Tbl, "/home/oskar/smooth_sailing/data/ws_right1/raw.bag");

    // double t_query = 1725632406;

    // Pose3 query_pose = queryPose(poseMap, t_query);
    // cout << query_pose.rotation().rpy() << endl;
    // cout << query_pose.translation() << endl;

    // Last 1725632700.00046491622924804688
    // First 1725632400.99974894523620605469

    return 0;
}