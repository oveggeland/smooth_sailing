#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <map>
#include <fstream>
#include <iomanip>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

#include <open3d/Open3D.h>
#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>
#include <open3d/core/nns/NearestNeighborSearch.h>
#include <open3d/core/nns/FixedRadiusIndex.h>

#include <yaml-cpp/yaml.h>

#include <tuple>


using namespace gtsam;
using namespace std;
using namespace open3d;

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

// Read camera to body transformation
Pose3 readTbc(const std::string& filename){
    YAML::Node config = YAML::LoadFile(filename);

    gtsam::Matrix4 Tcb;
    auto Tcb_node = config["T_cam_imu"];
    
    // Fill the matrix with the values from the YAML node
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            Tcb(i, j) = Tcb_node[i][j].as<double>();
        }
    }

    return gtsam::Pose3(Tcb).inverse(); 
}


std::map<double, Pose3> buildSensorPoseMap(const std::string& nav_file, Pose3 Tbs){
    std::map<double, Pose3> poseMap;
    std::ifstream file(nav_file);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << nav_file << std::endl;
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

            poseMap[ts] = Pose3(rotation, translation).compose(Tbs);
        }
    }

    file.close();
    return poseMap;
}



class CloudQuery{
    public:
        t::geometry::PointCloud raw_cloud_;
        core::nns::NearestNeighborSearch nns_;
        
        CloudQuery(t::geometry::PointCloud pcd): 
            raw_cloud_(pcd), nns_(core::nns::NearestNeighborSearch(pcd.GetPointAttr("timestamps").Reshape({-1, 1})))
        { 
            nns_.KnnIndex();  // Build the KDTree index
        }

        t::geometry::PointCloud subCloudQuery(_Float64 ts, _Float64 dt_thresh){
            // Step 4: Perform a radius search for points within the range [query_timestamp - 5, query_timestamp + 5]
            auto query_point = open3d::core::Tensor(std::vector<_Float64>(1, ts), {1, 1}, open3d::core::Dtype::Float64);
            auto result = nns_.FixedRadiusSearch(query_point, dt_thresh);

            // Step 5: Output the results
            auto indices = std::get<0>(result);

            return raw_cloud_.SelectByIndex(indices.To(open3d::core::Dtype::Int64));
        }
};

void visualizeCloud(t::geometry::PointCloud cloud) {
    // Convert tensor-based point cloud to legacy format for visualization
    open3d::geometry::PointCloud legacy_cloud = cloud.ToLegacy();
    
    // Visualize the point cloud using Open3D's built-in visualization
    open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(legacy_cloud)}, 
                                          "Point Cloud Visualization");
}


void compareClouds(t::geometry::PointCloud full_cloud, t::geometry::PointCloud sub_cloud) {
    // Convert tensor-based point clouds to legacy format for visualization
    open3d::geometry::PointCloud legacy_full_cloud = full_cloud.ToLegacy();
    open3d::geometry::PointCloud legacy_sub_cloud = sub_cloud.ToLegacy();

    // Assign colors for distinguishing the full cloud and the subset cloud
    legacy_full_cloud.PaintUniformColor(Eigen::Vector3d(0.6, 0.6, 0.6));  // Gray for full cloud
    legacy_sub_cloud.PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));  // Red for subset cloud

    // Combine both clouds in a single visualization
    open3d::visualization::DrawGeometries({
        std::make_shared<open3d::geometry::PointCloud>(legacy_full_cloud),
        std::make_shared<open3d::geometry::PointCloud>(legacy_sub_cloud)
    }, "Full Cloud vs Sub Cloud Comparison");
}



camera::PinholeCameraIntrinsic ReadCameraIntrinsics(const std::string& file_path) {
    // Load the YAML file
    YAML::Node config = YAML::LoadFile(file_path);
    
    // Extract intrinsic parameters (fx, fy, cx, cy)
    std::vector<double> intrinsics = config["intrinsics"].as<std::vector<double>>();
    double fx = intrinsics[0];
    double fy = intrinsics[1];
    double cx = intrinsics[2];
    double cy = intrinsics[3];

    // Extract resolution (width, height)
    std::vector<int> resolution = config["resolution"].as<std::vector<int>>();
    int width = resolution[0];
    int height = resolution[1];

    // Create and return Open3D camera intrinsic object
    return open3d::camera::PinholeCameraIntrinsic(resolution[0], resolution[1], fx, fy, cx, cy);
}



// Function to set camera extrinsics from gtsam::Pose3
void SetCameraExtrinsicsFromPose(const gtsam::Pose3& pose, open3d::camera::PinholeCameraParameters& camera_params) {
    // Extract the rotation matrix (3x3) and translation vector (3x1) from gtsam::Pose3
    Eigen::Matrix3d rotation = pose.rotation().matrix();  // Get rotation matrix
    Eigen::Vector3d translation = pose.translation();     // Get translation vector

    // Create a 4x4 extrinsics matrix for Open3D
    Eigen::Matrix4d extrinsics = Eigen::Matrix4d::Identity();  // Initialize with identity matrix

    // Set rotation (top-left 3x3 part of the 4x4 matrix)
    extrinsics.block<3, 3>(0, 0) = rotation;

    // Set translation (top-right 3x1 part of the 4x4 matrix)
    extrinsics.block<3, 1>(0, 3) = translation;

    // Assign the extrinsics to Open3D camera parameters
    camera_params.extrinsic_ = extrinsics;
}


void visualizePointCloud(t::geometry::PointCloud cloud, open3d::camera::PinholeCameraParameters cam_params) {
    // Convert tensor-based point cloud to legacy format for visualization
    open3d::geometry::PointCloud legacy_cloud = cloud.ToLegacy();
    
    // Create a Visualizer instance
    auto vis = std::make_shared<open3d::visualization::Visualizer>();
    vis->CreateVisualizerWindow("Point Cloud Visualization", 1440, 1080);

    // Add the point cloud to the visualizer
    vis->AddGeometry(std::make_shared<open3d::geometry::PointCloud>(legacy_cloud));

    // Set the camera parameters
    vis->GetViewControl().ConvertFromPinholeCameraParameters(cam_params, true);

    // Run the visualizer
    vis->Run();
    
    // Destroy the visualizer window
    vis->DestroyVisualizerWindow();
}

void ColorPointCloudWithColormap(open3d::t::geometry::PointCloud& pointcloud) {
    // Extract the point positions
    const auto& positions = pointcloud.GetPointPositions();
    size_t num_points = positions.GetShape(0);

    // Extract z-components and normalize to [0, 255]
    std::vector<uint8_t> z_values(num_points);
    auto positions_ptr = positions.GetDataPtr<float>();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();


    // Extract the intensities

    if (!pointcloud.HasPointAttr("scalar_intensities")) {
        std::cerr << "Point cloud does not have 'intensities' channel." << std::endl;
        return;
    }
    cout << "HALLO" << endl;
    const auto intensities = pointcloud.GetPointAttr("scalar_intensities");
    
    cout << "HEI" << endl;
    // Convert tensor to std::vector<uint8_t>
    std::vector<float> intensities_vec(num_points);
    std::memcpy(intensities_vec.data(), intensities.GetDataPtr<float>(), num_points * sizeof(float));
    

    cout << "HEI" << endl;
    // Normalize intensities to [0, 1]
    auto min_intensity = *std::min_element(intensities_vec.begin(), intensities_vec.end());
    auto max_intensity = *std::max_element(intensities_vec.begin(), intensities_vec.end());
    std::vector<float> normalized_intensities(num_points);
    for (size_t i = 0; i < num_points; ++i) {
        normalized_intensities[i] = (intensities_vec[i] - min_intensity) / static_cast<float>(max_intensity - min_intensity);
    }

    for (size_t i = 0; i < num_points; ++i) {
        float z = positions_ptr[i * 3 + 2]; // z-component
        if (z < min_z) min_z = z;
        if (z > max_z) max_z = z;
    }

    for (size_t i = 0; i < num_points; ++i) {
        float z = positions_ptr[i * 3 + 2];
        z_values[i] = static_cast<uint8_t>(255.0 * (z - min_z) / (max_z - min_z));
    }
    
    // Create a cv::Mat for the z-values and apply colormap
    cv::Mat z_values_mat(num_points, 1, CV_8UC1, z_values.data());
    cv::Mat color_map_mat;
    cv::applyColorMap(z_values_mat, color_map_mat, cv::COLORMAP_WINTER);

    // Convert color_map_mat to std::vector
    std::vector<uint8_t> colors_vec(color_map_mat.total() * color_map_mat.elemSize());
    std::memcpy(colors_vec.data(), color_map_mat.data, colors_vec.size());

    // Create a color tensor
    open3d::core::Tensor colors_tensor({num_points, 3}, open3d::core::Float32);
    auto colors_ptr = colors_tensor.GetDataPtr<float>();

    // Assign colors to the tensor
    for (size_t i = 0; i < num_points; ++i) {
        colors_ptr[i * 3 + 2] = normalized_intensities[i]*colors_vec[i * 3] / 255.0f;     // Red
        colors_ptr[i * 3 + 1] = normalized_intensities[i]*colors_vec[i * 3 + 1] / 255.0f; // Green
        colors_ptr[i * 3] = normalized_intensities[i]*colors_vec[i * 3 + 2] / 255.0f; // Blue
    }

    cout << "HFERFES" << endl;
    // Set the color attribute
    pointcloud.SetPointAttr("colors", colors_tensor);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "image_reconstruction_node");
    ros::NodeHandle nh("~");
    
    std::string workspace;
    if (!nh.getParam("/ws", workspace)){
        cout << "Error: No workspace provided" << endl;
        exit(1);
    }

    Pose3 Tbc = readTbc(workspace + "calib/ext.yaml");
    auto poseMap = buildSensorPoseMap(workspace + "nav.txt", Tbc);
    cout << "HEI" << endl;
    // Read pointcloud
    t::geometry::PointCloud pcd;
    if (!t::io::ReadPointCloudFromPLY(workspace + "processed.ply", pcd, open3d::io::ReadPointCloudOption()))
        cout << "READ FAILED" << endl;

    const auto& attributes = pcd.GetPointAttr();
    
    // Iterate through attributes and print their names
    for (const auto& attribute : attributes) {
        std::cout << "Attribute name: " << attribute.first << std::endl;
    }

    cout << workspace + "processed.ply" << endl;
    ColorPointCloudWithColormap(pcd);

    // Make a subcloud query object
    //CloudQuery pcd_query(pcd);

    // Read camera intrinsics
    open3d::camera::PinholeCameraParameters camera_params;
    camera_params.intrinsic_ = ReadCameraIntrinsics(workspace + "calib/int.yaml");


    // Create a Visualizer instance
    auto vis = std::make_shared<open3d::visualization::Visualizer>();
    vis->CreateVisualizerWindow("Point Cloud Visualization", 1440, 1080, 0, 0, false);
    vis->AddGeometry(std::make_shared<open3d::geometry::PointCloud>(pcd.ToLegacy()));

    vis->GetRenderOption().point_size_ = 7;


    // Iterate through bag and read images (to get timestamps)
    rosbag::Bag bag(workspace + "cooked.bag");

    // Specify the topic we are interested in
    rosbag::View view(bag, rosbag::TopicQuery("/blackfly_node/image"));

    // Iterate through the messages in the topic
    int cnt = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        // Check if the message is of type sensor_msgs::Image
        sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
        if (img_msg != nullptr) {
            // Extract the timestamp of the image
            _Float64 ts = img_msg->header.stamp.toSec();
            std::cout << "Image timestamp: " << ts << std::endl;

            // Query pose
            Pose3 T_cam;
            if(!queryPose(poseMap, ts, T_cam)){
                cout << "No pose at " << ts << endl;
                continue;
            }
            SetCameraExtrinsicsFromPose(T_cam.inverse(), camera_params);

            // Add the point cloud to the visualizer
            //vis->ClearGeometries();
            //vis->AddGeometry(std::make_shared<open3d::geometry::PointCloud>(pcd_query.subCloudQuery(ts, 3).ToLegacy()));
            
            // Update camera pose
            vis->GetViewControl().ConvertFromPinholeCameraParameters(camera_params, true);

            vis->PollEvents();
            vis->UpdateRender();

            vis->CaptureScreenImage(workspace + "reconstructed_images/frame_" + std::to_string(cnt) + ".png");
            cnt ++;
        }

        if (!ros::ok()){
            break;
        }
    }

    return 0;
}
