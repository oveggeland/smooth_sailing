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



void buildPointCloud(std::map<double, Pose3> poseMap, const std::string& bag_filename) {
    // Open the bag
    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);

    // Define the topic of interest
    std::string topic = "/livox_lidar_node/pointcloud2";
    
    // Set up a ROSBag view for the topic
    rosbag::View view(bag, rosbag::TopicQuery(topic));

    cout << std::fixed << std::setprecision(20); // Set precision


    open3d::geometry::PointCloud point_cloud;// = open3d::geometry::PointCloud(); // Initialize pointcloud

    // Iterate over the messages
    for (rosbag::MessageInstance const m : view) {
        // Check if the message is a PointCloud2 message
        sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_msg != nullptr) {
            double t0 = cloud_msg->header.stamp.toSec();

            pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
            pcl::fromROSMsg(*cloud_msg, pcl_cloud);

            cout << "New point cloud at " << t0 << endl;
            cout << "Number of points is: " << pcl_cloud.points.size() << endl;

            int cnt = 0;

            std::vector<Eigen::Vector3d> transformed_points;
            transformed_points.reserve(pcl_cloud.points.size());

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
                    gtsam::Point3 gtsam_point(point.x, point.y, point.z);
                    transformed_points.push_back(T.transformFrom(gtsam_point));
                }
                cnt ++;
            }

            point_cloud += open3d::geometry::PointCloud(transformed_points);
        }
    }


    // Save the PointCloud to a file
    std::string filename = "/home/oskar/navigation/data/ws_right2/raw.pcd";
    open3d::io::WritePointCloud(filename, point_cloud);

    // Visualize pointcloud in c++
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Open3D PointCloud Viewer", 800, 600);

    // Add the PointCloud to the visualizer
    visualizer.AddGeometry(std::make_shared<open3d::geometry::PointCloud>(point_cloud));

    // Run the visualizer
    visualizer.Run();

    // Close the visualizer window
    visualizer.DestroyVisualizerWindow();

    // Close the bag    
    bag.close();
}





int main(){
    cout << std::fixed << std::setprecision(20); // Set precision

    std::map<double, Pose3> poseMap = buildPoseMap("/home/oskar/navigation/data/ws_right2/nav.txt");
    

    buildPointCloud(poseMap, "/home/oskar/navigation/data/ws_right2/raw.bag");

    // double t_query = 1725632406;

    // Pose3 query_pose = queryPose(poseMap, t_query);
    // cout << query_pose.rotation().rpy() << endl;
    // cout << query_pose.translation() << endl;

    // Last 1725632700.00046491622924804688
    // First 1725632400.99974894523620605469

    return 0;
}