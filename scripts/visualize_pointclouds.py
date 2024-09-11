#! /bin/python3 

import open3d as o3d
import rospy
import os

def update_pointcloud(vis, pointclouds, current_index):
    # Save current view control parameters
    view_control = vis.get_view_control()
    camera_params = view_control.convert_to_pinhole_camera_parameters()

    # Clear and update the new point cloud
    vis.clear_geometries()
    vis.add_geometry(pointclouds[current_index])
    vis.update_geometry(pointclouds[current_index])

    # Restore the view
    view_control.convert_from_pinhole_camera_parameters(camera_params)
    
    # Refresh the visualizer
    vis.update_renderer()


def visualize_pointclouds(pointclouds, pointcloud_names):
    current_index = [0]  # To modify inside the callback
    
    # Key callback for switching point clouds
    def right_arrow_callback(vis):
        current_index[0] = (current_index[0] + 1) % len(pointclouds)
        update_pointcloud(vis, pointclouds, current_index[0])
        print(f"Showing PointCloud: {pointcloud_names[current_index[0]]}")

    def left_arrow_callback(vis):
        current_index[0] = (current_index[0] - 1) % len(pointclouds)
        update_pointcloud(vis, pointclouds, current_index[0])
        print(f"Showing PointCloud: {pointcloud_names[current_index[0]]}")
        
    # Create the visualization window
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=f"PointCloud Viewer: {pointcloud_names[current_index[0]]}")

    # Add the first point cloud
    update_pointcloud(vis, pointclouds, current_index[0])
    vis.get_view_control().set_zoom(0.7)
    
    # Register key callbacks for left and right arrow keys
    vis.register_key_callback(262, right_arrow_callback)  # Right arrow key
    vis.register_key_callback(263, left_arrow_callback)   # Left arrow key

    # Run the visualization
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    rospy.init_node("visualizer_node")
    
    ws = rospy.get_param("/ws")
    cloud_path = os.path.join(ws, "clouds")
    
    clouds = []
    names = []
    
    for name in os.listdir(cloud_path):
        print("Loading cloud: ", name)

        names.append(name)
        
        cloud = o3d.t.io.read_point_cloud(os.path.join(cloud_path, name)).to_legacy()
        #cloud = cloud.translate(-cloud.get_center())
        clouds.append(cloud)
        
    visualize_pointclouds(clouds, names)