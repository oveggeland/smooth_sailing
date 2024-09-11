#! /bin/python3 

import open3d as o3d
import rospy
import os

from t_pointcloud import t_grid_down_sample

if __name__ == "__main__":
    rospy.init_node("processing_node")
    ws = rospy.get_param("/ws")
    cloud_path = os.path.join(ws, "clouds")
    
    pcd = o3d.t.io.read_point_cloud(os.path.join(ws, "raw.pcd"))

    # Common things
    pcd = pcd.translate(-pcd.get_center()) # Normalize position values
    pcd_cleaned, inliers = pcd.remove_radius_outliers(10, 0.2) # Outlier filtering
    
    # Save raw.pcd
    o3d.t.io.write_point_cloud(os.path.join(cloud_path, "clean.ply"), pcd_cleaned)
    
    pcd_ds = pcd_cleaned.voxel_down_sample(0.5)
    o3d.t.io.write_point_cloud(os.path.join(cloud_path, "voxel_sampled05.ply"), pcd_ds)
    
    pcd_ds = pcd_cleaned.voxel_down_sample(0.1)
    o3d.t.io.write_point_cloud(os.path.join(cloud_path, "voxel_sampled01.ply"), pcd_ds)
    
    pcd_grid = t_grid_down_sample(pcd_cleaned, 0.5)
    o3d.t.io.write_point_cloud(os.path.join(cloud_path, "grid_sampled05.ply"), pcd_grid)
    
    pcd_grid = t_grid_down_sample(pcd_cleaned, 0.1)
    o3d.t.io.write_point_cloud(os.path.join(cloud_path, "grid_sampled01.ply"), pcd_grid)