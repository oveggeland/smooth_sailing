#! /bin/python3 

import open3d as o3d
import rospy
import os

from t_pointcloud import t_grid_down_sample, t_filter



def apply_filter(pcd, fnc):
    size0 = pcd.point['positions'].shape[0]
    pcd_filtered = fnc(pcd)
    size1 = pcd_filtered.point['positions'].shape[0]
    print(f"Remaining points: {size1}/{size0}")
    return pcd_filtered


if __name__ == "__main__":
    rospy.init_node("post_processing_node")
    ws = rospy.get_param("/ws")
    
    pcd = o3d.t.io.read_point_cloud(os.path.join(ws, "raw.ply"))

    # Remove points with distance less than some threshold
    d_min, d_max = 15, 100
    print(f"Applying distance threshold ({d_min}, {d_max})")
    pcd = apply_filter(pcd, lambda x: t_filter(x, channel="distances", min=d_min**2, max=d_max**2))
    
    # Statistical outlier removal
    nn, std_ratio = 10, 1
    print(f"Removing statistical outliers with number of neighbors: {nn} std ratio: {std_ratio}")
    pcd = apply_filter(pcd, lambda x: x.remove_statistical_outliers(nn, std_ratio)[0])
    
    # Down-sample based on planar grid
    grid_size = 0.1
    print(f"Perform planar grid down-sampling with grid size: {grid_size}")
    pcd = apply_filter(pcd, lambda x: t_grid_down_sample(x, grid_size))
    
    
    o3d.t.io.write_point_cloud(os.path.join(ws, "processed.ply"), pcd)