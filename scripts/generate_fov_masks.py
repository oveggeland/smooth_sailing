#! /bin/python3 

import numpy as np
import open3d as o3d

DEG2RAD = np.pi / 180

def get_camera_points(ch):
    u0, v0, w, h = ch.optimal_roi
    uv = np.array([
        [u0, u0+w, u0+w, u0],
        [v0, v0, v0+h, v0+h],
        [1, 1, 1, 1]
    ])

    K = ch.optimal_mtx
    return np.linalg.solve(K, uv)

def get_lidar_points(fov, theta_steps=180):
    radius = np.tan(0.5*DEG2RAD*fov)
    theta_linspace = np.linspace(2*np.pi, 0, theta_steps, dtype=np.float32)

    points = np.zeros((3, theta_steps))
    points[0] = 1
    points[1] = radius*np.cos(theta_linspace)
    points[2] = radius*np.sin(theta_linspace)

    return points

class FovGenerator:
    def __init__(self, points, max_dist, save_path) -> None:
        self.points = points
        self.max_dist = max_dist
        
        self.save_path = save_path
        if not os.path.isdir(save_path):
            os.makedirs(save_path)

    def new_mesh(self, T, ts):
        height = T[2, 3]
        wPlp = T[:3, :3] @ self.points

        # Deal with under horizon points
        below_horizon = wPlp[2] > 0
        wPlp_below = wPlp[:, below_horizon]
        wPlp_below = wPlp_below * (-height / wPlp_below[2])

        max_xy = np.sqrt(self.max_dist**2 - height**2) 
        xy_norm = np.linalg.norm(wPlp_below[:2], axis=0)
        out_of_bounds = xy_norm > max_xy
        wPlp_below[:2, out_of_bounds] = wPlp_below[:2, out_of_bounds] * (max_xy / xy_norm[out_of_bounds])

        wPlp[:, below_horizon] = wPlp_below

        # Deal with over horizon points
        wPlp_above = wPlp[:, ~below_horizon]
        wPlp_above = wPlp_above * (height / wPlp_above[2])
        wPlp_above[2] = - wPlp_above[2]
        wPlp_above[:2] = wPlp_above[:2] * (max_xy / np.linalg.norm(wPlp_above[:2], axis=0))
        wPlp[:, ~below_horizon] = wPlp_above

        # Rotate to correct frame and save
        wPwp = wPlp + T[:3, 3].reshape((3, 1))
        wPwp[2] = 0

        # Create and save mesh
        vertices = wPwp.T
        triangles = np.array([
            np.zeros(vertices.shape[0]-3),
            np.arange(1, vertices.shape[0]-2),
            np.arange(2, vertices.shape[0]-1)
        
        ]).T

        fov_mesh = o3d.geometry.TriangleMesh(
            o3d.cpu.pybind.utility.Vector3dVector(vertices), 
            o3d.cpu.pybind.utility.Vector3iVector(triangles))        

        o3d.io.write_triangle_mesh(os.path.join(self.save_path, f"{ts:.2f}.ply"), fov_mesh)

import os
import rospy 
import rosbag
import gtsam
import pandas as pd

def interpolate_pose3(pose1, pose2, t):
    assert 0 <= t <= 1, "Interpolation factor t must be between 0 and 1"

    # Linear on translation
    t_int = (1 - t) * pose1.translation() + t * pose2.translation()
    
    # Spherical interpolation on rotation
    rot_int = pose1.rotation().slerp(t, pose2.rotation())

    return gtsam.Pose3(rot_int, t_int)

class poseExtractor:
    def __init__(self, nav_file) -> None:
        df = pd.read_csv(nav_file)
        
        n_rows = len(df)
        self.ts = np.zeros(n_rows)
        self.poses = np.empty(n_rows, dtype=object)
        
        for i, row in df.iterrows():
            # Store timestamp
            self.ts[i] = row['ts']
            
            # Create Pose3 object using (x, y, z, roll, pitch, yaw)
            translation = gtsam.Point3(row['x'], row['y'], row['z'])
            rotation = gtsam.Rot3.Ypr(row['yaw'], row['pitch'], row['roll'])  # Yaw, pitch, roll order
            
            # Store Pose3 object in the array
            self.poses[i] = gtsam.Pose3(rotation, translation)


    def get_pose(self, t_query):
        idx = np.searchsorted(self.ts, t_query, side="left")
        if idx == 0 or idx == self.ts.size:
            return None
        
        t0, t1 = self.ts[idx-1], self.ts[idx]
        pose0, pose1 = self.poses[idx-1], self.poses[idx]
        
        t = (t_query - t0) / (t1 - t0)
        return interpolate_pose3(pose0, pose1, t)

import yaml

def readExt(ext_yaml, label):
        with open(ext_yaml, 'r') as ext_yaml:
            ext = yaml.safe_load(ext_yaml)
            return gtsam.Pose3(np.array(ext[label]))

if __name__ == "__main__":
    rospy.init_node("fov_generator_node")
    
    ws = rospy.get_param("/ws")
    exp = rospy.get_param("/exp")
    
    exp_path = os.path.join(ws, "exp", exp)
    
    bag = rosbag.Bag(os.path.join(ws, "cooked.bag"))
    
    # Using conservative estimates!!
    z0 = -16.5
    fov = 70
    
    points = get_lidar_points(fov)
    fg = FovGenerator(points, 100, os.path.join(exp_path, "fov"))
    poseQuery = poseExtractor(os.path.join(exp_path, "navigation", "nav.csv"))
    
    ext = rospy.get_param("ext_file")
    cTl = readExt(ext, "T_lidar_cam").inverse()
    bTc = readExt(ext, "T_cam_imu").inverse()
    bTl = bTc.compose(cTl)
    
    # Iterate through all LiDAR frame poses and generate a fov mask
    for (_, msg, _) in bag.read_messages(topics=["/livox_lidar_node/pointcloud2"]):
        ts = msg.header.stamp.to_sec()
        
        
        wTb = poseQuery.get_pose(ts)
        if wTb is None:
            continue
        
        wTl = wTb.compose(bTl)

        fg.new_mesh(wTl.matrix(), ts)
        