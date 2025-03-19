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
            
        self.global_mesh = o3d.geometry.TriangleMesh()
        
        
    def save_global(self):
        self.global_mesh.simplify_vertex_clustering(0.5)
        o3d.io.write_triangle_mesh(os.path.join(self.save_path, f"global.ply"), self.global_mesh)

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
            np.zeros(vertices.shape[0]-2),
            np.arange(1, vertices.shape[0]-1),
            np.arange(2, vertices.shape[0])
        
        ]).T
        

        fov_mesh = o3d.geometry.TriangleMesh(
            o3d.cpu.pybind.utility.Vector3dVector(vertices), 
            o3d.cpu.pybind.utility.Vector3iVector(triangles))        

        o3d.io.write_triangle_mesh(os.path.join(self.save_path, f"{ts:.2f}.ply"), fov_mesh)

        self.global_mesh += fov_mesh
        
        if rospy.is_shutdown():
            exit()

import os
import rospy 
import rosbag
import gtsam
import pandas as pd
from image_rendering import CameraHandle

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

def find_in_yaml(file, key, default=None):
    with open(file, "r") as yaml_file:
        content = yaml.safe_load(yaml_file)
        
        try:
            return content[key]
        except:
            print(file, "does not contain key", key)
            return default

if __name__ == "__main__":
    rospy.init_node("fov_generator_node")
    
    ws = rospy.get_param("/ws")
    exp = rospy.get_param("/exp")
    
    exp_path = os.path.join(ws, "exp", exp)
    
    bag = rosbag.Bag(os.path.join(ws, "cooked.bag"))
    # Using conservative estimates!!
    info = rospy.get_param("map_config")
    
    fov = find_in_yaml(info, "lidar_fov")
    lidar_max_dist = find_in_yaml(info, "lidar_max_dist")
    camera_max_dist = find_in_yaml(info, "camera_max_dist")
    
    lidar_points = get_lidar_points(fov)
    lidar_fov_generator = FovGenerator(lidar_points, lidar_max_dist, os.path.join(exp_path, "lidar_fov"))
    
    cam = CameraHandle(rospy.get_param("int_file"))
    camera_points = get_camera_points(cam)
    cam_fov_generator = FovGenerator(camera_points, camera_max_dist, os.path.join(exp_path, "camera_fov"))
    
    poseQuery = poseExtractor(os.path.join(exp_path, "navigation", "nav.csv"))
    
    ext = rospy.get_param("ext_file")
    cTl = readExt(ext, "T_lidar_cam").inverse()
    bTc = readExt(ext, "T_cam_imu").inverse()
    bTl = bTc.compose(cTl)
    
    # Iterate through all LiDAR frame poses and generate a fov mask
    for (topic, msg, _) in bag.read_messages(topics=["/livox_lidar_node/pointcloud2", "/blackfly_node/image"]):
        #1725811901.5753195 1725812101.5755522
        ts = msg.header.stamp.to_sec()
        
        wTb = poseQuery.get_pose(ts)
        if wTb is None:
            continue
        
        if topic == "/livox_lidar_node/pointcloud2":
            wTl = wTb.compose(bTl).matrix()
            lidar_fov_generator.new_mesh(wTl, ts)
        elif topic == "/blackfly_node/image":
            wTc = wTb.compose(bTc).matrix()
            cam_fov_generator.new_mesh(wTc, ts)
        
    lidar_fov_generator.save_global()
    cam_fov_generator.save_global()