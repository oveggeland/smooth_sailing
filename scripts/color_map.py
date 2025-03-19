#! /bin/python3 

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import cv2 as cv

from scipy.optimize import minimize

import pandas as pd

import rospy
import yaml
import os
import signal
from scipy.interpolate import interp1d
import rosbag
import open3d as o3d

import copy

from image_rendering import CameraHandle
from t_pointcloud import t_get_channel

DEG2RAD = (np.pi / 180)
RAD2DEG = (1 / DEG2RAD)

import gtsam

def shutdown_hook():
    print("SHUTDOWN")
    rospy.signal_shutdown("SHUT THE HELL DOWN")

signal.signal(signal.SIGINT, lambda sig, frame: shutdown_hook())
signal.signal(signal.SIGTERM, lambda sig, frame: shutdown_hook())

def create_image_map(cam, bagpath, t0, t1):
    bag = rosbag.Bag(bagpath)
    
    img_map = {}
    for _, msg, _ in bag.read_messages(topics=["/blackfly_node/image"]):
        ts = msg.header.stamp.to_sec()
        img = cam.get_undistorted_image(msg, False)
        
        img_map[ts] = img
            
    return img_map


def get_closest_image_idx(img_map, ts):
    t_image = img_map.keys()
    idx = np.searchsorted(t_image, ts)

    return img_map[idx]


def interpolate_pose3(pose1, pose2, t):
    assert 0 <= t <= 1, "Interpolation factor t must be between 0 and 1"

    # Linear on translation
    t_int = (1 - t) * pose1.translation() + t * pose2.translation()
    
    # Spherical interpolation on rotation
    rot_int = pose1.rotation().slerp(t, pose2.rotation())

    return gtsam.Pose3(rot_int, t_int)
    

def readYaml(file, label):
    with open(file, 'r') as f:
        return yaml.safe_load(f)[label]
    
 

def readExt(ext_yaml, label):
        with open(ext_yaml, 'r') as ext_yaml:
            ext = yaml.safe_load(ext_yaml)
            return gtsam.Pose3(np.array(ext[label]))

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


if __name__ == "__main__":
    rospy.init_node("post_processing_node")
    ws = rospy.get_param("/ws")
    exp = rospy.get_param("/exp")
    
    bagpath = os.path.join(ws, "cooked.bag")
    bag = rosbag.Bag(bagpath)
    
    cam = CameraHandle(rospy.get_param("int_file"))
    
    exp_path = os.path.join(ws, "exp", exp)
    raw_cloud_path = os.path.join(exp_path, "raw_clouds")
    colored_cloud_path = os.path.join(exp_path, "colored_clouds")
    os.makedirs(colored_cloud_path, exist_ok=True)
    
    pose_extractor = poseExtractor(os.path.join(exp_path, "navigation", "nav.csv"))
    
    # Get cam->imu transformation
    Tbc = readExt(rospy.get_param("ext_file"), "T_cam_imu").inverse()
    
    for cloud_name in sorted(os.listdir(raw_cloud_path)):
        pcd = o3d.t.io.read_point_cloud(os.path.join(raw_cloud_path, cloud_name))
        pcd.paint_uniform_color((0.0, 0.0, 0.0))
        
        ts = t_get_channel(pcd, "timestamps").flatten()
        pcd.point.color_count = np.zeros((ts.size, 1))
        
        t0, t1 = ts.min(), ts.max()
        print(f"Next cloud at time interval: ({t0}, {t1})")


        # Iterate over images
        for _, msg, _ in bag.read_messages(topics=["/blackfly_node/image"], start_time=rospy.Time(t0), end_time=rospy.Time(t1)):
            t_image = msg.header.stamp.to_sec()
            print("Image at:", t_image)
        
            Twb = pose_extractor.get_pose(t_image)
            if Twb is None:
                continue # No pose available
        
            T_cam = Twb.compose(Tbc).inverse()
            img = cam.get_undistorted_image(msg, False)
            img = cv.cvtColor(img, cv.COLOR_RGB2BGR) / 255.0
        
            inlier_mask = abs(ts-t_image) < 3
            inlier_idx = np.where(inlier_mask)[0]
        
            pcd_inliers = pcd.select_by_mask(inlier_mask)
            pcd_inliers.transform(T_cam.matrix())
            
            xyz = t_get_channel(pcd_inliers, "positions")
            
            uv = cam.project_points(xyz.T)
            in_fov = cam.uv_in_roi(uv)     

        
            if np.any(in_fov):
                uv = np.rint(uv[:, in_fov]).astype(int)
                inlier_idx = inlier_idx[in_fov]
                colors = img[uv[1], uv[0]]

                counts = pcd.point.color_count[inlier_idx].numpy()

                pcd.point.colors[inlier_idx] = (pcd.point.colors[inlier_idx]*counts + colors) / (counts+1)
                pcd.point.color_count[inlier_idx] += 1
            
            if rospy.is_shutdown():
                break
            
        
        
        # Now let's remove the non-colored points
        counts = t_get_channel(pcd, "color_count").flatten()
        pcd = pcd.select_by_mask(counts > 0)    
    
        o3d.t.io.write_point_cloud(os.path.join(colored_cloud_path, cloud_name), pcd)
    
        #o3d.visualization.draw_geometries([pcd.to_legacy()])
        
        if rospy.is_shutdown():
            break