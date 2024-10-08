#! /bin/python3 

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from image_rendering import CameraHandle, ImageRenderer, FovRenderer
import pandas as pd
import cv2 as cv
import rospy
import yaml
import os
import signal
import rosbag

from t_pointcloud import t_color_by_channel, t_color_enhance_by_channel, t_get_channel, t_color_relative_topo

import pandas as pd
import open3d as o3d
import gtsam

DEG2RAD = (np.pi / 180)
RAD2DEG = (1 / DEG2RAD)

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
    rospy.init_node("image_reconstruction_node")
    ws = rospy.get_param("/ws")
    exp = rospy.get_param("/exp")
    
    # Make output path
    image_path = os.path.join(ws, "exp", exp, "images")
    subfolders = ["topo", "binary", "optical", "overlay"]
    for folder in subfolders:
        if not os.path.exists(os.path.join(image_path, folder)):
            os.makedirs(os.path.join(image_path, folder))


    # Read fov.meshes
    fov_masks = {}
    fov_path = os.path.join(ws, "exp", exp, "fov")
    for file in sorted(os.listdir(fov_path)):
        mesh = o3d.io.read_triangle_mesh(os.path.join(fov_path, file))
        
        ts = float(file[:-4])
        fov_masks[ts] = mesh

    # Read clouds
    clouds = {}
    cloud_path = os.path.join(ws, "exp", exp, "processed_clouds")
    for cloud_file in sorted(os.listdir(cloud_path)):
        pcd = o3d.t.io.read_point_cloud(os.path.join(cloud_path, cloud_file))
        #pcd = t_normalized_topo_color(pcd)
        #t_color_by_channel(pcd, "intensities", cv.COLORMAP_HOT, 1, 99)
        #t_color_enhance_by_channel(pcd, "intensities", p_upper=95) # Everything with less than 30p intensity should have reduced visibility
        t_color_relative_topo(pcd)

        ts = t_get_channel(pcd, "timestamps")
        clouds[ts.min()] = pcd
    
        
    # Info file
    info = rospy.get_param("map_config")
    
    dt = readYaml(info, "reconstruction_dt")
    interval = readYaml(info, "reconstruction_interval")
    
    # Read bag
    bag = rosbag.Bag(os.path.join(ws, "cooked.bag"))
    
    # Navigation file
    nav_file = os.path.join(ws, "exp",  exp, "navigation", "nav.csv")
    pose_extractor = poseExtractor(nav_file)
    
    # Get cam->imu transformation
    Tbc = readExt(rospy.get_param("ext_file"), "T_cam_imu").inverse()
    
    # Camera object
    cam = CameraHandle(rospy.get_param("int_file"))
    image_renderer = ImageRenderer(cam, clouds, info)
    fov_renderer = FovRenderer(cam, fov_masks, info)
    
    
    # Iterate over bag
    for seq, (_, msg, _) in enumerate(bag.read_messages(topics=["/blackfly_node/image"], 
                                                        end_time=rospy.Time(bag.get_start_time() + rospy.get_param("max_time_interval")))):
        if rospy.is_shutdown():
            break
        
        if seq % interval != 0:
            continue # Skip image

        
        ts = msg.header.stamp.to_sec()
        Twb = pose_extractor.get_pose(ts)
        if Twb is None:
            continue # No pose available
        
        # Update clouds to current sliding window
        image_renderer.update_pointcloud(ts, dt) 
        fov_renderer.update_mesh(ts, dt)

        Twc = Twb.compose(Tbc)
        img_topo = image_renderer.render_image(Twc.inverse().matrix()) # Topography image 
        img_topo = np.expand_dims(cv.cvtColor(img_topo, cv.COLOR_BGR2GRAY), 2)
        img_binary = np.where(img_topo, 255, 0).astype(np.uint8)

        img_optical = cam.get_undistorted_image(msg, True)
        
        R = Twc.rotation()
        t = Twc.translation()
        t[2] = -16.5
        Twc = gtsam.Pose3(R, t)
        
        img_fov = fov_renderer.render_image(Twc.inverse().matrix()) 
        
        cv.imshow("TEST", img_fov)
        k = cv.waitKey(1)
        if k == ord('q'):
            break
        
        # Save to file
        cv.imwrite(os.path.join(image_path, "topo", f"frame_{seq:04d}.png"), img_topo)
        cv.imwrite(os.path.join(image_path, "binary", f"frame_{seq:04d}.png"), img_binary)
        cv.imwrite(os.path.join(image_path, "optical", f"frame_{seq:04d}.png"), img_optical)
        cv.imwrite(os.path.join(image_path, "fov", f"frame_{seq:04d}.png"), img_fov)
        
        if readYaml(info, "reconstruction_do_overlay"):
            img_overlay = np.where(img_topo, img_topo, img_optical) # Topo values where relevant
            img_overlay = np.where(img_fov, img_overlay, 0.5*img_overlay) # Highlight fov mask
            cv.imwrite(os.path.join(image_path, "overlay", f"frame_{seq:04d}.png"), img_overlay)