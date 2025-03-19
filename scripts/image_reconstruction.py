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

from t_pointcloud import t_color_by_channel, t_get_channel

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
    
            
    # Info file
    info = rospy.get_param("map_config")
    
    dt = readYaml(info, "reconstruction_dt")
    interval = readYaml(info, "reconstruction_interval")
    
    # Make output path
    image_path = os.path.join(ws, "exp", exp, "images", str(dt))
    subfolders = ["rgb", "intensity", "topo", "binary", "optical", "overlay", "fov", "compare"]
    for folder in subfolders:
        os.makedirs(os.path.join(image_path, folder), exist_ok=True)


    # Read fov.meshes
    print("Reading fov meshes")
    fov_masks = {}
    fov_path = os.path.join(ws, "exp", exp, "lidar_fov")
    if dt > 0:
        for file in sorted(os.listdir(fov_path)):
            mesh = o3d.io.read_triangle_mesh(os.path.join(fov_path, file))

            if file[:-4] != "global":
                ts = float(file[:-4])
                fov_masks[ts] = mesh
    else:
        fov_masks[0] = o3d.io.read_triangle_mesh(os.path.join(fov_path, "global.ply"))

    # Read clouds
    rgb_clouds = {}
    intensity_clouds = {}
    topo_clouds = {}
    cloud_path = os.path.join(ws, "exp", exp, "raw_clouds")

    for cloud_file in sorted(os.listdir(cloud_path)):      
        print("Read cloud", cloud_file)
        
        pcd_topo = o3d.t.io.read_point_cloud(os.path.join(cloud_path, cloud_file))
        t_color_by_channel(pcd_topo, "topo", cv.COLORMAP_VIRIDIS, -1, 3)
        
        pcd_intensity = pcd_topo.clone()
        t_color_by_channel(pcd_intensity, "intensities", cv.COLORMAP_COOL, 0, 50)
        
        ts = t_get_channel(pcd_topo, "timestamps")
        intensity_clouds[ts.min()] = pcd_intensity
        topo_clouds[ts.min()] = pcd_topo        
    # Read bag
    bag = rosbag.Bag(os.path.join(ws, "cooked.bag"))
    
    # Navigation file
    nav_file = os.path.join(ws, "exp",  exp, "navigation", "nav.csv")
    pose_extractor = poseExtractor(nav_file)
    
    # Get cam->imu transformation
    Tbc = readExt(rospy.get_param("ext_file"), "T_cam_imu").inverse()
    
    # Camera object
    cam = CameraHandle(rospy.get_param("int_file"))
    
    # render objects
    intensity_renderer = ImageRenderer(cam, intensity_clouds, info)
    topo_renderer = ImageRenderer(cam, topo_clouds, info)
    fov_renderer = FovRenderer(cam, fov_masks, info)
    
    
    # Iterate over bag
    for seq, (_, msg, _) in enumerate(bag.read_messages(topics=["/blackfly_node/image"], 
                                                        start_time=rospy.Time(bag.get_start_time() + readYaml(info, "t0_rel")),
                                                        end_time=rospy.Time(bag.get_start_time() + readYaml(info, "t1_rel")))):
        if rospy.is_shutdown():
            print("ROS IS SHUTDOWN")
            break
        
        if seq < 400 or seq % interval != 0:
            continue # Skip image

        
        ts = msg.header.stamp.to_sec()
        label = f"{ts:.1f}"
        Twb = pose_extractor.get_pose(ts)
        if Twb is None:
            continue # No pose available
        
        # Update clouds to current sliding window
        intensity_renderer.update_pointcloud(ts, dt) 
        topo_renderer.update_pointcloud(ts, dt)
        fov_renderer.update_mesh(ts, dt)

        Twc = Twb.compose(Tbc)
        Tcw = Twc.inverse().matrix()
        
        # Render reconstructed images and fov mask
        img_intensity = intensity_renderer.render_image(Tcw)
        img_topo = topo_renderer.render_image(Tcw)
        img_fov = fov_renderer.render_image(Tcw)         

        # Apply mask to reconstructed images
        where = ~np.any(img_topo, axis=2)
        img_intensity[where] = img_fov[where]
        img_topo[where] = img_fov[where]

        img_optical = cam.get_undistorted_image(msg, True)
        
        # if readYaml(info, "reconstruction_do_overlay"):
        #     img_overlay = img_optical.copy()
        #     alpha = 0.3
        #     img_overlay[img_bin] = (1-alpha)*img_overlay[img_bin] + alpha*np.array([0, 255, 0])# = np.where(where, img_topo, img_optical) # Topo values where relevant
        #     img_overlay = np.where(img_fov, img_overlay, 0.5*img_overlay) # Highlight fov mask
        #     cv.imwrite(os.path.join(image_path, "overlay", f"frame_{label}.png"), img_overlay)
            
        #img_optical = np.where(img_fov, img_optical, 0)
        
        #img_compare = np.vstack((
        #    np.hstack((img_optical, img_rgb)),
        #    np.hstack((img_topo, img_intensity))
        #))
            
        # Save to file
        #cv.imwrite(os.path.join(image_path, "rgb", f"frame_{label}.png"), img_rgb)
        cv.imwrite(os.path.join(image_path, "intensity", f"frame_{label}.png"), img_intensity)
        cv.imwrite(os.path.join(image_path, "topo", f"frame_{label}.png"), img_topo)
        #cv.imwrite(os.path.join(image_path, "binary", f"frame_{label}.png"), img_binary)
        cv.imwrite(os.path.join(image_path, "optical", f"frame_{label}.png"), img_optical)
        #cv.imwrite(os.path.join(image_path, "fov", f"frame_{label}.png"), img_fov)
        #cv.imwrite(os.path.join(image_path, "compare", f"frame_{label}.png"), img_compare)
        
        if rospy.is_shutdown():
            break