#! /bin/python3 

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from image_rendering import CameraHandle, ImageRenderer
import pandas as pd
import cv2 as cv
import rospy
import yaml
import os
import signal
import rosbag

from t_pointcloud import t_color_by_channel, t_color_enhance_by_channel

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

def readExt(ext_yaml, label):
        with open(ext_yaml, 'r') as ext_yaml:
            ext = yaml.safe_load(ext_yaml)
            return gtsam.Pose3(np.array(ext[label]))

class poseExtractor:
    def __init__(self, nav_file) -> None:
        df = pd.read_csv(nav_file)
        
        n_rows = len(df)
        print(n_rows)
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



IMAGE_INTERVAL = 5  #TODO, align this with optical images (Also move optical images here, no need for them before this happens)
if __name__ == "__main__":
    rospy.init_node("image_reconstruction_node")
    ws = rospy.get_param("/ws")
    
    # Read pointcloud
    pcd = o3d.t.io.read_point_cloud(os.path.join(ws, "processed.ply"))    
    t_color_by_channel(pcd, "intensities", cv.COLORMAP_HOT, 1, 99)
    #t_color_enhance_by_channel(pcd, "intensities", p_upper=30) # Everything with less than 30p intensity should have reduced visibility
    
    # Read bag
    bag = rosbag.Bag(os.path.join(ws, "cooked.bag"))
    
    # Navigation file
    nav_file = os.path.join(ws, "navigation", "nav.csv")
    pose_extractor = poseExtractor(nav_file)
    
    # Get cam->imu transformation
    Tbc = readExt(rospy.get_param("ext_file"), "T_cam_imu").inverse()
    
    # Camera object
    cam = CameraHandle(rospy.get_param("int_file"))
    image_renderer = ImageRenderer(cam, pcd.to_legacy())

    # Iterate over bag
    seq = 0
    for (_, msg, _) in bag.read_messages(topics=["/blackfly_node/image"]):
        ts = msg.header.stamp.to_sec()
        
        Twb = pose_extractor.get_pose(ts)
        if Twb is not None and (seq % 5 == 0):
            Twc = Twb.compose(Tbc)
            
            img = image_renderer.render_image(T_cam=Twc.inverse().matrix())
            
            # Save to file
            cv.imwrite(os.path.join(ws, "images", "topo_enhanced", f"frame_{seq:04d}.png"), img)
            
            cv.imshow("TEST", img)
            k = cv.waitKey(1)
            if k == ord('q'):
                break
        
        seq += 1
        
        if rospy.is_shutdown():
            break