#! /bin/python3 
import os
import rospy
import rosbag
import numpy as np
import cv2
from pyproj import Transformer
import pandas as pd
from image_rendering import CameraHandle

def cookbook(ws):
    in_bag = os.path.join(ws, "raw.bag")
    out_bag = os.path.join(ws, "cooked.bag")

    with rosbag.Bag(out_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(in_bag).read_messages():
            # This also replaces tf timestamps under the assumption 
            # that all transforms in the message share the same timestamp
            if topic == "/tf" and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
                
def extract_images(ws, interval=5):
    bag_file = os.path.join(ws, "raw.bag")
    output_folder = os.path.join(ws, "images", "optical")
    cam = CameraHandle(rospy.get_param("int_file"))

    # Check if the output folder exists, create if it doesn't
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        # Iterate over messages in the specified topic
        seq = 0
        for topic, msg, t in bag.read_messages(topics=["/blackfly_node/image"]):
            if (seq % interval) == 0:
                img = cam.get_undistorted_image(msg, crop_to_roi=True)
                cv2.imwrite(os.path.join(output_folder, f"frame_{seq:04d}.png"), img)
                
                if rospy.is_shutdown():
                    break
                
                
            seq += 1
            

def project_position(lat, lng, reference_frame="EPSG:6052"):
    projection = Transformer.from_crs("EPSG:4326", "EPSG:6052")
    y_gnss, x_gnss = projection.transform(lat, lng)
    return np.array([x_gnss, y_gnss])

def extract_ship_data(ws):
    bag = rosbag.Bag(os.path.join(ws, "raw.bag")) # We know ship data comes in correct order
    
    msg_count = bag.get_type_and_topic_info()[1]["/ship/nav"].message_count
    ship_data = np.zeros((msg_count, 9)) # ts, north, east, heave, roll, pitch, heading, course, speed

    for i, (_, msg, _) in enumerate(bag.read_messages(topics=["/ship/nav"])):
        ship_data[i, 0] = msg.header.stamp.to_sec()
        
        x_north, x_east = project_position(msg.lat, msg.lng) # TODO: This is not adjusted with the same offset as navigation data
        ship_data[i, 1] = x_north
        ship_data[i, 2] = x_east
        
        ship_data[i, 3] = msg.heave
        ship_data[i, 4] = msg.roll
        ship_data[i, 5] = msg.pitch
        ship_data[i, 6] = msg.heading
        ship_data[i, 7] = msg.cog
        ship_data[i, 8] = msg.sog
    
    df = pd.DataFrame(ship_data, columns=["ts", "x", "y", "z", "roll", "pitch", "heading", "course", "speed"])
    
    # Check if the output folder exists, create if it doesn't
    output_path = os.path.join(ws, "navigation")
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    df.to_csv(os.path.join(output_path, "ship.csv"))


if __name__ == "__main__":
    rospy.init_node("cookbook_node")
    
    ws = rospy.get_param("/ws")
    
    print("Cooking data bag...")
    cookbook(ws)
    
    print("Extracting images...")
    extract_images(ws, interval=5)
    
    print("Extracting ship data...")
    extract_ship_data(ws)