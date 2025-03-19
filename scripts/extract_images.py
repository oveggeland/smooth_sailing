#! /bin/python3 
import os
import rospy
import rosbag
import cv2
from image_rendering import CameraHandle


def extract_images(ws, interval=5, calib_file="/home/oskar/smooth_sailing/src/smooth_sailing/cfg/calib/int_right.yaml"):
    bag_file = os.path.join(ws, "cooked.bag")
    output_folder = os.path.join(ws, "images", "optical")
    cam = CameraHandle(calib_file)

    # Check if the output folder exists, create if it doesn't
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        # Iterate over messages in the specified topic
        seq = 0
        for topic, msg, t in bag.read_messages(topics=["/blackfly_node/image"]):
            if (seq % interval) == 0:
                label = str(int(1000*msg.header.stamp.to_sec())) + ".png"
                img = cam.get_undistorted_image(msg, crop_to_roi=True)
                cv2.imwrite(os.path.join(output_folder, label), img)
                
                
                if rospy.is_shutdown():
                    break
                
            seq += 1
            
            
if __name__ == "__main__":
    ws ="/home/oskar/smooth_sailing/data/long3"
    extract_images(ws, 1)