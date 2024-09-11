#! /bin/python3 
import os
import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def save_images_from_bag(bag_file, image_topic, output_folder):
    # Initialize CvBridge
    bridge = CvBridge()

    # Check if the output folder exists, create if it doesn't
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        # Iterate over messages in the specified topic
        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            # Convert the ROS image message to a CV2 image
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Create a filename for the image
            image_filename = os.path.join(output_folder, 'image_{:04d}.png'.format(t.to_nsec()))

            # Save the image to the specified folder
            cv2.imwrite(image_filename, cv_image)

            print(f"Saved image: {image_filename}")


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('image_extractor', anonymous=True)
    
    ws = rospy.get_param("/ws")

    # Path to the ROS bag file
    bag_file = os.path.join(ws, "cooked.bag")

    # Image topic to extract
    image_topic = '/blackfly_node/image'

    # Output folder to save images
    output_folder = os.path.join(ws, "images")

    # Extract and save images
    save_images_from_bag(bag_file, image_topic, output_folder)