#! /bin/python3 

import os
import rospy
import rosbag


if __name__ == "__main__":
    rospy.init_node("cookbook_node")
    
    ws = rospy.get_param("/ws")
    
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