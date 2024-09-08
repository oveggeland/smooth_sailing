#! /bin/python3 

import rosbag
import sys

if __name__ == "__main__":
    in_bag = sys.argv[1]
    out_bag = in_bag[:-4] + "_cooked.bag"

    with rosbag.Bag(out_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(in_bag).read_messages():
            # This also replaces tf timestamps under the assumption 
            # that all transforms in the message share the same timestamp
            if topic == "/tf" and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)