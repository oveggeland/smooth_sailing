#! /bin/python3 

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation as Rot

import pandas as pd

from gnss import project_position

DEG2RAD = (np.pi / 180)
RAD2DEG = (1 / DEG2RAD)

def extract_ship_data(bagpath):
    bag = rosbag.Bag(bagpath)
    
    ship_cnt = bag.get_type_and_topic_info()[1]["/ship/nav"].message_count
    ship_data = np.zeros((ship_cnt, 9)) # ts, north, east, heave, roll, pitch, heading, course, speed

    for i, (_, msg, _) in enumerate(rosbag.Bag(bagpath).read_messages(topics=["/ship/nav"])):
        ship_data[i, 0] = msg.header.stamp.to_sec()
        
        x_north, x_east = project_position(msg.lat, msg.lng)
        ship_data[i, 1] = x_north
        ship_data[i, 2] = x_east
        
        ship_data[i, 3] = msg.heave
        ship_data[i, 4] = msg.roll
        ship_data[i, 5] = msg.pitch
        ship_data[i, 6] = msg.heading
        ship_data[i, 7] = msg.cog
        ship_data[i, 8] = msg.sog
    
    return ship_data    


def extract_nav_data(path):
    df = pd.read_csv(path, sep=", ")
    return df.to_numpy()



def predict_height_from_ship_data(ship_data, lever_arm):
    n = ship_data.shape[0]
    
    height = np.zeros(n)
    for i in range(ship_data.shape[0]):
        height[i] = (Rot.from_euler('xyz', ship_data[i, 4:7], degrees=True).as_matrix() @ lever_arm)[2]

    return height





def compare_navigation(ship_data, nav_data):
    # Ship stuff
    t_ship, north_ship, east_ship = ship_data[:, :3].T
    heading_ship = ship_data[:, 6]
    h_pred = predict_height_from_ship_data(ship_data, lever_arm=np.array([0, 10, -17]))
    
    
    t_sys = nav_data.loc[:, 't'].to_numpy()
    heading_sys = nav_data.loc[:, ' y.1'].to_numpy()*RAD2DEG + 180 # Force heading to be in range (0, 360), not (-180, 180)
    height_sys = nav_data.loc[:, ' z'].to_numpy()

    
    plt.figure("True heading")
    plt.plot(t_ship, heading_ship, label="ship")
    plt.plot(t_sys, heading_sys, label="sys")
    plt.legend()
    
    plt.figure("Normalized heading")
    plt.plot(t_ship, heading_ship - heading_ship.mean(), label="ship")
    plt.plot(t_sys, heading_sys - heading_sys.mean(), label="sys")
    plt.legend()
    
    plt.figure("Comparing height")
    plt.plot(t_ship, h_pred - h_pred.mean(), label="ship")
    plt.plot(t_sys, height_sys - height_sys.mean(), label="sys")
    plt.legend()
    
    plt.show()
    
    
def prepare_ship_data(bagpath, outpath):
    data = extract_ship_data(bagpath)
    np.savetxt(outpath, data, header="ts north east heave roll pitch heading course speed")

if __name__ == "__main__":
    # Load the CSV file without a header
    ship_data_bag = "/home/oskar/navigation/data/right/motion_1.bag"
    #ship_data_bag = "/home/oskar/navigation/data/right/ice_motion.bag"
    ship_data_csv = "/home/oskar/navigation/src/smooth_sailing/data/ship_traj.txt"
    #prepare_ship_data(ship_data_bag, ship_data_csv)

    ship_data = np.loadtxt(ship_data_csv)

    nav_data_path = "/home/oskar/navigation/src/smooth_sailing/data/traj.txt"
    nav_data = pd.read_csv(nav_data_path, sep=",")
    
    compare_navigation(ship_data, nav_data)