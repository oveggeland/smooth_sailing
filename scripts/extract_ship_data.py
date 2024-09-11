#! /bin/python3 

import numpy as np
import pandas as pd
import rosbag
import sys 
import os

import rospy

import matplotlib.pyplot as plt
from pyproj import Transformer


def project_position(lat, lng, reference_frame="EPSG:6052"):
    projection = Transformer.from_crs("EPSG:4326", "EPSG:6052")
    y_gnss, x_gnss = projection.transform(lat, lng)
    return np.array([x_gnss, y_gnss])


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
    
    df = pd.DataFrame(ship_data, columns=["ts", "x", "y", "z", "roll", "pitch", "heading", "course", "speed"])
    return df    


def plot_ship_data(df):
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Data Overview', fontsize=16)

    # Plot x and y
    cmap = axs[0, 0].scatter(df['y'], df['x'], c=df['ts'].astype(int), cmap='viridis')
    axs[0, 0].set_xlabel('East [m]')
    axs[0, 0].set_ylabel('North [m]')
    axs[0, 0].set_title('Scatter Plot of x and y')
    fig.colorbar(cmap, ax=axs[0, 0], label='Time [s]')

    # Plot roll, pitch, heading, and course
    axs[0, 1].plot(df['ts'].values, df['roll'].values, label='roll', color='c')
    axs[0, 1].plot(df['ts'].values, df['pitch'].values, label='pitch', color='m')
    axs[0, 1].plot(df['ts'].values, df['heading'].values, label='heading', color='y')
    axs[0, 1].plot(df['ts'].values, df['course'].values, label='course', color='k')
    axs[0, 1].set_ylabel('roll / pitch / heading / course')
    axs[0, 1].legend()
    axs[0, 1].set_title('Roll, Pitch, Heading, and Course vs. Time')

    # Plot z
    axs[1, 0].plot(df['ts'].values, df['z'].values, label='z', color='g')
    axs[1, 0].set_ylabel('z')
    axs[1, 0].legend()
    axs[1, 0].set_title('z vs. Time')

    # Plot speed
    axs[1, 1].plot(df['ts'].values, df['speed'].values, label='speed', color='orange')
    axs[1, 1].set_ylabel('speed')
    axs[1, 1].legend()
    axs[1, 1].set_title('Speed vs. Time')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])


if __name__ == "__main__":
    rospy.init_node("ship_data_node")
    
    ws_path = rospy.get_param("/ws")
    df = extract_ship_data(os.path.join(ws_path, "ship.bag"))

    
    df.to_csv(os.path.join(ws_path, "ship_nav.txt"), index=False)
    
    plot_ship_data(df)
    plt.savefig(os.path.join(ws_path, "ship_data.png"))
    plt.show()