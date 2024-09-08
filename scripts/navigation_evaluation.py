#! /bin/python3 

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as Rot

import pandas as pd

import rospy
import yaml
import os
import signal

DEG2RAD = (np.pi / 180)
RAD2DEG = (1 / DEG2RAD)


def shutdown_hook():
    plt.close('all')

signal.signal(signal.SIGINT, lambda sig, frame: shutdown_hook())
signal.signal(signal.SIGTERM, lambda sig, frame: shutdown_hook())


def predict_offset(rpy, lever_arm):
    return Rot.from_euler('xyz', rpy, degrees=True).as_matrix() @ lever_arm

def predict_position_from_ship_data(ship_data, lever_arm):
    # Extract pose of ship
    rpy = ship_data[["roll", "pitch", "heading"]].to_numpy()
    
    # Estimate lever arm offset from ship orientation
    xyz_offset = Rot.from_euler('xyz', rpy, degrees=True).as_matrix() @ lever_arm
        
    return ship_data[["x", "y", "z"]].to_numpy() + xyz_offset


def ssa(angles, deg=True):
    halfrot = 180 if deg else np.pi
    angles = np.where(angles < -halfrot, angles + 2*halfrot, angles)
    angles = np.where(angles > halfrot, angles - 2*halfrot, angles)
    return angles

def compare_navigation(nav_data, ship_data, lever_arm):
    nav_time = nav_data["ts"].to_numpy()
    nav_heading = nav_data["yaw"].to_numpy()*RAD2DEG
    nav_height = nav_data["z"].to_numpy()
    
    ship_time = ship_data["ts"].to_numpy()
    ship_heading = ship_data["heading"].to_numpy()
    ship_predicted_position = predict_position_from_ship_data(ship_data, lever_arm=lever_arm)
    ship_height_prediction = ship_predicted_position[:, 2]

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    axs = axs.flatten()  # Flatten the 2x2 grid into a 1D array for easy access

    # Plot "True heading" on the first subplot
    axs[0].plot(ship_time, ship_heading, label="ship")
    axs[0].plot(nav_time, nav_heading, label="sys")
    axs[0].set_title("True heading")
    axs[0].legend()

    # Plot "Normalized heading" on the second subplot
    axs[1].plot(ship_time, ssa(ship_heading - ship_heading[0]), label="ship")
    axs[1].plot(nav_time, ssa(nav_heading - nav_heading[0]), label="sys")
    axs[1].set_title("Normalized heading")
    axs[1].legend()

    # Plot "Comparing height" on the third subplot
    axs[2].plot(ship_time, ship_height_prediction - ship_height_prediction.mean(), label="ship")
    axs[2].plot(nav_time, nav_height - nav_height.mean(), label="sys")
    axs[2].set_title("Comparing height")
    axs[2].legend()
    
    # Compare position and predicted position
    ax = axs[3]
    
    # First scatter plot with t0 for color
    tmin = min(nav_time.min(), ship_time.min())
    tmax = min(nav_time.max(), ship_time.max())
    
    ax.scatter(nav_data['y'], nav_data['x'], c=nav_time, vmin=tmin, vmax=tmax, cmap='viridis', label='Navigation estimate',  marker='o')
    cmap = ax.scatter(ship_predicted_position[:, 1], ship_predicted_position[:, 0], c=ship_time, vmin=tmin, vmax=tmax, cmap='viridis',
                      label='Ship prediction', marker='v')

    # Add colorbars for each scatter plot
    cbar1 = fig.colorbar(cmap, ax=ax, orientation='vertical', label='Time')

    # Add labels and title
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_title('Scatter plot with two datasets')
    ax.legend()

    # Show the plot
    plt.tight_layout()

def find_in_yaml(file, key, default=None):
    with open(file, "r") as yaml_file:
        content = yaml.safe_load(yaml_file)
        
        try:
            return content[key]
        except:
            print(file, "does not contain key", key)
            return default
        

def extract_nav_data(path):
    return  pd.read_csv(path, sep=",")



def plot_navigation(nav_data):
    # Create the figure and subplots (3x2 layout)
    fig, axs = plt.subplots(3, 2, figsize=(12, 10), num="Overview")
    pos_ax, v_ax, rot_ax, acc_bias_ax, gyro_bias_ax, height_ax = axs.flatten()

    # Cartesian position (North-East)
    cmap = pos_ax.scatter(nav_data["y"], nav_data["x"], c=nav_data["ts"], cmap='viridis')
    pos_ax.set_xlabel('East [m]')
    pos_ax.set_ylabel('North [m]')
    pos_ax.set_title('Cartesian position')
    fig.colorbar(cmap, ax=pos_ax, label='Time [s]')

    # Velocity
    v_ax.plot(nav_data["vx"], label="Velocity X")
    v_ax.plot(nav_data["vy"], label="Velocity Y")
    v_ax.plot(nav_data["vz"], label="Velocity Z")
    v_ax.set_xlabel('Time [s]')
    v_ax.set_ylabel('Velocity [m/s]')
    v_ax.legend()

    # Orientation (Roll, Pitch, Yaw)
    rot_ax.plot(nav_data["roll"], label='Roll')
    rot_ax.plot(nav_data["pitch"], label='Pitch')
    rot_ax.plot(nav_data["yaw"], label='Yaw')
    rot_ax.set_xlabel('Time [s]')
    rot_ax.set_ylabel('Angle [rad]')
    rot_ax.legend()

    # Acceleration bias
    acc_bias_ax.plot(nav_data["bax"], label='Acc Bias X')
    acc_bias_ax.plot(nav_data["bay"], label='Acc Bias Y')
    acc_bias_ax.plot(nav_data["baz"], label='Acc Bias Z')
    acc_bias_ax.set_xlabel('Time [s]')
    acc_bias_ax.set_ylabel('Acceleration Bias [m/s²]')
    acc_bias_ax.legend()

    # Gyroscope bias
    gyro_bias_ax.plot(nav_data["bgx"], label='Gyro Bias X')
    gyro_bias_ax.plot(nav_data["bgy"], label='Gyro Bias Y')
    gyro_bias_ax.plot(nav_data["bgz"], label='Gyro Bias Z')
    gyro_bias_ax.set_xlabel('Time [s]')
    gyro_bias_ax.set_ylabel('Gyroscope Bias [rad/s]')
    gyro_bias_ax.legend()

    # Relative height
    height_ax.plot(nav_data["z"])
    height_ax.set_xlabel('Time [s]')
    height_ax.set_ylabel('Relative height [m]')

    # Adjust layout to avoid overlapping of labels
    plt.tight_layout(pad=3.0)

if __name__ == "__main__":
    rospy.init_node("navigation_evaluation_node")

    
    config_file = rospy.get_param("config_file")
    ws = find_in_yaml(config_file, "workspace")
    
    # Extract ship data
    ship_data = pd.read_csv(os.path.join(ws, "ship_nav.txt"))
    
    # Extract navigation estimates
    nav_data = extract_nav_data(os.path.join(ws, "nav.txt"))
    
    plot_navigation(nav_data)
    plt.savefig(os.path.join(ws, "trajectory.png"))
    
    compare_navigation(nav_data, ship_data, find_in_yaml(config_file, "lever_arm"))
    plt.savefig(os.path.join(ws, "ship_comparison.png"))
    
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("Plotting interrupted. Closing all figures.")
        plt.close('all')
