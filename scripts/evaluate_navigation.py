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
    nav_time = nav_time - nav_time.min()
    nav_heading = nav_data["yaw"].to_numpy()*RAD2DEG
    nav_height = nav_data["z"].to_numpy()
    
    ship_time = ship_data["ts"].to_numpy()
    ship_time = ship_time - ship_time.min()
    ship_heading = ship_data["heading"].to_numpy()
    ship_predicted_position = predict_position_from_ship_data(ship_data, lever_arm=lever_arm)
    ship_height_prediction = ship_predicted_position[:, 2]

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    ax0, ax1, ax2, ax3 = axs.flatten()  # Flatten the 2x2 grid into a 1D array for easy access

    # Plot "True heading" on the first subplot
    ax0.plot(ship_time, ship_heading, label="Ship")
    ax0.plot(nav_time, nav_heading, label="System")
    ax0.set_ylabel("True heading [deg]")
    ax0.legend()

    # Plot "Normalized heading" on the second subplot
    ax1.plot(ship_time, ssa(ship_heading - ship_heading[int(ship_time.size/2)]), label="Ship")
    ax1.plot(nav_time, ssa(nav_heading - nav_heading[int(ship_time.size/2)]), label="System")
    ax1.set_ylabel("Normalized heading [deg]")
    ax1.legend()

    # Plot "Comparing height" on the third subplot
    ax2.plot(ship_time, ship_height_prediction - ship_height_prediction.mean(), label="Ship prediction")
    ax2.plot(nav_time, nav_height - nav_height.mean(), label="System")
    ax2.set_ylabel("Vertical displacement [m]")
    ax2.legend()
    
    # Compare roll and pitch
    ship_roll = -ship_data["roll"].values
    ship_pitch = ship_data["pitch"].values

    sys_roll = nav_data["roll"].values*RAD2DEG
    sys_pitch = nav_data["pitch"].values*RAD2DEG
    
    ax3.plot(nav_data["ts"].values, sys_roll - sys_roll.mean(), c='r', linestyle='dashed', label="System roll")
    ax3.plot(nav_data["ts"].values, sys_pitch - sys_pitch.mean(), c='b', linestyle='dashed', label="System pitch")
    ax3.plot(ship_data["ts"].values, ship_roll - ship_roll.mean(), c='g', label="Ship roll")
    ax3.plot(ship_data["ts"].values, ship_pitch - ship_pitch.mean(), c='y',  label="Ship pitch")
    ax3.set_ylabel("Relative attitudes [deg]")
    ax3.legend()

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
    v_ax.plot(nav_data["ts"].values, nav_data["vx"].values, label="Velocity X")
    v_ax.plot(nav_data["ts"].values, nav_data["vy"].values, label="Velocity Y")
    v_ax.plot(nav_data["ts"].values, nav_data["vz"].values, label="Velocity Z")
    v_ax.set_xlabel('Time [s]')
    v_ax.set_ylabel('Velocity [m/s]')
    v_ax.legend()

    # Orientation (Roll, Pitch, Yaw)
    rot_ax.plot(nav_data["ts"].values, nav_data["roll"].values, label='Roll')
    rot_ax.plot(nav_data["ts"].values, nav_data["pitch"].values, label='Pitch')
    rot_ax.plot(nav_data["ts"].values, nav_data["yaw"].values, label='Yaw')
    rot_ax.set_xlabel('Time [s]')
    rot_ax.set_ylabel('Angle [rad]')
    rot_ax.legend()

    # Acceleration bias
    acc_bias_ax.plot(nav_data["ts"].values, nav_data["bax"].values, label='Acc Bias X')
    acc_bias_ax.plot(nav_data["ts"].values, nav_data["bay"].values, label='Acc Bias Y')
    acc_bias_ax.plot(nav_data["ts"].values, nav_data["baz"].values, label='Acc Bias Z')
    acc_bias_ax.set_xlabel('Time [s]')
    acc_bias_ax.set_ylabel('Acceleration Bias [m/s²]')
    acc_bias_ax.legend()

    # Gyroscope bias
    gyro_bias_ax.plot(nav_data["ts"].values, nav_data["bgx"].values, label='Gyro Bias X')
    gyro_bias_ax.plot(nav_data["ts"].values, nav_data["bgy"].values, label='Gyro Bias Y')
    gyro_bias_ax.plot(nav_data["ts"].values, nav_data["bgz"].values, label='Gyro Bias Z')
    gyro_bias_ax.set_xlabel('Time [s]')
    gyro_bias_ax.set_ylabel('Gyroscope Bias [rad/s]')
    gyro_bias_ax.legend()

    # Relative height
    height_ax.plot(nav_data["ts"].values, nav_data["z"].values)
    height_ax.set_xlabel('Time [s]')
    height_ax.set_ylabel('Relative height [m]')

    # Adjust layout to avoid overlapping of labels
    plt.tight_layout(pad=3.0)


def evaluate_gnss(ship_data, gnss_data, nav_data, lever_arm):
    # Ship prediction
    pred_pos = predict_position_from_ship_data(ship_data, lever_arm)
    pred_time = ship_data["ts"].values
    pred_north, pred_east = pred_pos[:, 0], pred_pos[:, 1]
    
    m_time, m_north, m_east = gnss_data.values.T

    # Find the min and max times
    t_min, t_max = max(pred_time.min(), m_time.min()), min(pred_time.max(), m_time.max())
    
    m_idx = ( m_time >= t_min ) & ( m_time <= t_max )
    m_time, m_north, m_east = m_time[m_idx], m_north[m_idx], m_east[m_idx]
    
    pred_east = np.interp(m_time, pred_time, pred_east)
    pred_north = np.interp(m_time, pred_time, pred_north)
    d_east = pred_east - m_east
    d_north = pred_north - m_north
    
    fig, axs = plt.subplots(1, 3, figsize=(15, 5), num="GNSS")
    ax0, ax1, ax2 = axs.flatten()
    
    ax0.scatter(pred_east, pred_north, c=m_time, marker='o', label="Ship prediction")
    ax0.scatter(m_east, m_north, marker='x', label="Measured position")
    ax0.legend()
        
    cmap = ax1.scatter(d_east, d_north, c=m_time)
    fig.colorbar(cmap, ax=ax1)
    
    ax2.plot(m_time, d_east, label="Eastern deviation")
    ax2.plot(m_time, d_north, label="Northern deviation")
    ax2.legend()
    

if __name__ == "__main__":
    rospy.init_node("navigation_evaluation_node")

    nav_config = rospy.get_param("nav_config")
    ws = rospy.get_param("/ws")
    
    # Extract ship data
    ship_data = pd.read_csv(os.path.join(ws, "navigation", "ship.csv"))
    
    # Extract navigation estimates
    nav_data = pd.read_csv(os.path.join(ws, "navigation", "nav.csv"))
    
    gnss_data = pd.read_csv(os.path.join(ws, "navigation", "gnss.csv"))
    
    lever_arm = find_in_yaml(nav_config, "lever_arm")
    
    
    evaluate_gnss(ship_data, gnss_data, nav_data, lever_arm)
    plt.savefig(os.path.join(ws, "navigation", "gnss_evaluation.png"))    
    
    compare_navigation(nav_data, ship_data, lever_arm)
    plt.savefig(os.path.join(ws, "navigation", "ship_comparison.png"))    

    plot_navigation(nav_data)
    plt.savefig(os.path.join(ws, "navigation", "trajectory.png"))
    
    
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("Plotting interrupted. Closing all figures.")
        plt.close('all')
