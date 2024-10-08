#! /bin/python3 

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from scipy.optimize import minimize

import pandas as pd

import rospy
import yaml
import os
import signal
from scipy.interpolate import interp1d

DEG2RAD = (np.pi / 180)
RAD2DEG = (1 / DEG2RAD)

"""
Interpolate values from source onto time stamps of target
"""
def interpolate(t_target, y_target, t_source, y_source):
    t_min, t_max = max(t_target.min(), t_source.min()), min(t_target.max(), t_source.max())

    target_idx = (t_target >= t_min) & (t_target <= t_max)
    
    return t_target[target_idx], y_target[target_idx], np.interp(t_target[target_idx], t_source, y_source)
        
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

# Smallest signed angle
def ssa(angles, deg=True):
    halfrot = 180 if deg else np.pi
    angles = np.where(angles < -halfrot, angles + 2*halfrot, angles)
    angles = np.where(angles > halfrot, angles - 2*halfrot, angles)
    return angles

# Smalles unsigned angle
def sua(angles, deg=True):
    rot = 360 if deg else 2*np.pi
    angles = np.where(angles < 0, angles + rot, angles)
    angles = np.where(angles > rot, angles - rot, angles)
    return angles

def compare_navigation(nav_data, ship_data, lever_arm, m_dz):
    nav_time = nav_data["ts"].to_numpy()
    nav_heading = nav_data["yaw"].to_numpy()*RAD2DEG
    nav_height = nav_data["z"].to_numpy()
    
    ship_time = ship_data["ts"].to_numpy()
    ship_heading = ship_data["heading"].to_numpy()
    ship_predicted_position = predict_position_from_ship_data(ship_data, lever_arm=lever_arm)
    ship_height_prediction = ship_predicted_position[:, 2]

    fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    ax0, ax1, ax2, ax3 = axs.flatten()  # Flatten the 2x2 grid into a 1D array for easy access

    # Plot "True heading" on the first subplot
    ax0.plot(ship_time, ship_heading, label="Ship")
    ax0.plot(nav_time, nav_heading, label="System")
    ax0.set_ylabel("True heading [deg]")
    ax0.legend()

    # Plot "Normalized heading" on the second subplot
    t0, t1 = max(ship_time.min(), nav_time.min()), min(ship_time.max(), nav_time.max())
    int_idx = (nav_time >= t0) & (nav_time <= t1)
    ship_heading_int = np.interp(nav_time[int_idx], ship_time, ssa(ship_heading))
    
    diff = sua(ship_heading_int - nav_heading[int_idx])
    
    ax1.plot(nav_time[int_idx], diff)
    ax1.axhline(diff.mean(), linestyle="dashed")
    ax1.set_ylabel("Difference in heading [deg]")
    ax1.legend()

    # Plot "Comparing height" on the third subplot
    ship_pred_norm = ship_height_prediction - ship_height_prediction.mean()
    nav_norm = nav_height - nav_height.mean()
    
    t, y_target, y_source = interpolate(ship_time, ship_pred_norm, nav_time, nav_norm)
    
    rmse = np.sqrt(np.mean((y_source-y_target)**2))
    #rmse = np.sqrt(np.mean((ship_pred_norm)**2))
    
    ax2.plot(t, y_target, label="Ship prediction")
    ax2.plot(t, y_source, label="System estimate")
    ax2.scatter(m_dz["ts"].values,- m_dz["altitude"].values + m_dz["altitude"].values.mean(), c='r', marker='x', label="Virtual measurement")
    ax2.set_title(f"RMSE: {rmse}")
    ax2.set_ylabel(f"Vertical displacement [m]")
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
    
    ax3.set_xlim(nav_time[0], nav_time[-1])

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
    height_ax.plot(nav_data["ts"].values, nav_data["z"].values, label="Altitude")
    #height_ax.plot(nav_data["ts"].values, nav_data["bz"].values, label="Altitude bias")
    height_ax.set_xlabel('Time [s]')
    height_ax.set_ylabel('Vertical displacement [m]')

    # Adjust layout to avoid overlapping of labels
    plt.tight_layout(pad=3.0)


def error(m_xy, ship_data, lever_arm):
    pred_pos = predict_position_from_ship_data(ship_data, lever_arm)[:, :2]
    return np.sum((pred_pos[:, :2] - m_xy)**2)
    

def find_optimal_lever_arm(ship_data, gnss_data, t0, t1):
    m_t, m_x, m_y = gnss_data.values.T
    
    t_ship = ship_data["ts"].values
    
    idx = (t_ship >= t0) & (t_ship <= t1)
    ship_data = ship_data.iloc[idx]
    
    # TODO: This is a bit shady, interpolation should probably be on ship data instead
    m_x = np.interp(ship_data["ts"].values, m_t, m_x)
    m_y = np.interp(ship_data["ts"].values, m_t, m_y)
    
    fnc = lambda L: error(np.stack((m_x, m_y), axis=1), ship_data, L)
    L0 = np.array([0, 9.5, -17])
    
    res = minimize(fnc, L0)
    print(res.x)

    


def evaluate_gnss(ship_data, gnss_data, nav_data, lever_arm):
    # Ship prediction
    pred_pos = predict_position_from_ship_data(ship_data, lever_arm)
    pred_time = ship_data["ts"].values
    pred_north, pred_east = pred_pos[:, 0], pred_pos[:, 1]
    
    m_time, m_north, m_east = gnss_data.values.T

    # Find the min and max times
    t_min, t_max = max(pred_time.min(), m_time.min()), min(pred_time.max(), m_time.max())
    find_optimal_lever_arm(ship_data, gnss_data, t_min, t_max)
    
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
    #ax2.plot(pred_time, ship_data["heading"].values*DEG2RAD, label="Ship heading")
    ax2.legend()
    
    
def heaading_vs_gnss(ship_data, gnss_data, nav_data, lever_arm):
    # Ship prediction
    pred_pos = predict_position_from_ship_data(ship_data, lever_arm)
    pred_time = ship_data["ts"].values
    pred_north, pred_east = pred_pos[:, 0], pred_pos[:, 1]
    
    m_time, m_north, m_east = gnss_data.values.T

    # Find the min and max times
    t_min, t_max = max(pred_time.min(), m_time.min()), min(pred_time.max(), m_time.max())
    find_optimal_lever_arm(ship_data, gnss_data, t_min, t_max)
    
    m_idx = ( m_time >= t_min ) & ( m_time <= t_max )
    m_time, m_north, m_east = m_time[m_idx], m_north[m_idx], m_east[m_idx]
    
    pred_east = np.interp(m_time, pred_time, pred_east)
    pred_north = np.interp(m_time, pred_time, pred_north)
    d_east = pred_east - m_east
    d_north = pred_north - m_north
    
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(15, 5), num="Heading vs GNSS deviation")
    ax0, ax1, ax2 = axs.flatten()
    
    # 
    ship_heading=ship_data["heading"].values
    sys_heading=nav_data["yaw"].values*DEG2RAD
    ax0.plot(pred_time, ship_heading, label="Ship heading")
    ax0.legend()

    
    ax1.plot(m_time, d_east, label="Eastern deviation")
    ax1.plot(m_time, d_north, label="Northern deviation")
    #ax2.plot(pred_time, ship_data["heading"].values*DEG2RAD, label="Ship heading")
    ax1.legend()

    # Biases
    ax2.plot(nav_data["ts"].values, nav_data["gnss_bias_north"].values, label="North")
    ax2.plot(nav_data["ts"].values, nav_data["gnss_bias_east"].values, label="East")
    ax2.legend()



def align_data(ship_data, nav_data):
    t_ship = ship_data["ts"].values
    t_nav = nav_data["ts"].values
    
    t0, t1 = max(t_ship.min(), t_nav.min()), min(t_ship.max(), t_nav.max())
    valid_ship_idx = (t_ship > t0) & (t_ship < t1)
    valid_nav_idx = (t_nav > t0) & (t_nav < t1)
    
    ship_data_aligned = ship_data[valid_ship_idx]

    interp = interp1d(t_nav, nav_data.values, axis=0)
    nav_data_aligned = pd.DataFrame(interp(t_ship[valid_ship_idx]), columns=nav_data.columns)
    
    #nav_data_aligned = np.interp(t_ship[valid_ship_idx], np.ones_like(nav_data.values[valid_nav_idx])*t_nav[valid_nav_idx].reshape((-1, 1)), nav_data.values[valid_nav_idx])
    
    return ship_data_aligned, nav_data_aligned
    
    
if __name__ == "__main__":
    rospy.init_node("navigation_evaluation_node")

    nav_config = rospy.get_param("nav_config")
    ws = rospy.get_param("/ws")
    exp = rospy.get_param("/exp")
    nav_path = os.path.join(ws, "exp", exp, "navigation")
    
    # Extract ship data
    ship_data = pd.read_csv(os.path.join(ws, "ship.csv"))
    
    # Extract navigation estimates
    nav_data = pd.read_csv(os.path.join(nav_path, "nav.csv"))
    
    # Align ship and nav data (by interpolation)
    ship_data_aligned, nav_data_aligned = align_data(ship_data, nav_data)
    
    ship_data_aligned.to_csv(os.path.join(nav_path, "ship_aligned.csv"), index=False)
    nav_data_aligned.to_csv(os.path.join(nav_path, "nav_aligned.csv"), index=False)


    gnss_data = pd.read_csv(os.path.join(nav_path, "gnss.csv"))
    
    lever_arm = find_in_yaml(nav_config, "lever_arm")
    
    m_dz = pd.read_csv(os.path.join(nav_path, "height.csv"))
    
    heaading_vs_gnss(ship_data, gnss_data, nav_data, lever_arm)
    
    evaluate_gnss(ship_data, gnss_data, nav_data, lever_arm)
    plt.savefig(os.path.join(nav_path, "gnss_evaluation.png"))    

    compare_navigation(nav_data, ship_data, lever_arm, m_dz)
    plt.savefig(os.path.join(nav_path, "ship_comparison.png"))    

    plot_navigation(nav_data)
    plt.savefig(os.path.join(nav_path, "trajectory.png"))
    
    
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("Plotting interrupted. Closing all figures.")
        plt.close('all')
